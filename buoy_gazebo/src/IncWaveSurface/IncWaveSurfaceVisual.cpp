// Copyright 2026 Monterey Bay Aquarium Research Institute
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "IncWaveSurfaceVisual.hpp"

#include <gz/msgs/double_v.pb.h>
#include <Hlms/Pbs/OgreHlmsPbsDatablock.h>
#include <Vao/OgreIndexBufferPacked.h>
#include <Vao/OgreVaoManager.h>
#include <Vao/OgreVertexArrayObject.h>
#include <Vao/OgreVertexBufferPacked.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Event.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Color.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/ogre2/Ogre2Conversions.hh>
#include <gz/rendering/ogre2/Ogre2Material.hh>
#include <gz/rendering/ogre2/Ogre2ParticleEmitter.hh>
#include <gz/rendering/ogre2/Ogre2Scene.hh>
#include <gz/rendering/ogre2/Ogre2Visual.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

#include <OgreHlmsSamplerblock.h>
#include <OgreItem.h>
#include <OgreMesh2.h>
#include <OgreMeshManager2.h>
#include <OgrePixelFormatGpu.h>
#include <OgreRenderSystem.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreStagingTexture.h>
#include <OgreSubMesh2.h>
#include <OgreTextureBox.h>
#include <OgreTextureGpu.h>
#include <OgreTextureGpuManager.h>

#include "IncWaveComponents.hpp"
#include "WaveSurfaceField.hpp"

// Set by CMake to gz-rendering's sky cubemap
#ifndef INC_WAVE_SURFACE_DEFAULT_ENV_MAP
#define INC_WAVE_SURFACE_DEFAULT_ENV_MAP ""
#endif


namespace buoy_gazebo
{
using inc_wave_surface::WaveSurfaceField;

class IncWaveSurfaceVisualPrivate
{
public:
  /// \brief Render thread, every frame
  void OnSceneUpdate();

  /// \brief Render thread, before the render engine goes away
  void OnRenderTeardown();

  /// \brief Fetches the wave components, then computes surfaces
  void WorkerLoop();

  bool FindVisual();
  bool CreateMesh();
  void Upload(double _centerX, double _centerY);
  void UploadNormalMap(const std::vector<std::vector<std::int8_t>> & _levels);
  void DestroyMesh();

  // Settings
  inc_wave_surface::SurfaceParams params;
  std::chrono::duration<double> updatePeriod{1.0 / 30.0};
  std::string followLink{"Buoy"};
  gz::math::Color color{0.02f, 0.15f, 0.25f, 1.0f};
  double opacity{0.75};
  /// \brief Lower makes the sun glints so small that they flicker as the
  /// chop moves
  double roughness{0.3};
  std::string environmentMap{INC_WAVE_SURFACE_DEFAULT_ENV_MAP};

  /// \brief The visual this plugin is attached to
  gz::sim::Entity entity{gz::sim::kNullEntity};

  gz::common::ConnectionPtr sceneUpdateConnection;
  gz::common::ConnectionPtr teardownConnection;

  /// \brief Set when the render thread first runs. Until then the plugin does
  /// nothing, so it costs nothing on a server that doesn't render.
  std::atomic<bool> renderActive{false};

  // PreUpdate's thread only
  gz::sim::Entity followEntity{gz::sim::kNullEntity};
  int followSearches{0};
  bool haveWorld{false};

  // Written by PreUpdate, guarded by stateMutex
  std::mutex stateMutex;
  std::condition_variable stateCv;
  std::string worldName;
  double simTime{0.0};
  bool haveFollow{false};
  double followX{0.0};
  double followY{0.0};
  std::uint64_t stateVersion{0};
  bool stop{false};

  std::thread worker;
  gz::transport::Node node;

  // Worker to render thread, guarded by frameMutex
  std::mutex frameMutex;
  bool layoutReady{false};
  int verticesPerSide{0};
  std::vector<std::uint32_t> indices;
  double boundHalfWidth{1.0};
  double boundHalfHeight{1.0};
  int normalMapSize{0};
  std::vector<float> frontVertices;
  std::vector<std::vector<std::int8_t>> frontNormalMap;
  bool frontNew{false};
  double frontCenterX{0.0};
  double frontCenterY{0.0};

  // Render thread only
  gz::rendering::ScenePtr scene;
  gz::rendering::VisualPtr visual;
  gz::rendering::MaterialPtr material;
  Ogre::SceneManager * sceneManager{nullptr};
  Ogre::SceneNode * ogreNode{nullptr};
  Ogre::Item * item{nullptr};
  Ogre::MeshPtr mesh;
  Ogre::SubMesh * subMesh{nullptr};
  Ogre::VertexBufferPacked * vertexBuffer{nullptr};
  Ogre::TextureGpu * normalTexture{nullptr};
  std::vector<float> renderVertices;
  std::vector<std::vector<std::int8_t>> renderNormalMap;
  bool failed{false};
};

//////////////////////////////////////////////////
void IncWaveSurfaceVisualPrivate::WorkerLoop()
{
  std::string service;
  {
    std::unique_lock<std::mutex> lock(this->stateMutex);
    this->stateCv.wait(lock, [this] {return this->stop || !this->worldName.empty();});
    if (this->stop) {
      return;
    }
    service = IncWaveComponentsService(this->worldName);
  }

  // The components, from the IncWaveSurface plugin on the server
  IncWaveComponents waves;
  const auto start = std::chrono::steady_clock::now();
  bool warned = false;
  while (true) {
    gz::msgs::Double_V reply;
    bool result = false;
    if (this->node.Request(service, 1000u, reply, result) && result &&
      UnpackIncWaveComponents(reply, waves))
    {
      break;
    }
    if (!warned && std::chrono::steady_clock::now() - start > std::chrono::seconds(10)) {
      gzwarn << "IncWaveSurfaceVisual: still waiting for [" << service
             << "]. Is the IncWaveSurface plugin loaded?" << std::endl;
      warned = true;
    }
    std::unique_lock<std::mutex> lock(this->stateMutex);
    if (this->stateCv.wait_for(lock, std::chrono::seconds(1), [this] {return this->stop;})) {
      return;
    }
  }

  WaveSurfaceField field;
  field.Init(this->params, waves);
  gzmsg << "IncWaveSurfaceVisual: " << field.Summary() << std::endl;
  {
    std::lock_guard<std::mutex> lock(this->frameMutex);
    this->verticesPerSide = field.VerticesPerSide();
    this->indices = field.Indices();
    this->boundHalfWidth = field.BoundHalfWidth();
    this->boundHalfHeight = field.BoundHalfHeight();
    this->normalMapSize = field.NormalMapSize();
    this->layoutReady = true;
  }

  // Compute a surface whenever the sim time or the followed link changes, at
  // most once per update period
  std::uint64_t doneVersion = ~std::uint64_t{0};
  std::vector<float> back;
  std::vector<std::vector<std::int8_t>> backNormalMap;
  while (true) {
    double t;
    bool haveFollowNow;
    double x;
    double y;
    {
      std::unique_lock<std::mutex> lock(this->stateMutex);
      this->stateCv.wait(
        lock, [this, doneVersion] {return this->stop || this->stateVersion != doneVersion;});
      if (this->stop) {
        return;
      }
      doneVersion = this->stateVersion;
      t = this->simTime;
      haveFollowNow = this->haveFollow;
      x = this->followX;
      y = this->followY;
    }
    const auto computeStart = std::chrono::steady_clock::now();

    if (haveFollowNow) {
      field.Follow(x, y);
    }
    field.Compute(t, back);
    backNormalMap = field.NormalMapLevels();
    {
      std::lock_guard<std::mutex> lock(this->frameMutex);
      std::swap(back, this->frontVertices);
      std::swap(backNormalMap, this->frontNormalMap);
      this->frontCenterX = field.CenterX();
      this->frontCenterY = field.CenterY();
      this->frontNew = true;
    }

    const auto next = computeStart +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(this->updatePeriod);
    std::unique_lock<std::mutex> lock(this->stateMutex);
    if (this->stateCv.wait_until(lock, next, [this] {return this->stop;})) {
      return;
    }
  }
}

//////////////////////////////////////////////////
void IncWaveSurfaceVisualPrivate::OnSceneUpdate()
{
  GZ_PROFILE("IncWaveSurfaceVisual::OnSceneUpdate");
  if (!this->renderActive) {
    this->renderActive = true;
    this->worker = std::thread(&IncWaveSurfaceVisualPrivate::WorkerLoop, this);
  }
  if (this->failed) {
    return;
  }

  if (this->item == nullptr) {
    {
      std::lock_guard<std::mutex> lock(this->frameMutex);
      if (!this->layoutReady) {
        return;
      }
    }
    if (!this->scene) {
      this->scene = gz::rendering::sceneFromFirstRenderEngine();
      if (!this->scene) {
        return;
      }
    }
    if (!this->FindVisual()) {
      return;
    }
    if (!this->CreateMesh()) {
      this->DestroyMesh();
      this->failed = true;
      return;
    }
  }

  bool fresh = false;
  double centerX = 0.0;
  double centerY = 0.0;
  {
    std::lock_guard<std::mutex> lock(this->frameMutex);
    if (this->frontNew) {
      std::swap(this->frontVertices, this->renderVertices);
      std::swap(this->frontNormalMap, this->renderNormalMap);
      centerX = this->frontCenterX;
      centerY = this->frontCenterY;
      this->frontNew = false;
      fresh = true;
    }
  }
  if (fresh) {
    this->Upload(centerX, centerY);
  }
}

//////////////////////////////////////////////////
bool IncWaveSurfaceVisualPrivate::FindVisual()
{
  if (this->visual) {
    return true;
  }
  // Breadth first search for the visual of this entity, as ShaderParam does
  std::list<gz::rendering::NodePtr> nodes;
  nodes.push_back(this->scene->RootVisual());
  while (!nodes.empty()) {
    auto node = nodes.front();
    nodes.pop_front();
    if (!node) {
      continue;
    }
    if (node->HasUserData("gazebo-entity")) {
      auto variant = node->UserData("gazebo-entity");
      const std::uint64_t * value = std::get_if<std::uint64_t>(&variant);
      if (value && *value == this->entity) {
        this->visual = std::dynamic_pointer_cast<gz::rendering::Visual>(node);
        break;
      }
    }
    for (unsigned int i = 0; i < node->ChildCount(); ++i) {
      nodes.push_back(node->ChildByIndex(i));
    }
  }
  return this->visual != nullptr;
}

//////////////////////////////////////////////////
bool IncWaveSurfaceVisualPrivate::CreateMesh()
{
  auto ogreScene = std::dynamic_pointer_cast<gz::rendering::Ogre2Scene>(this->scene);
  auto ogreVisual = std::dynamic_pointer_cast<gz::rendering::Ogre2Visual>(this->visual);
  if (!ogreScene || !ogreVisual) {
    gzerr << "IncWaveSurfaceVisual needs the ogre2 render engine. No wave surface."
          << std::endl;
    return false;
  }
  this->sceneManager = ogreScene->OgreSceneManager();
  Ogre::VaoManager * vaoManager =
    this->sceneManager->getDestinationRenderSystem()->getVaoManager();

  std::vector<std::uint32_t> triangles;
  int side;
  double halfWidth;
  double halfHeight;
  int textureSize;
  {
    std::lock_guard<std::mutex> lock(this->frameMutex);
    triangles = this->indices;
    side = this->verticesPerSide;
    halfWidth = this->boundHalfWidth;
    halfHeight = this->boundHalfHeight;
    textureSize = this->normalMapSize;
  }
  const std::size_t numVertices = static_cast<std::size_t>(side) * side;
  if (numVertices == 0 || triangles.empty()) {
    gzerr << "IncWaveSurfaceVisual: empty mesh. No wave surface." << std::endl;
    return false;
  }

  // Rewritten whenever the worker has a new surface. Flat until then.
  Ogre::VertexElement2Vec elements;
  elements.push_back(Ogre::VertexElement2(Ogre::VET_FLOAT3, Ogre::VES_POSITION));
  elements.push_back(Ogre::VertexElement2(Ogre::VET_FLOAT3, Ogre::VES_NORMAL));
  elements.push_back(Ogre::VertexElement2(Ogre::VET_FLOAT3, Ogre::VES_TANGENT));
  elements.push_back(Ogre::VertexElement2(Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES));
  std::vector<float> initial(numVertices * WaveSurfaceField::kFloatsPerVertex, 0.0f);
  for (std::size_t v = 0; v < numVertices; ++v) {
    initial[v * WaveSurfaceField::kFloatsPerVertex + 5] = 1.0f;
    initial[v * WaveSurfaceField::kFloatsPerVertex + 6] = 1.0f;
  }
  this->vertexBuffer = vaoManager->createVertexBuffer(
    elements, numVertices, Ogre::BT_DYNAMIC_PERSISTENT, initial.data(), false);

  // The triangles never change
  Ogre::IndexBufferPacked * indexBuffer = vaoManager->createIndexBuffer(
    Ogre::IndexBufferPacked::IT_32BIT, triangles.size(), Ogre::BT_IMMUTABLE,
    triangles.data(), false);

  Ogre::VertexBufferPackedVec vertexBuffers;
  vertexBuffers.push_back(this->vertexBuffer);
  Ogre::VertexArrayObject * vao = vaoManager->createVertexArrayObject(
    vertexBuffers, indexBuffer, Ogre::OT_TRIANGLE_LIST);

  static int meshCount = 0;
  this->mesh = Ogre::MeshManager::getSingleton().createManual(
    "inc_wave_surface_" + std::to_string(meshCount++),
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  this->subMesh = this->mesh->createSubMesh();
  this->subMesh->mVao[Ogre::VpNormal].push_back(vao);
  this->subMesh->mVao[Ogre::VpShadow].push_back(vao);
  // Fixed bounds that hold every surface, around the mesh center
  const Ogre::Aabb bounds(
    Ogre::Vector3::ZERO,
    Ogre::Vector3(
      static_cast<Ogre::Real>(halfWidth), static_cast<Ogre::Real>(halfWidth),
      static_cast<Ogre::Real>(halfHeight)));
  this->mesh->_setBounds(bounds, false);
  this->mesh->_setBoundingSphereRadius(bounds.getRadius());

  // Set up the item as Ogre2Visual::AttachGeometry does, so picking works
  this->item = this->sceneManager->createItem(this->mesh, Ogre::SCENE_DYNAMIC);
  this->item->setName(this->visual->Name() + "_inc_wave_surface");
  this->item->getUserObjectBindings().setUserAny(Ogre::Any(this->visual->Id()));
  this->item->setVisibilityFlags(
    this->visual->VisibilityFlags() &
    ~gz::rendering::Ogre2ParticleEmitter::kParticleVisibilityFlags);
  this->item->setCastShadows(false);

  // Water: a dielectric with sharp reflections. Transparent mode fades the
  // diffuse but keeps the reflections.
  this->material = this->scene->CreateMaterial();
  this->material->SetAmbient(this->color);
  this->material->SetDiffuse(this->color);
  this->material->SetSpecular(gz::math::Color(1.0f, 1.0f, 1.0f, 1.0f));
  this->material->SetRoughness(static_cast<float>(this->roughness));
  this->material->SetMetalness(0.0f);
  this->material->SetCastShadows(false);
  if (!this->environmentMap.empty()) {
    if (gz::common::exists(this->environmentMap)) {
      this->material->SetEnvironmentMap(this->environmentMap);
    } else {
      gzwarn << "IncWaveSurfaceVisual: environment map [" << this->environmentMap
             << "] not found" << std::endl;
    }
  }
  auto ogreMaterial = std::dynamic_pointer_cast<gz::rendering::Ogre2Material>(this->material);
  Ogre::HlmsPbsDatablock * datablock = ogreMaterial->Datablock();
  if (this->opacity < 1.0) {
    datablock->setTransparency(
      static_cast<float>(this->opacity), Ogre::HlmsPbsDatablock::Transparent);
  }
  // Seen from below too
  Ogre::HlmsMacroblock macroblock(*datablock->getMacroblock());
  macroblock.mCullMode = Ogre::CULL_NONE;
  datablock->setMacroblock(macroblock);

  // The chop's normal map, rewritten with the vertices. It has every mip
  // level, so the chop blends smoothly with distance.
  if (textureSize > 0) {
    std::uint8_t levels = 1;
    while ((textureSize >> levels) > 0) {
      ++levels;
    }
    Ogre::TextureGpuManager * textureManager =
      this->sceneManager->getDestinationRenderSystem()->getTextureGpuManager();
    // The PBS shaders sample every texture as a 2D array, so this is one too,
    // with one slice
    static int textureCount = 0;
    this->normalTexture = textureManager->createTexture(
      "inc_wave_surface_chop_" + std::to_string(textureCount++),
      Ogre::GpuPageOutStrategy::Discard, Ogre::TextureFlags::ManualTexture,
      Ogre::TextureTypes::Type2DArray);
    this->normalTexture->setResolution(textureSize, textureSize, 1u);
    this->normalTexture->setPixelFormat(Ogre::PFG_RG8_SNORM);
    this->normalTexture->setNumMipmaps(levels);
    this->normalTexture->scheduleTransitionTo(Ogre::GpuResidency::Resident);

    // Flat until the first surface
    std::vector<std::vector<std::int8_t>> flat(levels);
    for (std::uint8_t level = 0; level < levels; ++level) {
      const std::size_t size = static_cast<std::size_t>(textureSize >> level);
      flat[level].assign(2 * size * size, 0);
    }
    this->UploadNormalMap(flat);

    Ogre::HlmsSamplerblock samplerblock;
    samplerblock.setAddressingMode(Ogre::TAM_WRAP);
    samplerblock.setFiltering(Ogre::TFO_ANISOTROPIC);
    samplerblock.mMaxAnisotropy = 8.0f;
    datablock->setTexture(Ogre::PBSM_NORMAL, this->normalTexture, &samplerblock);
  }
  this->item->setDatablock(datablock);

  // Vertices are relative to the mesh center. This node puts the center in
  // place and undoes the visual's own pose and scale.
  this->ogreNode = ogreVisual->Node()->createChildSceneNode(Ogre::SCENE_DYNAMIC);
  this->ogreNode->setInheritScale(false);
  this->ogreNode->attachObject(this->item);
  return true;
}

//////////////////////////////////////////////////
void IncWaveSurfaceVisualPrivate::Upload(double _centerX, double _centerY)
{
  const std::size_t numVertices = this->vertexBuffer->getNumElements();
  if (this->renderVertices.size() != numVertices * WaveSurfaceField::kFloatsPerVertex) {
    return;
  }
  auto * vertices = static_cast<float *>(this->vertexBuffer->map(0, numVertices));
  std::memcpy(vertices, this->renderVertices.data(), this->renderVertices.size() * sizeof(float));
  this->vertexBuffer->unmap(Ogre::UO_KEEP_PERSISTENT);
  this->UploadNormalMap(this->renderNormalMap);

  // Place the mesh center in the visual's frame
  const gz::math::Pose3d pose = this->visual->WorldPose();
  gz::math::Vector3d scale = this->visual->WorldScale();
  scale.Max(gz::math::Vector3d(1e-9, 1e-9, 1e-9));
  const gz::math::Vector3d center =
    pose.Rot().RotateVectorReverse(gz::math::Vector3d(_centerX, _centerY, 0.0) - pose.Pos()) /
    scale;
  this->ogreNode->setPosition(gz::rendering::Ogre2Conversions::Convert(center));
  this->ogreNode->setOrientation(
    gz::rendering::Ogre2Conversions::Convert(pose.Rot().Inverse()));
}

//////////////////////////////////////////////////
void IncWaveSurfaceVisualPrivate::UploadNormalMap(
  const std::vector<std::vector<std::int8_t>> & _levels)
{
  if (this->normalTexture == nullptr ||
    _levels.size() != this->normalTexture->getNumMipmaps())
  {
    return;
  }
  Ogre::TextureGpuManager * textureManager =
    this->sceneManager->getDestinationRenderSystem()->getTextureGpuManager();
  const std::uint32_t fullSize = this->normalTexture->getWidth();
  for (std::size_t level = 0; level < _levels.size(); ++level) {
    const std::uint32_t size = std::max<std::uint32_t>(1u, fullSize >> level);
    const std::vector<std::int8_t> & texels = _levels[level];
    if (texels.size() != 2u * size * size) {
      return;
    }
    Ogre::StagingTexture * staging = textureManager->getStagingTexture(
      size, size, 1u, 1u, Ogre::PFG_RG8_SNORM);
    staging->startMapRegion();
    Ogre::TextureBox box = staging->mapRegion(size, size, 1u, 1u, Ogre::PFG_RG8_SNORM);
    for (std::uint32_t row = 0; row < size; ++row) {
      std::memcpy(box.at(0, row, 0), &texels[2u * size * row], 2u * size);
    }
    staging->stopMapRegion();
    staging->upload(box, this->normalTexture, static_cast<std::uint8_t>(level));
    textureManager->removeStagingTexture(staging);
  }
}

//////////////////////////////////////////////////
void IncWaveSurfaceVisualPrivate::DestroyMesh()
{
  if (this->item != nullptr && this->sceneManager != nullptr) {
    this->sceneManager->destroyItem(this->item);
  }
  this->item = nullptr;
  if (this->ogreNode != nullptr && this->sceneManager != nullptr) {
    this->sceneManager->destroySceneNode(this->ogreNode);
  }
  this->ogreNode = nullptr;
  // Upload leaves it persistently mapped
  if (this->vertexBuffer != nullptr &&
    this->vertexBuffer->getMappingState() != Ogre::MS_UNMAPPED)
  {
    this->vertexBuffer->unmap(Ogre::UO_UNMAP_ALL);
  }
  if (this->subMesh != nullptr && this->sceneManager != nullptr) {
    // The shadow pass shares the normal pass's VAO
    Ogre::SubMesh::destroyVaos(
      this->subMesh->mVao[Ogre::VpNormal],
      this->sceneManager->getDestinationRenderSystem()->getVaoManager());
    this->subMesh->mVao[Ogre::VpNormal].clear();
    this->subMesh->mVao[Ogre::VpShadow].clear();
  }
  this->subMesh = nullptr;
  this->vertexBuffer = nullptr;
  if (this->mesh) {
    Ogre::MeshManager::getSingleton().remove(this->mesh->getName());
    this->mesh.reset();
  }
  if (this->material && this->scene) {
    this->scene->DestroyMaterial(this->material);
  }
  this->material.reset();
  // After the material, which uses it
  if (this->normalTexture != nullptr && this->sceneManager != nullptr) {
    this->sceneManager->getDestinationRenderSystem()->getTextureGpuManager()->destroyTexture(
      this->normalTexture);
  }
  this->normalTexture = nullptr;
}

//////////////////////////////////////////////////
void IncWaveSurfaceVisualPrivate::OnRenderTeardown()
{
  this->DestroyMesh();
  this->visual.reset();
  this->scene.reset();
}

//////////////////////////////////////////////////
IncWaveSurfaceVisual::IncWaveSurfaceVisual()
: dataPtr(std::make_unique<IncWaveSurfaceVisualPrivate>())
{
}

//////////////////////////////////////////////////
IncWaveSurfaceVisual::~IncWaveSurfaceVisual()
{
  this->dataPtr->sceneUpdateConnection.reset();
  this->dataPtr->teardownConnection.reset();
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->stateMutex);
    this->dataPtr->stop = true;
  }
  this->dataPtr->stateCv.notify_all();
  if (this->dataPtr->worker.joinable()) {
    this->dataPtr->worker.join();
  }
}

//////////////////////////////////////////////////
void IncWaveSurfaceVisual::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & /*_ecm*/,
  gz::sim::EventManager & _eventMgr)
{
  auto & d = *this->dataPtr;
  auto & p = d.params;
  d.entity = _entity;

  d.followLink = _sdf->Get<std::string>("follow_link", d.followLink).first;
  const double rate = _sdf->Get<double>("update_rate", 30.0).first;
  d.updatePeriod = std::chrono::duration<double>(rate > 0.0 ? 1.0 / rate : 0.0);

  p.size = _sdf->Get<double>("size", p.size).first;
  p.fineSize = _sdf->Get<double>("fine_size", p.fineSize).first;
  p.fineSpacing = _sdf->Get<double>("fine_spacing", p.fineSpacing).first;
  p.coarseSpacing = _sdf->Get<double>("coarse_spacing", p.coarseSpacing).first;
  p.maxVerticesPerAxis = _sdf->Get<int>("max_vertices_per_axis", p.maxVerticesPerAxis).first;
  p.skirt = _sdf->Get<double>("skirt", p.skirt).first;

  const std::string chop = _sdf->Get<std::string>("chop", "tail").first;
  if (chop == "none") {
    p.chop = inc_wave_surface::ChopMode::kNone;
  } else if (chop == "wind") {
    p.chop = inc_wave_surface::ChopMode::kWind;
  } else {
    if (chop != "tail") {
      gzwarn << "IncWaveSurfaceVisual: unknown chop [" << chop
             << "], expected none, tail or wind. Using tail." << std::endl;
    }
    p.chop = inc_wave_surface::ChopMode::kTail;
  }
  p.windSpeed = _sdf->Get<double>("wind_speed", p.windSpeed).first;
  if (_sdf->HasElement("wind_direction")) {
    // Compass degrees the wind comes from, to the math angle it blows to, the
    // way IncidentWaves converts WaveDir
    const double from = _sdf->Get<double>("wind_direction");
    p.windDirection = GZ_DTOR(90.0 - (from + 180.0));
  }
  p.chopTailExponent = _sdf->Get<double>("chop_tail_exponent", p.chopTailExponent).first;
  p.chopSpreading = _sdf->Get<double>("chop_spreading", p.chopSpreading).first;
  p.chopCrossover = _sdf->Get<double>("chop_crossover", p.chopCrossover).first;
  p.choppiness = _sdf->Get<double>("choppiness", p.choppiness).first;
  p.chopPatchSize = _sdf->Get<double>("chop_patch_size", p.chopPatchSize).first;
  p.chopResolution = _sdf->Get<int>("chop_resolution", p.chopResolution).first;
  p.chopSeed = _sdf->Get<unsigned int>("chop_seed", p.chopSeed).first;
  p.chopNormalMap = _sdf->Get<bool>("chop_normal_map", p.chopNormalMap).first;
  p.threads = _sdf->Get<int>("threads", p.threads).first;

  d.color = _sdf->Get<gz::math::Color>("color", d.color).first;
  d.opacity = std::clamp(_sdf->Get<double>("opacity", d.opacity).first, 0.0, 1.0);
  d.roughness = std::clamp(_sdf->Get<double>("roughness", d.roughness).first, 0.0, 1.0);
  d.environmentMap = _sdf->Get<std::string>("environment_map", d.environmentMap).first;

  // The render thread does the rendering work, as ShaderParam does
  d.sceneUpdateConnection = _eventMgr.Connect<gz::sim::events::SceneUpdate>(
    std::bind(&IncWaveSurfaceVisualPrivate::OnSceneUpdate, this->dataPtr.get()));
  d.teardownConnection = _eventMgr.Connect<gz::sim::events::RenderTeardown>(
    std::bind(&IncWaveSurfaceVisualPrivate::OnRenderTeardown, this->dataPtr.get()));
}

//////////////////////////////////////////////////
void IncWaveSurfaceVisual::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("IncWaveSurfaceVisual::PreUpdate");
  auto & d = *this->dataPtr;
  if (!d.renderActive) {
    return;
  }

  std::string world;
  if (!d.haveWorld) {
    auto name = _ecm.Component<gz::sim::components::Name>(gz::sim::worldEntity(_ecm));
    if (name != nullptr && !name->Data().empty()) {
      world = name->Data();
      d.haveWorld = true;
    }
  }

  if (!d.followLink.empty() && d.followEntity == gz::sim::kNullEntity) {
    d.followEntity = _ecm.EntityByComponents(
      gz::sim::components::Link(), gz::sim::components::Name(d.followLink));
    if (d.followEntity == gz::sim::kNullEntity && ++d.followSearches == 100) {
      gzwarn << "IncWaveSurfaceVisual: no link named [" << d.followLink
             << "]. The surface stays centered on the origin." << std::endl;
    }
  }
  bool haveFollow = false;
  double x = 0.0;
  double y = 0.0;
  if (d.followEntity != gz::sim::kNullEntity) {
    const gz::math::Pose3d pose = gz::sim::worldPose(d.followEntity, _ecm);
    x = pose.Pos().X();
    y = pose.Pos().Y();
    haveFollow = true;
  }
  const double t = std::chrono::duration<double>(_info.simTime).count();

  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(d.stateMutex);
    if (!world.empty()) {
      d.worldName = world;
      changed = true;
    }
    if (t != d.simTime || haveFollow != d.haveFollow || x != d.followX || y != d.followY) {
      d.simTime = t;
      d.haveFollow = haveFollow;
      d.followX = x;
      d.followY = y;
      changed = true;
    }
    if (changed) {
      ++d.stateVersion;
    }
  }
  if (changed) {
    d.stateCv.notify_all();
  }
}

}  // namespace buoy_gazebo

GZ_ADD_PLUGIN(
  buoy_gazebo::IncWaveSurfaceVisual,
  gz::sim::System,
  buoy_gazebo::IncWaveSurfaceVisual::ISystemConfigure,
  buoy_gazebo::IncWaveSurfaceVisual::ISystemPreUpdate);
