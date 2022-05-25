// Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
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

#include "TessendorfWaves.hpp"
#include "MeshUtils.hpp"

#include <chrono>
#include <list>
#include <memory>
#include <mutex>
#include <string>

#include <OgreItem.h>
#include <OgreMesh2.h>
#include <ignition/rendering/ogre2/Ogre2Conversions.hh>
#include <ignition/rendering/ogre2/Ogre2Geometry.hh>
#include <ignition/rendering/ogre2/Ogre2Mesh.hh>

#include <ignition/rendering/Visual.hh>
#include <ignition/rendering/Mesh.hh>
#include <ignition/rendering/Scene.hh>
#include "ignition/rendering/RenderingIface.hh"

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/Entity.hh>
#include "ignition/gazebo/rendering/Events.hh"
#include <ignition/gazebo/components/Name.hh>

namespace buoy
{
struct TessendorfWavesPrivate
{
  std::mutex mutex;
  std::string visualName;
  ignition::rendering::VisualPtr visual;
  ignition::rendering::ScenePtr scene;
  ignition::gazebo::Entity entity = ignition::gazebo::kNullEntity;
  std::chrono::steady_clock::duration currentSimTime;
  ignition::common::ConnectionPtr connection{nullptr};
  
  void OnUpdate();
};

TessendorfWaves::TessendorfWaves()
  : dataPtr(std::make_unique<TessendorfWavesPrivate>())
{

}

void TessendorfWaves::Configure(const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &_eventMgr)
{
  this->dataPtr->entity = _entity;
  auto nameComp = _ecm.Component<ignition::gazebo::components::Name>(_entity);
  this->dataPtr->visualName = nameComp->Data();

  // connect to the SceneUpdate event
  // the callback is executed in the rendering thread so do all
  // rendering operations in that thread
  this->dataPtr->connection =
    _eventMgr.Connect<ignition::gazebo::events::SceneUpdate>(
    std::bind(&TessendorfWavesPrivate::OnUpdate, this->dataPtr.get()));
}

//////////////////////////////////////////////////
void TessendorfWaves::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->currentSimTime = _info.simTime;
}

//////////////////////////////////////////////////
void TessendorfWavesPrivate::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->visualName.empty())
    return;

  if (!this->scene)
    this->scene = ignition::rendering::sceneFromFirstRenderEngine();

  if (!this->scene)
    return;

  if (!this->visual)
  {
    // this does a breadth first search for visual with the entity id
    // \todo(anyone) provide a helper function in RenderUtil to search for
    // visual by entity id?
    auto rootVis = scene->RootVisual();
    std::list<ignition::rendering::NodePtr> nodes;
    nodes.push_back(rootVis);
    while (!nodes.empty())
    {
      auto n = nodes.front();
      nodes.pop_front();
      if (n && n->HasUserData("gazebo-entity"))
      {
        // RenderUti stores gazebo-entity user data as int
        // \todo(anyone) Change this to uint64_t in Ignition H?
        auto variant = n->UserData("gazebo-entity");
        const int *value = std::get_if<int>(&variant);
        if (value && *value == static_cast<int>(this->entity))
        {
          this->visual = std::dynamic_pointer_cast<ignition::rendering::Visual>(n);
          break;
        }
      }
      for (unsigned int i = 0; i < n->ChildCount(); ++i)
        nodes.push_back(n->ChildByIndex(i));
    }
  }

  if (!this->visual)
    return;

  ignition::rendering::GeometryPtr geo = this->visual->GeometryByIndex(0U);
  ignition::rendering::MeshPtr derived = \
    std::dynamic_pointer_cast<ignition::rendering::Mesh>(geo);
  ignition::rendering::Ogre2MeshPtr ogre_derived = \
    std::dynamic_pointer_cast<ignition::rendering::Ogre2Mesh>(derived);
  Ogre::MovableObject *movable = ogre_derived->OgreObject();
  Ogre::Item *ogreItem = static_cast<Ogre::Item*>(movable);

  // mesh data to retrieve
  size_t vertexCount;
  size_t indexCount;
  Ogre::Vector3 *vertices;
  Ogre::uint32 *indices;

  // Get the mesh information
  mesh_utils::getMeshInformation(ogreItem->getMesh(), vertexCount,
    vertices, indexCount, indices,
    ogreItem->getParentNode()->_getDerivedPosition(),
    ogreItem->getParentNode()->_getDerivedOrientation(),
    ogreItem->getParentNode()->_getDerivedScale());
  ignerr << vertexCount << std::endl;
  /*
  
  Ogre::SubMesh* subMesh = mesh->getSubMesh(0);
  Ogre::VertexArrayObjectArray vaos = subMesh->mVao[0];

  if (!vaos.empty())
  {
      const Ogre::VertexBufferPackedVec& vertexBuffers = vaos[0]->getVertexBuffers();
      auto count = vertexBuffers[0]->getNumElements();
      AsyncTicketPtr asyncTicket = vertexBuffers[0]->readRequest(0, vertexBuffers[0]->getNumElements());
      const uint8* vertexData = static_cast<const uint8*>(asyncTicket->map());
      void* data = malloc(vertexBuffers[0]->getTotalSizeBytes());
      memcpy(data, vertexData, vertexBuffers[0]->getTotalSizeBytes());
      asyncTicket->unmap();
      struct Vertices {
          float px, py, pz;
          float nx, ny, nz;
          float texx, texy;
      };
      //some temp manipulation with data
      float* start = reinterpret_cast<float*>(data);
      float px = *start++;
      float py = *start++;
      float pz = *start++;

      py = 44;
      vertexBuffers[0]->upload(data, 0, vertexBuffers[0]->getNumElements());
  }
*/
}
}  // namespace buoy

IGNITION_ADD_PLUGIN(
  buoy::TessendorfWaves,
  ignition::gazebo::System,
  buoy::TessendorfWaves::ISystemConfigure,
  buoy::TessendorfWaves::ISystemPreUpdate)

