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

#include <OgreEntity.h>
#include <OgreMesh.h>
#include <ignition/rendering/ogre/OgreConversions.hh>
#include <ignition/rendering/ogre/OgreGeometry.hh>
#include <ignition/rendering/ogre/OgreMesh.hh>

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
  ignerr << "configured TessendorfWaves on " << this->dataPtr->visualName << std::endl;
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
  ignerr << "1 OnUpdate TessendorfWavesPrivate" << std::endl;

  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->visualName.empty())
    return;
  ignerr << "2 OnUpdate TessendorfWavesPrivate" << std::endl;

  if (!this->scene)
    this->scene = ignition::rendering::sceneFromFirstRenderEngine();
  ignerr << "3 OnUpdate TessendorfWavesPrivate" << std::endl;

  if (!this->scene)
    return;
  ignerr << "4 OnUpdate TessendorfWavesPrivate" << std::endl;

  if (!this->visual)
  {
    ignerr << "5 OnUpdate TessendorfWavesPrivate" << std::endl;

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
  ignerr << "6 OnUpdate TessendorfWavesPrivate" << std::endl;

  if (!this->visual)
    return;

  ignerr << "7 OnUpdate TessendorfWavesPrivate geometry count = " << this->visual->GeometryCount() << std::endl;

  ignition::rendering::GeometryPtr geo = this->visual->GeometryByIndex(0U);
  if (geo)
  {
    ignerr << "7.1 OnUpdate TessendorfWavesPrivate " << geo << std::endl;
  }
  ignerr << "8 OnUpdate TessendorfWavesPrivate" << std::endl;
  ignition::rendering::MeshPtr derived(std::dynamic_pointer_cast<ignition::rendering::Mesh>(geo));
  ignerr << "9 OnUpdate TessendorfWavesPrivate " << derived << std::endl;
  if (derived)
  {
    ignition::rendering::OgreMeshPtr ogre_derived(std::dynamic_pointer_cast<ignition::rendering::OgreMesh>(derived));
    if (ogre_derived)
    {
      ignerr << "9.1 OnUpdate TessendorfWavesPrivate " << ogre_derived << std::endl;
      Ogre::MovableObject *movable = ogre_derived->OgreObject();
      ignerr << "10 OnUpdate TessendorfWavesPrivate" << std::endl;

      Ogre::Entity *ogreEntity = static_cast<Ogre::Entity*>(movable);
      ignerr << "11 OnUpdate TessendorfWavesPrivate" << std::endl;

      // mesh data to retrieve
      size_t vertexCount;
      size_t indexCount;
      Ogre::Vector3 *vertices;
      uint64_t *indices;

      if (ogreEntity)
      {
        ignerr << "12 OnUpdate TessendorfWavesPrivate" << std::endl;
        if (ogreEntity->getMesh().get())
        {
          ignerr << "13 OnUpdate TessendorfWavesPrivate" << std::endl;

          // Get the mesh information
          mesh_utils::getMeshInformation(ogreEntity->getMesh().get(), vertexCount,
            vertices, indexCount, indices,
            ignition::rendering::OgreConversions::Convert(
              ogreEntity->getParentNode()->_getDerivedPosition()),
            ignition::rendering::OgreConversions::Convert(
              ogreEntity->getParentNode()->_getDerivedOrientation()),
            ignition::rendering::OgreConversions::Convert(
              ogreEntity->getParentNode()->_getDerivedScale()));

          ignerr << vertexCount << std::endl;
        }
      }
    }
    else
    {
      ignerr << "ogre derived not init OnUpdate TessendorfWavesPrivate" << std::endl;
    }
  }
  else
  {
    ignerr << "derived not init OnUpdate TessendorfWavesPrivate" << std::endl;
  }
}
}  // namespace buoy

IGNITION_ADD_PLUGIN(
  buoy::TessendorfWaves,
  ignition::gazebo::System,
  buoy::TessendorfWaves::ISystemConfigure,
  buoy::TessendorfWaves::ISystemPreUpdate)

