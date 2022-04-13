/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "PneumaticSpring.hh"

#include <ignition/msgs/double.pb.h>

#include <string>

#include <ignition/common/Profiler.hh>
#include <ignition/math/PID.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/JointPosition.hh"
//#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/Model.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::PneumaticSpringPrivate
{


  /// \brief Spring Type
  public: int SpringType;  //TODO:  Need to enumerate this better instead of just using integers


  /// \brief Spring Constant  
  public: double SpringConst;

  /// \brief Piston Diameter (inches)  - Currently Unused
  public: double PistonDiam;

  /// \brief Rod Diameter (inches) - Currently Unused
  public: double RodDiam;

  /// \brief Piston-End Dead Volume (inches^3)  - Currently Unused
  public: double PistonEndVolume_0;

  /// \brief Piston-End Dead Volume (inches^3) - Currently Unused
  public: double RodEndVolume_0;

  /// \brief Joint Entity
  public: Entity jointEntity;

  /// \brief Model interface
  public: Model model{kNullEntity};
};

//////////////////////////////////////////////////
PneumaticSpring::PneumaticSpring()
  : dataPtr(std::make_unique<PneumaticSpringPrivate>())
{
}



/////////////////////////////////////////////////
double SdfParamDouble(
    const std::shared_ptr<const sdf::Element> &_sdf,
    const std::string& _field,
    double _default)
{
  return _sdf->Get<double>(_field, _default).first;
}



//////////////////////////////////////////////////
void PneumaticSpring::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{

  this->dataPtr->SpringType    = SdfParamDouble(_sdf, "SpringType", 0); 
  this->dataPtr->SpringConst    = SdfParamDouble(_sdf, "SpringConst", 1);
  this->dataPtr->PistonDiam    = SdfParamDouble(_sdf, "PistonDiam", 5);
  this->dataPtr->RodDiam    = SdfParamDouble(_sdf, "RodDiam", 1.5);


  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "PneumaticSpring plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF
  auto jointName = _sdf->Get<std::string>("JointName");
  if (jointName.empty())
  {
    ignerr << "PneumaticSpring found an empty jointName parameter. "
           << "Failed to initialize.";
    return;
  }

  this->dataPtr->jointEntity = this->dataPtr->model.JointByName(_ecm,
      jointName);
  if (this->dataPtr->jointEntity == kNullEntity)
  {
    ignerr << "Joint with name[" << jointName << "] not found. "
    << "The PneumaticSpring may not influence this joint.\n";
    return;
  }



}

//////////////////////////////////////////////////
void PneumaticSpring::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("PneumaticSpring::PreUpdate");

  // If the joint hasn't been identified yet, the plugin is disabled
  if (this->dataPtr->jointEntity == kNullEntity)
    return;

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;


  // Create joint position component if one doesn't exist
  auto jointPosComp =
      _ecm.Component<components::JointPosition>(this->dataPtr->jointEntity);
  if (jointPosComp == nullptr)
  {
    _ecm.CreateComponent(
        this->dataPtr->jointEntity, components::JointPosition());
  }
  // We just created the joint position component, give one iteration for the
  // physics system to update its size
  if (jointPosComp == nullptr || jointPosComp->Data().empty())
    return;


  double x = jointPosComp->Data().at(0);  //TODO: Figure out if (0) for the index is always correct, some OR code has a process of finding the index for this argument.
  double force = -this->dataPtr->SpringConst*x;

       auto forceComp =
          _ecm.Component<components::JointForceCmd>(this->dataPtr->jointEntity);
      if (forceComp == nullptr)
      {
        _ecm.CreateComponent(this->dataPtr->jointEntity,
                             components::JointForceCmd({force}));
      }
      else
      { 
          forceComp->Data()[0] += force;  //Add force to existing forces.
      }         

}


IGNITION_ADD_PLUGIN(PneumaticSpring,
                    ignition::gazebo::System,
                    PneumaticSpring::ISystemConfigure,
                    PneumaticSpring::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(PneumaticSpring,
                          "ignition::gazebo::systems::PneumaticSpring")
