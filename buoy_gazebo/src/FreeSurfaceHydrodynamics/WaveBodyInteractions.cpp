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

#include "WaveBodyInteractions.hpp"
#include <ignition/common/Profiler.hh>
#include <ignition/common/Console.hh>
#include <ignition/gazebo/Types.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/math/PID.hh>
#include <ignition/msgs.hh>
#include <ignition/msgs/double.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <stdio.h>
#include "FS_Hydrodynamics.hpp"
#include "LinearIncidentWave.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace Eigen;

namespace buoy_gazebo
{
  class WaveBodyInteractionsPrivate
  {
  public:
    WaveBodyInteractionsPrivate() : FloatingBody(IncRef){}; // Constructor needs to assign reference or compile error of un-assigned referenced occurs

    /// \brief Model interface
    ignition::gazebo::Model model{ignition::gazebo::kNullEntity};

    /// \brief Link entity
    ignition::gazebo::Entity linkEntity;

    /// \brief Incident wave implementation
    LinearIncidentWave Inc;

    /// \brief Reference to Incident wave implementation
    LinearIncidentWave &IncRef = Inc;

    /// \brief  Free-Surface hydrodynamics implementation
    FS_HydroDynamics FloatingBody;

    double TargetWindingCurrent;

    double WindingCurrent;

    double Ve;

    /// \brief Ignition communication node.
    ignition::transport::Node node;

    /// \brief Callback for User Commanded Current subscription
    /// \param[in] _msg Current
    void OnUserCmdCurr(const ignition::msgs::Double &_msg);
  };

  //////////////////////////////////////////////////
  WaveBodyInteractions::WaveBodyInteractions()
      : dataPtr(std::make_unique<WaveBodyInteractionsPrivate>())
  {
  }

  /////////////////////////////////////////////////
  double SdfParamDouble(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_field,
      double _default)
  {
    return _sdf->Get<double>(_field, _default).first;
  }

  //////////////////////////////////////////////////
  void WaveBodyInteractions::Configure(
      const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager & /*_eventMgr*/)
  {
    std::cout << "In Configure " << std::endl;
    this->dataPtr->model = ignition::gazebo::Model(_entity);
    if (!this->dataPtr->model.Valid(_ecm))
    {
      ignerr << "WaveBodyInteractions plugin should be attached to a model entity. "
             << "Failed to initialize." << std::endl;
      return;
    }

    // Get params from SDF.
    if (!_sdf->HasElement("LinkName"))
    {
      ignerr << "You musk specify a <LinkName> for the wavebodyinteraction plugin to act upon"
             << "Failed to initialize." << std::endl;
      return;
    }
    auto linkName = _sdf->Get<std::string>("LinkName");
    std::cout << "LinkName = " << linkName << std::endl;

    this->dataPtr->linkEntity = this->dataPtr->model.LinkByName(_ecm, linkName);
    if (!_ecm.HasEntity(this->dataPtr->linkEntity))
    {
      ignerr << "Link name" << linkName << "does not exist";
      return;
    }

    double A = .5 + ((float)(rand() % 20) / 10);
    double T = 3.0 + (rand() % 9);
    // this->dataptr->Inc.SetToPiersonMoskowitzSpectrum(2*A, 0);
    this->dataPtr->Inc.SetToMonoChromatic(2 * A, T, 0);

    std::string HydrodynamicsBaseFilename = "/home/hamilton/buoy_ws/src/buoy_sim/buoy_gazebo/src/FreeSurfaceHydrodynamics/HydrodynamicCoeffs/BuoyA5";
    this->dataPtr->FloatingBody.ReadWAMITData_FD(HydrodynamicsBaseFilename);
    this->dataPtr->FloatingBody.ReadWAMITData_TD(HydrodynamicsBaseFilename);
    this->dataPtr->FloatingBody.SetTimestepSize(.01); // TODO:  Need to get timestep size from ecm.
  }

  //////////////////////////////////////////////////
  void WaveBodyInteractions::PreUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm)
  {

    IGN_PROFILE("#WaveBodyInteractions::PreUpdate");
    // Nothing left to do if paused.
    if (_info.paused)
    {
      return;
    }
    auto SimTime = std::chrono::duration<double>(_info.simTime).count();
    std::cout << "In PreUpdate SimTime = " << SimTime << std::endl;

    // \TODO(anyone): Support rewind
    if (_info.dt < std::chrono::steady_clock::duration::zero())
    {
      ignwarn << "Detected jump back in time [" << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count() << "s]. System may not work properly." << std::endl;
    }
  }

  //////////////////////////////////////////////////
  void WaveBodyInteractions::Update(
      const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm)
  {
    IGN_PROFILE("#WaveBodyInteractions::Update");
    // Nothing left to do if paused.
    if (_info.paused)
    {
      return;
    }

    // auto SimTime = std::chrono::duration < double > (_info.simTime).count();
  }

  //////////////////////////////////////////////////
  void WaveBodyInteractions::PostUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm)
  {
    IGN_PROFILE("#WaveBodyInteractions::PostUpdate");
    // Nothing left to do if paused.
    if (_info.paused)
    {
      return;
    }

    // auto SimTime = std::chrono::duration < double > (_info.simTime).count();
  }

  //////////////////////////////////////////////////
  void WaveBodyInteractionsPrivate::OnUserCmdCurr(const ignition::msgs::Double &_msg)
  {
    std::cout << "In OnUserCmdCurr" << std::endl;
  }
  } // namespace buoy_gazebo
  

  IGNITION_ADD_PLUGIN(
      buoy_gazebo::WaveBodyInteractions,
      ignition::gazebo::System,
      buoy_gazebo::WaveBodyInteractions::ISystemConfigure,
      buoy_gazebo::WaveBodyInteractions::ISystemPreUpdate,
      buoy_gazebo::WaveBodyInteractions::ISystemUpdate,
      buoy_gazebo::WaveBodyInteractions::ISystemPostUpdate);
  
/*
  IGNITION_ADD_PLUGIN(
      WaveBodyInteractions,
      ignition::gazebo::System,
      WaveBodyInteractions::ISystemConfigure,
      WaveBodyInteractions::ISystemPreUpdate,
      WaveBodyInteractions::ISystemUpdate,
      WaveBodyInteractions::ISystemPostUpdate);

  IGNITION_ADD_PLUGIN_ALIAS(WaveBodyInteractions,
                            "ignition::gazebo::systems::WaveBodyInteractions")
*/