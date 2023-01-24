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

#include <gz/msgs/double.pb.h>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <gz/sim/Link.hh>
#include <gz/sim/components/AngularAcceleration.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/math/PID.hh>
#include <gz/msgs.hh>
#include <gz/plugin/Register.hh>


#include "FreeSurfaceHydrodynamics/FS_Hydrodynamics.hpp"
#include "FreeSurfaceHydrodynamics/LinearIncidentWave.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace buoy_gazebo
{
class WaveBodyInteractionsPrivate
{
public:
  WaveBodyInteractionsPrivate()
  : FloatingBody(
      IncRef)
  {
  }                    // Constructor needs to assign reference or compile error
                       // of un-assigned referenced occurs

/// \brief Model interface
  gz::sim::Model model{gz::sim::kNullEntity};

/// \brief Link entity
  gz::sim::Entity linkEntity;

/// \brief Incident wave implementation
  LinearIncidentWave Inc;

/// \brief Reference to Incident wave implementation
  LinearIncidentWave & IncRef = Inc;

/// \brief  Free-Surface hydrodynamics implementation
  FS_HydroDynamics FloatingBody;

/// \brief Location of Waterplane in Link Frame
  gz::math::Vector3d WaterplaneOrigin;

/// \brief Pose of Waterplane (CS p) in Link Frame (CS b)
  gz::math::Pose3<double> b_Pose_p;
};

//////////////////////////////////////////////////
WaveBodyInteractions::WaveBodyInteractions()
: dataPtr(std::make_unique<WaveBodyInteractionsPrivate>())
{
}

/////////////////////////////////////////////////
double SdfParamDouble(
  const std::shared_ptr<const sdf::Element> & _sdf,
  const std::string & _field, double _default)
{
  return _sdf->Get<double>(_field, _default).first;
}
LinearIncidentWave Inc2;

//////////////////////////////////////////////////
void WaveBodyInteractions::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & /*_eventMgr*/)
{
  this->dataPtr->model = gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    ignerr <<
      "WaveBodyInteractions plugin should be attached to a model entity. " <<
      "Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF.
  if (!_sdf->HasElement("LinkName")) {
    ignerr << "You musk specify a <LinkName> for the wavebodyinteraction "
      "plugin to act upon" <<
      "Failed to initialize." << std::endl;
    return;
  }
  auto linkName = _sdf->Get<std::string>("LinkName");

  this->dataPtr->linkEntity = this->dataPtr->model.LinkByName(_ecm, linkName);
  if (!_ecm.HasEntity(this->dataPtr->linkEntity)) {
    ignerr << "Link name" << linkName << "does not exist";
    return;
  }

  double Hs = SdfParamDouble(_sdf, "Hs", 3.0);
  double Tp = SdfParamDouble(_sdf, "Tp", 8.);

  if (Tp > 0) {
    this->dataPtr->Inc.SetToPiersonMoskowitzSpectrum(Hs, Tp, 0.0, 180.0);
  } else {
    this->dataPtr->Inc.SetToMonoChromatic(Hs / 2.0, -Tp, 0.0, 180.0);
  }

  Inc2.SetToMonoChromatic(.1, 20.0, 0.0, 180.0);
// this->dataPtr->Inc = Inc2;

  std::string HydrodynamicsBaseFilename =
    ament_index_cpp::get_package_share_directory("buoy_description") +
    "/models/mbari_wec_base/hydrodynamic_coeffs/BuoyA5";
  this->dataPtr->FloatingBody.ReadWAMITData_FD(HydrodynamicsBaseFilename);
  this->dataPtr->FloatingBody.ReadWAMITData_TD(HydrodynamicsBaseFilename);
  // TODO(anyone):  Need to get timestep size from ecm.
  this->dataPtr->FloatingBody.SetTimestepSize(.01);

  gz::sim::Link baseLink(this->dataPtr->linkEntity);

  baseLink.EnableAccelerationChecks(
    _ecm,
    true);             // Allow access to last-timestep's acceleration in Configure()

  double S = SdfParamDouble(_sdf, "S", 5.47);
  double S11 = SdfParamDouble(_sdf, "S11", 1.37);
  double S22 = SdfParamDouble(_sdf, "S22", 1.37);
  this->dataPtr->FloatingBody.SetWaterplane(S, S11, S22);

  double COB_x = SdfParamDouble(_sdf, "COB_x", 0);
  double COB_y = SdfParamDouble(_sdf, "COB_y", 0);
  double COB_z = SdfParamDouble(_sdf, "COB_z", -.18);
  this->dataPtr->FloatingBody.SetCOB(COB_x, COB_y, COB_z);
  double Vol = SdfParamDouble(_sdf, "Vol", 1.75);
  this->dataPtr->FloatingBody.SetVolume(Vol);

  double WaterplaneOrigin_x = SdfParamDouble(_sdf, "WaterplaneOrigin_x", 0.0);
  double WaterplaneOrigin_y = SdfParamDouble(_sdf, "WaterplaneOrigin_y", 0.0);
  double WaterplaneOrigin_z = SdfParamDouble(_sdf, "WaterplaneOrigin_z", 2.45);
  this->dataPtr->b_Pose_p.Set(WaterplaneOrigin_x, WaterplaneOrigin_y, WaterplaneOrigin_z, 0, 0, 0);

  Eigen::VectorXd b(6);
  b(0) = 300.0;
  b(1) = 300.0;
  b(2) = 900.0;
  b(3) = 400.0;
  b(4) = 400.0;
  b(5) = 100.0;
  this->dataPtr->FloatingBody.SetDampingCoeffs(b);

  Eigen::VectorXd Cd(6);
  Cd(0) = .5;
  Cd(1) = .5;
  Cd(2) = .5;
  Cd(3) = .5;
  Cd(4) = .5;
  Cd(5) = .1;
  this->dataPtr->FloatingBody.SetDragCoeffs(Cd);

  Eigen::VectorXd Area(6);
  Area(0) = 5.0;
  Area(1) = 2.0;
  Area(2) = 2.0;
  Area(3) = 1.0;
  Area(4) = 1.0;
  Area(5) = 1.0;
  this->dataPtr->FloatingBody.SetAreas(Area);
}

#define EPSILON 0.0000001;
bool AreSame(double a, double b)
{
  return fabs(a - b) < EPSILON;
}
//////////////////////////////////////////////////
void WaveBodyInteractions::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("#WaveBodyInteractions::PreUpdate");
  // Nothing left to do if paused.
  if (_info.paused) {
    return;
  }
  auto SimTime = std::chrono::duration<double>(_info.simTime).count();

  double dt = std::chrono::duration<double>(_info.dt).count();
  if (!AreSame(dt, dataPtr->FloatingBody.GetTimestepSize())) {
    // Would prefer to set this in Configure,
    //   but not sure how to access it or if it's guaranteed to be setup when configure is called.
    std::cout << " Setting timestep size " << std::endl;
    // dataPtr->FloatingBody.SetTimestepSize(dt);
  }

  // \TODO(anyone): Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero()) {
    ignwarn <<
      "Detected jump back in time [" <<
      std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count() <<
      "s]. System may not work properly." << std::endl;
  }

  gz::sim::Link baseLink(this->dataPtr->linkEntity);
  auto w_xddot = baseLink.WorldLinearAcceleration(_ecm);
  auto w_omegadot = baseLink.WorldAngularAcceleration(_ecm);
  auto w_xdot = baseLink.WorldLinearVelocity(_ecm);
  auto w_omega = baseLink.WorldAngularVelocity(_ecm);

  auto w_Pose_b = gz::sim::worldPose(this->dataPtr->linkEntity, _ecm);
  auto w_Pose_p = w_Pose_b * this->dataPtr->b_Pose_p;
  std::cout << "w_Pose_b = " << w_Pose_b << std::endl;
  std::cout << "w_Pose_p = " << w_Pose_p << std::endl;

  gz::math::Vector3<double> p_xddot = w_Pose_p.Rot().Inverse() * *w_xddot;
  gz::math::Vector3<double> p_omegadot = w_Pose_p.Rot().Inverse() * *w_omegadot;
  std::cout << "w_omegadot = " << w_omegadot.value() << std::endl;
  std::cout << "p_omegadot = " << p_omegadot << std::endl;

  gz::math::Vector3<double> p_xdot = w_Pose_p.Rot().Inverse() * *w_xdot;
  gz::math::Vector3<double> p_omega = w_Pose_p.Rot().Inverse() * *w_omega;
  std::cout << "w_xdot = " << w_xdot.value() << std::endl;
  std::cout << "p_xdot = " << p_xdot << std::endl;

  Eigen::VectorXd x(6);
  x << w_Pose_p.X(), w_Pose_p.Y(), w_Pose_p.Z(), w_Pose_p.Roll(), w_Pose_p.Pitch(), w_Pose_p.Yaw();
  std::cout << "x(6) = " << x.transpose() << std::endl;
  Eigen::VectorXd BuoyancyForce(6);
  BuoyancyForce = this->dataPtr->FloatingBody.BuoyancyForce(x);
  std::cout << "Buoyancy Force = " << BuoyancyForce.transpose() << std::endl;

// Compute Buoyancy Force
  gz::math::Vector3d w_FBp(BuoyancyForce(0), BuoyancyForce(1), BuoyancyForce(2));
  // Needs to be adjusted for yaw only
  gz::math::Vector3d w_MBp(
    cos(x(5)) * BuoyancyForce(3) - sin(x(5)) * BuoyancyForce(4),
    sin(x(5)) * BuoyancyForce(3) + cos(x(5)) * BuoyancyForce(4),
    BuoyancyForce(5));             // Needs to be adjusted for yaw only
  // Add contribution due to force offset from origin
  w_MBp += (w_Pose_b.Rot().RotateVector(this->dataPtr->b_Pose_p.Pos())).Cross(w_FBp);
  std::cout << "Buoyancy: applied moment = " << w_MBp << std::endl;

// Compute Memory part of Radiation Force
  Eigen::VectorXd xddot(6);
  xddot << p_xddot.X(), p_xddot.Y(), p_xddot.Z(), p_omegadot.X(), p_omegadot.Y(), p_omegadot.Z();
  std::cout << "xddot = " << xddot.transpose() << std::endl;
  Eigen::VectorXd MemForce(6);
  // Note negative sign, FS_Hydrodynamics returns force required to move body in prescirbed way,
  //  force of water on body is opposite.
  MemForce = -this->dataPtr->FloatingBody.RadiationForce(xddot);
  std::cout << " MemForce = " << MemForce.transpose() << std::endl;
  gz::math::Vector3d w_FRp(MemForce(0), MemForce(1), MemForce(2));
  // Needs to be adjusted for yaw only
  gz::math::Vector3d w_MRp(
    1 * (cos(x(5)) * MemForce(3) - sin(x(5)) * MemForce(4)),
    1 * (sin(x(5)) * MemForce(3) + cos(x(5)) * MemForce(4)),
    MemForce(5));             // Needs to be adjusted for yaw only
  // Add contribution due to force offset from origin
  w_MRp += (w_Pose_b.Rot().RotateVector(this->dataPtr->b_Pose_p.Pos())).Cross(w_FRp);
  std::cout << "Radiation: applied moment = " << w_MRp << std::endl;

// Compute Wave Exciting Force
  Eigen::VectorXd ExtForce(6);
  ExtForce = this->dataPtr->FloatingBody.ExcitingForce();
  std::cout << "Exciting Force = " << ExtForce.transpose() << std::endl;
  gz::math::Vector3d w_FEp(ExtForce(0), ExtForce(1), ExtForce(2));
  // Needs to be adjusted for yaw only
  gz::math::Vector3d w_MEp(
    1 * (cos(x(5)) * ExtForce(3) - sin(x(5)) * ExtForce(4)),
    1 * (sin(x(5)) * ExtForce(3) + cos(x(5)) * ExtForce(4)),
    ExtForce(5));             // Needs to be adjusted for yaw only

  std::cout << "Exciting: applied force = " << w_FEp << std::endl;
  std::cout << "Exciting: applied moment = " << w_MEp << std::endl;
//  w_FEp[0] = 0.0;
//  w_FEp[1] = 0.0;
// w_FEp[2] = 0.0;
//  w_MEp[0] = 0.0;
//  w_MEp[1] = 0.0;
//  w_MEp[2] = 0.0;
  // Add contribution due to force offset from origin
  w_MEp += (w_Pose_b.Rot().RotateVector(this->dataPtr->b_Pose_p.Pos())).Cross(w_FEp);

  Eigen::VectorXd vel(6);
  vel << p_xdot.X(), p_xdot.Y(), p_xdot.Z(), p_omega.X(), p_omega.Y(), p_omega.Z();
#if 0
// Compute Linear Drag
  Eigen::VectorXd LinDampingForce(6);
  LinDampingForce = this->dataPtr->FloatingBody.LinearDampingForce(vel);
  gz::math::Vector3d w_FLDp(LinDampingForce(0), LinDampingForce(1), LinDampingForce(2));
  // Needs to be adjusted for yaw only
  gz::math::Vector3d w_MLDp(
    1 * (cos(x(5)) * LinDampingForce(3) - sin(x(5)) * LinDampingForce(4)),
    1 * (sin(x(5)) * LinDampingForce(3) + cos(x(5)) * LinDampingForce(4)),
    LinDampingForce(5));             // Needs to be adjusted for yaw only
  std::cout << "Linear Damping: applied force = " << w_FLDp << std::endl;
  std::cout << "Linear Damping: applied moment = " << w_MLDp << std::endl;
  // Add contribution due to force offset from origin
  w_MLDp += (w_Pose_b.Rot().RotateVector(this->dataPtr->b_Pose_p.Pos())).Cross(w_FLDp);
#endif

// Compute Viscous Drag (quadratic)
  Eigen::VectorXd DragForce(6);
  DragForce = this->dataPtr->FloatingBody.ViscousDragForce(vel);
  gz::math::Vector3d w_FVDp(DragForce(0), DragForce(1), DragForce(2));
  // Needs to be adjusted for yaw only
  gz::math::Vector3d w_MVDp(
    1 * (cos(x(5)) * DragForce(3) - sin(x(5)) * DragForce(4)),
    1 * (sin(x(5)) * DragForce(3) + cos(x(5)) * DragForce(4)),
    DragForce(5));             // Needs to be adjusted for yaw only
  std::cout << "Viscous Drag: applied force = " << w_FVDp << std::endl;
  std::cout << "Viscous Drag: applied moment = " << w_MVDp << std::endl;
  // Add contribution due to force offset from origin
  w_MVDp += (w_Pose_b.Rot().RotateVector(this->dataPtr->b_Pose_p.Pos())).Cross(w_FVDp);

  baseLink.AddWorldWrench(_ecm, w_FBp + w_FRp + w_FEp + w_FVDp, w_MBp + w_MRp + w_MEp + w_MVDp);
}

//////////////////////////////////////////////////
void WaveBodyInteractions::Update(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("#WaveBodyInteractions::Update");
  // Nothing left to do if paused.
  if (_info.paused) {
    return;
  }
}

//////////////////////////////////////////////////
void WaveBodyInteractions::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("#WaveBodyInteractions::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused) {
    return;
  }
}

}  // namespace buoy_gazebo

GZ_ADD_PLUGIN(
  buoy_gazebo::WaveBodyInteractions, gz::sim::System,
  buoy_gazebo::WaveBodyInteractions::ISystemConfigure,
  buoy_gazebo::WaveBodyInteractions::ISystemPreUpdate,
  buoy_gazebo::WaveBodyInteractions::ISystemUpdate,
  buoy_gazebo::WaveBodyInteractions::ISystemPostUpdate);
