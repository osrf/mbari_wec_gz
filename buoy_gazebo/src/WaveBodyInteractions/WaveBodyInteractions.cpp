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

#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/URI.hh>
#include <gz/math/PID.hh>
#include <gz/msgs.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularAcceleration.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>

#include "IncidentWaves/IncWaveState.hpp"

#include "FS_Hydrodynamics.hpp"
#include "LinearIncidentWave.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace buoy_gazebo
{
class WaveBodyInteractionsPrivate
{
public:
  /// \brief Model interface
  gz::sim::Model model{gz::sim::kNullEntity};

  /// \brief Link entity
  gz::sim::Entity linkEntity;

  /// \brief Incident wave implementation
  //  std::shared_ptr<LinearIncidentWave> Inc =
  //  std::make_shared<LinearIncidentWave> (); LinearIncidentWave Inc;

  /// \brief  Free-Surface hydrodynamics implementation
  FS_HydroDynamics FloatingBody;

  /// \brief Location of Waterplane in Link Frame
  gz::math::Vector3d WaterplaneOrigin;

  /// \brief Pose of Waterplane (CS p) in Link Frame (CS b)
  gz::math::Pose3<double> b_Pose_p;

  gz::sim::Entity IncWaveEntity{gz::sim::kNullEntity};
};

//////////////////////////////////////////////////
WaveBodyInteractions::WaveBodyInteractions()
: dataPtr(std::make_unique<WaveBodyInteractionsPrivate>()) {}

/////////////////////////////////////////////////
double SdfParamDouble(
  const std::shared_ptr<const sdf::Element> & _sdf,
  const std::string & _field, double _default)
{
  return _sdf->Get<double>(_field, _default).first;
}

//////////////////////////////////////////////////
void WaveBodyInteractions::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & /*_eventMgr*/)
{
  this->dataPtr->model = gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    ignerr
      << "WaveBodyInteractions plugin should be attached to a model entity. "
      << "Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF.
  if (!_sdf->HasElement("LinkName")) {
    ignerr << "You musk specify a <LinkName> for the wavebodyinteraction "
      "plugin to act upon"
           << "Failed to initialize." << std::endl;
    return;
  }
  auto linkName = _sdf->Get<std::string>("LinkName");

  this->dataPtr->linkEntity = this->dataPtr->model.LinkByName(_ecm, linkName);
  if (!_ecm.HasEntity(this->dataPtr->linkEntity)) {
    ignerr << "Link name" << linkName << "does not exist";
    return;
  }

  gz::sim::Link baseLink(this->dataPtr->linkEntity);

  baseLink.EnableAccelerationChecks(_ecm, true);
  baseLink.EnableVelocityChecks(_ecm, true);

  // Get world and gravity
  gz::sim::Entity worldEntity =
    _ecm.EntityByComponents(gz::sim::components::World());
  auto gravity = _ecm.Component<gz::sim::components::Gravity>(worldEntity);
  this->dataPtr->FloatingBody.SetGravity(-gravity->Data()[2]);

  double S = SdfParamDouble(_sdf, "S", 5.47);
  double S11 = SdfParamDouble(_sdf, "S11", 1.37);
  double S22 = SdfParamDouble(_sdf, "S22", 1.37);
  this->dataPtr->FloatingBody.SetWaterplane(S, S11, S22);

  double COB_x = SdfParamDouble(_sdf, "COB_x", 0.0);
  double COB_y = SdfParamDouble(_sdf, "COB_y", 0.0);
  double COB_z = SdfParamDouble(_sdf, "COB_z", -0.18);
  this->dataPtr->FloatingBody.SetCOB(COB_x, COB_y, COB_z);
  double Vol = SdfParamDouble(_sdf, "Vol", 1.75);
  this->dataPtr->FloatingBody.SetVolume(Vol);

  double WaterplaneOrigin_x = SdfParamDouble(_sdf, "WaterplaneOrigin_x", 0.0);
  double WaterplaneOrigin_y = SdfParamDouble(_sdf, "WaterplaneOrigin_y", 0.0);
  double WaterplaneOrigin_z = SdfParamDouble(_sdf, "WaterplaneOrigin_z", 2.45);
  this->dataPtr->b_Pose_p.Set(
    WaterplaneOrigin_x, WaterplaneOrigin_y,
    WaterplaneOrigin_z, 0.0, 0.0, 0.0);

  std::string hydro_coeff_uri_string =
    _sdf->Get<std::string>("hydro_coeffs_uri");
  gz::common::URI hydro_coeff_uri(hydro_coeff_uri_string);

  std::string HydrodynamicsBaseFilename;
  if (!hydro_coeff_uri.Scheme().compare("package")) {
    std::string delimiter = "/";
    std::string path = hydro_coeff_uri.Path().Str();
    std::string package_share_dir = path.substr(0, path.find(delimiter));
    std::string base_filenm = path.substr(path.find(delimiter), path.length());
    HydrodynamicsBaseFilename =
      ament_index_cpp::get_package_share_directory(package_share_dir) +
      base_filenm;
  } else {
    ignerr << "Only package: URI scheme has been implemented" << std::endl;
    return;
  }

  this->dataPtr->FloatingBody.ReadWAMITData_FD(HydrodynamicsBaseFilename);
  this->dataPtr->FloatingBody.ReadWAMITData_TD(HydrodynamicsBaseFilename);
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
  if (_info.iterations == 1) {  // First iteration, set timestep size.
    double dt = std::chrono::duration<double>(_info.dt).count();
    dataPtr->FloatingBody.SetTimestepSize(dt);
    gzdbg << " Set Wave Forcing timestep size:  dt = " << dt << std::endl;
  }

  // \TODO(anyone): Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero()) {
    ignwarn
      << "Detected jump back in time ["
      << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
      << "s]. System may not work properly." << std::endl;
  }

  // Retrieve pointer to Incident Wave from ECM, and assign to Floating Body
  this->dataPtr->IncWaveEntity =
    _ecm.EntityByComponents(gz::sim::components::Name("IncidentWaves"));
  buoy_gazebo::IncWaveState inc_wave_state;
  if (_ecm.EntityHasComponentType(
      this->dataPtr->IncWaveEntity,
      buoy_gazebo::components::IncWaveState().TypeId()))
  {
    auto inc_wave_state_comp =
      _ecm.Component<buoy_gazebo::components::IncWaveState>(
      this->dataPtr->IncWaveEntity);
    inc_wave_state = buoy_gazebo::IncWaveState(inc_wave_state_comp->Data());
    this->dataPtr->FloatingBody.AssignIncidentWave(inc_wave_state.Inc);
  }

  gz::sim::Link baseLink(this->dataPtr->linkEntity);

  gzdbg << "baseLink.Name = " << baseLink.Name(_ecm).value() << std::endl;

  auto w_xddot_b = baseLink.WorldLinearAcceleration(_ecm).value();
  auto w_omegadot_b = baseLink.WorldAngularAcceleration(_ecm).value();
  auto w_xdot_b = baseLink.WorldLinearVelocity(_ecm).value();
  auto w_omega_b = baseLink.WorldAngularVelocity(_ecm).value();

  auto w_Pose_b = gz::sim::worldPose(this->dataPtr->linkEntity, _ecm);
  auto w_Pose_p = w_Pose_b * this->dataPtr->b_Pose_p;
  gzdbg << "w_Pose_b = " << w_Pose_b << std::endl;
  gzdbg << "w_Pose_p = " << w_Pose_p << std::endl;

  // gz::math::Vector3<double> p_xdot = w_Pose_p.Rot().Inverse() * *w_xdot;
  gz::math::Vector3<double> w_xdot_p =
    w_xdot_b +
    w_omega_b.Cross(w_Pose_b.Rot() * this->dataPtr->b_Pose_p.Pos());
  gz::math::Vector3<double> w_omega_p =
    w_omega_b;    // Waterplane Coord sys is parallel to body C.S.
  gzdbg << "w_xdot_b = " << w_xdot_b << std::endl;
  gzdbg << "w_xdot_p = " << w_xdot_p << std::endl;
  gzdbg << "w_omega_b = " << w_omega_b << std::endl;
  gzdbg << "w_omega_p = " << w_omega_p << std::endl;

  gz::math::Vector3<double> w_xddot_p =
    w_xddot_b +
    w_omegadot_b.Cross(w_Pose_b.Rot() * this->dataPtr->b_Pose_p.Pos()) +
    w_omega_b.Cross(
    w_omega_b.Cross(w_Pose_b.Rot() * this->dataPtr->b_Pose_p.Pos()));
  gz::math::Vector3<double> w_omegadot_p =
    w_omegadot_b;        // Waterplane Coord sys is parallel to body C.S.
  gzdbg << "w_xddot_b = " << w_xddot_b << std::endl;
  gzdbg << "w_xddot_p = " << w_xddot_p << std::endl;
  gzdbg << "w_omegadot_b = " << w_omegadot_b << std::endl;
  gzdbg << "w_omegadot_p = " << w_omegadot_p << std::endl;
  gz::math::Vector3<double> b_xddot_p = w_Pose_p.Rot().Inverse() * w_xddot_p;
  gz::math::Vector3<double> b_omegadot_p =
    w_Pose_p.Rot().Inverse() * w_omegadot_p;

  inc_wave_state.x = w_Pose_p.X();
  inc_wave_state.y = w_Pose_p.Y();

  Eigen::VectorXd x(6);
  x << w_Pose_p.X(), w_Pose_p.Y(), w_Pose_p.Z(), w_Pose_p.Roll(),
    w_Pose_p.Pitch(), w_Pose_p.Yaw();
  gzdbg << "x(6) = " << x.transpose() << std::endl;
  Eigen::VectorXd BuoyancyForce(6);
  BuoyancyForce = this->dataPtr->FloatingBody.BuoyancyForce(x);
  gzdbg << "Buoyancy Force at waterplane = " << BuoyancyForce.transpose()
        << std::endl;

  // Compute Buoyancy Force
  gz::math::Vector3d w_FBp(BuoyancyForce(0), BuoyancyForce(1),
    BuoyancyForce(2));
  // Needs to be adjusted for yaw only
  gz::math::Vector3d w_MBp(
    cos(x(5)) * BuoyancyForce(3) - sin(x(5)) * BuoyancyForce(4),
    sin(x(5)) * BuoyancyForce(3) + cos(x(5)) * BuoyancyForce(4),
    BuoyancyForce(5));    // Needs to be adjusted for yaw only
  // Add contribution due to force offset from origin
  w_MBp +=
    (w_Pose_b.Rot().RotateVector(this->dataPtr->b_Pose_p.Pos())).Cross(w_FBp);
  gzdbg << "Buoyancy: applied moment = " << w_MBp << std::endl;

  // Compute Memory part of Radiation Force based on accelerations in the body
  // frame, result is in body frame
  Eigen::VectorXd xddot(6);
  //  xddot << p_xddotp.X(), p_xddot.Y(), p_xddot.Z(), p_omegadot.X(),
  //  p_omegadot.Y(), p_omegadot.Z();
  // xddot << w_xddot_p.X()+2.27*w_omegadot_p.Y(),
  // w_xddot_p.Y()+2.27*w_omegadot_p.X(), w_xddot_p.Z(), w_omegadot_p.X(),
  // w_omegadot_p.Y(), p_omegadot.Z();
  xddot << b_xddot_p.X(), b_xddot_p.Y(), b_xddot_p.Z(), b_omegadot_p.X(),
    b_omegadot_p.Y(), b_omegadot_p.Z();
  // xddot << 0.0, 0.0, 0.0, b_omegadot_p.X(), b_omegadot_p.Y(),
  // b_omegadot_p.Z();

  gzdbg << "xddot = " << xddot.transpose() << std::endl;
  gzdbg << "xddot = " << xddot.transpose() << std::endl;
  Eigen::VectorXd MemForce(6);
  // Note negative sign, FS_Hydrodynamics returns force required to move body in
  // prescribed way,
  //  force of water on body is opposite.
  MemForce = -this->dataPtr->FloatingBody.RadiationForce(xddot);
  gzdbg << " MemForce at Waterplane = " << MemForce.transpose() << std::endl;
  gz::math::Vector3d w_FRp(MemForce(0), MemForce(1), MemForce(2));
  // Needs to be adjusted for yaw only becaues of small pitch/roll assumption
  gz::math::Vector3d w_MRp(
    1 * (cos(x(5)) * MemForce(3) - sin(x(5)) * MemForce(4)),
    1 * (sin(x(5)) * MemForce(3) + cos(x(5)) * MemForce(4)),
    MemForce(5));    // Needs to be adjusted for yaw only

  // Add contribution due to force offset from origin
  w_MRp +=
    (w_Pose_b.Rot().RotateVector(this->dataPtr->b_Pose_p.Pos())).Cross(w_FRp);
  gzdbg << "Radiation: applied moment = " << w_MRp << std::endl;

  gzdbg << std::endl;
  // Compute Wave Exciting Force
  Eigen::VectorXd ExtForce(6);
  ExtForce = this->dataPtr->FloatingBody.ExcitingForce();
  gzdbg << "Exciting Force = " << ExtForce.transpose() << std::endl;
  gz::math::Vector3d w_FEp(ExtForce(0), ExtForce(1), ExtForce(2));
  // Needs to be adjusted for yaw only
  gz::math::Vector3d w_MEp(
    1 * (cos(x(5)) * ExtForce(3) - sin(x(5)) * ExtForce(4)),
    1 * (sin(x(5)) * ExtForce(3) + cos(x(5)) * ExtForce(4)),
    ExtForce(5));    // Needs to be adjusted for yaw only

  gzdbg << "Exciting: applied force = " << w_FEp << std::endl;
  gzdbg << "Exciting: applied moment = " << w_MEp << std::endl;

  // Add contribution due to force offset from origin
  w_MEp +=
    (w_Pose_b.Rot().RotateVector(this->dataPtr->b_Pose_p.Pos())).Cross(w_FEp);

  baseLink.AddWorldWrench(_ecm, w_FBp + w_FRp + w_FEp, w_MBp + w_MRp + w_MEp);
  // baseLink.AddWorldWrench(_ecm, w_FBp + w_FRp, w_MBp + w_MRp);
  // baseLink.AddWorldWrench(_ecm, w_FBp, w_MBp);

  // push buoy x-y location back to incident wave plugin, temporary.
  if (this->dataPtr->IncWaveEntity != gz::sim::kNullEntity) {
    _ecm.SetComponentData<buoy_gazebo::components::IncWaveState>(
      this->dataPtr->IncWaveEntity, inc_wave_state);
  }
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
