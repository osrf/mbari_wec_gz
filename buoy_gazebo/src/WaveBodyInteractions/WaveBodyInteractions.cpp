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
      Inc)
  {
  }                    // Constructor needs to assign reference or compile error
                       // of un-assigned referenced occurs

/// \brief Model interface
  gz::sim::Model model{gz::sim::kNullEntity};

/// \brief Link entity
  gz::sim::Entity linkEntity;

/// \brief Incident wave implementation
  LinearIncidentWave Inc;

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

  auto SpectrumType = _sdf->Get<std::string>("IncWaveSpectrumType");

  if (!SpectrumType.compare("MonoChromatic")) {
    gzdbg << "SpectrumType " << SpectrumType << std::endl;
    double A = SdfParamDouble(_sdf, "A", 0.0);
    double T = SdfParamDouble(_sdf, "T", 14.0);
    double phase = SdfParamDouble(_sdf, "Phase", 0.0);
    this->dataPtr->Inc.SetToMonoChromatic(A, T, phase, 180.0);
  }

  if (!SpectrumType.compare("Bretschneider")) {
    gzdbg << "SpectrumType " << SpectrumType << std::endl;
    double Hs = SdfParamDouble(_sdf, "Hs", 0.0);
    double Tp = SdfParamDouble(_sdf, "Tp", 14.0);
    gzdbg << "Hs = " << Hs << "  Tp = " << Tp << std::endl;
    this->dataPtr->Inc.SetToBretschneiderSpectrum(Hs, Tp, 180.0);
  }

  if (!SpectrumType.compare("Custom")) {
    gzwarn << "SpectrumType " << SpectrumType << "Spectrum Not Yet Implemented" << std::endl;
    // this->dataPtr->Inc.SetToCustomSpectrum(omega, S, 180.0);
  }


  gz::sim::Link baseLink(this->dataPtr->linkEntity);

  baseLink.EnableAccelerationChecks(
    _ecm, true);             // Allow access to last-timestep's acceleration in Configure()

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
    WaterplaneOrigin_x,
    WaterplaneOrigin_y,
    WaterplaneOrigin_z,
    0.0, 0.0, 0.0);

  std::string HydrodynamicsBaseFilename =
    ament_index_cpp::get_package_share_directory("buoy_description") +
    "/models/mbari_wec_base/hydrodynamic_coeffs/BuoyA5";
  this->dataPtr->FloatingBody.ReadWAMITData_FD(HydrodynamicsBaseFilename);
  this->dataPtr->FloatingBody.ReadWAMITData_TD(HydrodynamicsBaseFilename);

}

#define EPSILON 0.0000001;
bool AreSame(const double & a, const double & b)
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
  if(_info.iterations == 1)  // First iteration, set timestep size.
  {
    double dt = std::chrono::duration<double>(_info.dt).count();
    dataPtr->FloatingBody.SetTimestepSize(dt);
    gzdbg << " Set Wave Forcing timestep size:  dt = " << dt << std::endl; 
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
  gzdbg << "w_Pose_b = " << w_Pose_b << std::endl;
  gzdbg << "w_Pose_p = " << w_Pose_p << std::endl;

  gz::math::Vector3<double> p_xddot = w_Pose_p.Rot().Inverse() * *w_xddot;
  gz::math::Vector3<double> p_omegadot = w_Pose_p.Rot().Inverse() * *w_omegadot;
  gzdbg << "w_omegadot = " << w_omegadot.value() << std::endl;
  gzdbg << "p_omegadot = " << p_omegadot << std::endl;

  gz::math::Vector3<double> p_xdot = w_Pose_p.Rot().Inverse() * *w_xdot;
  gz::math::Vector3<double> p_omega = w_Pose_p.Rot().Inverse() * *w_omega;
  gzdbg << "w_xdot = " << w_xdot.value() << std::endl;
  gzdbg << "p_xdot = " << p_xdot << std::endl;

  Eigen::VectorXd x(6);
  x << w_Pose_p.X(), w_Pose_p.Y(), w_Pose_p.Z(), w_Pose_p.Roll(), w_Pose_p.Pitch(), w_Pose_p.Yaw();
  gzdbg << "x(6) = " << x.transpose() << std::endl;
  Eigen::VectorXd BuoyancyForce(6);
  BuoyancyForce = this->dataPtr->FloatingBody.BuoyancyForce(x);
  gzdbg << "Buoyancy Force = " << BuoyancyForce.transpose() << std::endl;

  double deta_dx{0.0}, deta_dy{0.0};
  double eta = this->dataPtr->Inc.eta(w_Pose_p.X(), w_Pose_p.Y(), SimTime, &deta_dx, &deta_dy);

  gz::msgs::Pose req;
  req.set_name("water_plane");
  req.mutable_position()->set_x(w_Pose_p.X());
  req.mutable_position()->set_y(w_Pose_p.Y());
  req.mutable_position()->set_z(eta);

  double roll = atan(deta_dx);
  double pitch = atan(deta_dy);
  double yaw = 0.0;

  double qx = sin(roll/2.0) * cos(pitch/2.0) * cos(yaw/2.0)
    - cos(roll/2.0) * sin(pitch/2.0) * sin(yaw/2.0);
  double qy = cos(roll/2.0) * sin(pitch/2.0) * cos(yaw/2.0)
    + sin(roll/2.0) * cos(pitch/2.0) * sin(yaw/2.0);
  double qz = cos(roll/2.0) * cos(pitch/2.0) * sin(yaw/2.0)
    - sin(roll/2.0) * sin(pitch/2.0) * cos(yaw/2.0);
  double qw = cos(roll/2.0) * cos(pitch/2.0) * cos(yaw/2.0)
    + sin(roll/2.0) * sin(pitch/2.0) * sin(yaw/2.0);

  req.mutable_orientation()->set_x(qx);
  req.mutable_orientation()->set_y(qy);
  req.mutable_orientation()->set_z(qz);
  req.mutable_orientation()->set_w(qw);

  std::function<void(const gz::msgs::Boolean &, const bool)> cb =
      [](const gz::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      gzerr << "Error sending move to request" << std::endl;
  };

  gz::transport::Node node;
  node.Request("/world/world_demo/set_pose", req, cb);

// Compute Buoyancy Force
  gz::math::Vector3d w_FBp(BuoyancyForce(0), BuoyancyForce(1), BuoyancyForce(2));
  // Needs to be adjusted for yaw only
  gz::math::Vector3d w_MBp(
    cos(x(5)) * BuoyancyForce(3) - sin(x(5)) * BuoyancyForce(4),
    sin(x(5)) * BuoyancyForce(3) + cos(x(5)) * BuoyancyForce(4),
    BuoyancyForce(5));             // Needs to be adjusted for yaw only
  // Add contribution due to force offset from origin
  w_MBp += (w_Pose_b.Rot().RotateVector(this->dataPtr->b_Pose_p.Pos())).Cross(w_FBp);
  gzdbg << "Buoyancy: applied moment = " << w_MBp << std::endl;

// Compute Memory part of Radiation Force
  Eigen::VectorXd xddot(6);
  xddot << p_xddot.X(), p_xddot.Y(), p_xddot.Z(), p_omegadot.X(), p_omegadot.Y(), p_omegadot.Z();
  gzdbg << "xddot = " << xddot.transpose() << std::endl;
  Eigen::VectorXd MemForce(6);
  // Note negative sign, FS_Hydrodynamics returns force required to move body in prescirbed way,
  //  force of water on body is opposite.
  MemForce = -this->dataPtr->FloatingBody.RadiationForce(xddot);
  gzdbg << " MemForce = " << MemForce.transpose() << std::endl;
  gz::math::Vector3d w_FRp(MemForce(0), MemForce(1), MemForce(2));
  // Needs to be adjusted for yaw only
  gz::math::Vector3d w_MRp(
    1 * (cos(x(5)) * MemForce(3) - sin(x(5)) * MemForce(4)),
    1 * (sin(x(5)) * MemForce(3) + cos(x(5)) * MemForce(4)),
    MemForce(5));             // Needs to be adjusted for yaw only

  // Add contribution due to force offset from origin
  w_MRp += (w_Pose_b.Rot().RotateVector(this->dataPtr->b_Pose_p.Pos())).Cross(w_FRp);
  gzdbg << "Radiation: applied moment = " << w_MRp << std::endl;

// Compute Wave Exciting Force
  Eigen::VectorXd ExtForce(6);
  ExtForce = this->dataPtr->FloatingBody.ExcitingForce();
  gzdbg << "Exciting Force = " << ExtForce.transpose() << std::endl;
  gz::math::Vector3d w_FEp(ExtForce(0), ExtForce(1), ExtForce(2));
  // Needs to be adjusted for yaw only
  gz::math::Vector3d w_MEp(
    1 * (cos(x(5)) * ExtForce(3) - sin(x(5)) * ExtForce(4)),
    1 * (sin(x(5)) * ExtForce(3) + cos(x(5)) * ExtForce(4)),
    ExtForce(5));             // Needs to be adjusted for yaw only

  gzdbg << "Exciting: applied force = " << w_FEp << std::endl;
  gzdbg << "Exciting: applied moment = " << w_MEp << std::endl;

  // Add contribution due to force offset from origin
  w_MEp += (w_Pose_b.Rot().RotateVector(this->dataPtr->b_Pose_p.Pos())).Cross(w_FEp);

  Eigen::VectorXd vel(6);
  vel << p_xdot.X(), p_xdot.Y(), p_xdot.Z(), p_omega.X(), p_omega.Y(), p_omega.Z();

  baseLink.AddWorldWrench(_ecm, w_FBp + w_FRp + w_FEp, w_MBp + w_MRp + w_MEp);
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
