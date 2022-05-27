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

#include <stdio.h>

#include <gz/msgs/double.pb.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/common/Console.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/math/PID.hh>
#include <gz/msgs.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>


#include "ElectroHydraulicPTO.hpp"
#include "ElectroHydraulicSoln.hpp"
#include "ElectroHydraulicState.hpp"


namespace buoy_gazebo
{
class ElectroHydraulicPTOPrivate
{
public:
  /// \brief Piston joint entity
  gz::sim::Entity PrismaticJointEntity{gz::sim::kNullEntity};

  /// \brief Piston area
  double PistonArea{1.0};

  /// \brief Rotor Inertia
  double RotorInertia{1.0};

  /// \brief Model interface
  gz::sim::Model model{gz::sim::kNullEntity};

  ElectroHydraulicSoln functor{};

  Eigen::VectorXd x{};

  double TargetWindingCurrent{0.0};

  double WindingCurrent{0.0};

  static constexpr double Ve{315.0};

  bool VelMode{false};

  /// \brief Gazebo communication node.
  gz::transport::Node node;
};

//////////////////////////////////////////////////
ElectroHydraulicPTO::ElectroHydraulicPTO()
: dataPtr(std::make_unique<ElectroHydraulicPTOPrivate>())
{
}


/////////////////////////////////////////////////
double SdfParamDouble(
  const std::shared_ptr<const sdf::Element> & _sdf,
  const std::string & _field,
  double _default)
{
  return _sdf->Get<double>(_field, _default).first;
}


//////////////////////////////////////////////////
void ElectroHydraulicPTO::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & /*_eventMgr*/)
{
  this->dataPtr->model = gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    gzerr << "ElectroHydraulicPTO plugin should be attached to a model entity. " <<
      "Failed to initialize." << std::endl;
    return;
  }


  // Get params from SDF for Prismatic Joint.
  auto PrismaticJointName = _sdf->Get<std::string>("PrismaticJointName");
  if (PrismaticJointName.empty()) {
    gzerr << "ElectroHydraulicPTO found an empty PrismaticJointName parameter. " <<
      "Failed to initialize.";
    return;
  }


  this->dataPtr->PrismaticJointEntity = this->dataPtr->model.JointByName(
    _ecm,
    PrismaticJointName);
  if (this->dataPtr->PrismaticJointEntity == gz::sim::kNullEntity) {
    gzerr << "Joint with name [" << PrismaticJointName << "] not found. " <<
      "The ElectroHydraulicPTO may not influence this joint.\n";
    return;
  } else {
    this->dataPtr->PistonArea = SdfParamDouble(_sdf, "PistonArea", this->dataPtr->PistonArea);
  }

  // Default to Parker F11-5  0.30in^3/rev
  static constexpr double PARKER_F11_5 = 0.30;  // in^3/rev
  this->dataPtr->functor.HydMotorDisp = SdfParamDouble(_sdf, "HydMotorDisp", PARKER_F11_5);
  this->dataPtr->RotorInertia = SdfParamDouble(_sdf, "RotorInertia", this->dataPtr->RotorInertia);

  if (_sdf->HasElement("VelMode")) {
    this->dataPtr->VelMode = true;
  }

  this->dataPtr->x.setConstant(2.0, 0.0);


  std::string pistonvel_topic = std::string("/pistonvel_") + PrismaticJointName;
  pistonvel_pub = node.Advertise<gz::msgs::Double>(pistonvel_topic);
  if (!pistonvel_pub) {
    gzerr << "Error advertising topic [" << pistonvel_topic << "]" << std::endl;
    return;
  }

  std::string rpm_topic = std::string("/rpm_") + PrismaticJointName;
  rpm_pub = node.Advertise<gz::msgs::Double>(rpm_topic);
  if (!rpm_pub) {
    gzerr << "Error advertising topic [" << rpm_topic << "]" << std::endl;
    return;
  }

  std::string deltaP_topic = std::string("/deltaP_") + PrismaticJointName;
  deltaP_pub = node.Advertise<gz::msgs::Double>(deltaP_topic);
  if (!deltaP_pub) {
    gzerr << "Error advertising topic [" << deltaP_topic << "]" << std::endl;
    return;
  }

  std::string targwindcurr_topic = std::string("/targwindcurr_") + PrismaticJointName;
  targwindcurr_pub = node.Advertise<gz::msgs::Double>(targwindcurr_topic);
  if (!targwindcurr_pub) {
    gzerr << "Error advertising topic [" << targwindcurr_topic << "]" << std::endl;
    return;
  }

  std::string windcurr_topic = std::string("/windcurr_") + PrismaticJointName;
  windcurr_pub = node.Advertise<gz::msgs::Double>(windcurr_topic);
  if (!windcurr_pub) {
    gzerr << "Error advertising topic [" << windcurr_topic << "]" << std::endl;
    return;
  }

  std::string battcurr_topic = std::string("/battcurr_") + PrismaticJointName;
  battcurr_pub = node.Advertise<gz::msgs::Double>(battcurr_topic);
  if (!battcurr_pub) {
    gzerr << "Error advertising topic [" << battcurr_topic << "]" << std::endl;
    return;
  }

  std::string loadcurr_topic = std::string("/loadcurr_") + PrismaticJointName;
  loadcurr_pub = node.Advertise<gz::msgs::Double>(loadcurr_topic);
  if (!loadcurr_pub) {
    gzerr << "Error advertising topic [" << loadcurr_topic << "]" << std::endl;
    return;
  }

  std::string scalefactor_topic = std::string("/scalefactor_") + PrismaticJointName;
  scalefactor_pub = node.Advertise<gz::msgs::Double>(scalefactor_topic);
  if (!scalefactor_pub) {
    gzerr << "Error advertising topic [" << scalefactor_topic << "]" << std::endl;
    return;
  }

  std::string retractfactor_topic = std::string("/retractfactor_") + PrismaticJointName;
  retractfactor_pub = node.Advertise<gz::msgs::Double>(retractfactor_topic);
  if (!retractfactor_pub) {
    gzerr << "Error advertising topic [" << retractfactor_topic << "]" << std::endl;
    return;
  }
}

//////////////////////////////////////////////////
void ElectroHydraulicPTO::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("#ElectroHydraulicPTO::PreUpdate");
  // Nothing left to do if paused.
  if (_info.paused) {
    return;
  }

  auto SimTime = std::chrono::duration<double>(_info.simTime).count();

  GZ_PROFILE("#ElectroHydraulicPTO::PreUpdate");

  // If the joints haven't been identified yet, the plugin is disabled
  if (this->dataPtr->PrismaticJointEntity == gz::sim::kNullEntity) {
    return;
  }

  // \TODO(anyone): Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero()) {
    gzwarn << "Detected jump back in time [" <<
      std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count() <<
      "s]. System may not work properly." << std::endl;
  }


  // Create joint velocity component for piston if one doesn't exist
  auto prismaticJointVelComp = _ecm.Component<gz::sim::components::JointVelocity>(
    this->dataPtr->PrismaticJointEntity);
  if (prismaticJointVelComp == nullptr) {
    _ecm.CreateComponent(
      this->dataPtr->PrismaticJointEntity, gz::sim::components::JointVelocity());
  }
  // We just created the joint velocity component, give one iteration for the
  // physics system to update its size
  if (prismaticJointVelComp == nullptr || prismaticJointVelComp->Data().empty()) {
    return;
  }

  // Retrieve Piston velocity, compute flow and provide as input to hydraulic solver.
  // TODO(anyone): Figure out if (0) for the index is always correct,
  // some OR code has a process of finding the index for this argument.
  double xdot = prismaticJointVelComp->Data().at(0);
  this->dataPtr->functor.Q = xdot * 39.4 * this->dataPtr->PistonArea;  // inch^3/second

  // Compute Resulting Rotor RPM and Force applied to Piston based on kinematics
  // and quasistatic forces.  These neglect oil compressibility and rotor inertia,
  // but do include mechanical and volumetric efficiency of hydraulic motor.
  // This is an implicit non-linear relation so iteration required,
  // performed by Eigen HybridNonLinearSolver

  buoy_gazebo::ElectroHydraulicState pto_state;
  if (_ecm.EntityHasComponentType(
      this->dataPtr->PrismaticJointEntity,
      buoy_gazebo::components::ElectroHydraulicState().TypeId()))
  {
    auto pto_state_comp =
      _ecm.Component<buoy_gazebo::components::ElectroHydraulicState>(
      this->dataPtr->PrismaticJointEntity);

    pto_state = buoy_gazebo::ElectroHydraulicState(pto_state_comp->Data());
  }

  if (pto_state.scale_command) {
    this->dataPtr->functor.I_Wind.ScaleFactor = pto_state.scale_command.value();
  } else {
    this->dataPtr->functor.I_Wind.ScaleFactor = DEFAULT_SCALE_FACTOR;
  }

  if (pto_state.retract_command) {
    this->dataPtr->functor.I_Wind.RetractFactor = pto_state.retract_command.value();
  } else {
    this->dataPtr->functor.I_Wind.RetractFactor = DEFAULT_RETRACT_FACTOR;
  }

  this->dataPtr->functor.I_Wind.current_override_ = pto_state.torque_command;
  if (pto_state.torque_command) {
    this->dataPtr->functor.I_Wind.UserCommandedCurrent = pto_state.torque_command.value();
  } else {
    this->dataPtr->functor.I_Wind.UserCommandedCurrent = 0.0;
  }

  this->dataPtr->functor.I_Wind.bias_override_ = pto_state.bias_current_command;
  if (pto_state.bias_current_command) {
    this->dataPtr->functor.I_Wind.BiasCurrent = pto_state.bias_current_command.value();
  } else {
    this->dataPtr->functor.I_Wind.BiasCurrent = DEFAULT_BIASCURRENT;
  }

  // Initial Guess based on perfect efficiency
  Eigen::HybridNonLinearSolver<ElectroHydraulicSoln> solver(this->dataPtr->functor);
  const int solver_info = solver.solveNumericalDiff(this->dataPtr->x);


  // Solve Electrical
  const double N = this->dataPtr->x[0U];
  double deltaP = this->dataPtr->x[1U];
  this->dataPtr->TargetWindingCurrent = this->dataPtr->functor.I_Wind.I;
  unsigned int seed{1U};
  this->dataPtr->WindingCurrent = this->dataPtr->TargetWindingCurrent + 0.001 *
    (rand_r(&seed) % 200 - 100);

  static const double eff_e = 0.85;
  static const double RPM_TO_RAD_PER_SEC = 2.0 * M_PI / 60.0;
  static const double P_conv = -1.375 * this->dataPtr->functor.I_Wind.TorqueConstantNMPerAmp *
    RPM_TO_RAD_PER_SEC;
  const double ShaftPower = P_conv * this->dataPtr->WindingCurrent * N;  // Watts
  double P = eff_e * ShaftPower;
  static const double Ri = 8.0;  // Ohms

  static const double two_a = 2.0 / Ri;
  static const double four_a = 2.0 * two_a;
  static const double neg_b = this->dataPtr->Ve / Ri;
  static const double neg_b_sq = neg_b * neg_b;

  // TODO(hamilton) temporary fix for NaN's when discriminant < 0.0
  // this happens when user commanded winding current is too large
  const double c = std::min(-P, neg_b_sq / four_a - 0.001 /* ensure discriminant > 0.0 */);
  // P = -c;

  const double sqrt_discriminant = sqrt(neg_b_sq - four_a * c);
  const double VBus1 = (neg_b + sqrt_discriminant) / two_a;
  const double VBus2 = (neg_b - sqrt_discriminant) / two_a;

  double VBus = VBus1 > VBus2 ? std::min(VBus1, 325.0) : std::min(VBus2, 325.0);

  double I_Batt = (VBus - this->dataPtr->Ve) / Ri;
  static const double I_BattMax = 7.0;

  if (I_Batt > I_BattMax) {  // Need to limit charge current
    I_Batt = I_BattMax;
    VBus = this->dataPtr->Ve + Ri * I_BattMax;
  }
  const double I_Load = P / VBus - I_Batt;


  // Assign Values
  pto_state.rpm = N;
  pto_state.voltage = VBus;
  pto_state.bcurrent = I_Batt;
  pto_state.wcurrent = this->dataPtr->WindingCurrent;
  pto_state.diff_press = deltaP;
  pto_state.bias_current = this->dataPtr->functor.I_Wind.BiasCurrent;
  pto_state.loaddc = I_Load;
  pto_state.scale = this->dataPtr->functor.I_Wind.ScaleFactor;
  pto_state.retract = this->dataPtr->functor.I_Wind.RetractFactor;
  pto_state.target_a = this->dataPtr->TargetWindingCurrent;

  _ecm.SetComponentData<buoy_gazebo::components::ElectroHydraulicState>(
    this->dataPtr->PrismaticJointEntity,
    pto_state);


  auto stampMsg = gz::sim::convert<gz::msgs::Time>(_info.simTime);

  gz::msgs::Double pistonvel;
  pistonvel.mutable_header()->mutable_stamp()->CopyFrom(stampMsg);
  pistonvel.set_data(xdot);

  gz::msgs::Double rpm;
  rpm.mutable_header()->mutable_stamp()->CopyFrom(stampMsg);
  rpm.set_data(N);

  gz::msgs::Double deltap;
  deltap.mutable_header()->mutable_stamp()->CopyFrom(stampMsg);
  deltap.set_data(deltaP);

  gz::msgs::Double targwindcurr;
  targwindcurr.mutable_header()->mutable_stamp()->CopyFrom(stampMsg);
  targwindcurr.set_data(this->dataPtr->TargetWindingCurrent);

  gz::msgs::Double windcurr;
  windcurr.mutable_header()->mutable_stamp()->CopyFrom(stampMsg);
  windcurr.set_data(this->dataPtr->WindingCurrent);

  gz::msgs::Double battcurr;
  battcurr.mutable_header()->mutable_stamp()->CopyFrom(stampMsg);
  battcurr.set_data(I_Batt);

  gz::msgs::Double loadcurr;
  loadcurr.mutable_header()->mutable_stamp()->CopyFrom(stampMsg);
  loadcurr.set_data(I_Load);

  gz::msgs::Double scalefactor;
  scalefactor.mutable_header()->mutable_stamp()->CopyFrom(stampMsg);
  scalefactor.set_data(this->dataPtr->functor.I_Wind.ScaleFactor);

  gz::msgs::Double retractfactor;
  retractfactor.mutable_header()->mutable_stamp()->CopyFrom(stampMsg);
  retractfactor.set_data(this->dataPtr->functor.I_Wind.RetractFactor);

  if (!pistonvel_pub.Publish(pistonvel)) {
    gzerr << "could not publish pistonvel" << std::endl;
  }

  if (!rpm_pub.Publish(rpm)) {
    gzerr << "could not publish rpm" << std::endl;
  }

  if (!deltaP_pub.Publish(deltap)) {
    gzerr << "could not publish deltaP" << std::endl;
  }

  if (!targwindcurr_pub.Publish(targwindcurr)) {
    gzerr << "could not publish targwindcurr" << std::endl;
  }

  if (!windcurr_pub.Publish(windcurr)) {
    gzerr << "could not publish windcurr" << std::endl;
  }

  if (!battcurr_pub.Publish(battcurr)) {
    gzerr << "could not publish battcurr" << std::endl;
  }

  if (!loadcurr_pub.Publish(loadcurr)) {
    gzerr << "could not publish loadcurr" << std::endl;
  }

  if (!scalefactor_pub.Publish(scalefactor)) {
    gzerr << "could not publish scalefactor" << std::endl;
  }

  if (!retractfactor_pub.Publish(retractfactor)) {
    gzerr << "could not publish retractfactor" << std::endl;
  }


  // Apply force if not in Velocity Mode, in which case a joint velocity is applied elsewhere
  // (likely by a test Fixture)
  if (!this->dataPtr->VelMode) {
    double piston_force = deltaP * this->dataPtr->PistonArea;
    // Create new component for this entitiy in ECM (if it doesn't already exist)
    auto forceComp = _ecm.Component<gz::sim::components::JointForceCmd>(
      this->dataPtr->PrismaticJointEntity);
    if (forceComp == nullptr) {
      _ecm.CreateComponent(
        this->dataPtr->PrismaticJointEntity,
        gz::sim::components::JointForceCmd({piston_force}));  // Create this iteration
    } else {
      forceComp->Data()[0] += piston_force;  // Add force to existing forces.
    }
  }
}
}  // namespace buoy_gazebo

GZ_ADD_PLUGIN(
  buoy_gazebo::ElectroHydraulicPTO,
  gz::sim::System,
  buoy_gazebo::ElectroHydraulicPTO::ISystemConfigure,
  buoy_gazebo::ElectroHydraulicPTO::ISystemPreUpdate);
