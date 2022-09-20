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

#include "ElectroHydraulicPTO.hpp"
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

#include <unsupported/Eigen/NonLinearOptimization>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "ElectroHydraulicState.hpp"
#include "ElectroHydraulicSoln.hpp"


namespace buoy_gazebo
{
class ElectroHydraulicPTOPrivate
{
public:
  /// \brief Piston joint entity
  ignition::gazebo::Entity PrismaticJointEntity{ignition::gazebo::kNullEntity};

  /// \brief Piston area
  double PistonArea{1.0};

  /// \brief Rotor Inertia
  double RotorInertia{1.0};

  /// \brief Model interface
  ignition::gazebo::Model model{ignition::gazebo::kNullEntity};

  ElectroHydraulicSoln functor{};

  Eigen::VectorXd x{};

  double TargetWindingCurrent{0.0};

  double WindingCurrent{0.0};

  static constexpr double Ve{315.0};

  bool VelMode{false};

  /// \brief Ignition communication node.
  ignition::transport::Node node;
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
  const ignition::gazebo::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  ignition::gazebo::EntityComponentManager & _ecm,
  ignition::gazebo::EventManager & /*_eventMgr*/)
{
  this->dataPtr->model = ignition::gazebo::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    ignerr << "ElectroHydraulicPTO plugin should be attached to a model entity. " <<
      "Failed to initialize." << std::endl;
    return;
  }


  // Get params from SDF for Prismatic Joint.
  auto PrismaticJointName = _sdf->Get<std::string>("PrismaticJointName");
  if (PrismaticJointName.empty()) {
    ignerr << "ElectroHydraulicPTO found an empty PrismaticJointName parameter. " <<
      "Failed to initialize.";
    return;
  }


  this->dataPtr->PrismaticJointEntity = this->dataPtr->model.JointByName(
    _ecm,
    PrismaticJointName);
  if (this->dataPtr->PrismaticJointEntity == ignition::gazebo::kNullEntity) {
    ignerr << "Joint with name [" << PrismaticJointName << "] not found. " <<
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
  pistonvel_pub = node.Advertise<ignition::msgs::Double>(pistonvel_topic);
  if (!pistonvel_pub) {
    ignerr << "Error advertising topic [" << pistonvel_topic << "]" << std::endl;
    return;
  }
}

//////////////////////////////////////////////////
void ElectroHydraulicPTO::PreUpdate(
  const ignition::gazebo::UpdateInfo & _info,
  ignition::gazebo::EntityComponentManager & _ecm)
{
  IGN_PROFILE("#ElectroHydraulicPTO::PreUpdate");
  // Nothing left to do if paused.
  if (_info.paused) {
    return;
  }

  auto SimTime = std::chrono::duration<double>(_info.simTime).count();

  // If the joints haven't been identified yet, the plugin is disabled
  if (this->dataPtr->PrismaticJointEntity == ignition::gazebo::kNullEntity) {
    return;
  }

  // \TODO(anyone): Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero()) {
    ignwarn << "Detected jump back in time [" <<
      std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count() <<
      "s]. System may not work properly." << std::endl;
  }


  // Create joint velocity component for piston if one doesn't exist
  auto prismaticJointVelComp = _ecm.Component<ignition::gazebo::components::JointVelocity>(
    this->dataPtr->PrismaticJointEntity);
  if (prismaticJointVelComp == nullptr) {
    _ecm.CreateComponent(
      this->dataPtr->PrismaticJointEntity, ignition::gazebo::components::JointVelocity());
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
  // TODO(hamilton) temporary fix for NaN situation. Should make this more robust
  // or at least parameterized.
  // Problem: If I repeatedly smash the PC with a -30 Amp winding current command, this solution
  // becomes unstable and rpm/pressure reach NaN and gazebo crashes. I'm clipping it
  // to the max absolute rpm from the winding current interpolation
  // (no extrapolation, default torque controller).
  // const double N = std::min(std::max(this->dataPtr->x[0U], -6790.0), 6790.0);
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

  auto stampMsg = ignition::gazebo::convert<ignition::msgs::Time>(_info.simTime);

  ignition::msgs::Double pistonvel;
  pistonvel.mutable_header()->mutable_stamp()->CopyFrom(stampMsg);
  pistonvel.set_data(xdot);


  if (!pistonvel_pub.Publish(pistonvel)) {
    ignerr << "could not publish pistonvel" << std::endl;
  }

  // Apply force if not in Velocity Mode, in which case a joint velocity is applied elsewhere
  // (likely by a test Fixture)
  if (!this->dataPtr->VelMode) {
    double piston_force = deltaP * this->dataPtr->PistonArea;
    // Create new component for this entitiy in ECM (if it doesn't already exist)
    auto forceComp = _ecm.Component<ignition::gazebo::components::JointForceCmd>(
      this->dataPtr->PrismaticJointEntity);
    if (forceComp == nullptr) {
      _ecm.CreateComponent(
        this->dataPtr->PrismaticJointEntity,
        ignition::gazebo::components::JointForceCmd({piston_force}));  // Create this iteration
    } else {
      forceComp->Data()[0] += piston_force;  // Add force to existing forces.
    }
  }
}
}  // namespace buoy_gazebo

IGNITION_ADD_PLUGIN(
  buoy_gazebo::ElectroHydraulicPTO,
  ignition::gazebo::System,
  buoy_gazebo::ElectroHydraulicPTO::ISystemConfigure,
  buoy_gazebo::ElectroHydraulicPTO::ISystemPreUpdate);
