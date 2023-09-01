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

#include <eigen3/unsupported/Eigen/NonLinearOptimization>

#include <gz/msgs/double.pb.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <buoy_utils/Constants.hpp>

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
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "ElectroHydraulicSoln.hpp"
#include "ElectroHydraulicState.hpp"
#include "ElectroHydraulicLoss.hpp"
#include "BatteryState.hpp"

#include <LatentData/LatentData.hpp>


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

  double Ve{300.0};

  /// \brief Default Scale Factor
  double DefaultScaleFactor{DEFAULT_SCALE_FACTOR};

  /// \brief Model interface
  gz::sim::Model model{gz::sim::kNullEntity};

  ElectroHydraulicSoln functor{};

  Eigen::VectorXd x{};

  static constexpr double Ri{7.0};
  static constexpr double I_BattChargeMax{7.0};
  static constexpr double MaxTargetVoltage{325.0};


// Dummy compensator pressure for ROS messages, not simulated
  static constexpr double CompensatorPressure{2.91};

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

  this->dataPtr->DefaultScaleFactor =
    SdfParamDouble(_sdf, "ScaleFactor", this->dataPtr->DefaultScaleFactor);

  if (_sdf->HasElement("BatterySoC")) {
    double SoC =
      SdfParamDouble(_sdf, "BatterySoC", 3.0 / 5.0);  // default to Ve = 300V
    this->dataPtr->Ve = 50.0 * SoC + 270.0;  // SoC (0.0 to 1.0) gives Ve (270V to 320V)
    gzdbg << "Battery SoC (" << SoC
          << ") -> Battery EMF (" << this->dataPtr->Ve << "V)" << std::endl;
  } else {
    this->dataPtr->Ve =
      SdfParamDouble(_sdf, "BatteryEMF", this->dataPtr->Ve);
    gzdbg << "Battery EMF (" << this->dataPtr->Ve << "V)" << std::endl;
  }

  if (_sdf->HasElement("VelMode")) {
    this->dataPtr->VelMode = true;
  }

  // Need to set to actual ram position for soft-stop at ends, mid-span for now
  this->dataPtr->functor.I_Wind.RamPosition = 40.0;

  this->dataPtr->x.setConstant(3, 0.0);
  this->dataPtr->x[2] = this->dataPtr->Ve;

  std::string pistonvel_topic = std::string("/pistonvel_") + PrismaticJointName;
  pistonvel_pub = node.Advertise<gz::msgs::Double>(pistonvel_topic);
  if (!pistonvel_pub) {
    gzerr << "Error advertising topic [" << pistonvel_topic << "]" << std::endl;
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

  // Create joint position component for piston if one doesn't exist
  auto prismaticJointPosComp = _ecm.Component<gz::sim::components::JointPosition>(
    this->dataPtr->PrismaticJointEntity);
  if (prismaticJointPosComp == nullptr) {
    _ecm.CreateComponent(
      this->dataPtr->PrismaticJointEntity, gz::sim::components::JointPosition());
  }
  // We just created the joint velocity component, give one iteration for the
  // physics system to update its size
  if (prismaticJointPosComp == nullptr || prismaticJointPosComp->Data().empty()) {
    return;
  }

  // Retrieve Piston velocity, compute flow and provide as input to hydraulic solver.
  // TODO(anyone): Figure out if (0) for the index is always correct,
  // some OR code has a process of finding the index for this argument.
  double xdot = prismaticJointVelComp->Data().at(0);
  this->dataPtr->functor.Q = xdot * buoy_utils::INCHES_PER_METER * this->dataPtr->PistonArea;

  double PistonPos = prismaticJointVelComp->Data().at(0);
  // this->dataPtr->functor.I_Wind.RamPosition = (2.03 - PistonPos) * buoy_utils::INCHES_PER_METER;
  this->dataPtr->functor.I_Wind.RamPosition = 40.0;

  // Compute Resulting Rotor RPM and Force applied to Piston based on kinematics
  // and quasistatic forces.  These neglect oil compressibility and rotor inertia,
  // but do include mechanical and volumetric efficiency of hydraulic motor.
  // This is an implicit non-linear relation so iteration required,
  // performed by Eigen HybridNonLinearSolver

  buoy_gazebo::BatteryState battery_state{};
  if (_ecm.EntityHasComponentType(
      this->dataPtr->PrismaticJointEntity,
      buoy_gazebo::components::BatteryState().TypeId()))
  {
    auto battery_state_comp =
      _ecm.Component<buoy_gazebo::components::BatteryState>(this->dataPtr->PrismaticJointEntity);

    battery_state = buoy_gazebo::BatteryState(battery_state_comp->Data());
  }

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

  buoy_gazebo::ElectroHydraulicLoss pto_loss;
  if (_ecm.EntityHasComponentType(
      this->dataPtr->PrismaticJointEntity,
      buoy_gazebo::components::ElectroHydraulicLoss().TypeId()))
  {
    auto pto_loss_comp =
      _ecm.Component<buoy_gazebo::components::ElectroHydraulicLoss>(
      this->dataPtr->PrismaticJointEntity);

    pto_loss = buoy_gazebo::ElectroHydraulicLoss(pto_loss_comp->Data());
  }

  buoy_gazebo::LatentData latent_data;
  if (_ecm.EntityHasComponentType(
      this->dataPtr->model.Entity(),
      buoy_gazebo::components::LatentData().TypeId()))
  {
    auto latent_data_comp =
      _ecm.Component<buoy_gazebo::components::LatentData>(this->dataPtr->model.Entity());

    latent_data = buoy_gazebo::LatentData(latent_data_comp->Data());
  }

  if (pto_state.scale_command) {
    this->dataPtr->functor.I_Wind.ScaleFactor = pto_state.scale_command.value();
  } else {
    this->dataPtr->functor.I_Wind.ScaleFactor = this->dataPtr->DefaultScaleFactor;
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

  this->dataPtr->functor.VBattEMF = this->dataPtr->Ve;
  this->dataPtr->functor.Ri = this->dataPtr->Ri;       // Ohms

  // See MINPACK documentation for details on this solver
  // Parameters and defaults are (Scalar = double):
  //           : factor(Scalar(100.))
  //           , maxfev(1000)
  //           , xtol(std::sqrt(NumTraits<Scalar>::epsilon()))
  //           , nb_of_subdiagonals(-1)
  //           , nb_of_superdiagonals(-1)
  //           , epsfcn(Scalar(0.)) {}
  Eigen::HybridNonLinearSolver<ElectroHydraulicSoln> solver(this->dataPtr->functor);
  solver.parameters.xtol = 0.0001;
  solver.parameters.maxfev = 1000;
  solver.diag.setConstant(3, 1.);
  solver.useExternalScaling = true;       // Improves solution stability dramatically.

  int solver_info;
  int i_try;
  for (i_try = 0; i_try < 4; i_try++) {
    // Initial condition based on perfect efficiency
    this->dataPtr->x[0] = buoy_utils::SecondsPerMinute * this->dataPtr->functor.Q /
      this->dataPtr->functor.HydMotorDisp;

    double WindCurr = this->dataPtr->functor.I_Wind(this->dataPtr->x[0U]);
    // 1.375 fudge factor required to match experiments, not yet sure why.
    const double T_applied = 1.375 * this->dataPtr->functor.I_Wind.TorqueConstantInLbPerAmp *
      WindCurr;
    this->dataPtr->x[1] = -T_applied / (this->dataPtr->functor.HydMotorDisp / (2 * M_PI));

    // Estimate VBus based on linearized battery
    double PBus = -this->dataPtr->x[0] * buoy_utils::RPM_TO_RAD_PER_SEC *
      T_applied * buoy_utils::NM_PER_INLB;
    this->dataPtr->x[2] = this->dataPtr->functor.Ri * PBus / this->dataPtr->Ve + this->dataPtr->Ve;

    solver_info = solver.solveNumericalDiff(this->dataPtr->x);
    if (solver_info == 1) {
      break;                   // Solution found so continue
    } else {
      this->dataPtr->functor.Q *= 0.95;  // Reduce piston speed slightly and try again
    }
  }

  if (i_try > 0) {
    std::stringstream warning;
    warning << "Warning: Reduced piston to achieve convergence" << std::endl;
    gzdbg << warning.str();
  }

  if (solver_info != 1) {
    std::stringstream warning;
    warning << "=================================" << std::endl;
    warning << "Warning: Numericals solver in ElectroHydraulicPTO did not converge" << std::endl;
    warning << "solver info: [" << solver_info << "]" << std::endl;
    warning << "=================================" << std::endl;
    gzdbg << warning.str();
  }

  // Solve Electrical
  const double N = this->dataPtr->x[0U];
  double deltaP = this->dataPtr->x[1U];
  double VBus = this->dataPtr->x[2U];
  VBus = std::min(VBus, this->dataPtr->MaxTargetVoltage);
  double BusPower = this->dataPtr->functor.BusPower;

  double I_Batt = (VBus - this->dataPtr->Ve) / this->dataPtr->Ri;
  if (I_Batt > this->dataPtr->I_BattChargeMax) {       // Need to limit charge current
    I_Batt = this->dataPtr->I_BattChargeMax;
    VBus = this->dataPtr->Ve + this->dataPtr->Ri * this->dataPtr->I_BattChargeMax;
  }

  double I_Load = 0.0;
  if (BusPower > 0) {
    I_Load = BusPower / VBus - I_Batt;
  }

  // Assign Values
  battery_state.voltage = VBus;
  battery_state.ips = 0.0;
  battery_state.vbalance = 2.76;
  battery_state.vstopcharge = 2.79;
  battery_state.gfault = 0.06;
  battery_state.hydrogen = 3.26;
  battery_state.status = 11264;

  pto_state.rpm = N;
  pto_state.voltage = VBus;
  pto_state.bcurrent = I_Batt;
  pto_state.wcurrent = this->dataPtr->functor.I_Wind.I;
  pto_state.diff_press = this->dataPtr->CompensatorPressure;
  if (deltaP >= 0) {       // Upper Pressure is > Lower Pressure
    pto_state.upper_hyd_press = deltaP;
    pto_state.lower_hyd_press = 0.0;
  } else {
    pto_state.upper_hyd_press = 0.0;
    pto_state.lower_hyd_press = -deltaP;
  }
  pto_state.bias_current = this->dataPtr->functor.I_Wind.BiasCurrent;
  pto_state.loaddc = I_Load;
  pto_state.scale = this->dataPtr->functor.I_Wind.ScaleFactor;
  pto_state.retract = this->dataPtr->functor.I_Wind.RetractFactor;
  pto_state.target_a = this->dataPtr->functor.I_Wind.I;


  pto_loss.hydraulic_motor_loss += 1.0;
  pto_loss.relief_valve_loss += 2.0;
  pto_loss.motor_drive_i2r_loss =
    this->dataPtr->functor.MotorDriveISquaredRLoss(this->dataPtr->functor.I_Wind.I);
  pto_loss.motor_drive_switching_loss = this->dataPtr->functor.MotorDriveSwitchingLoss(VBus);
  pto_loss.motor_drive_friction_loss = this->dataPtr->functor.MotorDriveFrictionLoss(N);
  pto_loss.battery_i2r_loss = I_Batt * I_Batt * this->dataPtr->Ri;

  double piston_force = -deltaP * this->dataPtr->PistonArea * buoy_utils::NEWTONS_PER_LB;

  latent_data.electro_hydraulic.valid = true;
  latent_data.electro_hydraulic.inst_power = VBus * (I_Batt + I_Load);
  latent_data.electro_hydraulic.rpm = N;
  latent_data.electro_hydraulic.force = piston_force;
  latent_data.electro_hydraulic.motor_drive_i2r_loss = pto_loss.motor_drive_i2r_loss;
  latent_data.electro_hydraulic.motor_drive_switching_loss = pto_loss.motor_drive_switching_loss;
  latent_data.electro_hydraulic.motor_drive_friction_loss = pto_loss.motor_drive_friction_loss;
  latent_data.electro_hydraulic.battery_i2r_loss = pto_loss.battery_i2r_loss;

  _ecm.SetComponentData<buoy_gazebo::components::BatteryState>(
    this->dataPtr->PrismaticJointEntity,
    battery_state);

  _ecm.SetComponentData<buoy_gazebo::components::ElectroHydraulicState>(
    this->dataPtr->PrismaticJointEntity,
    pto_state);

  auto stampMsg = gz::sim::convert<gz::msgs::Time>(_info.simTime);

  _ecm.SetComponentData<buoy_gazebo::components::ElectroHydraulicLoss>(
    this->dataPtr->PrismaticJointEntity,
    pto_loss);

  _ecm.SetComponentData<buoy_gazebo::components::LatentData>(
    this->dataPtr->model.Entity(),
    latent_data);

  gz::msgs::Double pistonvel;
  pistonvel.mutable_header()->mutable_stamp()->CopyFrom(stampMsg);
  pistonvel.set_data(xdot);


  if (!pistonvel_pub.Publish(pistonvel)) {
    gzerr << "could not publish pistonvel" << std::endl;
  }

  // Apply force if not in Velocity Mode, in which case a joint velocity is applied elsewhere
  // (likely by a test Fixture)
  if (!this->dataPtr->VelMode) {
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
