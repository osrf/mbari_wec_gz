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

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "ElectroHydraulicState.hpp"
#include "ElectroHydraulicSoln.hpp"

#include "JustInterp/JustInterp.hpp"

namespace buoy_gazebo
{
class ElectroHydraulicPTOPrivate
{
public:
  /// \brief Piston joint entity
  ignition::gazebo::Entity PrismaticJointEntity;

  /// \brief Piston area
  double PistonArea;

  /// \brief Rotor Inertia
  double RotorInertia;

  /// \brief Model interface
  ignition::gazebo::Model model {ignition::gazebo::kNullEntity};

  ElectroHydraulicSoln functor;

  Eigen::VectorXd x;

  double TargetWindingCurrent;

  double WindingCurrent;

  double Ve;

  bool VelMode;

  /// \brief Ignition communication node.
  ignition::transport::Node node;

  /// \brief Callback for User Commanded Current subscription
  /// \param[in] _msg Current
  void OnUserCmdCurr(const ignition::msgs::Double & _msg);

  /// \brief Callback for BiasCurrent subscription
  /// \param[in] _msg Current
  void OnUserBiasCurr(const ignition::msgs::Double & _msg);

  /// \brief Callback for ScaleFactor subscription
  /// \param[in] _msg Current
  void OnScale(const ignition::msgs::Double & _msg);

  /// \brief Callback for RetractFactor subscription
  /// \param[in] _msg Current
  void OnRetract(const ignition::msgs::Double & _msg);
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
    this->dataPtr->PistonArea = SdfParamDouble(_sdf, "PistonArea", 1.);
  }

  // Default to Parker F11-5  0.30in^3/rev
  this->dataPtr->functor.HydMotorDisp = SdfParamDouble(_sdf, "HydMotorDisp", 0.30);
  this->dataPtr->RotorInertia = SdfParamDouble(_sdf, "RotorInertia", 1);

  if (_sdf->HasElement("VelMode")) {
    this->dataPtr->VelMode = true;
  } else {
    this->dataPtr->VelMode = false;
  }

  this->dataPtr->Ve = 315;


  // Set Default Damping Relation
  {
    std::vector<double> P {0, 2800, 3000};
    std::vector<double> hyd_eff_v {1, 1, 0};
    this->dataPtr->functor.reliefValve.SetData(P.size(), P.data(), hyd_eff_v.data());
  }


  this->dataPtr->x.setConstant(2, 0.);

  // Subscribe to commands
  std::string topic = ignition::transport::TopicUtils::AsValidTopic(
    "/model/" +
    this->dataPtr->model.Name(_ecm) + "/joint/" + PrismaticJointName +
    "/UserCommandedCurr");
  if (topic.empty()) {
    std::cout << "###Failed to create topic for joint [" << PrismaticJointName <<
      "]" << std::endl;
    return;
  }
  this->dataPtr->node.Subscribe(
    topic, &ElectroHydraulicPTOPrivate::OnUserCmdCurr,
    this->dataPtr.get());
  std::cout << "###ElectroHydraulicPTO subscribing to Double messages on [" << topic <<
    "]" << std::endl;


  topic = ignition::transport::TopicUtils::AsValidTopic(
    "/model/" +
    this->dataPtr->model.Name(_ecm) + "/joint/" + PrismaticJointName +
    "/BiasCurrent");
  if (topic.empty()) {
    std::cout << "###Failed to create topic for joint [" << PrismaticJointName <<
      "]" << std::endl;
    return;
  }
  this->dataPtr->node.Subscribe(
    topic, &ElectroHydraulicPTOPrivate::OnUserBiasCurr,
    this->dataPtr.get());
  std::cout << "###ElectroHydraulicPTO subscribing to Double messages on [" << topic <<
    "]" << std::endl;


  topic = ignition::transport::TopicUtils::AsValidTopic(
    "/model/" +
    this->dataPtr->model.Name(_ecm) + "/joint/" + PrismaticJointName +
    "/ScaleFactor");
  if (topic.empty()) {
    std::cout << "###Failed to create topic for joint [" << PrismaticJointName <<
      "]" << std::endl;
    return;
  }
  this->dataPtr->node.Subscribe(
    topic, &ElectroHydraulicPTOPrivate::OnScale,
    this->dataPtr.get());
  std::cout << "###ElectroHydraulicPTO subscribing to Double messages on [" << topic <<
    "]" << std::endl;


  topic = ignition::transport::TopicUtils::AsValidTopic(
    "/model/" +
    this->dataPtr->model.Name(_ecm) + "/joint/" + PrismaticJointName +
    "/RetractFactor");
  if (topic.empty()) {
    std::cout << "###Failed to create topic for joint [" << PrismaticJointName <<
      "]" << std::endl;
    return;
  }
  this->dataPtr->node.Subscribe(
    topic, &ElectroHydraulicPTOPrivate::OnRetract,
    this->dataPtr.get());
  std::cout << "###ElectroHydraulicPTO subscribing to Double messages on [" << topic <<
    "]" << std::endl;


  std::string pistonvel_topic = std::string("/pistonvel_") + PrismaticJointName;
  pistonvel_pub = node.Advertise<ignition::msgs::Double>(pistonvel_topic);
  if (!pistonvel_pub) {
    ignerr << "Error advertising topic [" << pistonvel_topic << "]" << std::endl;
    return;
  }

  std::string rpm_topic = std::string("/rpm_") + PrismaticJointName;
  rpm_pub = node.Advertise<ignition::msgs::Double>(rpm_topic);
  if (!rpm_pub) {
    ignerr << "Error advertising topic [" << rpm_topic << "]" << std::endl;
    return;
  }

  std::string deltaP_topic = std::string("/deltaP_") + PrismaticJointName;
  deltaP_pub = node.Advertise<ignition::msgs::Double>(deltaP_topic);
  if (!deltaP_pub) {
    ignerr << "Error advertising topic [" << deltaP_topic << "]" << std::endl;
    return;
  }

  std::string targwindcurr_topic = std::string("/targwindcurr_") + PrismaticJointName;
  targwindcurr_pub = node.Advertise<ignition::msgs::Double>(targwindcurr_topic);
  if (!targwindcurr_pub) {
    ignerr << "Error advertising topic [" << targwindcurr_topic << "]" << std::endl;
    return;
  }

  std::string windcurr_topic = std::string("/windcurr_") + PrismaticJointName;
  windcurr_pub = node.Advertise<ignition::msgs::Double>(windcurr_topic);
  if (!windcurr_pub) {
    ignerr << "Error advertising topic [" << windcurr_topic << "]" << std::endl;
    return;
  }

  std::string battcurr_topic = std::string("/battcurr_") + PrismaticJointName;
  battcurr_pub = node.Advertise<ignition::msgs::Double>(battcurr_topic);
  if (!battcurr_pub) {
    ignerr << "Error advertising topic [" << battcurr_topic << "]" << std::endl;
    return;
  }

  std::string loadcurr_topic = std::string("/loadcurr_") + PrismaticJointName;
  loadcurr_pub = node.Advertise<ignition::msgs::Double>(loadcurr_topic);
  if (!loadcurr_pub) {
    ignerr << "Error advertising topic [" << loadcurr_topic << "]" << std::endl;
    return;
  }

  std::string scalefactor_topic = std::string("/scalefactor_") + PrismaticJointName;
  scalefactor_pub = node.Advertise<ignition::msgs::Double>(scalefactor_topic);
  if (!scalefactor_pub) {
    ignerr << "Error advertising topic [" << scalefactor_topic << "]" << std::endl;
    return;
  }

  std::string retractfactor_topic = std::string("/retractfactor_") + PrismaticJointName;
  retractfactor_pub = node.Advertise<ignition::msgs::Double>(retractfactor_topic);
  if (!retractfactor_pub) {
    ignerr << "Error advertising topic [" << retractfactor_topic << "]" << std::endl;
    return;
  }
}


//////////////////////////////////////////////////
void ElectroHydraulicPTO::PreUpdate(
  const ignition::gazebo::UpdateInfo & _info,
  ignition::gazebo::EntityComponentManager & _ecm)
{
  const int n = 2;
  int info;

  IGN_PROFILE("ElectroHydraulicPTO::Update");
  // Nothing left to do if paused.
  if (_info.paused) {
    return;
  }

  auto SimTime = std::chrono::duration<double>(_info.simTime).count();

  IGN_PROFILE("#ElectroHydraulicPTO::PreUpdate");

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
  this->dataPtr->functor.Q = xdot * 39.4 * this->dataPtr->PistonArea;    // inch^3/second

  // Retrieve Piston position and simulation time, and provide as input to hydraulic solver.
  // double x = prismaticJointVelComp->Data().at(0);
  // TODO(anyone): Figure out if (0) for the index is always correct,
  // some OR code has a process of finding the index for this argument.
  this->dataPtr->functor.I_Wind.PistonPos = 12.12;    // x*39.4;  // inch^3/second
  // Set Sim time in IWind so it knows when to timeout user and bias current commands.
  this->dataPtr->functor.I_Wind.SimTime = SimTime;

  // Compute Resulting Rotor RPM and Force applied to Piston based on kinematics
  // and quasistatic forces.  These neglect oil compressibility and rotor inertia,
  // but do include mechanical and volumetric efficiency of hydraulic motor.
  // This is an implicit non-linear relation so iteration required,
  // performed by Eigen HybridNonLinearSolver

  // Preclude changing User Commanded Current while it may be being read.
  this->dataPtr->functor.I_Wind.UserCommandMutex.lock();
  // Initial Guess based on perfect efficiency
  // this->dataPtr->x[0] = 60*this->dataPtr->functor.Q/this->dataPtr->functor.HydMotorDisp;
  // this->dataPtr->x[1] = this->dataPtr->functor.T_applied/(this->dataPtr->functor.HydMotorDisp);
  Eigen::HybridNonLinearSolver<ElectroHydraulicSoln> solver(this->dataPtr->functor);
  info = solver.solveNumericalDiff(this->dataPtr->x);
  // info = solver.hybrd1(this->dataPtr->x);
  this->dataPtr->functor.I_Wind.UserCommandMutex.unlock();


  // Solve Electrical
  double N = this->dataPtr->x[0];
  double deltaP = this->dataPtr->x[1];
  // Shame to have to re-compute this, but small effort...
  this->dataPtr->TargetWindingCurrent = this->dataPtr->functor.I_Wind(N);
  // Create seed on thread using current time
  unsigned int seed = (unsigned) time(NULL);
  this->dataPtr->WindingCurrent = this->dataPtr->TargetWindingCurrent + 0.001 *
    (rand_r(&seed) % 200 - 100);

  double eff_e = 0.85;
  double ShaftPower = -1.375 * this->dataPtr->functor.I_Wind.TorqueConstantNMPerAmp *
    this->dataPtr->WindingCurrent * 2 * M_PI * N / 60;                      // Watts
  double P = eff_e * ShaftPower;
  double Ri = 8;      // Ohms

  double a = (1.0 / Ri);
  double b = -this->dataPtr->Ve / Ri;
  double c = -P;

  double VBus;
  double VBus1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
  double VBus2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);

  if (VBus1 > VBus2) {
    VBus = VBus1;
  } else {
    VBus = VBus2;
  }

  if (VBus > 325) {
    VBus = 325;
  }

  double I_Batt = (VBus - this->dataPtr->Ve) / Ri;
  double I_BattMax = 7;

  if (I_Batt > I_BattMax) {    // Need to limit charge current
    I_Batt = I_BattMax;
    VBus = this->dataPtr->Ve + Ri * I_BattMax;
  }
  double I_Load = P / VBus - I_Batt;


  // Report results
  // std::cout << SimTime << "  " << "  " << xdot / .0254 << "    " << N << "   "
  //           << deltaP << "   " << VBus << "   " << this->dataPtr->TargetWindingCurrent
  //           << "  " << this->dataPtr->WindingCurrent << "  "  << I_Batt << "   "
  //           << I_Load << std::endl;


  ignition::msgs::Double pistonvel, rpm, deltap, targwindcurr, windcurr,
    battcurr, loadcurr, scalefactor, retractfactor;
  pistonvel.set_data(xdot);
  rpm.set_data(N);
  deltap.set_data(deltaP);
  targwindcurr.set_data(this->dataPtr->TargetWindingCurrent);
  windcurr.set_data(this->dataPtr->WindingCurrent);
  battcurr.set_data(I_Batt);
  loadcurr.set_data(I_Load);
  scalefactor.set_data(this->dataPtr->functor.I_Wind.ScaleFactor);
  retractfactor.set_data(this->dataPtr->functor.I_Wind.RetractFactor);


  if (!pistonvel_pub.Publish(pistonvel)) {
    ignerr << "could not publish pistonvel" << std::endl;
  }

  if (!rpm_pub.Publish(rpm)) {
    ignerr << "could not publish rpm" << std::endl;
  }

  if (!deltaP_pub.Publish(deltap)) {
    ignerr << "could not publish deltaP" << std::endl;
  }

  if (!targwindcurr_pub.Publish(targwindcurr)) {
    ignerr << "could not publish targwindcurr" << std::endl;
  }

  if (!windcurr_pub.Publish(windcurr)) {
    ignerr << "could not publish windcurr" << std::endl;
  }

  if (!battcurr_pub.Publish(battcurr)) {
    ignerr << "could not publish battcurr" << std::endl;
  }

  if (!loadcurr_pub.Publish(loadcurr)) {
    ignerr << "could not publish loadcurr" << std::endl;
  }

  if (!scalefactor_pub.Publish(scalefactor)) {
    ignerr << "could not publish scalefactor" << std::endl;
  }

  if (!retractfactor_pub.Publish(retractfactor)) {
    ignerr << "could not publish retractfactor" << std::endl;
  }


  // Assign Values
  ElectroHydraulicState PTO_Values;
  PTO_Values.xdot = xdot / 0.0254;
  PTO_Values.N = N;
  PTO_Values.deltaP = deltaP;
  PTO_Values.VBus = VBus;
  PTO_Values.TargetWindingCurrent = this->dataPtr->TargetWindingCurrent;
  PTO_Values.WindingCurrent = this->dataPtr->WindingCurrent;
  PTO_Values.I_Batt = I_Batt;
  PTO_Values.I_Load = I_Load;
  PTO_Values.ScaleFactor = this->dataPtr->functor.I_Wind.ScaleFactor;
  PTO_Values.RetractFactor = this->dataPtr->functor.I_Wind.RetractFactor;


  // Create new component for this entitiy in ECM (if it doesn't already exist)
  auto PTO_State_comp = _ecm.Component<buoy_gazebo::PTO_State>(this->dataPtr->PrismaticJointEntity);
  if (PTO_State_comp == nullptr) {
    _ecm.CreateComponent(
      this->dataPtr->PrismaticJointEntity,
      buoy_gazebo::PTO_State({PTO_Values}));
  } else {
    PTO_State_comp->Data() = PTO_Values;
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
        ignition::gazebo::components::JointForceCmd({piston_force}));    // Create this iteration
    } else {
      forceComp->Data()[0] += piston_force;    // Add force to existing forces.
    }
  }
}

//////////////////////////////////////////////////
void ElectroHydraulicPTO::Update(
  const ignition::gazebo::UpdateInfo & _info,
  ignition::gazebo::EntityComponentManager & _ecm)
{
  IGN_PROFILE("#ElectroHydraulicPTO::Update");
  // Nothing left to do if paused.
  if (_info.paused) {
    return;
  }

  // auto SimTime = std::chrono::duration < double > (_info.simTime).count();
}

//////////////////////////////////////////////////
void ElectroHydraulicPTO::PostUpdate(
  const ignition::gazebo::UpdateInfo & _info,
  const ignition::gazebo::EntityComponentManager & _ecm)
{
  IGN_PROFILE("#ElectroHydraulicPTO::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused) {
    return;
  }

  // auto SimTime = std::chrono::duration < double > (_info.simTime).count();
}


//////////////////////////////////////////////////
void ElectroHydraulicPTOPrivate::OnUserCmdCurr(const ignition::msgs::Double & _msg)
{
  this->functor.I_Wind.SetUserCommandedCurrent(_msg.data());
}

//////////////////////////////////////////////////
void ElectroHydraulicPTOPrivate::OnUserBiasCurr(const ignition::msgs::Double & _msg)
{
  this->functor.I_Wind.SetBiasCurrent(_msg.data());
}

//////////////////////////////////////////////////
void ElectroHydraulicPTOPrivate::OnScale(const ignition::msgs::Double & _msg)
{
  this->functor.I_Wind.SetScaleFactor(_msg.data());
}

//////////////////////////////////////////////////
void ElectroHydraulicPTOPrivate::OnRetract(const ignition::msgs::Double & _msg)
{
  this->functor.I_Wind.SetRetractFactor(_msg.data());
}
}  // namespace buoy_gazebo

IGNITION_ADD_PLUGIN(
  buoy_gazebo::ElectroHydraulicPTO,
  ignition::gazebo::System,
  buoy_gazebo::ElectroHydraulicPTO::ISystemConfigure,
  buoy_gazebo::ElectroHydraulicPTO::ISystemPreUpdate,
  buoy_gazebo::ElectroHydraulicPTO::ISystemUpdate,
  buoy_gazebo::ElectroHydraulicPTO::ISystemPostUpdate);
