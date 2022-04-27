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

#include "PolytropicPneumaticSpring.hpp"

#include <ignition/msgs/double.pb.h>

#include <ignition/common/Profiler.hh>
#include <ignition/math/PID.hh>
#include <ignition/msgs.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <memory>
#include <string>

#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/Model.hh"

namespace buoy_gazebo
{

struct PolytropicPneumaticSpringPrivate
{
  /// \brief Name of cylinder
  std::string name;

  /// \brief is hysteresis present in piston travel direction
  bool hysteresis;

  // SpringType type;

  /// \brief adiabatic index
  double n, n1, n2;

  /// \brief piston stroke (m)
  double stroke;

  /// \brief initial piston position (m)
  double x0, x1, x2;

  /// \brief piston area (m^3)
  double pistonArea;

  /// \brief Piston-End Dead Volume (m^3)
  double deadVolume;

  /// \brief initial Pressure (Pa)
  double P0, P1, P2;

  /// \brief initial Temp (K)
  double T0;

  /// \brief R (specific gas)
  double R;

  /// \brief specific heat of gas under constant pressure (kJ/(kg K))
  double c_p;

  /// \brief initial Volume (m^3)
  double V0, V1, V2;

  /// \brief mass of gas (kg)
  double mass;

  /// \brief c=mR(specific) (Pa*m^3)/K
  double c;

  /// \brief current Volume (m^3)
  double V;

  /// \brief current Pressure of gas (Pa)
  double P;

  /// \brief current Temperature of gas (K)
  double T;

  /// \brief current Force of gas on piston (N)
  double F;

  /// \brief current rate of heat loss (Watts)
  double Q_rate;

  /// \brief move piston along prescribed velocity for sysID
  bool debug_prescribed_velocity;

  /// \brief Joint Entity
  ignition::gazebo::Entity jointEntity;

  /// \brief Model interface
  ignition::gazebo::Model model {ignition::gazebo::kNullEntity};
};

//////////////////////////////////////////////////
PolytropicPneumaticSpring::PolytropicPneumaticSpring()
: dataPtr(std::make_unique<PolytropicPneumaticSpringPrivate>())
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
void PolytropicPneumaticSpring::computeForce(const double & x, const double & v, const double & n)
{
  this->dataPtr->V = this->dataPtr->deadVolume + x * this->dataPtr->pistonArea;
  this->dataPtr->P = this->dataPtr->P0 *
    pow(this->dataPtr->V0 / this->dataPtr->V, this->dataPtr->n);
  this->dataPtr->T = this->dataPtr->P * this->dataPtr->V / this->dataPtr->c;

  if (fabs(n - 1.4) < 1.0e-7) {this->dataPtr->Q_rate = 0.0;} else {
    this->dataPtr->Q_rate = (1.0 - n / 1.4) * (this->dataPtr->c_p / this->dataPtr->R) *
      this->dataPtr->P * this->dataPtr->pistonArea * v;
  }

  this->dataPtr->F = this->dataPtr->P * this->dataPtr->pistonArea;

  ignwarn << "V (" << this->dataPtr->name << "):" << this->dataPtr->V << std::endl;
  ignwarn << "P (" << this->dataPtr->name << "):" << this->dataPtr->P << std::endl;
  ignwarn << "T (" << this->dataPtr->name << "):" << this->dataPtr->T << std::endl;
}

//////////////////////////////////////////////////
void PolytropicPneumaticSpring::Configure(
  const ignition::gazebo::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  ignition::gazebo::EntityComponentManager & _ecm,
  ignition::gazebo::EventManager & /*_eventMgr*/)
{
  this->dataPtr->name = _sdf->Get<std::string>("chamber", "upper_adiabatic").first;
  ignwarn << "name: " << this->dataPtr->name << std::endl;

  this->dataPtr->stroke = SdfParamDouble(_sdf, "stroke", 2.03);
  this->dataPtr->pistonArea = SdfParamDouble(_sdf, "piston_area", 0.0127);
  this->dataPtr->deadVolume = SdfParamDouble(_sdf, "dead_volume", 0.0266);
  this->dataPtr->T0 = SdfParamDouble(_sdf, "T0", 283.15);
  this->dataPtr->R = SdfParamDouble(_sdf, "R_specific", 0.2968);
  this->dataPtr->c_p = SdfParamDouble(_sdf, "c_p", 1.04);
  this->dataPtr->debug_prescribed_velocity = _sdf->Get<bool>(
    "debug_prescribed_velocity", false).first;

  this->dataPtr->hysteresis = _sdf->Get<bool>("hysteresis", false).first;
  if (this->dataPtr->hysteresis) {
    this->dataPtr->n1 = SdfParamDouble(_sdf, "n1", 1.4);
    this->dataPtr->n2 = SdfParamDouble(_sdf, "n2", 1.4);
    this->dataPtr->P1 = SdfParamDouble(_sdf, "P1", 410240);
    this->dataPtr->P2 = SdfParamDouble(_sdf, "P2", 410240);
    this->dataPtr->x1 = SdfParamDouble(_sdf, "x1", 0.9921);
    this->dataPtr->x2 = SdfParamDouble(_sdf, "x2", 0.9921);

    this->dataPtr->V1 = this->dataPtr->deadVolume + \
      this->dataPtr->x1 * this->dataPtr->pistonArea;
    ignwarn << "V1: " << this->dataPtr->V1 << std::endl;
    this->dataPtr->V2 = this->dataPtr->deadVolume + \
      this->dataPtr->x2 * this->dataPtr->pistonArea;
    ignwarn << "V2: " << this->dataPtr->V2 << std::endl;

    this->dataPtr->c = this->dataPtr->P1 * this->dataPtr->V1 / this->dataPtr->T0;
    ignwarn << "c: " << this->dataPtr->c << std::endl;
    this->dataPtr->mass = this->dataPtr->c / this->dataPtr->R;
    ignwarn << "mass: " << this->dataPtr->mass << std::endl;
  } else {
    this->dataPtr->n = SdfParamDouble(_sdf, "n", 1.4);
    this->dataPtr->P0 = SdfParamDouble(_sdf, "P0", 410240);
    this->dataPtr->x0 = SdfParamDouble(_sdf, "x0", 0.9921);
    this->dataPtr->V0 = this->dataPtr->deadVolume + \
      this->dataPtr->x0 * this->dataPtr->pistonArea;

    ignwarn << "V0: " << this->dataPtr->V0 << std::endl;
    this->dataPtr->c = this->dataPtr->P0 * this->dataPtr->V0 / this->dataPtr->T0;
    ignwarn << "c: " << this->dataPtr->c << std::endl;
    this->dataPtr->mass = this->dataPtr->c / this->dataPtr->R;
    ignwarn << "mass: " << this->dataPtr->mass << std::endl;
  }

  this->dataPtr->model = ignition::gazebo::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    ignerr << "PolytropicPneumaticSpring plugin should be attached to a model entity. " <<
      "Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF
  auto jointName = _sdf->Get<std::string>("JointName");
  if (jointName.empty()) {
    ignerr << "PolytropicPneumaticSpring found an empty jointName parameter. " <<
      "Failed to initialize.";
    return;
  }

  this->dataPtr->jointEntity = this->dataPtr->model.JointByName(
    _ecm,
    jointName);
  if (this->dataPtr->jointEntity == ignition::gazebo::kNullEntity) {
    ignerr << "Joint with name[" << jointName << "] not found. " <<
      "The PolytropicPneumaticSpring may not influence this joint.\n";
    return;
  }

  std::string force_topic = std::string("/force_") + this->dataPtr->name;
  force_pub = node.Advertise<ignition::msgs::Double>(force_topic);
  if (!force_pub) {
    ignerr << "Error advertising topic [" << force_topic << "]" << std::endl;
    return;
  }

  std::string pressure_topic = std::string("/pressure_") + this->dataPtr->name;
  pressure_pub = node.Advertise<ignition::msgs::Double>(pressure_topic);
  if (!pressure_pub) {
    ignerr << "Error advertising topic [" << pressure_topic << "]" << std::endl;
    return;
  }

  std::string volume_topic = std::string("/volume_") + this->dataPtr->name;
  volume_pub = node.Advertise<ignition::msgs::Double>(volume_topic);
  if (!volume_pub) {
    ignerr << "Error advertising topic [" << volume_topic << "]" << std::endl;
    return;
  }

  std::string temperature_topic = std::string("/temperature_") + this->dataPtr->name;
  temperature_pub = node.Advertise<ignition::msgs::Double>(temperature_topic);
  if (!temperature_pub) {
    ignerr << "Error advertising topic [" << temperature_topic << "]" << std::endl;
    return;
  }

  std::string heat_rate_topic = std::string("/heat_rate_") + this->dataPtr->name;
  heat_rate_pub = node.Advertise<ignition::msgs::Double>(heat_rate_topic);
  if (!heat_rate_pub) {
    ignerr << "Error advertising topic [" << heat_rate_topic << "]" << std::endl;
    return;
  }
}

//////////////////////////////////////////////////
void PolytropicPneumaticSpring::PreUpdate(
  const ignition::gazebo::UpdateInfo & _info,
  ignition::gazebo::EntityComponentManager & _ecm)
{
  IGN_PROFILE("PolytropicPneumaticSpring::PreUpdate");

  // If the joint hasn't been identified yet, the plugin is disabled
  if (this->dataPtr->jointEntity == ignition::gazebo::kNullEntity) {
    return;
  }

  // TODO(anyone): Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero()) {
    ignwarn << "Detected jump back in time [" <<
      std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count() <<
      "s]. System may not work properly." << std::endl;
  }

  // Nothing left to do if paused.
  if (_info.paused) {
    return;
  }


  // Create joint position component if one doesn't exist
  auto jointPosComp =
    _ecm.Component<ignition::gazebo::components::JointPosition>(this->dataPtr->jointEntity);
  if (jointPosComp == nullptr) {
    _ecm.CreateComponent(
      this->dataPtr->jointEntity, ignition::gazebo::components::JointPosition());
  }
  // We just created the joint position component, give one iteration for the
  // physics system to update its size
  if (jointPosComp == nullptr || jointPosComp->Data().empty()) {
    return;
  }

  // Create joint velocity component if one doesn't exist
  auto jointVelComp =
    _ecm.Component<ignition::gazebo::components::JointVelocity>(this->dataPtr->jointEntity);
  if (jointVelComp == nullptr) {
    _ecm.CreateComponent(
      this->dataPtr->jointEntity, ignition::gazebo::components::JointVelocity());
  }
  // We just created the joint velocity component, give one iteration for the
  // physics system to update its size
  if (jointVelComp == nullptr || jointVelComp->Data().empty()) {
    return;
  }


  // TODO(anyone): Figure out if (0) for the index is always correct,
  // some OR code has a process of finding the index for this argument.
  double x = jointPosComp->Data().at(0);
  double v = jointVelComp->Data().at(0);
  ignwarn << "velocity: " << v << std::endl;
  if (this->dataPtr->name.compare(0, 5, std::string("upper")) == 0) {
    x = this->dataPtr->stroke - x;
    v *= -1.0;
    ignwarn << "swap direction (" << this->dataPtr->name << ")" << std::endl;
  }

  if (this->dataPtr->hysteresis) {
    ignwarn << "hysteresis (" << this->dataPtr->name << "): " <<
      this->dataPtr->hysteresis << std::endl;
    if (v >= 0.0) {
      this->dataPtr->n = this->dataPtr->n1;
      ignwarn << "polytropic index for increasing volume (" << this->dataPtr->name << "): " <<
        this->dataPtr->n << std::endl;
      this->dataPtr->V0 = this->dataPtr->V1;
      this->dataPtr->P0 = this->dataPtr->P1;
    } else {
      this->dataPtr->n = this->dataPtr->n2;
      ignwarn << "polytropic index for decreasing volume (" << this->dataPtr->name << "): " <<
        this->dataPtr->n << std::endl;
      this->dataPtr->V0 = this->dataPtr->V2;
      this->dataPtr->P0 = this->dataPtr->P2;
    }
  }

  computeForce(x, v, this->dataPtr->n);
  if (this->dataPtr->name.compare(0, 5, std::string("upper")) == 0) {
    this->dataPtr->F *= -1.0;
    ignwarn << "swap F for direction of piston (" << this->dataPtr->name << "): " <<
      this->dataPtr->F << std::endl;
  } else {
    ignwarn << "F (" << this->dataPtr->name << "):" << this->dataPtr->F << std::endl;
  }

  // TODO(anyone): use sensor to access to F (load cell?), P (pressure sensor),
  // and T (thermometer)

  ignition::msgs::Double force, pressure, volume, temperature, heat_rate;
  force.set_data(this->dataPtr->F);
  pressure.set_data(this->dataPtr->P);
  volume.set_data(this->dataPtr->V);
  temperature.set_data(this->dataPtr->T);
  heat_rate.set_data(this->dataPtr->Q_rate);

  if (!force_pub.Publish(force)) {
    ignerr << "could not publish force" << std::endl;
  }

  if (!pressure_pub.Publish(pressure)) {
    ignerr << "could not publish pressure" << std::endl;
  }

  if (!volume_pub.Publish(volume)) {
    ignerr << "could not publish volume" << std::endl;
  }

  if (!temperature_pub.Publish(temperature)) {
    ignerr << "could not publish temperature" << std::endl;
  }

  if (!heat_rate_pub.Publish(heat_rate)) {
    ignerr << "could not publish heat loss rate" << std::endl;
  }

  if (!this->dataPtr->debug_prescribed_velocity) {
    auto forceComp =
      _ecm.Component<ignition::gazebo::components::JointForceCmd>(this->dataPtr->jointEntity);
    if (forceComp == nullptr) {
      _ecm.CreateComponent(
        this->dataPtr->jointEntity,
        ignition::gazebo::components::JointForceCmd({this->dataPtr->F}));
    } else {
      forceComp->Data()[0] += this->dataPtr->F;    // Add force to existing forces.
    }
  } else {
    double period = 2.0;    // sec

    double piston_velocity = this->dataPtr->stroke * cos(
      2.0 * 3.14159265358979 *
      std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count() / period);
    auto joint_vel = _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(
      this->dataPtr->jointEntity);
    if (joint_vel == nullptr) {
      _ecm.CreateComponent(
        this->dataPtr->jointEntity,
        ignition::gazebo::components::JointVelocityCmd({piston_velocity}));
    } else {
      *joint_vel = ignition::gazebo::components::JointVelocityCmd({piston_velocity});
    }
  }
}
}  // namespace buoy_gazebo

IGNITION_ADD_PLUGIN(
  buoy_gazebo::PolytropicPneumaticSpring,
  ignition::gazebo::System,
  buoy_gazebo::PolytropicPneumaticSpring::ISystemConfigure,
  buoy_gazebo::PolytropicPneumaticSpring::ISystemPreUpdate)
