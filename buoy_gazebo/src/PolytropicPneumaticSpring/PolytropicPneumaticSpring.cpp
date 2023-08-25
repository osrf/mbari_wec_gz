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

#include <gz/msgs/double.pb.h>
#include <gz/msgs/time.pb.h>

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/PID.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>

#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>

#include "SpringState.hpp"
#include <LatentData/LatentData.hpp>


using namespace std::chrono_literals;

namespace buoy_gazebo
{
struct PolytropicPneumaticSpringConfig
{
  /// \brief Name of cylinder
  std::string name;

  bool is_upper{false};

  /// \brief is hysteresis present in piston travel direction
  bool hysteresis{false};

  /// \brief hysteresis velocity deadzone (m/s)
  double vel_dz_lower{-0.1}, vel_dz_upper{0.1};

  /// \brief adiabatic index (gamma) == 1.4 for diatomic gas (e.g. N2)
  static constexpr double ADIABATIC_INDEX{1.4};

  /// \brief initial piston position
  static constexpr double INITIAL_PISTON_POSITION{0.7};

  /// \brief polytropic index
  double n0{ADIABATIC_INDEX}, n1{ADIABATIC_INDEX}, n2{ADIABATIC_INDEX};

  bool is_adiabatic{false};

  /// \brief piston stroke (m)
  double stroke{2.03};

  /// \brief piston area (m^3)
  double piston_area{0.0127};

  /// \brief Piston-End Dead Volume (m^3)
  double dead_volume{0.0266};

  /// \brief measure of valve opening cross-section and duration (meter-seconds)
  double valve_absement{49e-7};

  /// \brief measure of pump opening cross-section and duration (meter-seconds)
  double pump_absement{10e-8};

  /// \brief pump differential pressure (Pa)
  double pump_pressure{1.7e+6};

  /// \brief mass flow of gas using pump (kg/s)
  double pump_mass_flow{pump_absement * pump_pressure};

  /// \brief initial Temp (K)
  double T0{283.15};

  /// \brief coef of heat transfer (1/s)
  double r{1.0};

  /// \brief Temp of environment (seawater) (K)
  double Tenv{283.15};

  /// \brief R (specific gas)
  double R{0.2968};

  /// \brief specific heat of gas under constant pressure (kJ/(kg K))
  double c_p{1.04};

  /// \brief initial Volume (m^3)
  double V0{dead_volume};

  /// \brief move piston along prescribed velocity for sysID
  bool debug_prescribed_velocity{false};

  /// \brief Joint Entity
  gz::sim::Entity jointEntity;

  /// \brief Model interface
  gz::sim::Model model{gz::sim::kNullEntity};

  bool VelMode{false};
};

struct PolytropicPneumaticSpringPrivate
{
  /// \brief polytropic index
  double n{PolytropicPneumaticSpringConfig::ADIABATIC_INDEX};

  /// \brief initial Pressure (Pa)
  double P0{410240};

  /// \brief mass of gas (kg)
  double mass{0.0};

  /// \brief c=mR(specific) (Pa*m^3)/K
  double c{0.0};

  /// \brief current Volume (m^3)
  double V{0.0266};

  /// \brief initial Volume (m^3)
  double V0{0.0266};  // takes value from config

  /// \brief current Pressure of gas (Pa)
  double P{P0};

  /// \brief current Temperature of gas (K)
  double T{283.15};

  /// \brief current Force of gas on piston (N)
  double F{0.0};

  /// \brief current rate of heat loss (Watts)
  double Q_rate{0.0};

  std::unique_ptr<const PolytropicPneumaticSpringConfig> config_{nullptr};
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

////////////////////////////////////////////////
void PolytropicPneumaticSpring::openValve(
  const int dt_nano, const double & pressure_diff,
  double & P0, const double & V0)
{
  GZ_ASSERT(V0 >= this->dataPtr->config_->dead_volume, "volume of chamber must be >= dead volume");

  const double dt_sec = dt_nano * GZ_NANO_TO_SEC;

  // want piston to drop 1 inch per second
  // so let mass flow from lower chamber to upper
  // results in initial pressure change (P0) via ideal gas law

  // P in Pascals (kg/(m*s^2)
  // multiply by absement (meter-seconds) to get mass_flow in kg/s
  const double mass_flow = this->dataPtr->config_->valve_absement * pressure_diff;
  const double delta_mass = dt_sec * mass_flow;  // kg of gas per step

  if (this->dataPtr->config_->is_upper) {
    this->dataPtr->mass += delta_mass;
  } else {
    this->dataPtr->mass -= delta_mass;
  }

  // Ideal Gas Law
  this->dataPtr->c = this->dataPtr->mass * this->dataPtr->config_->R;
  const double mRT = this->dataPtr->c * this->dataPtr->T;
  P0 = mRT / V0;
}

////////////////////////////////////////////////
void PolytropicPneumaticSpring::pumpOn(
  const int dt_nano,
  double & P0, const double & V0)
{
  GZ_ASSERT(V0 >= this->dataPtr->config_->dead_volume, "volume of chamber must be >= dead volume");

  const double dt_sec = dt_nano * GZ_NANO_TO_SEC;

  // want piston to raise 2 inches per minute
  // so pump mass flow from upper chamber to lower
  // results in initial pressure change (P0) via ideal gas law

  const double delta_mass = dt_sec * this->dataPtr->config_->pump_mass_flow;  // kg of gas per step

  if (this->dataPtr->config_->is_upper) {
    this->dataPtr->mass -= delta_mass;
  } else {
    this->dataPtr->mass += delta_mass;
  }

  // Ideal Gas Law
  this->dataPtr->c = this->dataPtr->mass * this->dataPtr->config_->R;
  const double mRT = this->dataPtr->c * this->dataPtr->T;
  P0 = mRT / V0;
}

void PolytropicPneumaticSpring::computeLawOfCoolingForce(const double & x, const int & dt_nano)
{
  // geometry: V = V_dead + A*x
  this->dataPtr->V = this->dataPtr->config_->dead_volume + x * this->dataPtr->config_->piston_area;

  // Newton's Law of Cooling (non-dimensionalized):
  // Tdot = r*(T_env - T(t)) -> T[n] = dt*r*(Tenv - T[n-1]) + T[n-1] (using forward difference)
  const double dt_sec = dt_nano * GZ_NANO_TO_SEC;
  const double dT =
    dt_sec * this->dataPtr->config_->r * (this->dataPtr->config_->Tenv - this->dataPtr->T);
  this->dataPtr->T += dT;

  // TODO(andermi) find Qdot (rate of heat transfer) from h, A, dT (Qdot = h*A*dT)
  // Get chamber surface area from CAD... not true cylinder
  // Also, since the chambers wrap around, h (heat transfer constant) is not quite water/steel/gas
  const double radius = 0.045;
  const double A = (2.0 * this->dataPtr->config_->piston_area) + 2.0*GZ_PI*radius*x;
  const double h = 11.3;  // (W/(m^2*K)) -- Water<->Mild Steel<->Gas
  this->dataPtr->Q_rate = h * A * dT;

  // Ideal Gas Law: P = (m*R)*T/V
  this->dataPtr->P = this->dataPtr->c * this->dataPtr->T / this->dataPtr->V;

  // F = P*A
  this->dataPtr->F = this->dataPtr->P * this->dataPtr->config_->piston_area;
}

//////////////////////////////////////////////////
void PolytropicPneumaticSpring::computePolytropicForce(const double & x, const double & v)
{
  const double V0 = this->dataPtr->V;
  const double P0 = this->dataPtr->P;
  // geometry: V = V_dead + A*x
  this->dataPtr->V = this->dataPtr->config_->dead_volume + x * this->dataPtr->config_->piston_area;
  // polytropic relationship: P = P0*(V0/V)^n
  this->dataPtr->P = P0 * pow(V0 / this->dataPtr->V, this->dataPtr->n);
  // Ideal Gas Law: T = P*V/(m*R)
  this->dataPtr->T = this->dataPtr->P * this->dataPtr->V / this->dataPtr->c;

  // no heat loss if adiabatic
  static const double cp_R = this->dataPtr->config_->c_p / this->dataPtr->config_->R;
  if (!this->dataPtr->config_->is_adiabatic) {
    // Rodrigues, M. J. (June 5, 2014). Heat Transfer During the Piston-Cylinder Expansion of a Gas
    // (Master's thesis, Oregon State University).
    // Retrieved from https://ir.library.oregonstate.edu/downloads/ww72bf399
    // heat loss rate for polytropic ideal gas:
    // dQ/dt = (1 - n/gamma)*(c_p/R)*P*A*dx/dt
    // TODO(andermi) get chamber surface area from CAD... not a true cylinder
    const double r = 0.045;
    const double A = (2.0 * this->dataPtr->config_->piston_area) + 2.0*GZ_PI*r*x;
    this->dataPtr->Q_rate =
      (1.0 - this->dataPtr->n / PolytropicPneumaticSpringConfig::ADIABATIC_INDEX) * cp_R *
      this->dataPtr->P * A * v;
  }

  // F = P*A
  this->dataPtr->F = this->dataPtr->P * this->dataPtr->config_->piston_area;
}

//////////////////////////////////////////////////
void PolytropicPneumaticSpring::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & /*_eventMgr*/)
{
  PolytropicPneumaticSpringConfig config;

  config.name = _sdf->Get<std::string>("chamber", "upper_adiabatic").first;
  gzdbg << "name: " << config.name << std::endl;

  config.is_upper = _sdf->Get<bool>("is_upper");
  gzdbg << "is upper? " << std::boolalpha << config.is_upper << std::noboolalpha << std::endl;

  config.valve_absement = SdfParamDouble(_sdf, "valve_absement", config.valve_absement);
  config.pump_absement = SdfParamDouble(_sdf, "pump_absement", config.pump_absement);
  config.pump_pressure = SdfParamDouble(_sdf, "pump_pressure", config.pump_pressure);
  // pump pressure in Pascals (kg/(m*s^2)
  // multiply by absement (meter-seconds) to get mass_flow in kg/s
  config.pump_mass_flow = config.pump_absement * config.pump_pressure;

  config.stroke = SdfParamDouble(_sdf, "stroke", config.stroke);
  config.piston_area = SdfParamDouble(_sdf, "piston_area", config.piston_area);
  config.dead_volume = SdfParamDouble(_sdf, "dead_volume", config.dead_volume);
  config.R = SdfParamDouble(_sdf, "R_specific", config.R);
  config.c_p = SdfParamDouble(_sdf, "c_p", config.c_p);
  config.debug_prescribed_velocity = _sdf->Get<bool>(
    "debug_prescribed_velocity", false).first;
  config.VelMode = _sdf->Get<bool>(
    "VelMode", false).first;
  config.T0 = SdfParamDouble(_sdf, "T0", config.T0);
  config.r = SdfParamDouble(_sdf, "r", config.r);

  config.hysteresis = _sdf->Get<bool>("hysteresis", false).first;
  config.vel_dz_lower = SdfParamDouble(_sdf, "velocity_deadzone_lower", config.vel_dz_lower);
  config.vel_dz_upper = SdfParamDouble(_sdf, "velocity_deadzone_upper", config.vel_dz_upper);
  if (config.hysteresis) {
    config.n1 = SdfParamDouble(_sdf, "n1", PolytropicPneumaticSpringConfig::ADIABATIC_INDEX);
    config.n2 = SdfParamDouble(_sdf, "n2", PolytropicPneumaticSpringConfig::ADIABATIC_INDEX);
    if ((fabs(config.n1 - PolytropicPneumaticSpringConfig::ADIABATIC_INDEX) < 1.0e-7) &&
      (fabs(config.n2 - PolytropicPneumaticSpringConfig::ADIABATIC_INDEX) < 1.0e-7))
    {
      config.is_adiabatic = true;
    } else {
      config.is_adiabatic = false;
    }
    this->dataPtr->n = config.n1;
  } else {  // no hysteresis
    config.n0 = SdfParamDouble(_sdf, "n", PolytropicPneumaticSpringConfig::ADIABATIC_INDEX);
    if (fabs(config.n0 - PolytropicPneumaticSpringConfig::ADIABATIC_INDEX) < 1.0e-7) {
      config.is_adiabatic = true;
    } else {
      config.is_adiabatic = false;
    }
    this->dataPtr->n = config.n0;
  }

  this->dataPtr->P0 = SdfParamDouble(_sdf, "P0", this->dataPtr->P0);
  config.V0 = SdfParamDouble(_sdf, "V0", config.V0);
  this->dataPtr->V0 = config.V0;

  gzdbg << "V0: " << config.V0 << std::endl;
  this->dataPtr->c = this->dataPtr->P0 * config.V0 / config.T0;  // m*R_specific
  gzdbg << "c: " << this->dataPtr->c << std::endl;
  this->dataPtr->mass = this->dataPtr->c / config.R;
  gzdbg << "mass: " << this->dataPtr->mass << std::endl;

  this->dataPtr->V = this->dataPtr->V0;
  this->dataPtr->P = this->dataPtr->P0;
  // Ideal Gas Law: T = P*V/(m*R_specific)
  this->dataPtr->T = this->dataPtr->P * this->dataPtr->V / this->dataPtr->c;

  config.model = gz::sim::Model(_entity);
  if (!config.model.Valid(_ecm)) {
    gzerr << "PolytropicPneumaticSpring plugin should be attached to a model entity. " <<
      "Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF
  auto jointName = _sdf->Get<std::string>("JointName");
  if (jointName.empty()) {
    gzerr << "PolytropicPneumaticSpring found an empty jointName parameter. " <<
      "Failed to initialize.";
    return;
  }

  config.jointEntity = config.model.JointByName(
    _ecm,
    jointName);
  if (config.jointEntity == gz::sim::kNullEntity) {
    gzerr << "Joint with name[" << jointName << "] not found. " <<
      "The PolytropicPneumaticSpring may not influence this joint.\n";
    return;
  }

  // set initial piston position
  if (config.is_upper) {
    double initial_piston_position =
      SdfParamDouble(
      _sdf,
      "initial_piston_position",
      PolytropicPneumaticSpringConfig::INITIAL_PISTON_POSITION);
    gz::sim::Joint joint(config.jointEntity);
    if (joint.Valid(_ecm)) {
      std::vector<double> pos{config.stroke - initial_piston_position};
      joint.ResetPosition(_ecm, pos);
    }
  }

  this->dataPtr->config_ = std::make_unique<const PolytropicPneumaticSpringConfig>(config);

  std::string force_topic = std::string("/force_") + this->dataPtr->config_->name;
  force_pub =
    node.Advertise<gz::msgs::Double>(
    gz::transport::TopicUtils::AsValidTopic(
      force_topic));
  if (!force_pub) {
    gzerr << "Error advertising topic [" << force_topic << "]" << std::endl;
    return;
  }

  std::string pressure_topic = std::string("/pressure_") + this->dataPtr->config_->name;
  pressure_pub =
    node.Advertise<gz::msgs::Double>(
    gz::transport::TopicUtils::AsValidTopic(
      pressure_topic));
  if (!pressure_pub) {
    gzerr << "Error advertising topic [" << pressure_topic << "]" << std::endl;
    return;
  }

  std::string volume_topic = std::string("/volume_") + this->dataPtr->config_->name;
  volume_pub =
    node.Advertise<gz::msgs::Double>(
    gz::transport::TopicUtils::AsValidTopic(
      volume_topic));
  if (!volume_pub) {
    gzerr << "Error advertising topic [" << volume_topic << "]" << std::endl;
    return;
  }

  std::string temperature_topic = std::string("/temperature_") + this->dataPtr->config_->name;
  temperature_pub =
    node.Advertise<gz::msgs::Double>(
    gz::transport::TopicUtils::AsValidTopic(
      temperature_topic));
  if (!temperature_pub) {
    gzerr << "Error advertising topic [" << temperature_topic << "]" << std::endl;
    return;
  }

  std::string heat_rate_topic = std::string("/heat_rate_") + this->dataPtr->config_->name;
  heat_rate_pub =
    node.Advertise<gz::msgs::Double>(
    gz::transport::TopicUtils::AsValidTopic(
      heat_rate_topic));
  if (!heat_rate_pub) {
    gzerr << "Error advertising topic [" << heat_rate_topic << "]" << std::endl;
    return;
  }
}

//////////////////////////////////////////////////
void PolytropicPneumaticSpring::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("PolytropicPneumaticSpring::PreUpdate");

  // If the joint hasn't been identified yet, the plugin is disabled
  if (this->dataPtr->config_->jointEntity == gz::sim::kNullEntity) {
    return;
  }

  // TODO(anyone): Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero()) {
    gzwarn << "Detected jump back in time [" <<
      std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count() <<
      "s]. System may not work properly." << std::endl;
  }

  // Nothing left to do if paused.
  if (_info.paused) {
    return;
  }

  // Create joint position component if one doesn't exist
  auto jointPosComp =
    _ecm.Component<gz::sim::components::JointPosition>(
    this->dataPtr->config_->jointEntity);
  if (jointPosComp == nullptr) {
    _ecm.CreateComponent(
      this->dataPtr->config_->jointEntity, gz::sim::components::JointPosition());
  }
  // We just created the joint position component, give one iteration for the
  // physics system to update its size
  if (jointPosComp == nullptr || jointPosComp->Data().empty()) {
    return;
  }

  // Create joint velocity component if one doesn't exist
  auto jointVelComp =
    _ecm.Component<gz::sim::components::JointVelocity>(
    this->dataPtr->config_->jointEntity);
  if (jointVelComp == nullptr) {
    _ecm.CreateComponent(
      this->dataPtr->config_->jointEntity, gz::sim::components::JointVelocity());
  }
  // We just created the joint velocity component, give one iteration for the
  // physics system to update its size
  if (jointVelComp == nullptr || jointVelComp->Data().empty()) {
    return;
  }

  buoy_gazebo::SpringState spring_state{};
  if (_ecm.EntityHasComponentType(
      this->dataPtr->config_->jointEntity,
      buoy_gazebo::components::SpringState().TypeId()))
  {
    auto spring_state_comp =
      _ecm.Component<buoy_gazebo::components::SpringState>(this->dataPtr->config_->jointEntity);

    spring_state = buoy_gazebo::SpringState(spring_state_comp->Data());
  }

  // TODO(anyone): Figure out if (0) for the index is always correct,
  // some OR code has a process of finding the index for this argument.
  double x = jointPosComp->Data().at(0);
  double v = jointVelComp->Data().at(0);

  if (this->dataPtr->config_->is_upper) {
    x = this->dataPtr->config_->stroke - x;
    v *= -1.0;
  }

  x = std::min(std::max(x, 0.0), this->dataPtr->config_->stroke);

  const int dt_nano =
    static_cast<int>(std::chrono::duration_cast<std::chrono::nanoseconds>(_info.dt).count());

  static const double PASCAL_TO_PSI = 1.450377e-4;  // PSI/Pascal
  const double pressure_diff = (spring_state.lower_psi - spring_state.upper_psi) / PASCAL_TO_PSI;

  if (spring_state.valve_command) {
    openValve(
      dt_nano, pressure_diff,
      this->dataPtr->P, this->dataPtr->V);
  } else if (spring_state.pump_command) {
    pumpOn(
      dt_nano,
      this->dataPtr->P, this->dataPtr->V);
  }

  if (this->dataPtr->config_->hysteresis) {
    if (v >= this->dataPtr->config_->vel_dz_upper) {
      this->dataPtr->n = this->dataPtr->config_->n1;
      computePolytropicForce(x, v);
    } else if (v <= this->dataPtr->config_->vel_dz_lower) {
      this->dataPtr->n = this->dataPtr->config_->n2;
      computePolytropicForce(x, v);
    } else {
      computeLawOfCoolingForce(x, dt_nano);
    }
  } else {
    if (this->dataPtr->config_->vel_dz_lower <= v &&
      v <= this->dataPtr->config_->vel_dz_upper)
    {
      computeLawOfCoolingForce(x, dt_nano);
    } else {
      computePolytropicForce(x, v);
    }
  }

  if (this->dataPtr->config_->is_upper) {
    this->dataPtr->F *= -1.0;
    this->dataPtr->Q_rate *= -1.0;
  }

  auto stampMsg = gz::sim::convert<gz::msgs::Time>(_info.simTime);

  if (this->dataPtr->config_->is_upper) {
    spring_state.range_finder = x;
    spring_state.upper_psi = PASCAL_TO_PSI * this->dataPtr->P;
  } else {
    spring_state.lower_psi = PASCAL_TO_PSI * this->dataPtr->P;
  }

  _ecm.SetComponentData<buoy_gazebo::components::SpringState>(
    this->dataPtr->config_->jointEntity,
    spring_state);

  buoy_gazebo::LatentData latent_data;
  if (_ecm.EntityHasComponentType(
      this->dataPtr->config_->model.Entity(),
      buoy_gazebo::components::LatentData().TypeId()))
  {
    auto latent_data_comp =
      _ecm.Component<buoy_gazebo::components::LatentData>(this->dataPtr->config_->model.Entity());

    latent_data = buoy_gazebo::LatentData(latent_data_comp->Data());
  }

  if (this->dataPtr->config_->is_upper) {
    latent_data.upper_spring.valid = true;
    latent_data.upper_spring.force = this->dataPtr->F;
    latent_data.upper_spring.T = this->dataPtr->T;
    latent_data.upper_spring.dQ_dt = this->dataPtr->Q_rate;
    latent_data.upper_spring.piston_position = x;
    latent_data.upper_spring.piston_velocity = v;
  } else {
    latent_data.lower_spring.valid = true;
    latent_data.lower_spring.force = this->dataPtr->F;
    latent_data.lower_spring.T = this->dataPtr->T;
    latent_data.lower_spring.dQ_dt = this->dataPtr->Q_rate;
    latent_data.lower_spring.piston_position = this->dataPtr->config_->stroke - x;
    latent_data.lower_spring.piston_velocity = -v;
  }

  _ecm.SetComponentData<buoy_gazebo::components::LatentData>(
    this->dataPtr->config_->model.Entity(),
    latent_data);

  gz::msgs::Double force;
  force.mutable_header()->mutable_stamp()->CopyFrom(stampMsg);
  force.set_data(this->dataPtr->F);

  // TODO(anyone): use sensor to access to P (pressure sensor) and T (thermometer)
  gz::msgs::Double pressure;
  pressure.mutable_header()->mutable_stamp()->CopyFrom(stampMsg);
  pressure.set_data(this->dataPtr->P);

  gz::msgs::Double volume;
  volume.mutable_header()->mutable_stamp()->CopyFrom(stampMsg);
  volume.set_data(this->dataPtr->V);

  gz::msgs::Double temperature;
  temperature.mutable_header()->mutable_stamp()->CopyFrom(stampMsg);
  temperature.set_data(this->dataPtr->T);

  gz::msgs::Double heat_rate;
  heat_rate.mutable_header()->mutable_stamp()->CopyFrom(stampMsg);
  heat_rate.set_data(this->dataPtr->Q_rate);

  gz::msgs::Double piston_velocity;
  piston_velocity.mutable_header()->mutable_stamp()->CopyFrom(stampMsg);
  piston_velocity.set_data(v);

  if (!force_pub.Publish(force)) {
    gzerr << "could not publish force" << std::endl;
  }

  if (!pressure_pub.Publish(pressure)) {
    gzerr << "could not publish pressure" << std::endl;
  }

  if (!volume_pub.Publish(volume)) {
    gzerr << "could not publish volume" << std::endl;
  }

  if (!temperature_pub.Publish(temperature)) {
    gzerr << "could not publish temperature" << std::endl;
  }

  if (!heat_rate_pub.Publish(heat_rate)) {
    gzerr << "could not publish heat loss rate" << std::endl;
  }

  static const bool debug_prescribed_velocity{this->dataPtr->config_->debug_prescribed_velocity};
  if (!debug_prescribed_velocity) {
    // Apply force if not in Velocity Mode, in which case a joint velocity is applied elsewhere
    // (likely by a test Fixture)
    if (!this->dataPtr->config_->VelMode) {
      auto forceComp =
        _ecm.Component<gz::sim::components::JointForceCmd>(
        this->dataPtr->config_->jointEntity);
      if (forceComp == nullptr) {
        _ecm.CreateComponent(
          this->dataPtr->config_->jointEntity,
          gz::sim::components::JointForceCmd({this->dataPtr->F}));
      } else {
        forceComp->Data()[0] += this->dataPtr->F;  // Add force to existing forces.
      }
    }
  } else {
    double period = 2.0;  // sec

    double piston_velocity = this->dataPtr->config_->stroke * cos(
      2.0 * GZ_PI *
      std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count() / period);
    auto joint_vel = _ecm.Component<gz::sim::components::JointVelocityCmd>(
      this->dataPtr->config_->jointEntity);
    if (joint_vel == nullptr) {
      _ecm.CreateComponent(
        this->dataPtr->config_->jointEntity,
        gz::sim::components::JointVelocityCmd({piston_velocity}));
    } else {
      *joint_vel = gz::sim::components::JointVelocityCmd({piston_velocity});
    }
  }
}
}  // namespace buoy_gazebo

GZ_ADD_PLUGIN(
  buoy_gazebo::PolytropicPneumaticSpring,
  gz::sim::System,
  buoy_gazebo::PolytropicPneumaticSpring::ISystemConfigure,
  buoy_gazebo::PolytropicPneumaticSpring::ISystemPreUpdate)
