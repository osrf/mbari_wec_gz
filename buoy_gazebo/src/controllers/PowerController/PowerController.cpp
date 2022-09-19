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

#include "PowerController.hpp"

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/msgs/wrench.pb.h>
#include <ignition/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <ros_ign_gazebo/Stopwatch.hpp>

#include <buoy_interfaces/msg/pc_record.hpp>
#include <buoy_interfaces/srv/pc_wind_curr_command.hpp>
#include <buoy_interfaces/srv/pc_scale_command.hpp>
#include <buoy_interfaces/srv/pc_retract_command.hpp>
#include <buoy_interfaces/srv/pc_bias_curr_command.hpp>
#include <buoy_interfaces/msg/pb_command_response.hpp>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "ElectroHydraulicPTO/ElectroHydraulicState.hpp"


using namespace std::chrono_literals;

namespace buoy_gazebo
{
struct PowerControllerROS2
{
  rclcpp::Node::SharedPtr node_{nullptr};
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_{nullptr};
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_handler_{nullptr};
  bool use_sim_time_{true};

  rclcpp::Publisher<buoy_interfaces::msg::PCRecord>::SharedPtr pc_pub_{nullptr};
  std::unique_ptr<rclcpp::Rate> pub_rate_{nullptr};
  static const rcl_interfaces::msg::FloatingPointRange valid_pub_rate_range_;
  buoy_interfaces::msg::PCRecord pc_record_;
  double pub_rate_hz_{10.0};
};
const rcl_interfaces::msg::FloatingPointRange PowerControllerROS2::valid_pub_rate_range_ =
  rcl_interfaces::msg::FloatingPointRange()
  .set__from_value(10.0F)
  .set__to_value(50.0F);

struct PowerControllerServices
{
  // PCWindCurrCommand -- Winding Current (Torque)
  rclcpp::Service<buoy_interfaces::srv::PCWindCurrCommand>::SharedPtr torque_command_service_{
    nullptr};
  std::function<void(std::shared_ptr<buoy_interfaces::srv::PCWindCurrCommand::Request>,
    std::shared_ptr<buoy_interfaces::srv::PCWindCurrCommand::Response>)> torque_command_handler_;

  ros_ign_gazebo::Stopwatch torque_command_watch_;
  rclcpp::Duration torque_command_duration_{0, 0U};
  static const rclcpp::Duration TORQUE_COMMAND_TIMEOUT;
  static const rcl_interfaces::msg::FloatingPointRange valid_wind_curr_range_;
  double wind_curr_{0.0};
  std::atomic<bool> torque_command_{false};
  std::atomic<bool> new_torque_command_{false};

  // PCScaleCommand -- winding current scale factor
  rclcpp::Service<buoy_interfaces::srv::PCScaleCommand>::SharedPtr scale_command_service_{nullptr};
  std::function<void(std::shared_ptr<buoy_interfaces::srv::PCScaleCommand::Request>,
    std::shared_ptr<buoy_interfaces::srv::PCScaleCommand::Response>)> scale_command_handler_;

  ros_ign_gazebo::Stopwatch scale_command_watch_;
  rclcpp::Duration scale_command_duration_{0, 0U};
  static const rclcpp::Duration SCALE_COMMAND_TIMEOUT;
  static const rcl_interfaces::msg::FloatingPointRange valid_scale_range_;
  double scale_{0.0};
  std::atomic<bool> scale_command_{false};
  std::atomic<bool> new_scale_command_{false};

  // PCRetractCommand -- winding current retract factor (additional scaling in retract direction)
  rclcpp::Service<buoy_interfaces::srv::PCRetractCommand>::SharedPtr retract_command_service_{
    nullptr};
  std::function<void(std::shared_ptr<buoy_interfaces::srv::PCRetractCommand::Request>,
    std::shared_ptr<buoy_interfaces::srv::PCRetractCommand::Response>)> retract_command_handler_;

  ros_ign_gazebo::Stopwatch retract_command_watch_;
  rclcpp::Duration retract_command_duration_{0, 0U};
  static const rclcpp::Duration RETRACT_COMMAND_TIMEOUT;
  static const rcl_interfaces::msg::FloatingPointRange valid_retract_range_;
  double retract_{0.0};
  std::atomic<bool> retract_command_{false};
  std::atomic<bool> new_retract_command_{false};

  // PCBiasCurrCommand -- winding current bias offset
  rclcpp::Service<buoy_interfaces::srv::PCBiasCurrCommand>::SharedPtr bias_curr_command_service_{
    nullptr};
  std::function<void(std::shared_ptr<buoy_interfaces::srv::PCBiasCurrCommand::Request>,
    std::shared_ptr<buoy_interfaces::srv::PCBiasCurrCommand::Response>)> bias_curr_command_handler_;

  ros_ign_gazebo::Stopwatch bias_curr_command_watch_;
  rclcpp::Duration bias_curr_command_duration_{0, 0U};
  static const rclcpp::Duration BIAS_CURR_COMMAND_TIMEOUT;
  static const rcl_interfaces::msg::FloatingPointRange valid_bias_curr_range_;
  double bias_curr_{0.0};
  std::atomic<bool> bias_curr_command_{false};
  std::atomic<bool> new_bias_curr_command_{false};

  std::mutex command_mutex_;
};
const rclcpp::Duration PowerControllerServices::TORQUE_COMMAND_TIMEOUT{2, 0U};
const rcl_interfaces::msg::FloatingPointRange PowerControllerServices::valid_wind_curr_range_ =
  rcl_interfaces::msg::FloatingPointRange()
  .set__from_value(-35.0F)
  .set__to_value(35.0F);

const rclcpp::Duration PowerControllerServices::SCALE_COMMAND_TIMEOUT{300, 0U};
const rcl_interfaces::msg::FloatingPointRange PowerControllerServices::valid_scale_range_ =
  rcl_interfaces::msg::FloatingPointRange()
  .set__from_value(0.5F)
  .set__to_value(1.4F);

const rclcpp::Duration PowerControllerServices::RETRACT_COMMAND_TIMEOUT{300, 0U};
const rcl_interfaces::msg::FloatingPointRange PowerControllerServices::valid_retract_range_ =
  rcl_interfaces::msg::FloatingPointRange()
  .set__from_value(0.4F)
  .set__to_value(1.0F);

const rclcpp::Duration PowerControllerServices::BIAS_CURR_COMMAND_TIMEOUT{10, 0U};
const rcl_interfaces::msg::FloatingPointRange PowerControllerServices::valid_bias_curr_range_ =
  rcl_interfaces::msg::FloatingPointRange()
  .set__from_value(-20.0F)
  .set__to_value(20.0F);

struct PowerControllerPrivate
{
  ignition::gazebo::Entity entity_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity jointEntity_{ignition::gazebo::kNullEntity};
  ignition::transport::Node node_;
  std::chrono::steady_clock::duration current_time_;

  std::mutex data_mutex_, next_access_mutex_, low_prio_mutex_;
  std::atomic<bool> pto_data_valid_{false};

  std::thread thread_executor_spin_, thread_publish_;
  std::atomic<bool> stop_{false}, paused_{true};

  int16_t seq_num{0};

  std::unique_ptr<PowerControllerROS2> ros_;
  std::unique_ptr<PowerControllerServices> services_;

  bool data_valid() const
  {
    return pto_data_valid_;
  }

  void handle_publish_rate(const rclcpp::Parameter & parameter)
  {
    double rate_hz = parameter.as_double();
    RCLCPP_INFO_STREAM(
      ros_->node_->get_logger(),
      "[ROS 2 Power Control] setting publish_rate: " << rate_hz);

    if (PowerControllerROS2::valid_pub_rate_range_.from_value > rate_hz ||
      rate_hz > PowerControllerROS2::valid_pub_rate_range_.to_value)
    {
      RCLCPP_WARN_STREAM(
        ros_->node_->get_logger(),
        "[ROS 2 Power Control] publish_rate out of bounds -- clipped between [" <<
          PowerControllerROS2::valid_pub_rate_range_.from_value << ", " <<
          PowerControllerROS2::valid_pub_rate_range_.to_value << "] Hz");
    }

    ros_->pub_rate_hz_ = std::min(
      std::max(
        rate_hz,
        PowerControllerROS2::valid_pub_rate_range_.from_value),
      PowerControllerROS2::valid_pub_rate_range_.to_value);

    // low prio data access
    std::unique_lock low(low_prio_mutex_);
    std::unique_lock next(next_access_mutex_);
    std::unique_lock data(data_mutex_);
    next.unlock();
    ros_->pub_rate_ = std::make_unique<rclcpp::Rate>(ros_->pub_rate_hz_);
    data.unlock();
  }

  void ros2_setup(const std::string & node_name, const std::string & ns)
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    ros_->node_ = rclcpp::Node::make_shared(node_name, ns);

    ros_->node_->set_parameter(
      rclcpp::Parameter(
        "use_sim_time",
        ros_->use_sim_time_));

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.floating_point_range = {PowerControllerROS2::valid_pub_rate_range_};
    ros_->node_->declare_parameter("publish_rate", ros_->pub_rate_hz_, descriptor);

    ros_->parameter_handler_ = ros_->node_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters)
      {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & parameter : parameters) {
          if (
            parameter.get_name() == "publish_rate" &&
            parameter.get_type() == rclcpp::PARAMETER_DOUBLE)
          {
            handle_publish_rate(parameter);
          }
        }
        return result;
      });

    ros_->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    ros_->executor_->add_node(ros_->node_);
    stop_ = false;

    auto spin = [this]()
      {
        while (rclcpp::ok() && !stop_) {
          ros_->executor_->spin_once();
        }
      };
    thread_executor_spin_ = std::thread(spin);
  }

  int8_t handle_command_service(
    double command_value,
    const rcl_interfaces::msg::FloatingPointRange & valid_range,
    double & services_command_value,
    rclcpp::Duration & duration,
    const rclcpp::Duration & timeout,
    std::atomic<bool> & services_command,
    std::atomic<bool> & new_command)
  {
    int8_t result = buoy_interfaces::msg::PBCommandResponse::OK;
    std::unique_lock lock(services_->command_mutex_);
    if (valid_range.from_value > command_value ||
      command_value > valid_range.to_value)
    {
      command_value = std::min(
        std::max(
          command_value,
          valid_range.from_value),
        valid_range.to_value);

      result = buoy_interfaces::msg::PBCommandResponse::BAD_INPUT;
    }

    services_command_value = command_value;
    duration = timeout;

    services_command = true;
    new_command = true;

    return result;
  }

  void setupServices()
  {
    // PCWindCurrCommand
    services_->torque_command_watch_.SetClock(ros_->node_->get_clock());
    services_->torque_command_handler_ =
      [this](const std::shared_ptr<buoy_interfaces::srv::PCWindCurrCommand::Request> request,
        std::shared_ptr<buoy_interfaces::srv::PCWindCurrCommand::Response> response)
      {
        RCLCPP_INFO_STREAM(
          ros_->node_->get_logger(),
          "[ROS 2 Power Control] PCWindCurrCommand Received [" << request->wind_curr << " Amps]");

        response->result.value = handle_command_service(
          request->wind_curr,
          services_->valid_wind_curr_range_,
          services_->wind_curr_,
          services_->torque_command_duration_,
          PowerControllerServices::TORQUE_COMMAND_TIMEOUT,
          services_->torque_command_,
          services_->new_torque_command_);

        if (response->result.value == response->result.BAD_INPUT) {
          RCLCPP_WARN_STREAM(
            ros_->node_->get_logger(),
            "[ROS 2 Spring Control] PCWindCurrCommand out of bounds -- clipped between [" <<
              services_->valid_wind_curr_range_.from_value << ", " <<
              services_->valid_wind_curr_range_.to_value << "] Amps");
        }
      };
    services_->torque_command_service_ =
      ros_->node_->create_service<buoy_interfaces::srv::PCWindCurrCommand>(
      "pc_wind_curr_command",
      services_->torque_command_handler_);

    // PCScaleCommand
    services_->scale_command_watch_.SetClock(ros_->node_->get_clock());
    services_->scale_command_handler_ =
      [this](const std::shared_ptr<buoy_interfaces::srv::PCScaleCommand::Request> request,
        std::shared_ptr<buoy_interfaces::srv::PCScaleCommand::Response> response)
      {
        RCLCPP_INFO_STREAM(
          ros_->node_->get_logger(),
          "[ROS 2 Power Control] PCScaleCommand Received [" << request->scale << "]");

        response->result.value = handle_command_service(
          request->scale,
          services_->valid_scale_range_,
          services_->scale_,
          services_->scale_command_duration_,
          PowerControllerServices::SCALE_COMMAND_TIMEOUT,
          services_->scale_command_,
          services_->new_scale_command_);

        if (response->result.value == response->result.BAD_INPUT) {
          RCLCPP_WARN_STREAM(
            ros_->node_->get_logger(),
            "[ROS 2 Spring Control] PCScaleCommand out of bounds -- clipped between [" <<
              services_->valid_scale_range_.from_value << ", " <<
              services_->valid_scale_range_.to_value << "]");
        }
      };
    services_->scale_command_service_ =
      ros_->node_->create_service<buoy_interfaces::srv::PCScaleCommand>(
      "pc_scale_command",
      services_->scale_command_handler_);

    // PCRetractCommand
    services_->retract_command_watch_.SetClock(ros_->node_->get_clock());
    services_->retract_command_handler_ =
      [this](const std::shared_ptr<buoy_interfaces::srv::PCRetractCommand::Request> request,
        std::shared_ptr<buoy_interfaces::srv::PCRetractCommand::Response> response)
      {
        RCLCPP_INFO_STREAM(
          ros_->node_->get_logger(),
          "[ROS 2 Power Control] PCRetractCommand Received [" << request->retract << "]");

        response->result.value = handle_command_service(
          request->retract,
          services_->valid_retract_range_,
          services_->retract_,
          services_->retract_command_duration_,
          PowerControllerServices::RETRACT_COMMAND_TIMEOUT,
          services_->retract_command_,
          services_->new_retract_command_);

        if (response->result.value == response->result.BAD_INPUT) {
          RCLCPP_WARN_STREAM(
            ros_->node_->get_logger(),
            "[ROS 2 Spring Control] PCRetractCommand out of bounds -- clipped between [" <<
              services_->valid_retract_range_.from_value << ", " <<
              services_->valid_retract_range_.to_value << "]");
        }
      };
    services_->retract_command_service_ =
      ros_->node_->create_service<buoy_interfaces::srv::PCRetractCommand>(
      "pc_retract_command",
      services_->retract_command_handler_);

    // PCBiasCurrCommand
    services_->bias_curr_command_watch_.SetClock(ros_->node_->get_clock());
    services_->bias_curr_command_handler_ =
      [this](const std::shared_ptr<buoy_interfaces::srv::PCBiasCurrCommand::Request> request,
        std::shared_ptr<buoy_interfaces::srv::PCBiasCurrCommand::Response> response)
      {
        RCLCPP_INFO_STREAM(
          ros_->node_->get_logger(),
          "[ROS 2 Power Control] PCBiasCurrCommand Received [" << request->bias_curr << " Amps]");

        response->result.value = handle_command_service(
          request->bias_curr,
          services_->valid_bias_curr_range_,
          services_->bias_curr_,
          services_->bias_curr_command_duration_,
          PowerControllerServices::BIAS_CURR_COMMAND_TIMEOUT,
          services_->bias_curr_command_,
          services_->new_bias_curr_command_);

        if (response->result.value == response->result.BAD_INPUT) {
          RCLCPP_WARN_STREAM(
            ros_->node_->get_logger(),
            "[ROS 2 Spring Control] PCBiasCurrCommand out of bounds -- clipped between [" <<
              services_->valid_bias_curr_range_.from_value << ", " <<
              services_->valid_bias_curr_range_.to_value << "]");
        }
      };
    services_->bias_curr_command_service_ =
      ros_->node_->create_service<buoy_interfaces::srv::PCBiasCurrCommand>(
      "pc_bias_curr_command",
      services_->bias_curr_command_handler_);
  }

  void manageCommandTimer(
    const std::string & command_name,
    buoy_utils::CommandTriState<> & command,
    ros_ign_gazebo::Stopwatch & watch,
    const rclcpp::Duration & duration)
  {
    // override
    if (command) {
      if (!watch.Running()) {
        RCLCPP_INFO_STREAM(
          ros_->node_->get_logger(),
          "Override " << command_name << " (" <<
            duration.seconds() << "s)");

        watch.Start(true);

        // stop overriding
      } else {
        if (watch.ElapsedRunTime() >= duration) {
          watch.Stop();
          command = false;

          RCLCPP_INFO_STREAM(
            ros_->node_->get_logger(),
            "Stopped overriding " << command_name << " after (" <<
              watch.ElapsedRunTime().seconds() << "s)");
        }
      }
    }
  }

  void manageCommandTimers(ElectroHydraulicState & state)
  {
    // override winding current (torque)
    manageCommandTimer(
      "Winding Current (Torque)",
      state.torque_command,
      services_->torque_command_watch_,
      services_->torque_command_duration_);

    // override scale factor
    manageCommandTimer(
      "Scale Factor",
      state.scale_command,
      services_->scale_command_watch_,
      services_->scale_command_duration_);

    // override retract factor
    manageCommandTimer(
      "Retract Factor",
      state.retract_command,
      services_->retract_command_watch_,
      services_->retract_command_duration_);

    // override bias current
    manageCommandTimer(
      "Bias Current",
      state.bias_current_command,
      services_->bias_curr_command_watch_,
      services_->bias_curr_command_duration_);
  }

  void manageCommandState(
    const std::string & command_name,
    buoy_utils::CommandTriState<> & command,
    std::atomic<bool> & services_command,
    std::atomic<bool> & new_command,
    const double & command_value,
    ros_ign_gazebo::Stopwatch & watch,
    rclcpp::Duration & duration,
    const rclcpp::Duration & timeout)
  {
    if (command.isFinished()) {
      std::unique_lock lock(services_->command_mutex_);
      services_command = false;
      duration = rclcpp::Duration(0, 0U);
      watch.Reset();
      command.reset();
    }

    if (new_command) {
      std::unique_lock lock(services_->command_mutex_);
      if (services_command) {
        if (command) {
          command = command_value;
          duration = timeout + watch.ElapsedRunTime();

          RCLCPP_INFO_STREAM(
            ros_->node_->get_logger(),
            "Continue Override " << command_name << " (" <<
              duration.seconds() << "s)");
        } else {
          command = command_value;
        }
      } else {
        command = services_command;
      }
      new_command = false;
    }
  }

  void manageCommandStates(ElectroHydraulicState & state)
  {
    manageCommandState(
      "Winding Current (Torque)",
      state.torque_command,
      services_->torque_command_,
      services_->new_torque_command_,
      services_->wind_curr_,
      services_->torque_command_watch_,
      services_->torque_command_duration_,
      PowerControllerServices::TORQUE_COMMAND_TIMEOUT);

    manageCommandState(
      "Scale Factor",
      state.scale_command,
      services_->scale_command_,
      services_->new_scale_command_,
      services_->scale_,
      services_->scale_command_watch_,
      services_->scale_command_duration_,
      PowerControllerServices::SCALE_COMMAND_TIMEOUT);

    manageCommandState(
      "Retract Factor",
      state.retract_command,
      services_->retract_command_,
      services_->new_retract_command_,
      services_->retract_,
      services_->retract_command_watch_,
      services_->retract_command_duration_,
      PowerControllerServices::RETRACT_COMMAND_TIMEOUT);

    manageCommandState(
      "Bias Current",
      state.bias_current_command,
      services_->bias_curr_command_,
      services_->new_bias_curr_command_,
      services_->bias_curr_,
      services_->bias_curr_command_watch_,
      services_->bias_curr_command_duration_,
      PowerControllerServices::BIAS_CURR_COMMAND_TIMEOUT);
  }
};

//////////////////////////////////////////////////
PowerController::PowerController()
: dataPtr(std::make_unique<PowerControllerPrivate>())
{
  this->dataPtr->ros_ = std::make_unique<PowerControllerROS2>();
  this->dataPtr->services_ = std::make_unique<PowerControllerServices>();
}

PowerController::~PowerController()
{
  // Stop ros2 threads
  this->dataPtr->stop_ = true;
  if (this->dataPtr->ros_->executor_) {
    this->dataPtr->ros_->executor_->cancel();
  }
  this->dataPtr->thread_executor_spin_.join();
  this->dataPtr->thread_publish_.join();
}

void PowerController::Configure(
  const ignition::gazebo::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  ignition::gazebo::EntityComponentManager & _ecm,
  ignition::gazebo::EventManager &)
{
  // Make sure the controller is attached to a valid model
  auto model = ignition::gazebo::Model(_entity);
  if (!model.Valid(_ecm)) {
    ignerr << "[ROS 2 Spring Control] Failed to initialize because [" <<
      model.Name(_ecm) << "] is not a model." << std::endl <<
      "Please make sure that ROS 2 Spring Control is attached to a valid model." << std::endl;
    return;
  }

  this->dataPtr->entity_ = _entity;

  // Get params from SDF
  auto jointName = _sdf->Get<std::string>("JointName");
  if (jointName.empty()) {
    ignerr << "PowerController found an empty JointName parameter. " <<
      "Failed to initialize.";
    return;
  }

  this->dataPtr->jointEntity_ = model.JointByName(_ecm, jointName);
  if (this->dataPtr->jointEntity_ == ignition::gazebo::kNullEntity) {
    ignerr << "Joint with name[" << jointName << "] not found. " <<
      "The PowerController may not influence this joint.\n";
    return;
  }

  // controller scoped name
  std::string scoped_name = ignition::gazebo::scopedName(_entity, _ecm, "/", false);

  // ROS node
  std::string ns = _sdf->Get<std::string>("namespace", scoped_name).first;
  if (ns.empty() || ns[0] != '/') {
    ns = '/' + ns;
  }
  std::string node_name = _sdf->Get<std::string>("node_name", "power_controller").first;

  this->dataPtr->ros2_setup(node_name, ns);

  RCLCPP_INFO_STREAM(
    this->dataPtr->ros_->node_->get_logger(),
    "[ROS 2 Spring Control] Setting up controller for [" << model.Name(_ecm) << "]");

  // Publisher
  std::string topic = _sdf->Get<std::string>("topic", "power_data").first;
  this->dataPtr->ros_->pc_pub_ =
    this->dataPtr->ros_->node_->create_publisher<buoy_interfaces::msg::PCRecord>(topic, 10);

  this->dataPtr->ros_->pub_rate_hz_ =
    _sdf->Get<double>("publish_rate", this->dataPtr->ros_->pub_rate_hz_).first;

  RCLCPP_INFO_STREAM(
    this->dataPtr->ros_->node_->get_logger(),
    "[ROS 2 Power Control] Set publish_rate param from SDF: " <<
      this->dataPtr->ros_->pub_rate_hz_);

  this->dataPtr->ros_->node_->set_parameter(
    rclcpp::Parameter(
      "publish_rate",
      this->dataPtr->ros_->pub_rate_hz_));

  auto publish = [this]()
    {
      while (rclcpp::ok() && !this->dataPtr->stop_) {
        if (this->dataPtr->ros_->pc_pub_->get_subscription_count() <= 0) {continue;}

        // Only update and publish if not paused.
        if (this->dataPtr->paused_) {continue;}

        buoy_interfaces::msg::PCRecord pc_record;
        // high prio data access
        std::unique_lock next(this->dataPtr->next_access_mutex_);
        std::unique_lock data(this->dataPtr->data_mutex_);
        next.unlock();

        if (this->dataPtr->data_valid()) {
          this->dataPtr->ros_->pc_record_.seq_num =
            this->dataPtr->seq_num++ % std::numeric_limits<int16_t>::max();
          pc_record = this->dataPtr->ros_->pc_record_;
          data.unlock();

          this->dataPtr->ros_->pc_pub_->publish(pc_record);
        } else {
          data.unlock();
        }

        this->dataPtr->ros_->pub_rate_->sleep();
      }
    };
  this->dataPtr->thread_publish_ = std::thread(publish);

  this->dataPtr->setupServices();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void PowerController::PreUpdate(
  const ignition::gazebo::UpdateInfo & _info,
  ignition::gazebo::EntityComponentManager & _ecm)
{
  IGN_PROFILE("PowerController::PreUpdate");

  this->dataPtr->paused_ = _info.paused;
  this->dataPtr->current_time_ = _info.simTime;

  if (_info.paused) {
    return;
  }

  buoy_gazebo::ElectroHydraulicState pto_state;
  if (_ecm.EntityHasComponentType(
      this->dataPtr->jointEntity_,
      buoy_gazebo::components::ElectroHydraulicState().TypeId()))
  {
    auto pto_state_comp =
      _ecm.Component<buoy_gazebo::components::ElectroHydraulicState>(
      this->dataPtr->jointEntity_);

    pto_state = buoy_gazebo::ElectroHydraulicState(pto_state_comp->Data());
  } else {
    return;
  }

  this->dataPtr->manageCommandTimers(pto_state);
  this->dataPtr->manageCommandStates(pto_state);

  _ecm.SetComponentData<buoy_gazebo::components::ElectroHydraulicState>(
    this->dataPtr->jointEntity_,
    pto_state);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void PowerController::PostUpdate(
  const ignition::gazebo::UpdateInfo & _info,
  const ignition::gazebo::EntityComponentManager & _ecm)
{
  IGN_PROFILE("PowerController::PostUpdate");

  this->dataPtr->paused_ = _info.paused;
  this->dataPtr->current_time_ = _info.simTime;

  if (_info.paused) {
    return;
  }

  if (!_ecm.EntityHasComponentType(
      this->dataPtr->jointEntity_,
      buoy_gazebo::components::ElectroHydraulicState().TypeId()))
  {
    // Pneumatic Spring hasn't updated values yet
    this->dataPtr->pto_data_valid_ = false;
    return;
  }

  auto pto_state_comp =
    _ecm.Component<buoy_gazebo::components::ElectroHydraulicState>(this->dataPtr->jointEntity_);

  // low prio data access
  std::unique_lock low(this->dataPtr->low_prio_mutex_);
  std::unique_lock next(this->dataPtr->next_access_mutex_);
  std::unique_lock data(this->dataPtr->data_mutex_);
  next.unlock();

  auto sec_nsec = ignition::math::durationToSecNsec(this->dataPtr->current_time_);

  this->dataPtr->ros_->pc_record_.header.stamp.sec = sec_nsec.first;
  this->dataPtr->ros_->pc_record_.header.stamp.nanosec = sec_nsec.second;
  this->dataPtr->ros_->pc_record_.rpm = pto_state_comp->Data().rpm;
  this->dataPtr->ros_->pc_record_.voltage = pto_state_comp->Data().voltage;
  this->dataPtr->ros_->pc_record_.bcurrent = pto_state_comp->Data().bcurrent;
  this->dataPtr->ros_->pc_record_.wcurrent = pto_state_comp->Data().wcurrent;
  this->dataPtr->ros_->pc_record_.diff_press = pto_state_comp->Data().diff_press;
  this->dataPtr->ros_->pc_record_.bias_current = pto_state_comp->Data().bias_current;
  this->dataPtr->ros_->pc_record_.loaddc = pto_state_comp->Data().loaddc;
  this->dataPtr->ros_->pc_record_.scale = pto_state_comp->Data().scale;
  this->dataPtr->ros_->pc_record_.retract = pto_state_comp->Data().retract;
  this->dataPtr->ros_->pc_record_.target_a = pto_state_comp->Data().target_a;
  this->dataPtr->ros_->pc_record_.target_a = pto_state_comp->Data().target_a;

  // TODO(anyone)
  // this->dataPtr->ros_->pc_record_.sd_rpm = pto_state_comp->Data().sd_rpm;
  // this->dataPtr->ros_->pc_record_.draw_curr_limit = pto_state_comp->Data().draw_curr_limit;
  // this->dataPtr->ros_->pc_record_.torque = pto_state_comp->Data().torque;
  // this->dataPtr->ros_->pc_record_.target_v = pto_state_comp->Data().target_v;
  // this->dataPtr->ros_->pc_record_.charge_curr_limit = pto_state_comp->Data().charge_curr_limit;

  // TODO(anyone) set the bits for this
  this->dataPtr->ros_->pc_record_.status = pto_state_comp->Data().status;

  this->dataPtr->pto_data_valid_ = true;
  data.unlock();
}
}  // namespace buoy_gazebo

IGNITION_ADD_PLUGIN(
  buoy_gazebo::PowerController,
  ignition::gazebo::System,
  buoy_gazebo::PowerController::ISystemConfigure,
  buoy_gazebo::PowerController::ISystemPreUpdate,
  buoy_gazebo::PowerController::ISystemPostUpdate)
