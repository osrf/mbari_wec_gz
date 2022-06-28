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

#include "NoOpController.hpp"

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>

#include <buoy_msgs/msg/pb_command_response.hpp>

#include "buoy_msgs/srv/bc_reset_command.hpp"
#include "buoy_msgs/srv/bender_command.hpp"
#include "buoy_msgs/srv/bus_command.hpp"
#include "buoy_msgs/srv/gain_command.hpp"
#include "buoy_msgs/srv/pc_batt_switch_command.hpp"
#include "buoy_msgs/srv/pc_charge_curr_lim_command.hpp"
#include "buoy_msgs/srv/pc_draw_curr_lim_command.hpp"
#include "buoy_msgs/srv/pc_std_dev_targ_command.hpp"
#include "buoy_msgs/srv/pcv_targ_max_command.hpp"
#include "buoy_msgs/srv/sc_reset_command.hpp"
#include "buoy_msgs/srv/tether_command.hpp"
#include "buoy_msgs/srv/tf_reset_command.hpp"
#include "buoy_msgs/srv/tf_set_actual_pos_command.hpp"
#include "buoy_msgs/srv/tf_set_charge_mode_command.hpp"
#include "buoy_msgs/srv/tf_set_curr_lim_command.hpp"
#include "buoy_msgs/srv/tf_set_mode_command.hpp"
#include "buoy_msgs/srv/tf_set_pos_command.hpp"
#include "buoy_msgs/srv/tf_set_state_machine_command.hpp"
#include "buoy_msgs/srv/tf_watch_dog_command.hpp"

#include <memory>
#include <string>


#define CREATE_SERVICE(type, prefix, cmd_var, service, \
  cmd_type, range_type) \
    services_->prefix##_command_handler_ = \
      [this](const std::shared_ptr<buoy_msgs::srv::type::Request> request, \
        std::shared_ptr<buoy_msgs::srv::type::Response> response) \
      { \
        RCLCPP_WARN_STREAM( \
          ros_->node_->get_logger(), \
          "[ROS 2 No-Op Control] " #type " Received [" << \
          request->cmd_var << "] -- Not Implemented"); \
\
        response->result.value = \
          handle_command_service<cmd_type, rcl_interfaces::msg::range_type>( \
            request->cmd_var, \
            services_->valid_##prefix##_range_); \
\
        if (response->result.value == response->result.BAD_INPUT) { \
          RCLCPP_WARN_STREAM( \
            ros_->node_->get_logger(), \
            "[ROS 2 No-Op Control] " #type " out of bounds -- [" << \
              services_->valid_##prefix##_range_.from_value << ", " << \
              services_->valid_##prefix##_range_.to_value << "]"); \
        } \
      }; \
    services_->prefix##_command_service_ = \
      ros_->node_->create_service<buoy_msgs::srv::type>( \
      #service, \
      services_->prefix##_command_handler_)


#define DECLARE_SERVICE(type, prefix, range_type) \
  rclcpp::Service<buoy_msgs::srv::type>::SharedPtr prefix##_command_service_{nullptr}; \
  std::function<void(std::shared_ptr<buoy_msgs::srv::type::Request>, \
    std::shared_ptr<buoy_msgs::srv::type::Response>)> prefix##_command_handler_; \
  static const rcl_interfaces::msg::range_type valid_##prefix##_range_

#define INIT_VALID_RANGE(prefix, range_type, low, high) \
  const rcl_interfaces::msg::range_type NoOpServices::valid_##prefix##_range_ = \
    rcl_interfaces::msg::range_type() \
    .set__from_value(low) \
    .set__to_value(high)


namespace buoy_gazebo
{
struct NoOpROS2
{
  rclcpp::Node::SharedPtr node_{nullptr};
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_{nullptr};
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_handler_{nullptr};
  bool use_sim_time_{true};
};

struct NoOpServices
{
  DECLARE_SERVICE(BenderCommand, bender, IntegerRange);
  DECLARE_SERVICE(BusCommand, bus, IntegerRange);
  DECLARE_SERVICE(GainCommand, gain, IntegerRange);
  DECLARE_SERVICE(PCBattSwitchCommand, pcbattswitch, IntegerRange);
  DECLARE_SERVICE(PCChargeCurrLimCommand, pcchargecurrlim, FloatingPointRange);
  DECLARE_SERVICE(PCDrawCurrLimCommand, pcdrawcurrlim, FloatingPointRange);
  DECLARE_SERVICE(PCStdDevTargCommand, pcstddevtarg, IntegerRange);
  DECLARE_SERVICE(PCVTargMaxCommand, pcvtargmax, FloatingPointRange);
  DECLARE_SERVICE(TetherCommand, tether, IntegerRange);
  DECLARE_SERVICE(TFSetActualPosCommand, tfsetactualpos, IntegerRange);
  DECLARE_SERVICE(TFSetChargeModeCommand, tfsetchargemode, IntegerRange);
  DECLARE_SERVICE(TFSetCurrLimCommand, tfsetcurrlim, IntegerRange);
  DECLARE_SERVICE(TFSetModeCommand, tfsetmode, IntegerRange);
  DECLARE_SERVICE(TFSetPosCommand, tfsetpos, IntegerRange);
  DECLARE_SERVICE(TFSetStateMachineCommand, tfsetstatemachine, IntegerRange);

  // BCResetCommand
  rclcpp::Service<buoy_msgs::srv::BCResetCommand>::SharedPtr bcreset_command_service_{nullptr};
  std::function<void(std::shared_ptr<buoy_msgs::srv::BCResetCommand::Request>,
    std::shared_ptr<buoy_msgs::srv::BCResetCommand::Response>)> bcreset_command_handler_;

  // SCResetCommand
  rclcpp::Service<buoy_msgs::srv::SCResetCommand>::SharedPtr screset_command_service_{nullptr};
  std::function<void(std::shared_ptr<buoy_msgs::srv::SCResetCommand::Request>,
    std::shared_ptr<buoy_msgs::srv::SCResetCommand::Response>)> screset_command_handler_;

  // TFResetCommand
  rclcpp::Service<buoy_msgs::srv::TFResetCommand>::SharedPtr tfreset_command_service_{nullptr};
  std::function<void(std::shared_ptr<buoy_msgs::srv::TFResetCommand::Request>,
    std::shared_ptr<buoy_msgs::srv::TFResetCommand::Response>)> tfreset_command_handler_;

  // TFWatchDogCommand
  rclcpp::Service<
      buoy_msgs::srv::TFWatchDogCommand
    >::SharedPtr tfwatchdog_command_service_{nullptr};
  std::function<void(std::shared_ptr<buoy_msgs::srv::TFWatchDogCommand::Request>,
    std::shared_ptr<buoy_msgs::srv::TFWatchDogCommand::Response>)> tfwatchdog_command_handler_;
};
INIT_VALID_RANGE(bender, IntegerRange, 0, 2);
INIT_VALID_RANGE(bus, IntegerRange, 0, 2);
INIT_VALID_RANGE(gain, IntegerRange, 0, 200);
INIT_VALID_RANGE(pcbattswitch, IntegerRange, 0, 1);
INIT_VALID_RANGE(pcchargecurrlim, FloatingPointRange, 2.0F, 12.0F);
INIT_VALID_RANGE(pcdrawcurrlim, FloatingPointRange, 2.0F, 12.0F);
INIT_VALID_RANGE(pcstddevtarg, IntegerRange, 0, 2000);
INIT_VALID_RANGE(pcvtargmax, FloatingPointRange, 320.0F, 330.0F);
INIT_VALID_RANGE(tether, IntegerRange, 0, 1);
INIT_VALID_RANGE(tfsetactualpos, IntegerRange, 0, 1);
INIT_VALID_RANGE(tfsetchargemode, IntegerRange, 0, 3);
INIT_VALID_RANGE(tfsetcurrlim, IntegerRange, 500, 5000);
INIT_VALID_RANGE(tfsetmode, IntegerRange, 0, 2);
INIT_VALID_RANGE(tfsetpos, IntegerRange, 0, 1);
INIT_VALID_RANGE(tfsetstatemachine, IntegerRange, 0, 1);


struct NoOpControllerPrivate
{
  std::thread thread_executor_spin_, thread_publish_;
  std::atomic<bool> stop_{false};

  std::unique_ptr<NoOpROS2> ros_;
  std::unique_ptr<NoOpServices> services_;

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

  template<typename T, typename U>
  int8_t handle_command_service(
    T command_value,
    const U & valid_range)
  {
    int8_t result = buoy_msgs::msg::PBCommandResponse::NOT_IMPLEMENTED;
    if (valid_range.from_value > command_value ||
      command_value > valid_range.to_value)
    {
      result = buoy_msgs::msg::PBCommandResponse::BAD_INPUT;
    }

    return result;
  }

  void setupServices()
  {
    CREATE_SERVICE(BenderCommand, bender, state, bender_command,
      uint16_t, IntegerRange);
    CREATE_SERVICE(BusCommand, bus, state, bus_command,
      uint16_t, IntegerRange);
    CREATE_SERVICE(GainCommand, gain, gain, gain_command,
      uint16_t, IntegerRange);
    CREATE_SERVICE(PCBattSwitchCommand, pcbattswitch, state, pc_batt_switch_command,
      uint16_t, IntegerRange);
    CREATE_SERVICE(PCChargeCurrLimCommand, pcchargecurrlim, charge_curr_lim, pc_charge_curr_lim_command,
      float, FloatingPointRange);
    CREATE_SERVICE(PCDrawCurrLimCommand, pcdrawcurrlim, max_batt_draw_current, pc_draw_curr_lim_command,
      float, FloatingPointRange);
    CREATE_SERVICE(PCStdDevTargCommand, pcstddevtarg, rpm_std_dev, pc_std_dev_targ_command,
      uint16_t, IntegerRange);
    CREATE_SERVICE(PCVTargMaxCommand, pcvtargmax, v_targ_max, pc_v_targ_max_command,
      float, FloatingPointRange);
    CREATE_SERVICE(TetherCommand, tether, state, tether_command,
      uint16_t, IntegerRange);
    CREATE_SERVICE(TFSetActualPosCommand, tfsetactualpos, position, tf_set_actual_pos_command,
      uint16_t, IntegerRange);
    CREATE_SERVICE(TFSetChargeModeCommand, tfsetchargemode, mode, tf_set_charge_mode_command,
      uint16_t, IntegerRange);
    CREATE_SERVICE(TFSetCurrLimCommand, tfsetcurrlim, curr_lim, tf_set_curr_lim_command,
      uint16_t, IntegerRange);
    CREATE_SERVICE(TFSetModeCommand, tfsetmode, mode, tf_set_mode_command,
      uint16_t, IntegerRange);
    CREATE_SERVICE(TFSetPosCommand, tfsetpos, position, tf_set_pos_command,
      uint16_t, IntegerRange);
    CREATE_SERVICE(TFSetStateMachineCommand, tfsetstatemachine, state,
      tf_set_state_machine_command, uint16_t, IntegerRange);

    // BCResetCommand
    services_->bcreset_command_handler_ =
      [this](const std::shared_ptr<buoy_msgs::srv::BCResetCommand::Request> request,
        std::shared_ptr<buoy_msgs::srv::BCResetCommand::Response> response)
      {
        RCLCPP_WARN_STREAM(
          ros_->node_->get_logger(),
          "[ROS 2 No-Op Control] BCResetCommand -- Not Implemented");

        response->result.value = response->result.NOT_IMPLEMENTED;
      };
    services_->bcreset_command_service_ =
      ros_->node_->create_service<buoy_msgs::srv::BCResetCommand>(
      "bc_reset_command",
      services_->bcreset_command_handler_);

    // SCResetCommand
    services_->screset_command_handler_ =
      [this](const std::shared_ptr<buoy_msgs::srv::SCResetCommand::Request> request,
        std::shared_ptr<buoy_msgs::srv::SCResetCommand::Response> response)
      {
        RCLCPP_WARN_STREAM(
          ros_->node_->get_logger(),
          "[ROS 2 No-Op Control] SCResetCommand -- Not Implemented");

        response->result.value = response->result.NOT_IMPLEMENTED;
      };
    services_->screset_command_service_ =
      ros_->node_->create_service<buoy_msgs::srv::SCResetCommand>(
      "sc_reset_command",
      services_->screset_command_handler_);

    // TFResetCommand
    services_->tfreset_command_handler_ =
      [this](const std::shared_ptr<buoy_msgs::srv::TFResetCommand::Request> request,
        std::shared_ptr<buoy_msgs::srv::TFResetCommand::Response> response)
      {
        RCLCPP_WARN_STREAM(
          ros_->node_->get_logger(),
          "[ROS 2 No-Op Control] TFResetCommand -- Not Implemented");

        response->result.value = response->result.NOT_IMPLEMENTED;
      };
    services_->tfreset_command_service_ =
      ros_->node_->create_service<buoy_msgs::srv::TFResetCommand>(
      "tf_reset_command",
      services_->tfreset_command_handler_);

    // TFWatchDogCommand
    services_->tfwatchdog_command_handler_ =
      [this](const std::shared_ptr<buoy_msgs::srv::TFWatchDogCommand::Request> request,
        std::shared_ptr<buoy_msgs::srv::TFWatchDogCommand::Response> response)
      {
        RCLCPP_WARN_STREAM(
          ros_->node_->get_logger(),
          "[ROS 2 No-Op Control] TFWatchDogCommand -- Not Implemented");

        response->result.value = response->result.NOT_IMPLEMENTED;
      };
    services_->tfwatchdog_command_service_ =
      ros_->node_->create_service<buoy_msgs::srv::TFWatchDogCommand>(
      "tf_watch_dog_command",
      services_->tfwatchdog_command_handler_);
  }
};

//////////////////////////////////////////////////
NoOpController::NoOpController()
: dataPtr(std::make_unique<NoOpControllerPrivate>())
{
  this->dataPtr->ros_ = std::make_unique<NoOpROS2>();
  this->dataPtr->services_ = std::make_unique<NoOpServices>();
}

NoOpController::~NoOpController()
{
  // Stop ros2 threads
  this->dataPtr->stop_ = true;
  if (this->dataPtr->ros_->executor_) {
    this->dataPtr->ros_->executor_->cancel();
  }
  this->dataPtr->thread_executor_spin_.join();
  this->dataPtr->thread_publish_.join();
}

void NoOpController::Configure(
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

  // controller scoped name
  std::string scoped_name = ignition::gazebo::scopedName(_entity, _ecm, "/", false);

  // ROS node
  std::string ns = _sdf->Get<std::string>("namespace", scoped_name).first;
  if (ns.empty() || ns[0] != '/') {
    ns = '/' + ns;
  }
  std::string node_name = _sdf->Get<std::string>("node_name", "noop_controller").first;

  this->dataPtr->ros2_setup(node_name, ns);

  RCLCPP_INFO_STREAM(
    this->dataPtr->ros_->node_->get_logger(),
    "[ROS 2 Spring Control] Setting up controller for [" << model.Name(_ecm) << "]");

  this->dataPtr->setupServices();
}

}  // namespace buoy_gazebo

IGNITION_ADD_PLUGIN(
  buoy_gazebo::NoOpController,
  ignition::gazebo::System,
  buoy_gazebo::NoOpController::ISystemConfigure)
