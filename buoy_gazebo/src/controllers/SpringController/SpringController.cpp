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

#include "SpringController.hpp"

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/msgs/wrench.pb.h>
#include <ignition/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <buoy_msgs/msg/sc_record.hpp>
#include <buoy_msgs/srv/valve_command.hpp>
#include <buoy_msgs/srv/pump_command.hpp>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "PolytropicPneumaticSpring/SpringState.hpp"

using namespace std::chrono_literals;

namespace buoy_gazebo
{
struct SpringControllerROS2
{
  rclcpp::Node::SharedPtr node_{nullptr};
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  rclcpp::Publisher<buoy_msgs::msg::SCRecord>::SharedPtr sc_pub_;
  std::unique_ptr<rclcpp::Rate> pub_rate_;
  buoy_msgs::msg::SCRecord sc_record_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_handler_;
};

struct SpringControllerServices
{
  rclcpp::Service<buoy_msgs::srv::ValveCommand>::SharedPtr valve_command_service_;
  std::function<void(std::shared_ptr<buoy_msgs::srv::ValveCommand::Request>,
    std::shared_ptr<buoy_msgs::srv::ValveCommand::Response>)> valve_command_handler_;
  rclcpp::Service<buoy_msgs::srv::PumpCommand>::SharedPtr pump_command_service_;
  std::function<void(std::shared_ptr<buoy_msgs::srv::PumpCommand::Request>,
    std::shared_ptr<buoy_msgs::srv::PumpCommand::Response>)> pump_command_handler_;
  std::mutex command_mutex_;
};

struct SpringControllerPrivate
{
  ignition::gazebo::Entity entity_;
  ignition::gazebo::Entity jointEntity_;
  ignition::transport::Node node_;
  std::function<void(const ignition::msgs::Wrench &)> ft_cb_;
  std::chrono::steady_clock::duration current_time_;

  std::mutex data_mutex_, next_access_mutex_, low_prio_mutex_;
  std::atomic<bool> spring_data_valid_{false}, load_cell_data_valid_{false};

  std::atomic<bool> valve_command_{false}, pump_command_{false};
  std::atomic<bool> has_new_command_{false}, reset_watch_{false};
  ignition::math::clock::duration command_duration_{0s};

  std::thread thread_executor_spin_, thread_publish_;
  std::atomic<bool> stop_{false}, paused_{true};

  int16_t seq_num{0};
  double pub_rate_hz_{10.0};

  std::unique_ptr<SpringControllerROS2> ros_;
  std::unique_ptr<SpringControllerServices> services_;

  bool data_valid() const
  {
    return spring_data_valid_ && load_cell_data_valid_;
  }

  void ros2_setup(const std::string & node_name, const std::string & ns)
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    ros_->node_ = rclcpp::Node::make_shared(node_name, ns);

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(10.0F).set__to_value(50.0F).set__step(1.0F);
    descriptor.floating_point_range = {range};
    ros_->node_->declare_parameter("publish_rate", pub_rate_hz_, descriptor);

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
            double rate_hz = parameter.as_double();
            RCLCPP_INFO_STREAM(
              ros_->node_->get_logger(),
              "[ROS 2 Spring Control] setting publish_rate: " << rate_hz);
            if (rate_hz < 10.0 || rate_hz > 50.0) {
              RCLCPP_WARN_STREAM(
                ros_->node_->get_logger(),
                "[ROS 2 Spring Control] publish_rate out of bounds -- clipped between [10,50]");
            }
            pub_rate_hz_ = std::min(std::max(rate_hz, 10.0), 50.0);
            // low prio data access
            std::unique_lock low(low_prio_mutex_);
            std::unique_lock next(next_access_mutex_);
            std::unique_lock data(data_mutex_);
            next.unlock();
            ros_->pub_rate_ = std::make_unique<rclcpp::Rate>(pub_rate_hz_);
            data.unlock();
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

  void setup_services()
  {
    // ValveCommand
    services_->valve_command_handler_ = \
      [this](const std::shared_ptr<buoy_msgs::srv::ValveCommand::Request> request,
        std::shared_ptr<buoy_msgs::srv::ValveCommand::Response> response)
      {
        RCLCPP_INFO_STREAM(
          ros_->node_->get_logger(),
          "[ROS 2 Spring Control] ValveCommand Received (" << request->duration_sec << "s)");
        std::unique_lock lock(services_->command_mutex_);
        // if already running pump, don't allow valve
        // unless for some reason we need to turn valve off (shouldn't get in that state)
        if (pump_command_) {
          if (request->duration_sec != request->OFF) {
            response->result.value = response->result.BUSY;
            RCLCPP_ERROR_STREAM(
              ros_->node_->get_logger(),
              "[ROS 2 Spring Control] ValveCommand cannot process" << \
                " while pump command is running...");
            return;
          }
        }

        if (request->duration_sec == request->OFF) {
          command_duration_ = 0s;
          valve_command_ = false;
          has_new_command_ = true;
        } else {
          uint16_t duration_sec{request->duration_sec};
          if (duration_sec > 20U) {
            duration_sec = std::min(
              std::max(duration_sec, static_cast<uint16_t>(1U)),
              static_cast<uint16_t>(20U));
            response->result.value = response->result.BAD_INPUT;
            RCLCPP_WARN_STREAM(
              ros_->node_->get_logger(),
              "[ROS 2 Spring Control] ValveCommand out of bounds -- clipped to 20s");
          }
          command_duration_ = ignition::math::clock::duration(std::chrono::seconds(duration_sec));
          valve_command_ = true;
          has_new_command_ = true;
        }
      };
    services_->valve_command_service_ = \
      ros_->node_->create_service<buoy_msgs::srv::ValveCommand>(
      "valve_command",
      services_->valve_command_handler_);

    // PumpCommand
    services_->pump_command_handler_ = \
      [this](const std::shared_ptr<buoy_msgs::srv::PumpCommand::Request> request,
        std::shared_ptr<buoy_msgs::srv::PumpCommand::Response> response)
      {
        RCLCPP_INFO_STREAM(
          ros_->node_->get_logger(),
          "[ROS 2 Spring Control] PumpCommand Received (" << request->duration_sec << "s)");
        std::unique_lock lock(services_->command_mutex_);
        // if already running valve, don't allow pump
        // unless for some reason we need to turn pump off (shouldn't get in that state)
        if (valve_command_) {
          if (request->duration_sec != request->OFF) {
            response->result.value = response->result.BUSY;
            RCLCPP_ERROR_STREAM(
              ros_->node_->get_logger(),
              "[ROS 2 Spring Control] PumpCommand cannot process" << \
                " while valve command is running...");
            return;
          }
        }

        if (request->duration_sec == request->OFF) {
          command_duration_ = 0s;
          pump_command_ = false;
          has_new_command_ = true;
        } else {
          uint16_t duration_sec{request->duration_sec};
          if (duration_sec > 20U) {
            duration_sec = std::min(
              std::max(duration_sec, static_cast<uint16_t>(1U)),
              static_cast<uint16_t>(20U));
            response->result.value = response->result.BAD_INPUT;
            RCLCPP_WARN_STREAM(
              ros_->node_->get_logger(),
              "[ROS 2 Spring Control] PumpCommand out of bounds -- clipped to 20s");
          }
          command_duration_ = ignition::math::clock::duration(std::chrono::seconds(duration_sec));
          pump_command_ = true;
          has_new_command_ = true;
        }
      };
    services_->pump_command_service_ = \
      ros_->node_->create_service<buoy_msgs::srv::PumpCommand>(
      "pump_command",
      services_->pump_command_handler_);
  }
};

//////////////////////////////////////////////////
SpringController::SpringController()
: dataPtr(std::make_unique<SpringControllerPrivate>())
{
  this->dataPtr->ros_ = std::make_unique<SpringControllerROS2>();
  this->dataPtr->services_ = std::make_unique<SpringControllerServices>();
}

SpringController::~SpringController()
{
  // Stop ros2 threads
  this->dataPtr->stop_ = true;
  if (this->dataPtr->ros_->executor_) {
    this->dataPtr->ros_->executor_->cancel();
  }
  this->dataPtr->thread_executor_spin_.join();
  this->dataPtr->thread_publish_.join();
}

void SpringController::Configure(
  const ignition::gazebo::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  ignition::gazebo::EntityComponentManager & _ecm,
  ignition::gazebo::EventManager &)
{
  // Make sure the controller is attached to a valid model
  auto model = ignition::gazebo::Model(_entity);
  if (!model.Valid(_ecm)) {
    ignerr << "[ROS 2 Spring Control] Failed to initialize because [" << \
      model.Name(_ecm) << "] is not a model." << std::endl << \
      "Please make sure that ROS 2 Spring Control is attached to a valid model." << std::endl;
    return;
  }

  this->dataPtr->entity_ = _entity;

  // Get params from SDF
  auto jointName = _sdf->Get<std::string>("JointName");
  if (jointName.empty()) {
    ignerr << "SpringController found an empty JointName parameter. " <<
      "Failed to initialize.";
    return;
  }

  this->dataPtr->jointEntity_ = model.JointByName(_ecm, jointName);
  if (this->dataPtr->jointEntity_ == ignition::gazebo::kNullEntity) {
    ignerr << "Joint with name[" << jointName << "] not found. " <<
      "The SpringController may not influence this joint.\n";
    return;
  }

  // controller scoped name
  std::string scoped_name = ignition::gazebo::scopedName(_entity, _ecm, "/", false);

  // ROS node
  std::string ns = _sdf->Get<std::string>("namespace", scoped_name).first;
  if (ns.empty() || ns[0] != '/') {
    ns = '/' + ns;
  }
  std::string node_name = _sdf->Get<std::string>("node_name", "spring_controller").first;

  this->dataPtr->ros2_setup(node_name, ns);

  RCLCPP_INFO_STREAM(
    this->dataPtr->ros_->node_->get_logger(),
    "[ROS 2 Spring Control] Setting up controller for [" << model.Name(_ecm) << "]");

  // Force Torque Sensor
  this->dataPtr->ft_cb_ = [this](const ignition::msgs::Wrench & _msg)
    {
      // low prio data access
      std::unique_lock low(this->dataPtr->low_prio_mutex_);
      std::unique_lock next(this->dataPtr->next_access_mutex_);
      std::unique_lock data(this->dataPtr->data_mutex_);
      next.unlock();
      this->dataPtr->ros_->sc_record_.load_cell = _msg.force().z();
      this->dataPtr->load_cell_data_valid_ = true;
      data.unlock();
    };
  if (!this->dataPtr->node_.Subscribe("/Universal_joint/force_torque", this->dataPtr->ft_cb_)) {
    ignerr << "Error subscribing to topic [" << "/Universal_joint/force_torque" << "]" << std::endl;
    return;
  }

  // Publisher
  std::string topic = _sdf->Get<std::string>("topic", "sc_record").first;
  this->dataPtr->ros_->sc_pub_ = \
    this->dataPtr->ros_->node_->create_publisher<buoy_msgs::msg::SCRecord>(topic, 10);

  this->dataPtr->pub_rate_hz_ = \
    _sdf->Get<double>("publish_rate", this->dataPtr->pub_rate_hz_).first;

  RCLCPP_INFO_STREAM(
    this->dataPtr->ros_->node_->get_logger(),
    "[ROS 2 Spring Control] Set publish_rate param from SDF: " << this->dataPtr->pub_rate_hz_);
  this->dataPtr->ros_->node_->set_parameter(
    rclcpp::Parameter(
      "publish_rate",
      this->dataPtr->pub_rate_hz_));

  auto publish = [this]()
    {
      while (rclcpp::ok() && !this->dataPtr->stop_) {
        if (this->dataPtr->ros_->sc_pub_->get_subscription_count() <= 0) {continue;}
        // Only update and publish if not paused.
        if (this->dataPtr->paused_) {continue;}

        buoy_msgs::msg::SCRecord sc_record;
        // high prio data access
        std::unique_lock next(this->dataPtr->next_access_mutex_);
        std::unique_lock data(this->dataPtr->data_mutex_);
        next.unlock();

        if (this->dataPtr->data_valid()) {
          this->dataPtr->ros_->sc_record_.seq_num = \
            this->dataPtr->seq_num++ % std::numeric_limits<int16_t>::max();
          sc_record = this->dataPtr->ros_->sc_record_;
          data.unlock();
          this->dataPtr->ros_->sc_pub_->publish(sc_record);
        } else {
          data.unlock();
        }

        this->dataPtr->ros_->pub_rate_->sleep();
      }
    };
  this->dataPtr->thread_publish_ = std::thread(publish);

  this->dataPtr->setup_services();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void SpringController::PreUpdate(
  const ignition::gazebo::UpdateInfo & _info,
  ignition::gazebo::EntityComponentManager & _ecm)
{
  IGN_PROFILE("SpringController::PreUpdate");

  this->dataPtr->paused_ = _info.paused;
  this->dataPtr->current_time_ = _info.simTime;

  if (!_ecm.EntityHasComponentType(
      this->dataPtr->jointEntity_,
      buoy_gazebo::components::SpringState().TypeId()))
  {
    return;
  }

  auto spring_state_comp = \
    _ecm.Component<buoy_gazebo::components::SpringState>(this->dataPtr->jointEntity_);

  std::unique_lock lock(this->dataPtr->services_->command_mutex_);
  if (this->dataPtr->reset_watch_) {
    if (spring_state_comp->Data().valve_command.active()) {
      RCLCPP_INFO_STREAM(
        this->dataPtr->ros_->node_->get_logger(),
        "Valve closed after (" << \
          std::chrono::duration_cast<std::chrono::seconds>(
          spring_state_comp->Data().command_watch.ElapsedRunTime()).count() << "s)");
    } else if (spring_state_comp->Data().pump_command.active()) {
      RCLCPP_INFO_STREAM(
        this->dataPtr->ros_->node_->get_logger(),
        "Pump off after (" << \
          std::chrono::duration_cast<std::chrono::seconds>(
          spring_state_comp->Data().command_watch.ElapsedRunTime()).count() << "s)");
    }
    spring_state_comp->Data().command_watch.Reset();
    spring_state_comp->Data().command_duration = 0s;
    spring_state_comp->Data().valve_command.reset();
    spring_state_comp->Data().pump_command.reset();
    this->dataPtr->reset_watch_ = false;
  }

  if (this->dataPtr->has_new_command_) {
    if (spring_state_comp->Data().valve_command || spring_state_comp->Data().pump_command) {
      spring_state_comp->Data().valve_command = this->dataPtr->valve_command_;
      spring_state_comp->Data().pump_command = this->dataPtr->pump_command_;
      spring_state_comp->Data().command_duration = \
        this->dataPtr->command_duration_ + \
        spring_state_comp->Data().command_watch.ElapsedRunTime();
    } else {
      spring_state_comp->Data().command_duration = this->dataPtr->command_duration_;
      spring_state_comp->Data().valve_command = this->dataPtr->valve_command_;
      spring_state_comp->Data().pump_command = this->dataPtr->pump_command_;
    }

    if (spring_state_comp->Data().valve_command) {
      RCLCPP_INFO_STREAM(
        this->dataPtr->ros_->node_->get_logger(),
        "Valve open (" << \
          std::chrono::duration_cast<std::chrono::seconds>(
          spring_state_comp->Data().command_duration).count() << "s)");
    } else if (spring_state_comp->Data().pump_command) {
      RCLCPP_INFO_STREAM(
        this->dataPtr->ros_->node_->get_logger(),
        "Pump on (" << \
          std::chrono::duration_cast<std::chrono::seconds>(
          spring_state_comp->Data().command_duration).count() << "s)");
    }

    this->dataPtr->has_new_command_ = false;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void SpringController::PostUpdate(
  const ignition::gazebo::UpdateInfo & _info,
  const ignition::gazebo::EntityComponentManager & _ecm)
{
  IGN_PROFILE("SpringController::PostUpdate");

  this->dataPtr->paused_ = _info.paused;
  this->dataPtr->current_time_ = _info.simTime;

  if (!_ecm.EntityHasComponentType(
      this->dataPtr->jointEntity_,
      buoy_gazebo::components::SpringState().TypeId()))
  {
    // Pneumatic Spring hasn't updated values yet
    this->dataPtr->spring_data_valid_ = false;
    return;
  }

  auto spring_state_comp = \
    _ecm.Component<buoy_gazebo::components::SpringState>(this->dataPtr->jointEntity_);

  if (spring_state_comp->Data().valve_command.isFinished()) {
    std::unique_lock lock(this->dataPtr->services_->command_mutex_);
    this->dataPtr->valve_command_ = false;
    this->dataPtr->command_duration_ = 0s;
    this->dataPtr->reset_watch_ = true;  // can't reset the watch in post-update,
                                         // so flag to do it in pre-update
  }

  if (spring_state_comp->Data().pump_command.isFinished()) {
    std::unique_lock lock(this->dataPtr->services_->command_mutex_);
    this->dataPtr->pump_command_ = false;
    this->dataPtr->command_duration_ = 0s;
    this->dataPtr->reset_watch_ = true;  // can't reset the watch in post-update,
                                         // so flag to do it in pre-update
  }

  // low prio data access
  std::unique_lock low(this->dataPtr->low_prio_mutex_);
  std::unique_lock next(this->dataPtr->next_access_mutex_);
  std::unique_lock data(this->dataPtr->data_mutex_);
  next.unlock();

  auto sec_nsec = ignition::math::durationToSecNsec(this->dataPtr->current_time_);

  this->dataPtr->ros_->sc_record_.header.stamp.sec = sec_nsec.first;
  this->dataPtr->ros_->sc_record_.header.stamp.nanosec = sec_nsec.second;
  // this->dataPtr->sc_record_.load_cell = spring_state_comp->Data().load_cell;
  this->dataPtr->ros_->sc_record_.range_finder = spring_state_comp->Data().range_finder;
  this->dataPtr->ros_->sc_record_.upper_psi = spring_state_comp->Data().upper_psi;
  this->dataPtr->ros_->sc_record_.lower_psi = spring_state_comp->Data().lower_psi;
  // this->dataPtr->sc_record_.epoch = spring_state_comp->Data().epoch_;
  // this->dataPtr->sc_record_.salinity = spring_state_comp->Data().salinity_;
  // this->dataPtr->sc_record_.temperature = spring_state_comp->Data().temperature_;
  // this->dataPtr->sc_record_.status = spring_state_comp->Data().status_;
  this->dataPtr->spring_data_valid_ = true;
  data.unlock();
}
}  // namespace buoy_gazebo

IGNITION_ADD_PLUGIN(
  buoy_gazebo::SpringController,
  ignition::gazebo::System,
  buoy_gazebo::SpringController::ISystemConfigure,
  buoy_gazebo::SpringController::ISystemPreUpdate,
  buoy_gazebo::SpringController::ISystemPostUpdate)
