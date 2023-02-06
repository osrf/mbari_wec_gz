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

#include <gz/msgs/wrench.pb.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <ros_gz_sim/Stopwatch.hpp>

#include <buoy_interfaces/msg/bc_record.hpp>

#include "ElectroHydraulicPTO/BatteryState.hpp"
#include "BatteryController.hpp"

using namespace std::chrono_literals;

namespace buoy_gazebo
{
struct BatteryControllerROS2
{
  rclcpp::Node::SharedPtr node_{nullptr};
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_{nullptr};
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_handler_{nullptr};
  bool use_sim_time_{true};

  rclcpp::Publisher<buoy_interfaces::msg::BCRecord>::SharedPtr bc_pub_{nullptr};
  std::unique_ptr<rclcpp::Rate> pub_rate_{nullptr};
  buoy_interfaces::msg::BCRecord bc_record_;
  double pub_rate_hz_{10.0};
};

struct BatteryControllerPrivate
{
  gz::sim::Entity entity_{gz::sim::kNullEntity};
  gz::sim::Entity jointEntity_{gz::sim::kNullEntity};
  gz::transport::Node node_;
  std::function<void(const gz::msgs::Wrench &)> ft_cb_;
  std::chrono::steady_clock::duration current_time_;

  std::mutex data_mutex_, next_access_mutex_, low_prio_mutex_;
  std::atomic<bool> battery_data_valid_{false}, load_cell_data_valid_{false};

  std::thread thread_executor_spin_, thread_publish_;
  std::atomic<bool> stop_{false}, paused_{true};

  int16_t seq_num{0};

  std::unique_ptr<BatteryControllerROS2> ros_;

  bool data_valid() const
  {
    return battery_data_valid_ && load_cell_data_valid_;
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
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(10.0F).set__to_value(50.0F).set__step(1.0F);
    descriptor.floating_point_range = {range};
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
            double rate_hz = parameter.as_double();
            RCLCPP_INFO_STREAM(
              ros_->node_->get_logger(),
              "[ROS 2 Battery Control] setting publish_rate: " << rate_hz);

            if (rate_hz < 10.0 || rate_hz > 50.0) {
              RCLCPP_WARN_STREAM(
                ros_->node_->get_logger(),
                "[ROS 2 Battery Control] publish_rate out of bounds -- clipped between [10,50]");
            }

            ros_->pub_rate_hz_ = std::min(std::max(rate_hz, 10.0), 50.0);

            // low prio data access
            std::unique_lock low(low_prio_mutex_);
            std::unique_lock next(next_access_mutex_);
            std::unique_lock data(data_mutex_);
            next.unlock();
            ros_->pub_rate_ = std::make_unique<rclcpp::Rate>(ros_->pub_rate_hz_);
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
        rclcpp::Rate rate(50.0);
        while (rclcpp::ok() && !stop_) {
          ros_->executor_->spin_once();
          rate.sleep();
        }
      };
    thread_executor_spin_ = std::thread(spin);
  }
};

/////////////////////////////////////////////////////////////////////////////////////////////
void BatteryController::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("BatteryController::PreUpdate");

  this->dataPtr->paused_ = _info.paused;
  this->dataPtr->current_time_ = _info.simTime;

  if (_info.paused) {
    return;
  }

  buoy_gazebo::BatteryState battery_state;
  if (_ecm.EntityHasComponentType(
      this->dataPtr->jointEntity_,
      buoy_gazebo::components::BatteryState().TypeId()))
  {
    auto battery_state_comp =
      _ecm.Component<buoy_gazebo::components::BatteryState>(this->dataPtr->jointEntity_);

    battery_state = buoy_gazebo::BatteryState(battery_state_comp->Data());
  } else {
    return;
  }

  _ecm.SetComponentData<buoy_gazebo::components::BatteryState>(
    this->dataPtr->jointEntity_,
    battery_state);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void BatteryController::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("BatteryController::PostUpdate");

  this->dataPtr->paused_ = _info.paused;
  this->dataPtr->current_time_ = _info.simTime;

  if (_info.paused) {
    return;
  }

  if (!_ecm.EntityHasComponentType(
      this->dataPtr->jointEntity_,
      buoy_gazebo::components::BatteryState().TypeId()))
  {
    // Values haven't been updated yet
    this->dataPtr->battery_data_valid_ = false;
    return;
  }

  auto battery_state_comp =
    _ecm.Component<buoy_gazebo::components::BatteryState>(this->dataPtr->jointEntity_);

  // low prio data access
  std::unique_lock low(this->dataPtr->low_prio_mutex_);
  std::unique_lock next(this->dataPtr->next_access_mutex_);
  std::unique_lock data(this->dataPtr->data_mutex_);
  next.unlock();

  auto sec_nsec = gz::math::durationToSecNsec(this->dataPtr->current_time_);

  this->dataPtr->ros_->bc_record_.header.stamp.sec = sec_nsec.first;
  this->dataPtr->ros_->bc_record_.header.stamp.nanosec = sec_nsec.second;
  this->dataPtr->ros_->bc_record_.voltage = battery_state_comp->Data().voltage;

  //  Currently existing on the physical buoy but won't change in simulation
  //  this->dataPtr->ros_->bc_record_.ips = battery_state_comp->Data().ips;
  //  this->dataPtr->ros_->bc_record_.vbalance = battery_state_comp->Data().vbalance;
  //  this->dataPtr->ros_->bc_record_.vstopcharge = battery_state_comp->Data().vstopcharge;
  //  this->dataPtr->ros_->bc_record_.gfault = battery_state_comp->Data().gfault;
  //  this->dataPtr->ros_->bc_record_.hydrogen = battery_state_comp->Data().hydrogen;

  this->dataPtr->ros_->bc_record_.status = battery_state_comp->Data().status;

  this->dataPtr->battery_data_valid_ = true;
  data.unlock();
}
}  // namespace buoy_gazebo

GZ_ADD_PLUGIN(
  buoy_gazebo::BatteryController,
  gz::sim::System,
  buoy_gazebo::BatteryController::ISystemConfigure,
  buoy_gazebo::BatteryController::ISystemPreUpdate,
  buoy_gazebo::BatteryController::ISystemPostUpdate)
