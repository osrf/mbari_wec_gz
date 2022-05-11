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
#include <ignition/plugin/Register.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include <rclcpp/rclcpp.hpp>

#include <buoy_msgs/msg/sc_record.hpp>

#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "PolytropicPneumaticSpring/SpringState.hpp"


struct buoy_gazebo::SpringControllerPrivate
{
  ignition::gazebo::Entity entity_;
  ignition::gazebo::Entity jointEntity_;
  rclcpp::Node::SharedPtr rosnode_{nullptr};
  ignition::transport::Node node_;
  std::function<void(const ignition::msgs::Wrench &)> ft_cb_;
  rclcpp::Publisher<buoy_msgs::msg::SCRecord>::SharedPtr sc_pub_;
  double pub_rate_hz_{10.0};
  std::unique_ptr<rclcpp::Rate> pub_rate_;
  std::chrono::steady_clock::duration current_time_;
  buoy_msgs::msg::SCRecord sc_record_;
  std::mutex data_mutex_, next_access_mutex_, low_prio_mutex_;
  bool data_valid_{false};
  std::thread thread_executor_spin_, thread_publish_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  bool stop_{false}, paused_{true};
  int16_t seq_num{0};
};


IGNITION_ADD_PLUGIN(
  buoy_gazebo::SpringController,
  ignition::gazebo::System,
  buoy_gazebo::SpringController::ISystemConfigure,
  buoy_gazebo::SpringController::ISystemPostUpdate)

namespace buoy_gazebo
{
//////////////////////////////////////////////////
SpringController::SpringController()
: dataPtr(std::make_unique<SpringControllerPrivate>())
{
}

SpringController::~SpringController()
{
  // Stop ros2 threads
  this->dataPtr->stop_ = true;
  this->dataPtr->executor_->cancel();
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
  std::string scoped_name = ignition::gazebo::scopedName(this->dataPtr->entity_, _ecm, "/", false);

  // ROS node
  std::string ns = _sdf->Get<std::string>("namespace", scoped_name).first;
  if (ns.empty() || ns[0] != '/') {
    ns = '/' + ns;
  }

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
    std::string node_name = _sdf->Get<std::string>("node_name", "spring_controller").first;
    this->dataPtr->rosnode_ = rclcpp::Node::make_shared(node_name, ns);
  }

  this->dataPtr->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  this->dataPtr->executor_->add_node(this->dataPtr->rosnode_);
  this->dataPtr->stop_ = false;

  auto spin = [this]()
    {
      while (rclcpp::ok() && !this->dataPtr->stop_) {
        this->dataPtr->executor_->spin_once();
      }
    };
  this->dataPtr->thread_executor_spin_ = std::thread(spin);


  RCLCPP_INFO_STREAM(
    this->dataPtr->rosnode_->get_logger(),
    "[ROS 2 Spring Control] Setting up controller for [" << model.Name(_ecm));

  // Force Torque Sensor
  this->dataPtr->ft_cb_ = [this](const ignition::msgs::Wrench & _msg)
    {
      // low prio data access
      std::unique_lock low(this->dataPtr->low_prio_mutex_);
      std::unique_lock next(this->dataPtr->next_access_mutex_);
      std::unique_lock data(this->dataPtr->data_mutex_);
      next.unlock();
      this->dataPtr->sc_record_.load_cell = _msg.force().z();
      data.unlock();
    };
  if (!this->dataPtr->node_.Subscribe("/Universal_joint/force_torque", this->dataPtr->ft_cb_)) {
    ignerr << "Error subscribing to topic [" << "/Universal_joint/force_torque" << "]" << std::endl;
    return;  // TODO(anyone) abort?
  }

  // Publisher
  std::string topic = _sdf->Get<std::string>("topic", "sc_record").first;
  this->dataPtr->sc_pub_ = \
    this->dataPtr->rosnode_->create_publisher<buoy_msgs::msg::SCRecord>(topic, 10);

  this->dataPtr->pub_rate_hz_ = \
    _sdf->Get<double>("publish_rate", this->dataPtr->pub_rate_hz_).first;
  this->dataPtr->pub_rate_ = std::make_unique<rclcpp::Rate>(this->dataPtr->pub_rate_hz_);

  auto publish = [this]()
    {
      while (rclcpp::ok() && !this->dataPtr->stop_) {
        if (this->dataPtr->sc_pub_->get_subscription_count() <= 0) {continue;}
        // Only update and publish if not paused.
        if (this->dataPtr->paused_) {continue;}

        buoy_msgs::msg::SCRecord sc_record;
        // high prio data access
        std::unique_lock next(this->dataPtr->next_access_mutex_);
        std::unique_lock data(this->dataPtr->data_mutex_);
        next.unlock();
        this->dataPtr->sc_record_.seq_num = \
          this->dataPtr->seq_num++ % std::numeric_limits<int16_t>::max();
        sc_record = this->dataPtr->sc_record_;
        data.unlock();

        if (this->dataPtr->data_valid_) {this->dataPtr->sc_pub_->publish(sc_record);}

        this->dataPtr->pub_rate_->sleep();
      }
    };
  this->dataPtr->thread_publish_ = std::thread(publish);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void SpringController::PostUpdate(
  const ignition::gazebo::UpdateInfo & _info,
  const ignition::gazebo::EntityComponentManager & _ecm)
{
  const auto model = ignition::gazebo::Model(this->dataPtr->entity_);

  if (!_ecm.EntityHasComponentType(
      this->dataPtr->jointEntity_,
      buoy_gazebo::SpringStateComponent().TypeId()))
  {
    // Pneumatic Spring hasn't updated values yet
    // low prio data access
    std::unique_lock low(this->dataPtr->low_prio_mutex_);
    std::unique_lock next(this->dataPtr->next_access_mutex_);
    std::unique_lock data(this->dataPtr->data_mutex_);
    next.unlock();
    this->dataPtr->data_valid_ = false;
    data.unlock();
    return;
  }

  auto spring_state_comp = \
    _ecm.Component<buoy_gazebo::SpringStateComponent>(this->dataPtr->jointEntity_);

  // low prio data access
  std::unique_lock low(this->dataPtr->low_prio_mutex_);
  std::unique_lock next(this->dataPtr->next_access_mutex_);
  std::unique_lock data(this->dataPtr->data_mutex_);
  next.unlock();
  this->dataPtr->data_valid_ = true;

  this->dataPtr->current_time_ = _info.simTime;
  this->dataPtr->paused_ = _info.paused;
  auto sec_nsec = ignition::math::durationToSecNsec(dataPtr->current_time_);

  this->dataPtr->sc_record_.header.stamp.sec = sec_nsec.first;
  this->dataPtr->sc_record_.header.stamp.nanosec = sec_nsec.second;
  // this->dataPtr->sc_record_.load_cell = spring_state_comp->Data().load_cell;
  this->dataPtr->sc_record_.range_finder = spring_state_comp->Data().range_finder;
  this->dataPtr->sc_record_.upper_psi = spring_state_comp->Data().upper_psi;
  this->dataPtr->sc_record_.lower_psi = spring_state_comp->Data().lower_psi;
  // this->dataPtr->sc_record_.epoch = spring_state_comp->Data().epoch_;
  // this->dataPtr->sc_record_.salinity = spring_state_comp->Data().salinity_;
  // this->dataPtr->sc_record_.temperature = spring_state_comp->Data().temperature_;
  // this->dataPtr->sc_record_.status = spring_state_comp->Data().status_;
  data.unlock();
}
}  // namespace buoy_gazebo
