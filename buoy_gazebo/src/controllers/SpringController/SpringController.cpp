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

#include <rclcpp/rclcpp.hpp>

#include <buoy_msgs/msg/sc_record.hpp>

#include <memory>
#include <string>
#include <vector>

#include "PolytropicPneumaticSpring/SpringState.hpp"


struct buoy_gazebo::SpringControllerPrivate
{
  ignition::gazebo::Entity entity_;
  ignition::gazebo::Entity jointEntity_;
  rclcpp::Node::SharedPtr rosnode_{nullptr};
  rclcpp::Publisher<buoy_msgs::msg::SCRecord>::SharedPtr sc_pub_;
  rclcpp::Rate pub_rate_{10.0};
  std::chrono::steady_clock::duration current_time_;
  buoy_msgs::msg::SCRecord sc_record_;
  std::mutex data_mutex_, next_access_mutex_, low_prio_mutex_;
  bool data_valid_{false};
  std::thread thread_executor_spin_, thread_publish_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  bool stop_{false};
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
  // Stop controller manager thread
  this->dataPtr->stop_ = true;
  this->dataPtr->executor_->cancel();
  this->dataPtr->thread_executor_spin_.join();
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
    ignerr << "SpringController found an empty jointName parameter. " <<
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
  auto scoped_name = ignition::gazebo::scopedName(this->dataPtr->entity_, _ecm, "/", false);

  // ROS node
  std::vector<std::string> arguments = {"--ros-args"};
  auto ns = _sdf->Get<std::string>("namespace", scoped_name).first;
  if (ns.empty() || ns[0] != '/') {
    ns = '/' + ns;
  }
  std::string ns_arg = std::string("__ns:=") + ns;
  arguments.push_back(RCL_REMAP_FLAG);
  arguments.push_back(ns_arg);
  std::vector<const char *> argv;
  for (const auto & arg : arguments) {
    argv.push_back(reinterpret_cast<const char *>(arg.data()));
  }
  if (!rclcpp::ok()) {
    rclcpp::init(static_cast<int>(argv.size()), argv.data());
    std::string node_name = "spring_controller";
    this->dataPtr->rosnode_ = rclcpp::Node::make_shared(node_name);
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

  // Publisher
  auto topic = _sdf->Get<std::string>("topic", "sc_record").first;
  this->dataPtr->sc_pub_ = \
    this->dataPtr->rosnode_->create_publisher<buoy_msgs::msg::SCRecord>(topic, 10);

  auto publish = [this]()
  {
    while (rclcpp::ok() && !this->dataPtr->stop_) {
      if (this->dataPtr->sc_pub_->get_subscription_count() <= 0)  continue;
      
      buoy_msgs::msg::SCRecord sc_record;
      // high prio data access
      std::unique_lock next(this->dataPtr->next_access_mutex_);
      std::unique_lock data(this->dataPtr->data_mutex_);
      next.unlock();
      this->dataPtr->sc_record_.seq_num = \
        this->dataPtr->seq_num++ % std::numeric_limits<int16_t>::max();
      sc_record = this->dataPtr->sc_record_;
      data.unlock();
      
      if(this->dataPtr->data_valid_)  this->dataPtr->sc_pub_->publish(sc_record);
      
      this->dataPtr->pub_rate_.sleep();
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

  auto spring_state_comp = \
    _ecm.Component<buoy_gazebo::SpringStateComponent>(this->dataPtr->jointEntity_);
  RCLCPP_DEBUG_STREAM(
    this->dataPtr->rosnode_->get_logger(), "[ROS 2 Spring Control] filling spring data [" <<
      model.Name(_ecm) << "] spring data component: " << \
      spring_state_comp << ")].");
  if (spring_state_comp == nullptr) {
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

  // low prio data access
  std::unique_lock low(this->dataPtr->low_prio_mutex_);
  std::unique_lock next(this->dataPtr->next_access_mutex_);
  std::unique_lock data(this->dataPtr->data_mutex_);
  next.unlock();
  this->dataPtr->data_valid_ = true;

  this->dataPtr->current_time_ = _info.simTime;
  auto sec_nsec = ignition::math::durationToSecNsec(dataPtr->current_time_);
  
  this->dataPtr->sc_record_.header.stamp.sec = sec_nsec.first;
  this->dataPtr->sc_record_.header.stamp.nanosec = sec_nsec.second;
  //this->dataPtr->sc_record_.seq_num = dataPtr->seq_num++;

  this->dataPtr->sc_record_.load_cell = spring_state_comp->Data().load_cell;
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
