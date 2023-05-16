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

#include <gz/msgs/imu.pb.h>

#include <memory>
#include <string>
#include <vector>
#include <limits>

#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>

#include <buoy_interfaces/msg/xb_record.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "buoy_utils/Rate.hpp"
#include "XBowAHRS.hpp"


struct buoy_gazebo::XBowAHRSPrivate
{
  gz::sim::Entity entity_;
  gz::sim::Entity linkEntity_;
  rclcpp::Node::SharedPtr rosnode_{nullptr};
  bool use_sim_time_{true};
  gz::transport::Node node_;
  std::function<void(const gz::msgs::IMU &)> imu_cb_;
  rclcpp::Publisher<buoy_interfaces::msg::XBRecord>::SharedPtr xb_pub_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_{nullptr};
  double pub_rate_hz_{10.0};
  std::unique_ptr<buoy_utils::SimRate> pub_rate_{nullptr};
  std::chrono::steady_clock::duration current_time_;
  buoy_interfaces::msg::XBRecord xb_record_;
  std::mutex data_mutex_, next_access_mutex_, low_prio_mutex_;
  std::atomic<bool> imu_data_valid_{false}, velocity_data_valid_{false};
  std::thread thread_executor_spin_, thread_publish_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::atomic<bool> stop_{false}, paused_{true};

  void set_xb_record_imu(const gz::msgs::IMU & _imu)
  {
    // xb_record_.header.stamp.sec = _imu.header().stamp().sec();
    // xb_record_.header.stamp.nanosec = _imu.header().stamp().nsec();
    xb_record_.header.frame_id = "Buoy";
    xb_record_.imu.header = xb_record_.header;
    xb_record_.imu.orientation.x = _imu.orientation().x();
    xb_record_.imu.orientation.y = _imu.orientation().y();
    xb_record_.imu.orientation.z = _imu.orientation().z();
    xb_record_.imu.orientation.w = _imu.orientation().w();
    xb_record_.imu.angular_velocity.x = _imu.angular_velocity().x();
    xb_record_.imu.angular_velocity.y = _imu.angular_velocity().y();
    xb_record_.imu.angular_velocity.z = _imu.angular_velocity().z();
    xb_record_.imu.linear_acceleration.x = _imu.linear_acceleration().x();
    xb_record_.imu.linear_acceleration.y = _imu.linear_acceleration().y();
    xb_record_.imu.linear_acceleration.z = _imu.linear_acceleration().z();
  }

  bool data_valid() const
  {
    return velocity_data_valid_ && imu_data_valid_;
  }
};


GZ_ADD_PLUGIN(
  buoy_gazebo::XBowAHRS,
  gz::sim::System,
  buoy_gazebo::XBowAHRS::ISystemConfigure,
  buoy_gazebo::XBowAHRS::ISystemPostUpdate)

namespace buoy_gazebo
{
//////////////////////////////////////////////////
XBowAHRS::XBowAHRS()
: dataPtr(std::make_unique<XBowAHRSPrivate>())
{
}

XBowAHRS::~XBowAHRS()
{
  // Stop ros2 threads
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  this->dataPtr->stop_ = true;
  this->dataPtr->executor_->cancel();
  this->dataPtr->thread_executor_spin_.join();
  this->dataPtr->thread_publish_.join();
}

void XBowAHRS::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager &)
{
  // Make sure the controller is attached to a valid model
  auto model = gz::sim::Model(_entity);
  if (!model.Valid(_ecm)) {
    gzerr << "[ROS 2 XBow AHRS] Failed to initialize because [" << \
      model.Name(_ecm) << "] is not a model." << std::endl << \
      "Please make sure that ROS 2 XBow AHRS is attached to a valid model." << std::endl;
    return;
  }
  const auto link_entity = model.LinkByName(_ecm, "Buoy");
  gz::sim::Link link(link_entity);
  link.EnableVelocityChecks(_ecm);

  this->dataPtr->entity_ = _entity;

  // Get params from SDF
  // controller scoped name
  std::string scoped_name = gz::sim::scopedName(this->dataPtr->entity_, _ecm, "/", false);

  // ROS node
  std::string ns = _sdf->Get<std::string>("namespace", scoped_name).first;
  if (ns.empty() || ns[0] != '/') {
    ns = '/' + ns;
  }

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  std::string node_name = _sdf->Get<std::string>("node_name", "xbow_ahrs").first;
  this->dataPtr->rosnode_ = rclcpp::Node::make_shared(node_name, ns);
  
  this->dataPtr->rosnode_->set_parameter(
      rclcpp::Parameter(
        "use_sim_time",
        this->dataPtr->use_sim_time_));

  this->dataPtr->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  this->dataPtr->executor_->add_node(this->dataPtr->rosnode_);
  this->dataPtr->stop_ = false;

  auto spin = [this]()
    {
      rclcpp::Rate rate(50.0);
      while (rclcpp::ok() && !this->dataPtr->stop_) {
        this->dataPtr->executor_->spin_once();
        rate.sleep();
      }
    };
  this->dataPtr->thread_executor_spin_ = std::thread(spin);

  RCLCPP_INFO_STREAM(
    this->dataPtr->rosnode_->get_logger(),
    "[ROS 2 XBow AHRS] Setting up controller for [" << model.Name(_ecm) << "]");

  // IMU Sensor
  this->dataPtr->imu_cb_ = [this](const gz::msgs::IMU & _imu)
    {
      // low prio data access
      std::unique_lock low(this->dataPtr->low_prio_mutex_);
      std::unique_lock next(this->dataPtr->next_access_mutex_);
      std::unique_lock data(this->dataPtr->data_mutex_);
      next.unlock();
      this->dataPtr->set_xb_record_imu(_imu);
      this->dataPtr->imu_data_valid_ = true;
      data.unlock();
    };
  if (!this->dataPtr->node_.Subscribe("/Buoy_link/xbow_imu", this->dataPtr->imu_cb_)) {
    gzerr << "Error subscribing to topic [" << "/Buoy_link/xbow_imu" << "]" << std::endl;
    return;
  }

  // Publisher
  std::string xb_topic = _sdf->Get<std::string>("xb_topic", "ahrs_data").first;
  this->dataPtr->xb_pub_ = \
    this->dataPtr->rosnode_->create_publisher<buoy_interfaces::msg::XBRecord>(xb_topic, 10);

  std::string imu_topic = _sdf->Get<std::string>("imu_topic", "xb_imu").first;
  this->dataPtr->imu_pub_ = \
    this->dataPtr->rosnode_->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);

  this->dataPtr->pub_rate_hz_ = \
    _sdf->Get<double>("publish_rate", this->dataPtr->pub_rate_hz_).first;
  this->dataPtr->pub_rate_ = std::make_unique<buoy_utils::SimRate>(
    this->dataPtr->pub_rate_hz_,
    this->dataPtr->rosnode_->get_clock());

  auto publish = [this]()
    {
      while (rclcpp::ok() && !this->dataPtr->stop_) {
        if (this->dataPtr->xb_pub_->get_subscription_count() <= 0 && \
          this->dataPtr->imu_pub_->get_subscription_count() <= 0) {continue;}
        // Only update and publish if not paused.
        if (this->dataPtr->paused_) {continue;}

        buoy_interfaces::msg::XBRecord xb_record;
        // high prio data access
        std::unique_lock next(this->dataPtr->next_access_mutex_);
        std::unique_lock data(this->dataPtr->data_mutex_);
        next.unlock();

        if (this->dataPtr->data_valid()) {
          xb_record = this->dataPtr->xb_record_;
          data.unlock();
          if (this->dataPtr->xb_pub_->get_subscription_count() > 0) {
            this->dataPtr->xb_pub_->publish(xb_record);
          }
          if (this->dataPtr->imu_pub_->get_subscription_count() > 0) {
            this->dataPtr->imu_pub_->publish(xb_record.imu);
          }
        } else {
          data.unlock();
        }
        this->dataPtr->pub_rate_->sleep();
      }
    };
  this->dataPtr->thread_publish_ = std::thread(publish);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void XBowAHRS::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & _ecm)
{
  this->dataPtr->paused_ = _info.paused;
  auto model = gz::sim::Model(this->dataPtr->entity_);
  const auto link_entity = model.LinkByName(_ecm, "Buoy");
  gz::sim::Link link(link_entity);
  auto v_world = link.WorldLinearVelocity(_ecm);  // assume x,y,z == ENU

  this->dataPtr->current_time_ = _info.simTime;
  auto sec_nsec = gz::math::durationToSecNsec(this->dataPtr->current_time_);

  // low prio data access
  std::unique_lock low(this->dataPtr->low_prio_mutex_);
  std::unique_lock next(this->dataPtr->next_access_mutex_);
  std::unique_lock data(this->dataPtr->data_mutex_);
  next.unlock();

  this->dataPtr->xb_record_.header.stamp.sec = sec_nsec.first;
  this->dataPtr->xb_record_.header.stamp.nanosec = sec_nsec.second;

  if (v_world) {
    this->dataPtr->xb_record_.ned_velocity.x = v_world->Y();
    this->dataPtr->xb_record_.ned_velocity.y = v_world->X();
    this->dataPtr->xb_record_.ned_velocity.z = -v_world->Z();
    this->dataPtr->velocity_data_valid_ = true;
  } else {
    this->dataPtr->velocity_data_valid_ = false;
  }
  data.unlock();
}
}  // namespace buoy_gazebo
