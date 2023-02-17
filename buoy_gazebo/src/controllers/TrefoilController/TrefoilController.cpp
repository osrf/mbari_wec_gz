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

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>

#include <buoy_interfaces/msg/tf_record.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "TrefoilController.hpp"

struct buoy_gazebo::TrefoilControllerPrivate
{
  rclcpp::Node::SharedPtr rosnode_{nullptr};
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

  gz::sim::Entity entity_;
  gz::transport::Node node_;
  std::chrono::steady_clock::duration current_time_;

  rclcpp::Publisher<buoy_interfaces::msg::TFRecord>::SharedPtr tf_pub_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_{nullptr};

  std::function<void(const gz::msgs::IMU &)> imu_cb_;
  std::function<void(const gz::msgs::Magnetometer &)> mag_cb_;
  double pub_rate_hz_{10.0};
  std::unique_ptr<rclcpp::Rate> pub_rate_{nullptr};
  buoy_interfaces::msg::TFRecord tf_record_;

  std::mutex data_mutex_, next_access_mutex_, low_prio_mutex_;
  std::atomic<bool> imu_data_valid_{false}, mag_data_valid_{false};

  std::thread thread_executor_spin_, thread_publish_;
  std::atomic<bool> stop_{false}, paused_{true};

  int16_t seq_num{0};

  void set_tf_record_imu(const gz::msgs::IMU & _imu)
  {
    tf_record_.header.stamp.sec = _imu.header().stamp().sec();
    tf_record_.header.stamp.nanosec = _imu.header().stamp().nsec();
    tf_record_.header.frame_id = "Trefoil";
    tf_record_.imu.header = tf_record_.header;
    tf_record_.imu.orientation.x = _imu.orientation().x();
    tf_record_.imu.orientation.y = _imu.orientation().y();
    tf_record_.imu.orientation.z = _imu.orientation().z();
    tf_record_.imu.orientation.w = _imu.orientation().w();
    tf_record_.imu.angular_velocity.x = _imu.angular_velocity().x();
    tf_record_.imu.angular_velocity.y = _imu.angular_velocity().y();
    tf_record_.imu.angular_velocity.z = _imu.angular_velocity().z();
    tf_record_.imu.linear_acceleration.x = _imu.linear_acceleration().x();
    tf_record_.imu.linear_acceleration.y = _imu.linear_acceleration().y();
    tf_record_.imu.linear_acceleration.z = _imu.linear_acceleration().z();
  }

  void set_tf_record_mag(const gz::msgs::Magnetometer & _mag)
  {
    tf_record_.header.stamp.sec = _mag.header().stamp().sec();
    tf_record_.header.stamp.nanosec = _mag.header().stamp().nsec();
    tf_record_.header.frame_id = "Trefoil";
    tf_record_.mag.header = tf_record_.header;
    tf_record_.mag.magnetic_field.x = _mag.field_tesla().x();
    tf_record_.mag.magnetic_field.y = _mag.field_tesla().y();
    tf_record_.mag.magnetic_field.z = _mag.field_tesla().z();
  }

  bool data_valid() const
  {
    return imu_data_valid_ && mag_data_valid_;
  }
};


GZ_ADD_PLUGIN(
  buoy_gazebo::TrefoilController,
  gz::sim::System,
  buoy_gazebo::TrefoilController::ISystemConfigure,
  buoy_gazebo::TrefoilController::ISystemPostUpdate)

namespace buoy_gazebo
{
//////////////////////////////////////////////////
TrefoilController::TrefoilController()
: dataPtr(std::make_unique<TrefoilControllerPrivate>())
{
}

TrefoilController::~TrefoilController()
{
  // Stop ros2 threads
  this->dataPtr->stop_ = true;
  this->dataPtr->executor_->cancel();
  this->dataPtr->thread_executor_spin_.join();
  this->dataPtr->thread_publish_.join();
}

void TrefoilController::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager &)
{
  // Make sure the controller is attached to a valid model
  auto model = gz::sim::Model(_entity);
  if (!model.Valid(_ecm)) {
    gzerr << "[ROS 2 Trefoil Controller] Failed to initialize because [" << \
      model.Name(_ecm) << "] is not a model." << std::endl << \
      "Please make sure that ROS 2 Trefoil Controller is attached to a valid model." << std::endl;
    return;
  }
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
  std::string node_name = _sdf->Get<std::string>("node_name", "Trefoil_controller").first;
  this->dataPtr->rosnode_ = rclcpp::Node::make_shared(node_name, ns);

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
    "[ROS 2 Trefoil Controller] Setting up controller for [" << model.Name(_ecm) << "]");

  // IMU Sensor
  this->dataPtr->imu_cb_ = [this](const gz::msgs::IMU & _imu)
    {
      // low prio data access
      std::unique_lock low(this->dataPtr->low_prio_mutex_);
      std::unique_lock next(this->dataPtr->next_access_mutex_);
      std::unique_lock data(this->dataPtr->data_mutex_);
      next.unlock();
      this->dataPtr->set_tf_record_imu(_imu);
      this->dataPtr->imu_data_valid_ = true;
      data.unlock();
    };
  if (!this->dataPtr->node_.Subscribe("/Trefoil_link/trefoil_imu", this->dataPtr->imu_cb_)) {
    gzerr << "Error subscribing to topic [" << "/Trefoil_link/trefoil_imu" << "]" << std::endl;
    return;
  }

  // Magnetometer Sensor
  this->dataPtr->mag_cb_ = [this](const gz::msgs::Magnetometer & _mag)
    {
      // low prio data access
      std::unique_lock low(this->dataPtr->low_prio_mutex_);
      std::unique_lock next(this->dataPtr->next_access_mutex_);
      std::unique_lock data(this->dataPtr->data_mutex_);
      next.unlock();
      this->dataPtr->set_tf_record_mag(_mag);
      this->dataPtr->mag_data_valid_ = true;
      data.unlock();
    };
  if (!this->dataPtr->node_.Subscribe("/Trefoil_link/trefoil_mag", this->dataPtr->mag_cb_)) {
    gzerr << "Error subscribing to topic [" << "/Trefoil_link/trefoil_mag" << "]" << std::endl;
    return;
  }

  // Publisher
  std::string tf_topic = _sdf->Get<std::string>("tf_topic", "trefoil_data").first;
  this->dataPtr->tf_pub_ = \
    this->dataPtr->rosnode_->create_publisher<buoy_interfaces::msg::TFRecord>(tf_topic, 10);

  std::string imu_topic = _sdf->Get<std::string>("imu_topic", "trefoil_imu").first;
  this->dataPtr->imu_pub_ = \
    this->dataPtr->rosnode_->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);

  std::string mag_topic = _sdf->Get<std::string>("mag_topic", "trefoil_mag").first;
  this->dataPtr->mag_pub_ = \
    this->dataPtr->rosnode_->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic, 10);

  this->dataPtr->pub_rate_hz_ = \
    _sdf->Get<double>("publish_rate", this->dataPtr->pub_rate_hz_).first;
  this->dataPtr->pub_rate_ = std::make_unique<rclcpp::Rate>(this->dataPtr->pub_rate_hz_);

  auto publish = [this]()
    {
      while (rclcpp::ok() && !this->dataPtr->stop_) {
        if (this->dataPtr->tf_pub_->get_subscription_count() <= 0 && \
          this->dataPtr->imu_pub_->get_subscription_count() <= 0 && \
          this->dataPtr->mag_pub_->get_subscription_count() <= 0) {continue;}
        // Only update and publish if not paused.
        if (this->dataPtr->paused_) {continue;}

        buoy_interfaces::msg::TFRecord tf_record;
        // high prio data access
        std::unique_lock next(this->dataPtr->next_access_mutex_);
        std::unique_lock data(this->dataPtr->data_mutex_);
        next.unlock();

        if (this->dataPtr->data_valid()) {
          this->dataPtr->tf_record_.seq_num =
            this->dataPtr->seq_num++ % std::numeric_limits<int16_t>::max();
          tf_record = this->dataPtr->tf_record_;
          data.unlock();
          if (this->dataPtr->tf_pub_->get_subscription_count() > 0) {
            this->dataPtr->tf_pub_->publish(tf_record);
          }
          if (this->dataPtr->imu_pub_->get_subscription_count() > 0) {
            this->dataPtr->imu_pub_->publish(tf_record.imu);
          }
          if (this->dataPtr->mag_pub_->get_subscription_count() > 0) {
            this->dataPtr->mag_pub_->publish(tf_record.mag);
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
void TrefoilController::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & _ecm)
{
  this->dataPtr->paused_ = _info.paused;
  if (_info.paused) {
    return;
  }

  //  Get Trefoil link's pose
  auto model = gz::sim::Model(this->dataPtr->entity_);
  auto link = model.LinkByName(_ecm, "Trefoil");
  auto pose = gz::sim::worldPose(link, _ecm);
  double depth = pose.Pos().Z();

  // low prio data access
  std::unique_lock low(this->dataPtr->low_prio_mutex_);
  std::unique_lock next(this->dataPtr->next_access_mutex_);
  std::unique_lock data(this->dataPtr->data_mutex_);
  next.unlock();

  auto sec_nsec = gz::math::durationToSecNsec(this->dataPtr->current_time_);

  this->dataPtr->tf_record_.header.stamp.sec = sec_nsec.first;
  this->dataPtr->tf_record_.header.stamp.nanosec = sec_nsec.second;
  //  Sea pressure: depth*rho*g, Pascal to psi
  this->dataPtr->tf_record_.pressure = (depth * 1025 * 9.8) / 6894.75729;

  //  Constants
  this->dataPtr->tf_record_.power_timeouts = 60;
  this->dataPtr->tf_record_.tether_voltage = 48.0;
  this->dataPtr->tf_record_.battery_voltage = 48.0;
  this->dataPtr->tf_record_.status = 0;
  this->dataPtr->tf_record_.vpe_status = 0;
  this->dataPtr->tf_record_.comms_timeouts = 60;
  this->dataPtr->tf_record_.motor_status = 0;
  this->dataPtr->tf_record_.motor_current = 0;
  this->dataPtr->tf_record_.encoder = 0;

  data.unlock();
}
}  // namespace buoy_gazebo
