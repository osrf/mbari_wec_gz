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

#include "LatentDataPublisher.hpp"

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

#include <buoy_interfaces/msg/latent_data.hpp>
#include <LatentData/LatentData.hpp>

#include "buoy_utils/Rate.hpp"


struct buoy_gazebo::LatentDataPublisherPrivate
{
  rclcpp::Node::SharedPtr rosnode_{nullptr};
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

  gz::sim::Entity entity_;
  gz::transport::Node node_;
  bool use_sim_time_{true};
  std::chrono::steady_clock::duration current_time_;

  rclcpp::Publisher<buoy_interfaces::msg::LatentData>::SharedPtr ros2_pub_{nullptr};

  double pub_rate_hz_{10.0};
  std::unique_ptr<buoy_utils::SimRate> pub_rate_{nullptr};
  buoy_interfaces::msg::LatentData latent_data_;
  std::atomic<bool> data_valid_{false};

  std::mutex data_mutex_, next_access_mutex_, low_prio_mutex_;

  std::thread thread_executor_spin_, thread_publish_;
  std::atomic<bool> stop_{false}, paused_{true};

  void copy_inc_wave_height(
    const buoy_gazebo::IncWaveHeightPoint & in,
    buoy_interfaces::msg::IncWaveHeight & out)
  {
    // all fixed points from SDF computed at SimTime (relative_time = 0.0)
    out.relative_time = 0.0;
    out.use_buoy_origin = in.use_buoy_origin;
    out.pose.pose.position.x = in.x;
    out.pose.pose.position.y = in.y;
    out.pose.pose.position.z = in.eta;
    out.pose.pose.orientation.x = in.qx;
    out.pose.pose.orientation.y = in.qy;
    out.pose.pose.orientation.z = in.qz;
    out.pose.pose.orientation.w = in.qw;
  }
};


GZ_ADD_PLUGIN(
  buoy_gazebo::LatentDataPublisher,
  gz::sim::System,
  buoy_gazebo::LatentDataPublisher::ISystemConfigure,
  buoy_gazebo::LatentDataPublisher::ISystemPostUpdate)

namespace buoy_gazebo
{
//////////////////////////////////////////////////
LatentDataPublisher::LatentDataPublisher()
: dataPtr(std::make_unique<LatentDataPublisherPrivate>())
{
}

LatentDataPublisher::~LatentDataPublisher()
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

void LatentDataPublisher::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager &)
{
  // Make sure the controller is attached to a valid model
  auto model = gz::sim::Model(_entity);
  if (!model.Valid(_ecm)) {
    gzerr << "[ROS 2 LatentData Publisher] Failed to initialize because [" << \
      model.Name(_ecm) << "] is not a model." << std::endl << \
      "Please make sure that ROS 2 LatentData Publisher is attached to a valid model." << std::endl;
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
  std::string node_name = _sdf->Get<std::string>("node_name", "latent_data").first;
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
    "[ROS 2 LatentData Publisher] Setting up controller for [" << model.Name(_ecm) << "]");

  // Set header
  auto sec_nsec = gz::math::durationToSecNsec(this->dataPtr->current_time_);
  this->dataPtr->latent_data_.header.stamp.sec = sec_nsec.first;
  this->dataPtr->latent_data_.header.stamp.nanosec = sec_nsec.second;

  // Publisher
  std::string ros2_topic = _sdf->Get<std::string>("ros2_topic", "latent_data").first;
  this->dataPtr->ros2_pub_ =
    this->dataPtr->rosnode_->create_publisher<buoy_interfaces::msg::LatentData>(ros2_topic, 10);

  this->dataPtr->pub_rate_hz_ =
    _sdf->Get<double>("publish_rate", this->dataPtr->pub_rate_hz_).first;
  this->dataPtr->pub_rate_ = std::make_unique<buoy_utils::SimRate>(
    this->dataPtr->pub_rate_hz_,
    this->dataPtr->rosnode_->get_clock());

  auto publish = [this]()
    {
      while (rclcpp::ok() && !this->dataPtr->stop_) {
        if (this->dataPtr->ros2_pub_->get_subscription_count() <= 0) {continue;}
        // Only update and publish if not paused.
        if (this->dataPtr->paused_) {continue;}

        buoy_interfaces::msg::LatentData latent_data;
        // high prio data access
        std::unique_lock next(this->dataPtr->next_access_mutex_);
        std::unique_lock data(this->dataPtr->data_mutex_);
        next.unlock();

        if (this->dataPtr->data_valid_) {
          latent_data = this->dataPtr->latent_data_;
          data.unlock();
          if (this->dataPtr->ros2_pub_->get_subscription_count() > 0) {
            this->dataPtr->ros2_pub_->publish(latent_data);
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
void LatentDataPublisher::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & _ecm)
{
  this->dataPtr->paused_ = _info.paused;
  if (_info.paused) {
    return;
  }

  this->dataPtr->current_time_ = _info.simTime;
  auto sec_nsec = gz::math::durationToSecNsec(this->dataPtr->current_time_);

  buoy_gazebo::LatentData latent_data;
  if (_ecm.EntityHasComponentType(
      this->dataPtr->entity_,
      buoy_gazebo::components::LatentData().TypeId()))
  {
    auto latent_data_comp =
      _ecm.Component<buoy_gazebo::components::LatentData>(this->dataPtr->entity_);

    latent_data = buoy_gazebo::LatentData(latent_data_comp->Data());
  } else {
    this->dataPtr->data_valid_ = false;
    return;
  }

  // low prio data access
  std::unique_lock low(this->dataPtr->low_prio_mutex_);
  std::unique_lock next(this->dataPtr->next_access_mutex_);
  std::unique_lock data(this->dataPtr->data_mutex_);
  next.unlock();

  this->dataPtr->latent_data_.header.stamp.sec = sec_nsec.first;
  this->dataPtr->latent_data_.header.stamp.nanosec = sec_nsec.second;

  this->dataPtr->latent_data_.inc_wave_heights.clear();
  this->dataPtr->latent_data_.inc_wave_heights.resize(latent_data.inc_wave_heights.points.size());

  std::size_t idx = 0U;
  for (; idx < latent_data.inc_wave_heights.points.size(); ++idx) {
    // all fixed points from SDF computed at SimTime (relative_time = 0.0)
    this->dataPtr->latent_data_.inc_wave_heights[idx].pose.header.stamp.sec =
      latent_data.inc_wave_heights.sec;
    this->dataPtr->latent_data_.inc_wave_heights[idx].pose.header.stamp.nanosec =
      latent_data.inc_wave_heights.nsec;
    this->dataPtr->copy_inc_wave_height(
      latent_data.inc_wave_heights.points[idx],
      this->dataPtr->latent_data_.inc_wave_heights[idx]);
  }

  this->dataPtr->latent_data_.upper_spring.force = latent_data.upper_spring.force;
  this->dataPtr->latent_data_.upper_spring.temperature = latent_data.upper_spring.T;
  this->dataPtr->latent_data_.upper_spring.heat_loss = latent_data.upper_spring.dQ_dt;
  this->dataPtr->latent_data_.upper_spring.piston_position =
    latent_data.upper_spring.piston_position;
  this->dataPtr->latent_data_.upper_spring.piston_velocity =
    latent_data.upper_spring.piston_velocity;
  this->dataPtr->latent_data_.lower_spring.force = latent_data.lower_spring.force;
  this->dataPtr->latent_data_.lower_spring.temperature = latent_data.lower_spring.T;
  this->dataPtr->latent_data_.lower_spring.heat_loss = latent_data.lower_spring.dQ_dt;
  this->dataPtr->latent_data_.lower_spring.piston_position =
    latent_data.lower_spring.piston_position;
  this->dataPtr->latent_data_.lower_spring.piston_velocity =
    latent_data.lower_spring.piston_velocity;

  this->dataPtr->latent_data_.electro_hydraulic.rpm =
    latent_data.electro_hydraulic.rpm;
  this->dataPtr->latent_data_.electro_hydraulic.upper_hydraulic_pressure =
    latent_data.electro_hydraulic.upper_hydraulic_pressure;
  this->dataPtr->latent_data_.electro_hydraulic.lower_hydraulic_pressure =
    latent_data.electro_hydraulic.lower_hydraulic_pressure;
  this->dataPtr->latent_data_.electro_hydraulic.force =
    latent_data.electro_hydraulic.force;
  this->dataPtr->latent_data_.electro_hydraulic.supplied_hydraulic_power =
    latent_data.electro_hydraulic.supplied_hydraulic_power;
  this->dataPtr->latent_data_.electro_hydraulic.hydraulic_motor_loss =
    latent_data.electro_hydraulic.hydraulic_motor_loss;
  this->dataPtr->latent_data_.electro_hydraulic.relief_valve_loss =
    latent_data.electro_hydraulic.relief_valve_loss;
  this->dataPtr->latent_data_.electro_hydraulic.motor_emf_power =
    latent_data.electro_hydraulic.motor_emf_power;
  this->dataPtr->latent_data_.electro_hydraulic.motor_drive_i2r_loss =
    latent_data.electro_hydraulic.motor_drive_i2r_loss;
  this->dataPtr->latent_data_.electro_hydraulic.motor_drive_switching_loss =
    latent_data.electro_hydraulic.motor_drive_switching_loss;
  this->dataPtr->latent_data_.electro_hydraulic.motor_drive_friction_loss =
    latent_data.electro_hydraulic.motor_drive_friction_loss;
  this->dataPtr->latent_data_.electro_hydraulic.load_dump_power =
    latent_data.electro_hydraulic.load_dump_power;
  this->dataPtr->latent_data_.electro_hydraulic.battery_i2r_loss =
    latent_data.electro_hydraulic.battery_i2r_loss;
  this->dataPtr->latent_data_.electro_hydraulic.battery_storage_power =
    latent_data.electro_hydraulic.battery_storage_power;

  this->dataPtr->latent_data_.wave_body.pose.position.x =
    latent_data.wave_body.pose.X();
  this->dataPtr->latent_data_.wave_body.pose.position.y =
    latent_data.wave_body.pose.Y();
  this->dataPtr->latent_data_.wave_body.pose.position.z =
    latent_data.wave_body.pose.Z();
  this->dataPtr->latent_data_.wave_body.pose.orientation.x =
    latent_data.wave_body.pose.Rot().X();
  this->dataPtr->latent_data_.wave_body.pose.orientation.y =
    latent_data.wave_body.pose.Rot().Y();
  this->dataPtr->latent_data_.wave_body.pose.orientation.z =
    latent_data.wave_body.pose.Rot().Z();
  this->dataPtr->latent_data_.wave_body.pose.orientation.w =
    latent_data.wave_body.pose.Rot().W();

  this->dataPtr->latent_data_.wave_body.twist.linear.x =
    latent_data.wave_body.twist.X();
  this->dataPtr->latent_data_.wave_body.twist.linear.y =
    latent_data.wave_body.twist.Y();
  this->dataPtr->latent_data_.wave_body.twist.linear.z =
    latent_data.wave_body.twist.Z();
  this->dataPtr->latent_data_.wave_body.twist.angular.x =
    latent_data.wave_body.twist.Roll();
  this->dataPtr->latent_data_.wave_body.twist.angular.y =
    latent_data.wave_body.twist.Pitch();
  this->dataPtr->latent_data_.wave_body.twist.angular.z =
    latent_data.wave_body.twist.Yaw();

  this->dataPtr->latent_data_.wave_body.buoyancy.force.x =
    latent_data.wave_body.buoyant_force.X();
  this->dataPtr->latent_data_.wave_body.buoyancy.force.y =
    latent_data.wave_body.buoyant_force.Y();
  this->dataPtr->latent_data_.wave_body.buoyancy.force.z =
    latent_data.wave_body.buoyant_force.Z();
  this->dataPtr->latent_data_.wave_body.buoyancy.torque.x =
    latent_data.wave_body.buoyant_moment.X();
  this->dataPtr->latent_data_.wave_body.buoyancy.torque.y =
    latent_data.wave_body.buoyant_moment.Y();
  this->dataPtr->latent_data_.wave_body.buoyancy.torque.z =
    latent_data.wave_body.buoyant_moment.Z();
  this->dataPtr->latent_data_.wave_body.buoyancy_total_power =
    latent_data.wave_body.buoyancy_total_power;

  this->dataPtr->latent_data_.wave_body.radiation.force.x =
    latent_data.wave_body.radiation_force.X();
  this->dataPtr->latent_data_.wave_body.radiation.force.y =
    latent_data.wave_body.radiation_force.Y();
  this->dataPtr->latent_data_.wave_body.radiation.force.z =
    latent_data.wave_body.radiation_force.Z();
  this->dataPtr->latent_data_.wave_body.radiation.torque.x =
    latent_data.wave_body.radiation_moment.X();
  this->dataPtr->latent_data_.wave_body.radiation.torque.y =
    latent_data.wave_body.radiation_moment.Y();
  this->dataPtr->latent_data_.wave_body.radiation.torque.z =
    latent_data.wave_body.radiation_moment.Z();
  this->dataPtr->latent_data_.wave_body.radiation_total_power =
    latent_data.wave_body.radiation_total_power;

  this->dataPtr->latent_data_.wave_body.excitation.force.x =
    latent_data.wave_body.exciting_force.X();
  this->dataPtr->latent_data_.wave_body.excitation.force.y =
    latent_data.wave_body.exciting_force.Y();
  this->dataPtr->latent_data_.wave_body.excitation.force.z =
    latent_data.wave_body.exciting_force.Z();
  this->dataPtr->latent_data_.wave_body.excitation.torque.x =
    latent_data.wave_body.exciting_moment.X();
  this->dataPtr->latent_data_.wave_body.excitation.torque.y =
    latent_data.wave_body.exciting_moment.Y();
  this->dataPtr->latent_data_.wave_body.excitation.torque.z =
    latent_data.wave_body.exciting_moment.Z();
  this->dataPtr->latent_data_.wave_body.excitation_total_power =
    latent_data.wave_body.excitation_total_power;

  this->dataPtr->latent_data_.piston_friction_force =
    latent_data.piston_friction_force;

  this->dataPtr->data_valid_ = latent_data.valid();

  this->dataPtr->data_valid_ = latent_data.valid();

  data.unlock();
}
}  // namespace buoy_gazebo
