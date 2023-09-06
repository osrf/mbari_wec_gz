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

#include "IncWaveHeight.hpp"

#include <string>
#include <tuple>

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <buoy_interfaces/srv/inc_wave_height.hpp>

#include <IncidentWaves/IncWaveState.hpp>
#include <LatentData/LatentData.hpp>


using namespace std::chrono_literals;

namespace buoy_gazebo
{
struct IncWaveHeightROS2
{
  rclcpp::Node::SharedPtr node_{nullptr};
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_{nullptr};
  bool use_sim_time_{true};
};

struct IncWaveHeightServices
{
  rclcpp::Service<buoy_interfaces::srv::IncWaveHeight>::SharedPtr inc_wave_height_service_{nullptr};
  std::function<void(std::shared_ptr<buoy_interfaces::srv::IncWaveHeight::Request>,
    std::shared_ptr<buoy_interfaces::srv::IncWaveHeight::Response>)> inc_wave_height_handler_;
};

struct IncWaveHeightPrivate
{
  gz::sim::Entity IncWaveEntity{gz::sim::kNullEntity};
  buoy_gazebo::IncWaveState inc_wave_state;

  gz::sim::Entity entity{gz::sim::kNullEntity};
  gz::sim::Model model{gz::sim::kNullEntity};

  std::chrono::steady_clock::duration current_time_;

  buoy_gazebo::IncWaveHeights inc_wave_heights;

  std::mutex data_mutex_, next_access_mutex_, low_prio_mutex_;
  std::atomic<bool> inc_wave_valid_{false};
  std::atomic<bool> got_new_request_{false};

  std::thread thread_executor_spin_;
  std::atomic<bool> stop_{false}, paused_{true};

  std::unique_ptr<IncWaveHeightROS2> ros_;
  std::unique_ptr<IncWaveHeightServices> services_;

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
        rclcpp::Rate rate(50.0);
        while (rclcpp::ok() && !stop_) {
          ros_->executor_->spin_once();
          rate.sleep();
        }
      };
    thread_executor_spin_ = std::thread(spin);
  }

  std::tuple<double, gz::math::Quaternion<double>> compute_eta(
    const double & _x,
    const double & _y,
    const double & SimTime,
    const bool use_buoy_origin)
  {
    double x = _x;
    double y = _y;
    if (use_buoy_origin) {
      x += this->inc_wave_state.x;
      y += this->inc_wave_state.y;
    }

    double deta_dx{0.0}, deta_dy{0.0};
    double eta = this->inc_wave_state.Inc.eta(
      x, y, SimTime, &deta_dx, &deta_dy);

    double roll = atan(deta_dx);
    double pitch = atan(deta_dy);
    double yaw = 0.0;

    gz::math::Quaternion<double> q =
      gz::math::Quaternion<double>::EulerToQuaternion(roll, pitch, yaw);

    return std::make_tuple(eta, q);
  }

  void setup_services()
  {
    // IncWaveHeight
    services_->inc_wave_height_handler_ =
      [this](const std::shared_ptr<buoy_interfaces::srv::IncWaveHeight::Request> request,
        std::shared_ptr<buoy_interfaces::srv::IncWaveHeight::Response> response)
      {
        RCLCPP_INFO_STREAM(
          ros_->node_->get_logger(),
          "[ROS 2 Incident Wave Height] IncWaveHeight Request received with ["
            << request->points.size() << "] point(s)");

        // high prio data access
        std::unique_lock next(next_access_mutex_);
        std::unique_lock data(data_mutex_);
        next.unlock();

        response->valid = this->inc_wave_valid_;
        if (!this->inc_wave_valid_) {
          data.unlock();
          return;
        }

        double SimTime = std::chrono::duration<double>(current_time_).count();
        auto sec_nsec = gz::math::durationToSecNsec(current_time_);

        response->heights.resize(request->points.size());
        double relative_time = request->relative_time;
        bool use_buoy_origin = request->use_buoy_origin;
        for (std::size_t idx = 0U; idx < request->points.size(); ++idx) {
          double x = request->points[idx].x;
          double y = request->points[idx].y;

          double eta{0.0};
          gz::math::Quaternion<double> q;
          std::tie(eta, q) = compute_eta(x, y, SimTime + relative_time, use_buoy_origin);

          response->heights[idx].relative_time = relative_time;
          response->heights[idx].use_buoy_origin = use_buoy_origin;
          response->heights[idx].pose.header.stamp.sec = sec_nsec.first;
          response->heights[idx].pose.header.stamp.nanosec = sec_nsec.second;
          response->heights[idx].pose.pose.position = request->points[idx];
          response->heights[idx].pose.pose.position.z = eta;

          response->heights[idx].pose.pose.orientation.x = q.X();
          response->heights[idx].pose.pose.orientation.y = q.Y();
          response->heights[idx].pose.pose.orientation.z = q.Z();
          response->heights[idx].pose.pose.orientation.w = q.W();
        }
        data.unlock();
      };
    services_->inc_wave_height_service_ =
      ros_->node_->create_service<buoy_interfaces::srv::IncWaveHeight>(
      "inc_wave_height",
      services_->inc_wave_height_handler_);
  }
};

//////////////////////////////////////////////////
IncWaveHeight::IncWaveHeight()
: dataPtr(std::make_unique<IncWaveHeightPrivate>())
{
  this->dataPtr->ros_ = std::make_unique<IncWaveHeightROS2>();
  this->dataPtr->services_ = std::make_unique<IncWaveHeightServices>();
}

IncWaveHeight::~IncWaveHeight()
{
  // Stop ros2 threads
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  this->dataPtr->stop_ = true;
  if (this->dataPtr->ros_->executor_) {
    this->dataPtr->ros_->executor_->cancel();
  }
  this->dataPtr->thread_executor_spin_.join();
}

void IncWaveHeight::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager &)
{
  // Make sure the controller is attached to a valid model
  this->dataPtr->entity = _entity;
  this->dataPtr->model = gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    gzerr << "[ROS 2 Incident Wave Height] Failed to initialize because [" <<
      this->dataPtr->model.Name(_ecm) << "] is not a model." << std::endl <<
      "Please make sure that ROS 2 Incident Wave Height is attached to a valid model." << std::endl;
    return;
  }

  // <points use_buoy_origin="true">
  //   <xy>1.2 3.4</xy>
  //   <xy>3.1 4.1</xy>
  // </points>
  if (_sdf->HasElement("points")) {
    const sdf::ElementPtr points = _sdf->GetElementImpl("points");
    if (points != nullptr) {
      bool use_buoy_origin{false};
      if (points->GetAttributeSet("use_buoy_origin")) {
        const sdf::ParamPtr p = points->GetAttribute("use_buoy_origin");
        if (p != nullptr) {
          bool result = p->Get(use_buoy_origin);
          if (!result) {
            use_buoy_origin = false;
          }
        }
      }

      this->dataPtr->inc_wave_heights.points.clear();
      bool first = true;
      sdf::ElementPtr e{nullptr};
      for (;;) {
        if (first) {
          e = points->GetElementImpl("xy");
          first = false;
        } else {
          e = e->GetNextElement();
        }
        if (e == nullptr) {
          break;
        }

        IncWaveHeightPoint pt;
        pt.use_buoy_origin = use_buoy_origin;
        std::string xy = e->Get<std::string>();
        std::stringstream ss(xy);
        ss >> pt.x; ss >> pt.y;
        if (ss.fail()) {
          gzerr << "[ROS 2 Incident Wave Height] Could not parse input point from SDF" << std::endl;
          continue;
        }
        this->dataPtr->inc_wave_heights.points.push_back(pt);

        this->dataPtr->got_new_request_ = true;
      }
    } else {
      gzerr << "[ROS 2 Incident Wave Height] Could not parse input points from SDF" << std::endl;
    }
  }

  // controller scoped name
  std::string scoped_name = gz::sim::scopedName(_entity, _ecm, "/", false);

  // ROS node
  std::string ns = _sdf->Get<std::string>("namespace", scoped_name).first;
  if (ns.empty() || ns[0] != '/') {
    ns = '/' + ns;
  }
  std::string node_name = _sdf->Get<std::string>("node_name", "inc_wave_service").first;

  this->dataPtr->ros2_setup(node_name, ns);

  RCLCPP_INFO_STREAM(
    this->dataPtr->ros_->node_->get_logger(),
    "[ROS 2 Incident Wave Height] Setting up service for ["
      << this->dataPtr->model.Name(_ecm) << "]");

  this->dataPtr->setup_services();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void IncWaveHeight::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("IncWaveHeight::PreUpdate");

  this->dataPtr->paused_ = _info.paused;
  this->dataPtr->current_time_ = _info.simTime;

  if (_info.paused) {
    return;
  }

  buoy_gazebo::LatentData latent_data;
  if (_ecm.EntityHasComponentType(
      this->dataPtr->entity,
      buoy_gazebo::components::LatentData().TypeId()))
  {
    auto latent_data_comp =
      _ecm.Component<buoy_gazebo::components::LatentData>(this->dataPtr->entity);

    latent_data = buoy_gazebo::LatentData(latent_data_comp->Data());
  }

  // low prio data access
  std::unique_lock low(this->dataPtr->low_prio_mutex_);
  std::unique_lock next(this->dataPtr->next_access_mutex_);
  std::unique_lock data(this->dataPtr->data_mutex_);
  next.unlock();

  if (!this->dataPtr->inc_wave_valid_) {
    data.unlock();
    return;
  }

  if (this->dataPtr->got_new_request_) {
    latent_data.inc_wave_heights = this->dataPtr->inc_wave_heights;
    this->dataPtr->got_new_request_ = false;
  }

  if (latent_data.inc_wave_heights.points.size() == 0U) {
    data.unlock();
    return;
  }

  double SimTime = std::chrono::duration<double>(this->dataPtr->current_time_).count();
  auto sec_nsec = gz::math::durationToSecNsec(this->dataPtr->current_time_);
  latent_data.inc_wave_heights.sec = sec_nsec.first;
  latent_data.inc_wave_heights.nsec = sec_nsec.second;

  std::size_t idx = 0U;
  for (; idx < latent_data.inc_wave_heights.points.size(); ++idx) {
    double use_buoy_origin = latent_data.inc_wave_heights.points[idx].use_buoy_origin;
    double x = latent_data.inc_wave_heights.points[idx].x;
    double y = latent_data.inc_wave_heights.points[idx].y;

    double eta{0.0};
    gz::math::Quaternion<double> q;
    std::tie(eta, q) = this->dataPtr->compute_eta(x, y, SimTime, use_buoy_origin);

    latent_data.inc_wave_heights.points[idx].eta = eta;
    latent_data.inc_wave_heights.points[idx].qx = q.X();
    latent_data.inc_wave_heights.points[idx].qy = q.Y();
    latent_data.inc_wave_heights.points[idx].qz = q.Z();
    latent_data.inc_wave_heights.points[idx].qw = q.W();
  }
  data.unlock();

  _ecm.SetComponentData<buoy_gazebo::components::LatentData>(
    this->dataPtr->entity,
    latent_data);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void IncWaveHeight::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("IncWaveHeight::Update");

  this->dataPtr->paused_ = _info.paused;
  this->dataPtr->current_time_ = _info.simTime;

  if (_info.paused) {
    return;
  }

  // low prio data access
  std::unique_lock low(this->dataPtr->low_prio_mutex_);
  std::unique_lock next(this->dataPtr->next_access_mutex_);
  std::unique_lock data(this->dataPtr->data_mutex_);
  next.unlock();

  this->dataPtr->IncWaveEntity =
    _ecm.EntityByComponents(gz::sim::components::Name("IncidentWaves"));

  if (this->dataPtr->IncWaveEntity == gz::sim::kNullEntity) {
    this->dataPtr->inc_wave_valid_ = false;
    return;
  }

  if (_ecm.EntityHasComponentType(
      this->dataPtr->IncWaveEntity,
      buoy_gazebo::components::IncWaveState().TypeId()))
  {
    auto inc_wave_state_comp =
      _ecm.Component<buoy_gazebo::components::IncWaveState>(
      this->dataPtr->IncWaveEntity);

    this->dataPtr->inc_wave_state = buoy_gazebo::IncWaveState(inc_wave_state_comp->Data());
    this->dataPtr->inc_wave_valid_ = true;
  } else {
    this->dataPtr->inc_wave_valid_ = false;
  }
  data.unlock();
}
}  // namespace buoy_gazebo

GZ_ADD_PLUGIN(
  buoy_gazebo::IncWaveHeight,
  gz::sim::System,
  buoy_gazebo::IncWaveHeight::ISystemConfigure,
  buoy_gazebo::IncWaveHeight::ISystemPreUpdate,
  buoy_gazebo::IncWaveHeight::ISystemPostUpdate)
