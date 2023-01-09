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

#include <ros_ign_gazebo/Stopwatch.hpp>

#include <buoy_interfaces/msg/sc_record.hpp>
#include <buoy_interfaces/srv/valve_command.hpp>
#include <buoy_interfaces/srv/pump_command.hpp>

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
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_{nullptr};
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_handler_{nullptr};
  bool use_sim_time_{true};

  rclcpp::Publisher<buoy_interfaces::msg::SCRecord>::SharedPtr sc_pub_{nullptr};
  std::unique_ptr<rclcpp::Rate> pub_rate_{nullptr};
  buoy_interfaces::msg::SCRecord sc_record_;
  double pub_rate_hz_{10.0};
};

struct SpringControllerServices
{
  rclcpp::Service<buoy_interfaces::srv::ValveCommand>::SharedPtr valve_command_service_{nullptr};
  std::function<void(std::shared_ptr<buoy_interfaces::srv::ValveCommand::Request>,
    std::shared_ptr<buoy_interfaces::srv::ValveCommand::Response>)> valve_command_handler_;

  rclcpp::Service<buoy_interfaces::srv::PumpCommand>::SharedPtr pump_command_service_{nullptr};
  std::function<void(std::shared_ptr<buoy_interfaces::srv::PumpCommand::Request>,
    std::shared_ptr<buoy_interfaces::srv::PumpCommand::Response>)> pump_command_handler_;

  ros_ign_gazebo::Stopwatch command_watch_;
  rclcpp::Duration command_duration_{0, 0U};

  std::atomic<bool> valve_command_{false}, pump_command_{false};
  std::atomic<bool> new_pump_command_{false}, new_valve_command_{false};

  std::mutex command_mutex_;
};

struct SpringControllerPrivate
{
  ignition::gazebo::Entity entity_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity jointEntity_{ignition::gazebo::kNullEntity};
  ignition::transport::Node node_;
  std::function<void(const ignition::msgs::Wrench &)> ft_cb_;
  std::chrono::steady_clock::duration current_time_;

  std::mutex data_mutex_, next_access_mutex_, low_prio_mutex_;
  std::atomic<bool> spring_data_valid_{false}, load_cell_data_valid_{false};

  std::thread thread_executor_spin_, thread_publish_;
  std::atomic<bool> stop_{false}, paused_{true};

  int16_t seq_num{0};

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
              "[ROS 2 Spring Control] setting publish_rate: " << rate_hz);

            if (rate_hz < 10.0 || rate_hz > 50.0) {
              RCLCPP_WARN_STREAM(
                ros_->node_->get_logger(),
                "[ROS 2 Spring Control] publish_rate out of bounds -- clipped between [10,50]");
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

  void setup_services()
  {
    services_->command_watch_.SetClock(ros_->node_->get_clock());

    // ValveCommand
    services_->valve_command_handler_ =
      [this](const std::shared_ptr<buoy_interfaces::srv::ValveCommand::Request> request,
        std::shared_ptr<buoy_interfaces::srv::ValveCommand::Response> response)
      {
        RCLCPP_INFO_STREAM(
          ros_->node_->get_logger(),
          "[ROS 2 Spring Control] ValveCommand Received (" << request->duration_sec << "s)");

        std::unique_lock lock(services_->command_mutex_);
        // if already running pump, don't allow valve
        // unless for some reason we need to turn valve off (shouldn't get in that state)
        if (services_->pump_command_) {
          if (request->duration_sec != request->OFF) {
            response->result.value = response->result.BUSY;

            RCLCPP_ERROR_STREAM(
              ros_->node_->get_logger(),
              "[ROS 2 Spring Control] ValveCommand cannot process" <<
                " while pump command is running...");

            return;
          }
        }

        if (request->duration_sec == request->OFF) {
          services_->command_duration_ = rclcpp::Duration(0, 0U);
          services_->valve_command_ = false;
          services_->new_valve_command_ = true;
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

          services_->command_duration_ =
            rclcpp::Duration::from_seconds(static_cast<double>(duration_sec));

          services_->valve_command_ = true;
          services_->new_valve_command_ = true;
        }
      };
    services_->valve_command_service_ =
      ros_->node_->create_service<buoy_interfaces::srv::ValveCommand>(
      "valve_command",
      services_->valve_command_handler_);

    // PumpCommand
    services_->pump_command_handler_ =
      [this](const std::shared_ptr<buoy_interfaces::srv::PumpCommand::Request> request,
        std::shared_ptr<buoy_interfaces::srv::PumpCommand::Response> response)
      {
        RCLCPP_INFO_STREAM(
          ros_->node_->get_logger(),
          "[ROS 2 Spring Control] PumpCommand Received (" << request->duration_mins << " mins)");

        std::unique_lock lock(services_->command_mutex_);
        // if already running valve, don't allow pump
        // unless for some reason we need to turn pump off (shouldn't get in that state)
        if (services_->valve_command_) {
          if (request->duration_mins != request->OFF) {
            response->result.value = response->result.BUSY;

            RCLCPP_ERROR_STREAM(
              ros_->node_->get_logger(),
              "[ROS 2 Spring Control] PumpCommand cannot process" <<
                " while valve command is running...");

            return;
          }
        }

        if (request->duration_mins == request->OFF) {
          services_->command_duration_ = rclcpp::Duration(0, 0U);
          services_->pump_command_ = false;
          services_->new_pump_command_ = true;
        } else {
          float duration_mins{request->duration_mins};
          if (duration_mins > 10U) {
            duration_mins = std::min(
              std::max(duration_mins, static_cast<float>(10U)),
              static_cast<float>(10U));

            response->result.value = response->result.BAD_INPUT;

            RCLCPP_WARN_STREAM(
              ros_->node_->get_logger(),
              "[ROS 2 Spring Control] PumpCommand out of bounds -- clipped to 10mins");
          }

          services_->command_duration_ =
            rclcpp::Duration::from_seconds(static_cast<double>(duration_mins) * 60.0);

          services_->pump_command_ = true;
          services_->new_pump_command_ = true;
        }
      };
    services_->pump_command_service_ =
      ros_->node_->create_service<buoy_interfaces::srv::PumpCommand>(
      "pump_command",
      services_->pump_command_handler_);
  }

  void manageCommandTimer(SpringState & state)
  {
    static double init_x = 0.0;
    static rclcpp::Duration pump_prev{0, 0U};

    // open valve
    if (state.valve_command && !services_->command_watch_.Running()) {
      RCLCPP_INFO_STREAM(
        ros_->node_->get_logger(),
        "Valve open (" <<
          services_->command_duration_.seconds() << "s)");

      services_->command_watch_.Start(true);
      state.status.bits().ReliefValveStatus = 1U;

      init_x = state.range_finder;

      // turn pump on
    } else if (state.pump_command.isRunning() && !services_->command_watch_.Running()) {
      RCLCPP_INFO_STREAM(
        ros_->node_->get_logger(),
        "Pump on (" <<
          services_->command_duration_.seconds() << "s)");

      services_->command_watch_.Start(true);
      state.status.bits().PumpStatus = 1U;
      state.status.bits().PumpToggle = 1U;
      pump_prev = services_->command_watch_.ElapsedRunTime();

      init_x = state.range_finder;

    } else {
      // close valve
      if (state.valve_command &&
        services_->command_watch_.ElapsedRunTime() >= services_->command_duration_)
      {
        services_->command_watch_.Stop();
        state.status.bits().ReliefValveStatus = 0U;
        state.valve_command = false;

        RCLCPP_INFO_STREAM(
          ros_->node_->get_logger(),
          "Valve closed after (" <<
            services_->command_watch_.ElapsedRunTime().seconds() << "s)");

        igndbg << "piston moved: " << (state.range_finder - init_x) /
        (services_->command_watch_.ElapsedRunTime().nanoseconds() * IGN_NANO_TO_SEC) <<
          " m/s" << std::endl;
      }

      if (state.pump_command) {
        // turn pump off
        if (services_->command_watch_.ElapsedRunTime() >= services_->command_duration_) {
          services_->command_watch_.Stop();
          state.status.bits().PumpStatus = 0U;
          state.status.bits().PumpToggle = 0U;
          state.pump_command = false;

          RCLCPP_INFO_STREAM(
            ros_->node_->get_logger(),
            "Pump off after (" <<
              services_->command_watch_.ElapsedRunTime().seconds() << "s)");

          igndbg << "piston moved: " << (state.range_finder - init_x) /
          (services_->command_watch_.ElapsedRunTime().nanoseconds() * IGN_NANO_TO_SEC) <<
            " m/s" << std::endl;
        } else {
          // set pump toggle -- linear pump servo drives back and forth
          if (floor(services_->command_watch_.ElapsedRunTime().seconds()) !=
            floor(pump_prev.seconds()))
          {
            state.status.bits().PumpToggle += 1U;
            state.status.bits().PumpToggle %= 2U;
          }
        }
        pump_prev = services_->command_watch_.ElapsedRunTime();
      }
    }
  }

  void manageCommandState(SpringState & state)
  {
    if (state.valve_command.isFinished()) {
      std::unique_lock lock(services_->command_mutex_);
      services_->valve_command_ = false;
      state.status.bits().ReliefValveRequest = 0U;
      services_->command_duration_ = rclcpp::Duration(0, 0U);
      services_->command_watch_.Reset();
      state.valve_command.reset();
    }

    if (state.pump_command.isFinished()) {
      std::unique_lock lock(services_->command_mutex_);
      services_->pump_command_ = false;
      state.status.bits().PumpRequest = 0U;
      services_->command_duration_ = rclcpp::Duration(0, 0U);
      services_->command_watch_.Reset();
      state.pump_command.reset();
    }


    if (services_->new_pump_command_ || services_->new_valve_command_) {
      std::unique_lock lock(services_->command_mutex_);

      if (state.valve_command || state.pump_command) {
        state.valve_command = services_->valve_command_;
        state.status.bits().ReliefValveRequest =
          static_cast<uint8_t>(services_->new_valve_command_);

        state.pump_command = services_->pump_command_;
        state.status.bits().PumpRequest = static_cast<uint8_t>(services_->new_pump_command_);

        services_->command_duration_ =
          services_->command_duration_ +
          services_->command_watch_.ElapsedRunTime();

        if (state.valve_command) {
          RCLCPP_INFO_STREAM(
            ros_->node_->get_logger(),
            "Valve open (" <<
              services_->command_duration_.seconds() << "s)");

        } else if (state.pump_command) {
          RCLCPP_INFO_STREAM(
            ros_->node_->get_logger(),
            "Pump on (" <<
              services_->command_duration_.seconds() << "s)");
        }
      } else {
        state.valve_command = services_->valve_command_;
        state.status.bits().ReliefValveRequest =
          static_cast<uint8_t>(services_->new_valve_command_);

        state.pump_command = services_->pump_command_;
        state.status.bits().PumpRequest = static_cast<uint8_t>(services_->new_pump_command_);
      }

      services_->new_pump_command_ = services_->new_valve_command_ = false;
    }
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
    ignerr << "[ROS 2 Spring Control] Failed to initialize because [" <<
      model.Name(_ecm) << "] is not a model." << std::endl <<
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
  std::string topic = _sdf->Get<std::string>("topic", "spring_data").first;
  this->dataPtr->ros_->sc_pub_ =
    this->dataPtr->ros_->node_->create_publisher<buoy_interfaces::msg::SCRecord>(topic, 10);

  this->dataPtr->ros_->pub_rate_hz_ =
    _sdf->Get<double>("publish_rate", this->dataPtr->ros_->pub_rate_hz_).first;

  RCLCPP_INFO_STREAM(
    this->dataPtr->ros_->node_->get_logger(),
    "[ROS 2 Spring Control] Set publish_rate param from SDF: " <<
      this->dataPtr->ros_->pub_rate_hz_);

  this->dataPtr->ros_->node_->set_parameter(
    rclcpp::Parameter(
      "publish_rate",
      this->dataPtr->ros_->pub_rate_hz_));

  auto publish = [this]()
    {
      while (rclcpp::ok() && !this->dataPtr->stop_) {
        if (this->dataPtr->ros_->sc_pub_->get_subscription_count() <= 0) {continue;}

        // Only update and publish if not paused.
        if (this->dataPtr->paused_) {continue;}

        buoy_interfaces::msg::SCRecord sc_record;
        // high prio data access
        std::unique_lock next(this->dataPtr->next_access_mutex_);
        std::unique_lock data(this->dataPtr->data_mutex_);
        next.unlock();

        if (this->dataPtr->data_valid()) {
          this->dataPtr->ros_->sc_record_.seq_num =
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

  if (_info.paused) {
    return;
  }

  buoy_gazebo::SpringState spring_state;
  if (_ecm.EntityHasComponentType(
      this->dataPtr->jointEntity_,
      buoy_gazebo::components::SpringState().TypeId()))
  {
    auto spring_state_comp =
      _ecm.Component<buoy_gazebo::components::SpringState>(this->dataPtr->jointEntity_);

    spring_state = buoy_gazebo::SpringState(spring_state_comp->Data());
  } else {
    return;
  }

  this->dataPtr->manageCommandTimer(spring_state);
  this->dataPtr->manageCommandState(spring_state);

  spring_state.status.bits().TetherPowerStatus = 1U;

  _ecm.SetComponentData<buoy_gazebo::components::SpringState>(
    this->dataPtr->jointEntity_,
    spring_state);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void SpringController::PostUpdate(
  const ignition::gazebo::UpdateInfo & _info,
  const ignition::gazebo::EntityComponentManager & _ecm)
{
  IGN_PROFILE("SpringController::PostUpdate");

  this->dataPtr->paused_ = _info.paused;
  this->dataPtr->current_time_ = _info.simTime;

  if (_info.paused) {
    return;
  }

  if (!_ecm.EntityHasComponentType(
      this->dataPtr->jointEntity_,
      buoy_gazebo::components::SpringState().TypeId()))
  {
    // Pneumatic Spring hasn't updated values yet
    this->dataPtr->spring_data_valid_ = false;
    return;
  }

  auto spring_state_comp =
    _ecm.Component<buoy_gazebo::components::SpringState>(this->dataPtr->jointEntity_);

  // low prio data access
  std::unique_lock low(this->dataPtr->low_prio_mutex_);
  std::unique_lock next(this->dataPtr->next_access_mutex_);
  std::unique_lock data(this->dataPtr->data_mutex_);
  next.unlock();

  auto sec_nsec = ignition::math::durationToSecNsec(this->dataPtr->current_time_);

  this->dataPtr->ros_->sc_record_.header.stamp.sec = sec_nsec.first;
  this->dataPtr->ros_->sc_record_.header.stamp.nanosec = sec_nsec.second;
  this->dataPtr->ros_->sc_record_.range_finder = spring_state_comp->Data().range_finder;
  this->dataPtr->ros_->sc_record_.upper_psi = spring_state_comp->Data().upper_psi;
  this->dataPtr->ros_->sc_record_.lower_psi = spring_state_comp->Data().lower_psi;

  // Currently existing but unused fields by physical buoy -- the CTD sensor is not in service
  // this->dataPtr->sc_record_.epoch = spring_state_comp->Data().epoch_;
  // this->dataPtr->sc_record_.salinity = spring_state_comp->Data().salinity_;
  // this->dataPtr->sc_record_.temperature = spring_state_comp->Data().temperature_;

  // TODO(andermi) set the bits for this
  this->dataPtr->ros_->sc_record_.status = spring_state_comp->Data().status;

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
