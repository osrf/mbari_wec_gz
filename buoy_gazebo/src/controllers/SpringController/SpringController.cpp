#include "SpringController.hpp"
#include "PolytropicPneumaticSpring/SpringState.hpp"

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>

#include <buoy_msgs/msg/sc_record.hpp>


struct buoy_gazebo::SpringControllerPrivate
{
  ignition::gazebo::Entity entity_;
  std::chrono::steady_clock::duration current_time_;
  rclcpp::Node::SharedPtr rosnode_{nullptr};
  std::thread thread_executor_spin_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  bool stop_{false};
  rclcpp::Publisher<buoy_msgs::msg::SCRecord>::SharedPtr sc_pub_;
  const buoy_gazebo::SpringStateComponent * spring_state_comp;
  int16_t seq_num{0};
};


IGNITION_ADD_PLUGIN(
    buoy_gazebo::SpringController,
    ignition::gazebo::System,
    buoy_gazebo::SpringController::ISystemConfigure,
    buoy_gazebo::SpringController::ISystemPostUpdate)

using namespace buoy_gazebo;



//////////////////////////////////////////////////
SpringController::SpringController()
: dataPtr(std::make_unique<SpringControllerPrivate>())
{
}

SpringController::~SpringController()
{
  // Stop controller manager thread
  this->dataPtr->stop_ = true;
  // this->dataPtr->executor_->remove_node(this->dataPtr->rosnode_);
  this->dataPtr->executor_->cancel();
  this->dataPtr->thread_executor_spin_.join();
}

void SpringController::Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &)
{
  // Make sure the controller is attached to a valid model
  const auto model = ignition::gazebo::Model(_entity);
  if (!model.Valid(_ecm))
  {
    // TODO(andermi) this should not use rclcpp logging
    // RCLCPP_ERROR(
    //  this->dataPtr->rosnode_->get_logger(),
    //  "[ROS 2 Spring Control] Failed to initialize because [%s] (Entity=%lu)] is not a model."
    //  "Please make sure that ROS 2 Spring Control is attached to a valid model.",
    //  model.Name(_ecm).c_str(), _entity);
    return;
  }
  
  this->dataPtr->entity_ = _entity;

  // controller scoped name
  auto scoped_name = ignition::gazebo::scopedName(this->dataPtr->entity_, _ecm, "/", false);

  // ROS node
  std::vector<std::string> arguments = {"--ros-args"};
  auto ns = _sdf->Get<std::string>("namespace", scoped_name).first;
  if (ns.empty() || ns[0] != '/')
  {
    ns = '/' + ns;
  }
  std::string ns_arg = std::string("__ns:=") + ns;
  arguments.push_back(RCL_REMAP_FLAG);
  arguments.push_back(ns_arg);
  std::vector<const char *> argv;
  for (const auto & arg : arguments)
  {
    argv.push_back(reinterpret_cast<const char *>(arg.data()));
  }
  if (!rclcpp::ok())
  {
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
    this->dataPtr->rosnode_->get_logger(), "[ROS 2 Spring Control] Setting up controller for [" <<
      model.Name(_ecm) << "] (Entity=" << _entity << ")].");

  // Publisher
  auto topic = _sdf->Get<std::string>("topic", "sc_record").first;
  this->dataPtr->sc_pub_ = this->dataPtr->rosnode_->create_publisher<buoy_msgs::msg::SCRecord>(topic, 10);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void SpringController::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  const auto model = ignition::gazebo::Model(this->dataPtr->entity_);
  RCLCPP_INFO_STREAM(
    this->dataPtr->rosnode_->get_logger(), "[ROS 2 Spring Control] filling spring data [" <<
      model.Name(_ecm) << "] (Entity=" << this->dataPtr->entity_ << ")].");
  this->dataPtr->current_time_ = _info.simTime;
  //if (this->dataPtr->sc_pub_->get_subscription_count() <= 0)
  //  return;
  
  auto sec_nsec = ignition::math::durationToSecNsec(_info.simTime);
  
  // Create new component for this entitiy in ECM (if it doesn't already exist)
  this->dataPtr->spring_state_comp = _ecm.Component<buoy_gazebo::SpringStateComponent>(this->dataPtr->entity_);
  if (this->dataPtr->spring_state_comp == nullptr)
    // Pneumatic Spring hasn't updated values yet
    return;
  
  buoy_msgs::msg::SCRecord sc_record;
  sc_record.header.stamp.sec = sec_nsec.first;
  sc_record.header.stamp.nanosec = sec_nsec.second;
  sc_record.seq_num = this->dataPtr->seq_num++;
  

  sc_record.load_cell = this->dataPtr->spring_state_comp->Data().load_cell;
  sc_record.range_finder = this->dataPtr->spring_state_comp->Data().range_finder;
  sc_record.upper_psi = this->dataPtr->spring_state_comp->Data().upper_psi;
  sc_record.lower_psi = this->dataPtr->spring_state_comp->Data().lower_psi;
  // sc_record.epoch = this->dataPtr->epoch_;
  // sc_record.salinity = this->dataPtr->salinity_;
  // sc_record.temperature = this->dataPtr->temperature_;
  // sc_record.status = this->dataPtr->status_;

  this->dataPtr->sc_pub_->publish(sc_record);
}



