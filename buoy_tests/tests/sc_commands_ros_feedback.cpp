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

#include <buoy_msgs/interface.hpp>

#include <gtest/gtest.h>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/TestFixture.hh>
#include <ignition/transport/Node.hh>

#include <chrono>
#include <memory>
#include <string>

// NOLINTNEXTLINE
using namespace std::chrono;

constexpr double INCHES_TO_METERS{0.0254};

class SCROSNode final : public buoy_msgs::Interface<SCROSNode>
{
public:
  float range_finder_{0.0F};
  uint16_t status_{0U};

  float pre_valve_range_finder_{0.0F}, post_valve_range_finder_{0.0F};

  float pre_pump_range_finder_{0.0F}, post_pump_range_finder_{0.0F};
  size_t pump_toggle_counter_{0U};

  ValveServiceResponseFuture valve_response_future_;
  PumpServiceResponseFuture pump_response_future_;

  explicit SCROSNode(const std::string & node_name)
  : buoy_msgs::Interface<SCROSNode>(node_name)
  {
    set_parameter(
      rclcpp::Parameter(
        "use_sim_time",
        true));

    node_ = std::shared_ptr<SCROSNode>(this, [](SCROSNode *) {});  // null deleter
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    stop_ = false;

    auto spin = [this]()
      {
        while (rclcpp::ok() && !stop_) {
          executor_->spin_once();
        }
      };
    thread_executor_spin_ = std::thread(spin);
  }

  ~SCROSNode()
  {
    stop_ = true;
    if (executor_) {
      executor_->cancel();
    }
    thread_executor_spin_.join();
  }

  void stop()
  {
    stop_ = true;
  }

  void send_valve_command()
  {
    auto request = std::make_shared<buoy_msgs::srv::ValveCommand::Request>();
    request->duration_sec = 5U;

    valve_response_future_ = valve_client_->async_send_request(request, valve_callback);
  }

  void send_pump_command()
  {
    auto request = std::make_shared<buoy_msgs::srv::PumpCommand::Request>();
    request->duration_sec = 20U;

    pump_response_future_ = pump_client_->async_send_request(request, pump_callback);
  }

private:
  friend CRTP;  // syntactic sugar (see https://stackoverflow.com/a/58435857/9686600)

  void spring_callback(const buoy_msgs::msg::SCRecord & data)
  {
    range_finder_ = data.range_finder;
    status_ = data.status;
  }

  std::thread thread_executor_spin_;
  std::atomic<bool> stop_{false};
  rclcpp::Node::SharedPtr node_{nullptr};
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_{nullptr};
};


class BuoySCTests : public ::testing::Test
{
protected:
  int iterations{0};
  std::unique_ptr<ignition::gazebo::TestFixture> fixture{nullptr};
  std::unique_ptr<SCROSNode> node{nullptr};
  ignition::gazebo::Entity buoyEntity{ignition::gazebo::kNullEntity};

  virtual void SetUp()
  {
    // Skip debug messages to run faster
    ignition::common::Console::SetVerbosity(3);

    // Setup fixture
    ignition::gazebo::ServerConfig config;
    config.SetSdfFile("mbari_wec.sdf");
    config.SetUpdateRate(0.0);

    fixture = std::make_unique<ignition::gazebo::TestFixture>(config);
    node = std::make_unique<SCROSNode>("test_sc_ros");

    fixture->
    OnConfigure(
      [&](
        const ignition::gazebo::Entity & _worldEntity,
        const std::shared_ptr<const sdf::Element> &,
        ignition::gazebo::EntityComponentManager & _ecm,
        ignition::gazebo::EventManager &)
      {
        auto world = ignition::gazebo::World(_worldEntity);

        buoyEntity = world.ModelByName(_ecm, "MBARI_WEC");
        EXPECT_NE(ignition::gazebo::kNullEntity, buoyEntity);
      }).
    OnPostUpdate(
      [&](
        const ignition::gazebo::UpdateInfo &,
        const ignition::gazebo::EntityComponentManager & _ecm)
      {
        iterations++;

        auto pose = ignition::gazebo::worldPose(buoyEntity, _ecm);

        // Expect buoy to stay more or less in the same place horizontally.
        EXPECT_LT(-0.001, pose.Pos().X());
        EXPECT_GT(0.001, pose.Pos().X());

        EXPECT_LT(-0.001, pose.Pos().Y());
        EXPECT_GT(0.001, pose.Pos().Y());

        // Buoy starts at Z == -2.0
        // It's slightly out of the water, so it falls ~1m
        EXPECT_LT(-3.020, pose.Pos().Z());

        // And it bounces back up beyond its starting point
        EXPECT_GT(-1.89, pose.Pos().Z());
      }).
    Finalize();
  }
};

//////////////////////////////////////////////////
TEST_F(BuoySCTests, SCValveROS)
{
  // Run simulation server
  int targetIterations{15000};
  fixture->Server()->Run(true /*blocking*/, targetIterations, false /*paused*/);

  EXPECT_EQ(targetIterations, iterations);
  rclcpp::Clock::SharedPtr clock = node->get_clock();
  EXPECT_EQ(static_cast<int>(clock->now().seconds()), static_cast<int>(iterations / 1000.0F));

  node->pre_valve_range_finder_ = node->range_finder_;
  node->send_valve_command();
  ASSERT_TRUE(node->valve_response_future_.valid());
  node->valve_response_future_.wait();
  EXPECT_EQ(
    node->valve_response_future_.get()->result.value,
    node->valve_response_future_.get()->result.OK);

  fixture->Server()->Run(true /*blocking*/, targetIterations, false /*paused*/);

  EXPECT_EQ(2.0F * targetIterations, iterations);
  EXPECT_EQ(static_cast<int>(clock->now().seconds()), static_cast<int>(iterations / 1000.0F));

  node->post_valve_range_finder_ = node->range_finder_;

  EXPECT_GT(
    node->post_valve_range_finder_,
    node->pre_valve_range_finder_ + 0.8F /*inch per sec*/ * INCHES_TO_METERS * 5.0F /*seconds*/) <<
    "Piston should extend 1 inch/sec for 5 seconds";

  EXPECT_LT(
    node->post_valve_range_finder_,
    node->pre_valve_range_finder_ + 1.2F /*inch per sec*/ * INCHES_TO_METERS * 5.0F /*seconds*/) <<
    "Piston should extend 1 inch/sec for 5 seconds";

  node->stop();

  // Sanity check that the test ran
  EXPECT_NE(ignition::gazebo::kNullEntity, buoyEntity);
}

//////////////////////////////////////////////////
TEST_F(BuoySCTests, SCPumpROS)
{
  // Run simulation server
  int targetIterations{15000};
  fixture->Server()->Run(true /*blocking*/, targetIterations, false /*paused*/);

  EXPECT_EQ(targetIterations, iterations);
  rclcpp::Clock::SharedPtr clock = node->get_clock();
  EXPECT_EQ(static_cast<int>(clock->now().seconds()), static_cast<int>(iterations / 1000.0F));

  node->pre_pump_range_finder_ = node->range_finder_;
  node->send_pump_command();
  ASSERT_TRUE(node->pump_response_future_.valid());
  node->pump_response_future_.wait();
  EXPECT_EQ(
    node->pump_response_future_.get()->result.value,
    node->pump_response_future_.get()->result.OK);

  fixture->Server()->Run(true /*blocking*/, targetIterations, false /*paused*/);

  EXPECT_EQ(2.0F * targetIterations, iterations);
  EXPECT_EQ(static_cast<int>(clock->now().seconds()), static_cast<int>(iterations / 1000.0F));

  node->post_pump_range_finder_ = node->range_finder_;

  EXPECT_GT(
    node->post_pump_range_finder_,
    node->pre_pump_range_finder_ - 2.2F /*inches per minute*/ * INCHES_TO_METERS *
    20.0F /*seconds*/ * 1.0F /*minute*/ / 60.0F /*seconds*/) << \
    "Piston should retract 2 inches/min for 20 seconds";

  EXPECT_LT(
    node->post_pump_range_finder_,
    node->pre_pump_range_finder_ - 1.8F /*inches per minute*/ * INCHES_TO_METERS *
    20.0F /*seconds*/ * 1.0F /*minute*/ / 60.0F /*seconds*/) << \
    "Piston should retract 2 inches/min for 20 seconds";

  node->stop();

  // Sanity check that the test ran
  EXPECT_NE(ignition::gazebo::kNullEntity, buoyEntity);
}
