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

#include <buoy_msgs/msg/pc_record.hpp>
#include <buoy_examples/torque_control_policy.hpp>

#include <chrono>
#include <memory>
#include <string>


// NOLINTNEXTLINE
using namespace std::chrono;

constexpr double INCHES_TO_METERS{0.0254};

class PCROSNode final : public buoy_msgs::Interface<PCROSNode>
{
public:
  rclcpp::Clock::SharedPtr clock_{nullptr};

  float wind_curr_{0.0F};

  PCWindCurrServiceResponseFuture pc_wind_curr_response_future_;

  explicit PCROSNode(const std::string & node_name)
  : buoy_msgs::Interface<PCROSNode>(node_name)
  {
    set_parameter(
      rclcpp::Parameter(
        "use_sim_time",
        true));

    node_ = std::shared_ptr<PCROSNode>(this, [](PCROSNode *) {});  // null deleter
    clock_ = this->get_clock();

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

  ~PCROSNode()
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

  void send_pc_wind_curr_command(const float & wind_curr)
  {
    auto request = std::make_shared<buoy_msgs::srv::PCWindCurrCommand::Request>();
    request->wind_curr = wind_curr;

    pc_wind_curr_response_future_ = pc_wind_curr_client_->async_send_request(
      request,
      pc_wind_curr_callback);
  }

private:
  friend CRTP;  // syntactic sugar (see https://stackoverflow.com/a/58435857/9686600)

  void power_callback(const buoy_msgs::msg::PCRecord & data)
  {
    wind_curr_ = data.wcurrent;
  }

  std::thread thread_executor_spin_;
  std::atomic<bool> stop_{false};
  rclcpp::Node::SharedPtr node_{nullptr};
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_{nullptr};
};


class BuoyPCTests : public ::testing::Test
{
protected:
  int iterations{0};
  std::unique_ptr<ignition::gazebo::TestFixture> fixture{nullptr};
  std::unique_ptr<PCROSNode> node{nullptr};
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
    node = std::make_unique<PCROSNode>("test_pc_ros");

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
TEST_F(BuoyPCTests, PCWindCurrROS)
{
  int preCmdIterations{15000}, feedbackCheckIterations{1000};

  // Run simulation server and wait for piston to settle
  fixture->Server()->Run(true /*blocking*/, preCmdIterations, false /*paused*/);
  EXPECT_EQ(preCmdIterations, iterations);
  EXPECT_EQ(
    static_cast<int>(node->clock_->now().seconds()),
    static_cast<int>(iterations / 1000.0F));

  const float wc{12.345F};
  EXPECT_NE(node->wind_curr_, wc);

  // Now send wind curr command
  node->send_pc_wind_curr_command(wc);
  ASSERT_TRUE(node->pc_wind_curr_response_future_.valid());
  node->pc_wind_curr_response_future_.wait();
  EXPECT_EQ(
    node->pc_wind_curr_response_future_.get()->result.value,
    node->pc_wind_curr_response_future_.get()->result.OK);

  // Run a bit for wind curr command to process
  fixture->Server()->Run(true /*blocking*/, feedbackCheckIterations, false /*paused*/);
  EXPECT_EQ(preCmdIterations + feedbackCheckIterations, iterations);
  EXPECT_EQ(
    static_cast<int>(node->clock_->now().seconds()),
    static_cast<int>(iterations / 1000.0F));

  EXPECT_GT(node->wind_curr_, wc - 0.1F);
  EXPECT_LT(node->wind_curr_, wc + 0.1F);

  // Run to allow ___ command to finish
  fixture->Server()->Run(true /*blocking*/, feedbackCheckIterations, false /*paused*/);
  EXPECT_EQ(preCmdIterations + 2 * feedbackCheckIterations, iterations);
  EXPECT_EQ(
    static_cast<int>(node->clock_->now().seconds()),
    static_cast<int>(iterations / 1000.0F));

  // Stop spinning node
  node->stop();

  // Sanity check that the test ran
  EXPECT_NE(ignition::gazebo::kNullEntity, buoyEntity);
}
