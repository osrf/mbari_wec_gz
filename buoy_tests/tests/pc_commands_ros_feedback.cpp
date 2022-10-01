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

#include <buoy_api/interface.hpp>

#include <gtest/gtest.h>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/TestFixture.hh>
#include <ignition/transport/Node.hh>

#include <buoy_interfaces/msg/pc_record.hpp>
#include <buoy_api/examples/torque_control_policy.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>


// NOLINTNEXTLINE
using namespace std::chrono;

constexpr double INCHES_TO_METERS{0.0254};

class PCROSNode final : public buoy_api::Interface<PCROSNode>
{
public:
  rclcpp::Clock::SharedPtr clock_{nullptr};

  float rpm_{0.0};
  float wind_curr_{0.0F};
  float bias_curr_{0.0F};
  float scale_{1.0};
  float retract_{0.6};
  float range_finder_{0.0F};

  PBTorqueControlPolicy torque_policy_;

  PCWindCurrServiceResponseFuture pc_wind_curr_response_future_;
  PCBiasCurrServiceResponseFuture pc_bias_curr_response_future_;
  PCScaleServiceResponseFuture pc_scale_response_future_;
  PCRetractServiceResponseFuture pc_retract_response_future_;

  explicit PCROSNode(const std::string & node_name)
  : buoy_api::Interface<PCROSNode>(node_name)
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

private:
  friend CRTP;  // syntactic sugar (see https://stackoverflow.com/a/58435857/9686600)

  void power_callback(const buoy_interfaces::msg::PCRecord & data)
  {
    rpm_ = data.rpm;
    wind_curr_ = data.wcurrent;
    bias_curr_ = data.bias_current;
    scale_ = data.scale;
    retract_ = data.retract;
  }

  void spring_callback(const buoy_interfaces::msg::SCRecord & data)
  {
    range_finder_ = data.range_finder;
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

        buoyEntity = world.ModelByName(_ecm, "MBARI_WEC_ROS");
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
TEST_F(BuoyPCTests, PCCommandsInROSFeedback)
{
  int preCmdIterations{15000}, feedbackCheckIterations{100};

  EXPECT_EQ(0U, iterations);

  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(
    static_cast<int>(node->clock_->now().seconds()),
    static_cast<int>(iterations / 1000.0F));

  ///////////////////////////////////////////
  // Check Default Winding Current Damping
  fixture->Server()->Run(true /*blocking*/, feedbackCheckIterations, false /*paused*/);
  EXPECT_EQ(feedbackCheckIterations, iterations);

  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(
    static_cast<int>(node->clock_->now().seconds()),
    static_cast<int>(iterations / 1000.0F));

  std::cout << node->torque_policy_ << std::endl;

  double expected_wind_curr =
    node->torque_policy_.WindingCurrentTarget(
    node->rpm_,
    node->scale_,
    node->retract_) + node->bias_curr_;
  EXPECT_GT(node->wind_curr_, expected_wind_curr - 0.1);
  EXPECT_LT(node->wind_curr_, expected_wind_curr + 0.1);

  // Run simulation server and wait for piston to settle
  fixture->Server()->Run(true /*blocking*/, preCmdIterations, false /*paused*/);
  EXPECT_EQ(preCmdIterations + feedbackCheckIterations, iterations);

  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(
    static_cast<int>(node->clock_->now().seconds()),
    static_cast<int>(iterations / 1000.0F));

  ///////////////////////////////////////////
  // Winding Current
  const float wc{12.345F};
  EXPECT_NE(node->wind_curr_, wc);

  // Now send wind curr command
  node->pc_wind_curr_response_future_ = node->send_pc_wind_curr_command(wc);
  ASSERT_TRUE(node->pc_wind_curr_response_future_.valid());
  node->pc_wind_curr_response_future_.wait();
  EXPECT_EQ(
    node->pc_wind_curr_response_future_.get()->result.value,
    node->pc_wind_curr_response_future_.get()->result.OK);

  // Run a bit for wind curr command to process
  fixture->Server()->Run(true /*blocking*/, feedbackCheckIterations, false /*paused*/);
  EXPECT_EQ(preCmdIterations + 2 * feedbackCheckIterations, iterations);

  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(
    static_cast<int>(node->clock_->now().seconds()),
    static_cast<int>(iterations / 1000.0F));

  EXPECT_GT(node->wind_curr_, wc - 0.1F);
  EXPECT_LT(node->wind_curr_, wc + 0.1F);

  ///////////////////////////////////////////
  // Scale
  const float scale{1.23F};
  EXPECT_NE(node->scale_, scale);

  // Now send scale command
  node->pc_scale_response_future_ = node->send_pc_scale_command(scale);
  ASSERT_TRUE(node->pc_scale_response_future_.valid());
  node->pc_scale_response_future_.wait();
  EXPECT_EQ(
    node->pc_scale_response_future_.get()->result.value,
    node->pc_scale_response_future_.get()->result.OK);

  // Run a bit for scale command to process
  fixture->Server()->Run(true /*blocking*/, feedbackCheckIterations, false /*paused*/);
  EXPECT_EQ(preCmdIterations + 3 * feedbackCheckIterations, iterations);

  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(
    static_cast<int>(node->clock_->now().seconds()),
    static_cast<int>(iterations / 1000.0F));

  EXPECT_GT(node->scale_, scale - 0.01F);
  EXPECT_LT(node->scale_, scale + 0.01F);

  ///////////////////////////////////////////
  // Retract
  const float retract{0.75F};
  EXPECT_NE(node->retract_, retract);

  // Now send retract command
  node->pc_retract_response_future_ = node->send_pc_retract_command(retract);
  ASSERT_TRUE(node->pc_retract_response_future_.valid());
  node->pc_retract_response_future_.wait();
  EXPECT_EQ(
    node->pc_retract_response_future_.get()->result.value,
    node->pc_retract_response_future_.get()->result.OK);

  // Run a bit for retract command to process
  fixture->Server()->Run(true /*blocking*/, feedbackCheckIterations, false /*paused*/);
  EXPECT_EQ(preCmdIterations + 4 * feedbackCheckIterations, iterations);

  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(
    static_cast<int>(node->clock_->now().seconds()),
    static_cast<int>(iterations / 1000.0F));

  EXPECT_GT(node->retract_, retract - 0.01F);
  EXPECT_LT(node->retract_, retract + 0.01F);

  ///////////////////////////////////////////////////////
  // Check Return to Default Winding Current Damping
  int torque_timeout_iterations{2000};
  fixture->Server()->Run(
    true /*blocking*/,
    torque_timeout_iterations - 2 * feedbackCheckIterations, false /*paused*/);
  EXPECT_EQ(
    preCmdIterations + 2 * feedbackCheckIterations + torque_timeout_iterations,
    iterations);

  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(
    static_cast<int>(node->clock_->now().seconds()),
    static_cast<int>(iterations / 1000.0F));

  expected_wind_curr =
    node->torque_policy_.WindingCurrentTarget(
    node->rpm_,
    node->scale_,
    node->retract_) + node->bias_curr_;
  EXPECT_GT(node->wind_curr_, expected_wind_curr - 0.1);
  EXPECT_LT(node->wind_curr_, expected_wind_curr + 0.1);

  ///////////////////////////////////////////
  // Bias Current
  const float bc{7.89F};
  EXPECT_NE(node->bias_curr_, bc);

  // Now send bias curr command
  node->pc_bias_curr_response_future_ = node->send_pc_bias_curr_command(bc);
  ASSERT_TRUE(node->pc_bias_curr_response_future_.valid());
  node->pc_bias_curr_response_future_.wait();
  EXPECT_EQ(
    node->pc_bias_curr_response_future_.get()->result.value,
    node->pc_bias_curr_response_future_.get()->result.OK);

  // Run a bit for bias curr command to move piston
  int bias_curr_iterations{9000}, bias_curr_timeout_iterations{10000};
  fixture->Server()->Run(true /*blocking*/, bias_curr_iterations, false /*paused*/);
  EXPECT_EQ(
    preCmdIterations + 2 * feedbackCheckIterations +
    torque_timeout_iterations + bias_curr_iterations,
    iterations);

  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(
    static_cast<int>(node->clock_->now().seconds()),
    static_cast<int>(iterations / 1000.0F));

  EXPECT_GT(node->bias_curr_, bc - 0.1F);
  EXPECT_LT(node->bias_curr_, bc + 0.1F);

  // TODO(andermi) fix this comparison when motor mode is fixed
  EXPECT_LT(node->range_finder_, 1.0);  // meters

  // Let bias curr command timeout
  fixture->Server()->Run(
    true /*blocking*/,
    bias_curr_timeout_iterations - bias_curr_iterations + feedbackCheckIterations,
    false /*paused*/);
  EXPECT_EQ(
    preCmdIterations + 3 * feedbackCheckIterations +
    torque_timeout_iterations + bias_curr_timeout_iterations,
    iterations);

  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(
    static_cast<int>(node->clock_->now().seconds()),
    static_cast<int>(iterations / 1000.0F));

  // check default bias curr
  EXPECT_GT(node->bias_curr_, -0.1F);
  EXPECT_LT(node->bias_curr_, 0.1F);

  ///////////////////////////////////////////
  // Stop spinning node
  node->stop();

  // Sanity check that the test ran
  EXPECT_NE(ignition::gazebo::kNullEntity, buoyEntity);
}
