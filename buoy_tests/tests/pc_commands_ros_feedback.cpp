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

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/transport/Node.hh>

#include <buoy_api/interface.hpp>
#include <buoy_api/examples/torque_control_policy.hpp>

#include <buoy_interfaces/msg/pc_record.hpp>


// Defines from Controller Firmware, behavior replicated here
#define TORQUE_CONSTANT 0.438   // 0.62 N-m/ARMS  0.428N-m/AMPS Flux Current
#define CURRENT_CMD_RATELIMIT 200  // A/second.  Set to zero to disable feature
#define TORQUE_CMD_TIMEOUT 2  // Torque Command Timeut, in secs. Set to zero to disable timeout
#define BIAS_CMD_TIMEOUT 10  // Bias Current Command Timeut, secs. Set to zero to disable timeout
#define DEFAULT_SCALE_FACTOR 1.0  // -RPM on Kollemogen is +RPM here and extension
#define MAX_SCALE_FACTOR 1.4
#define MIN_SCALE_FACTOR 0.5
#define DEFAULT_RETRACT_FACTOR 0.6
#define MAX_RETRACT_FACTOR 1.0
#define MIN_RETRACT_FACTOR 0.4
#define DEFAULT_BIASCURRENT 0.0  // Start with zero bias current
#define MAX_BIASCURRENT 20.0  // Max allowable winding bias current Magnitude that can be applied
#define MAX_WINDCURRENTLIMIT 35.0  // Winding Current Limit, Amps.  Limit on internal target
#define SC_RANGE_MIN 0.0  // Inches
#define SC_RANGE_MAX 80.0  // Inches
#define STOP_RANGE 10.0  // Inches from SC_RANGE_MIN and SC_RANGE_MAX to increase generator torque
// Max amount to modify RPM in determining WindingCurrentLimit near ends of stroke
#define MAX_RPM_ADJUSTMENT 5000.0


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

    this->set_params();

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

  double winding_current_limiter(const double & I)
  {
    double LimitedI = I;
    double AdjustedN = rpm_;
    const double RamPosition = (SC_RANGE_MAX - (range_finder_ / 0.0254));
    if (rpm_ >= 0.0) {  // Retracting
      const double min_region = SC_RANGE_MIN + STOP_RANGE;
      if (RamPosition < min_region) {
        // boost RPM by fraction of max adjustment to limit current
        AdjustedN += MAX_RPM_ADJUSTMENT * (min_region - RamPosition) / min_region;
      }
      const double CurrLim =
        -AdjustedN * 2.0 * MAX_WINDCURRENTLIMIT / 1000.0 + 385.0;  // Magic nums
      LimitedI = std::min(LimitedI, CurrLim);
    } else {  // Extending
      const double max_region = SC_RANGE_MAX - STOP_RANGE;
      if (RamPosition > max_region) {
        // boost RPM by fraction of max adjustment to limit current
        AdjustedN -= MAX_RPM_ADJUSTMENT * (RamPosition - max_region) / max_region;
      }
      const double CurrLim =
        -AdjustedN * 2.0 * MAX_WINDCURRENTLIMIT / 1000.0 - 385.0;  // Magic nums
      LimitedI = std::max(LimitedI, CurrLim);
    }

    LimitedI = std::min(std::max(LimitedI, -MAX_WINDCURRENTLIMIT), MAX_WINDCURRENTLIMIT);
    return LimitedI;
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


  void set_params()
  {
    this->declare_parameter("torque_constant", torque_policy_.Torque_constant);
    torque_policy_.Torque_constant = this->get_parameter("torque_constant").as_double();

    this->declare_parameter(
      "n_spec", std::vector<double>(
        torque_policy_.N_Spec.begin(),
        torque_policy_.N_Spec.end()));
    std::vector<double> temp_double_arr = this->get_parameter("n_spec").as_double_array();
    torque_policy_.N_Spec.assign(temp_double_arr.begin(), temp_double_arr.end());

    this->declare_parameter(
      "torque_spec", std::vector<double>(
        torque_policy_.Torque_Spec.begin(),
        torque_policy_.Torque_Spec.end()));
    temp_double_arr = this->get_parameter("torque_spec").as_double_array();
    torque_policy_.Torque_Spec.assign(temp_double_arr.begin(), temp_double_arr.end());

    torque_policy_.update_params();
    RCLCPP_INFO_STREAM(rclcpp::get_logger(this->get_name()), torque_policy_);
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
  std::unique_ptr<gz::sim::TestFixture> fixture{nullptr};
  std::unique_ptr<PCROSNode> node{nullptr};
  gz::sim::Entity buoyEntity{gz::sim::kNullEntity};

  virtual void SetUp()
  {
    // Skip debug messages to run faster
    gz::common::Console::SetVerbosity(3);

    // Setup fixture
    gz::sim::ServerConfig config;
    config.SetSdfFile("mbari_wec.sdf");
    config.SetUpdateRate(0.0);

    fixture = std::make_unique<gz::sim::TestFixture>(config);
    node = std::make_unique<PCROSNode>("pb_torque_controller");  // same name as example to grab
                                                                 // params

    fixture->
    OnConfigure(
      [&](
        const gz::sim::Entity & _worldEntity,
        const std::shared_ptr<const sdf::Element> &,
        gz::sim::EntityComponentManager & _ecm,
        gz::sim::EventManager &)
      {
        auto world = gz::sim::World(_worldEntity);

        buoyEntity = world.ModelByName(_ecm, "MBARI_WEC_ROS");
        EXPECT_NE(gz::sim::kNullEntity, buoyEntity);
      }).
    OnPostUpdate(
      [&](
        const gz::sim::UpdateInfo &,
        const gz::sim::EntityComponentManager & _ecm)
      {
        iterations++;

        auto pose = gz::sim::worldPose(buoyEntity, _ecm);

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
  expected_wind_curr = node->winding_current_limiter(expected_wind_curr);
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
  expected_wind_curr = node->winding_current_limiter(expected_wind_curr);
  EXPECT_GT(node->wind_curr_, expected_wind_curr - 0.2);
  EXPECT_LT(node->wind_curr_, expected_wind_curr + 0.2);

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
  EXPECT_LT(node->range_finder_, 2.03);  // meters

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
  EXPECT_NE(gz::sim::kNullEntity, buoyEntity);
}
