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

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gz/common/Console.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/TestFixture.hh>

#include <buoy_tests/srv/run_server.hpp>

#include <rclcpp/rclcpp.hpp>


// NOLINTNEXTLINE
using namespace std::chrono;

TEST(BuoyTests, RunServer)
{
  // Skip debug messages to run faster
  gz::common::Console::SetVerbosity(4);

  // Setup fixture
  gz::sim::ServerConfig config;
  config.SetSdfFile("mbari_wec_sinusoidal_piston.sdf");
  config.SetUpdateRate(0.0);

  size_t iterations{0U};

  gz::sim::Entity buoyEntity{gz::sim::kNullEntity};
  std::unique_ptr<gz::sim::TestFixture> fixture =
    std::make_unique<gz::sim::TestFixture>(config);
  fixture->
  OnConfigure(
    [&](
      const gz::sim::Entity & _worldEntity,
      const std::shared_ptr<const sdf::Element> &,
      gz::sim::EntityComponentManager & _ecm,
      gz::sim::EventManager &)
    {
      auto world = gz::sim::World(_worldEntity);

      buoyEntity = world.ModelByName(_ecm, "MBARI_WEC_SINUSOIDAL_PISTON");
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

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("gz_fixture_server");
  std::atomic<bool> stop = false;

  rclcpp::Service<buoy_tests::srv::RunServer>::SharedPtr service =
    node->create_service<buoy_tests::srv::RunServer>(
    "run_server",
    [&](const std::shared_ptr<buoy_tests::srv::RunServer::Request> request,
    std::shared_ptr<buoy_tests::srv::RunServer::Response> response)
    {
      if (request->iterations == 0U) {
        RCLCPP_INFO(
          rclcpp::get_logger("run_server"),
          "Incoming request to shutdown");
        stop = true;
        rclcpp::shutdown();
        response->success = true;
        return;
      } else {
        RCLCPP_INFO_STREAM(
          rclcpp::get_logger("run_server"),
          "Incoming request\niterations: " << request->iterations);
      }

      const size_t initial_iterations = iterations;

      static const bool blocking = true;
      static const bool paused = false;
      response->success = fixture->Server()->Run(blocking, request->iterations, paused);
      response->iterations = iterations - initial_iterations;

      EXPECT_EQ(iterations - initial_iterations, request->iterations);

      RCLCPP_INFO_STREAM(
        rclcpp::get_logger("run_server"),
        "Response: " << std::boolalpha << response->success << std::noboolalpha);
    }
    );

  RCLCPP_INFO(rclcpp::get_logger("run_server"), "Ready to run test server.");

  // rclcpp::spin(node);
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);

  rclcpp::Rate rate(50.0);
  while (rclcpp::ok() && !stop) {
    executor->spin_once();
    rate.sleep();
  }

  if (executor) {
    executor->cancel();
  }

  std::this_thread::sleep_for(1s);  // needed for launch_test to know we shut down

  RCLCPP_INFO(rclcpp::get_logger("run_server"), "Shutting down test server.");
}
