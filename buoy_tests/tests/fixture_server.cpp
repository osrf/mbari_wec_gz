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

#include <rclcpp/rclcpp.hpp>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/TestFixture.hh>
#include <ignition/transport/Node.hh>

#include <buoy_tests/srv/run_server.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>


TEST(BuoyTests, RunServer)
{
  // Skip debug messages to run faster
  ignition::common::Console::SetVerbosity(3);

  // Setup fixture
  ignition::gazebo::ServerConfig config;
  config.SetSdfFile("mbari_wec.sdf");
  config.SetUpdateRate(0.0);

  size_t iterations{0U};

  ignition::gazebo::Entity buoyEntity{ignition::gazebo::kNullEntity};
  std::unique_ptr<ignition::gazebo::TestFixture> fixture =
    std::make_unique<ignition::gazebo::TestFixture>(config);
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

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("gz_fixture_server");

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

  rclcpp::spin(node);
  RCLCPP_INFO(rclcpp::get_logger("run_server"), "Shutting down test server.");
}
