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

#include <ignition/common/Console.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/TestFixture.hh>
#include <ignition/transport/Node.hh>

#include <memory>

//////////////////////////////////////////////////
TEST(BuoyTests, NoInputs)
{
  // Skip debug messages to run faster
  ignition::common::Console::SetVerbosity(3);

  // Setup fixture
  ignition::gazebo::ServerConfig config;
  config.SetSdfFile("mbari_wec.sdf");
  config.SetUpdateRate(0.0);

  ignition::gazebo::TestFixture fixture(config);

  int iterations{0};
  ignition::gazebo::Entity buoyEntity{ignition::gazebo::kNullEntity};

  fixture.
  OnConfigure(
    [&](const ignition::gazebo::Entity & _worldEntity,
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

  // Run simulation server
  int targetIterations{15000};
  fixture.Server()->Run(true /*blocking*/, targetIterations, false /*paused*/);

  // Sanity check that the test ran
  EXPECT_EQ(targetIterations, iterations);
  EXPECT_NE(ignition::gazebo::kNullEntity, buoyEntity);
}
