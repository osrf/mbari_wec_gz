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

#include <memory>

#include <gz/common/Console.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/transport/Node.hh>

//////////////////////////////////////////////////
TEST(BuoyTests, NoInputs)
{
  // Skip debug messages to run faster
  gz::common::Console::SetVerbosity(3);

  // Setup fixture
  gz::sim::ServerConfig config;
  config.SetSdfFile("mbari_wec_test.sdf");
  config.SetUpdateRate(0.0);

  gz::sim::TestFixture fixture(config);

  int iterations{0};
  gz::sim::Entity buoyEntity{gz::sim::kNullEntity};

  fixture.
  OnConfigure(
    [&](const gz::sim::Entity & _worldEntity,
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

  // Run simulation server
  int targetIterations{15000};
  fixture.Server()->Run(true /*blocking*/, targetIterations, false /*paused*/);

  // Sanity check that the test ran
  EXPECT_EQ(targetIterations, iterations);
  EXPECT_NE(gz::sim::kNullEntity, buoyEntity);
}
