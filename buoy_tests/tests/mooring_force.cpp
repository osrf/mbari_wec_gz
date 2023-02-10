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

#include <gz/common/Console.hh>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

#include <gz/transport/Node.hh>

//////////////////////////////////////////////////
TEST(BuoyTests, MooringForce)
{
  // Skip debug messages to run faster TODO change to 3 for PR
  gz::common::Console::SetVerbosity(4);

  // Setup fixture
  gz::sim::ServerConfig config;
  config.SetSdfFile("mbari_wec.sdf");

  gz::sim::TestFixture fixture(config);

  // Anchor pose, defined in SDF parameter anchor_position to MooringForce
  // plugin in buoy_description/models/mbari_wec/model.sdf.em
  gz::math::Vector3d anchorPos(80.0, 100.0, -84.0);

  // Expect buoy to stay within a horizontal radius of anchor, the
  // distance calculated from the vertical depth V and the
  // hypotenuse that is the length of the mooring line L.
  const double DEPTH = 82.0;
  const double MOORING_LENGTH = 160.0;
  //double maxRadius = sqrt(MOORING_LENGTH * MOORING_LENGTH - DEPTH * DEPTH);
  double maxRadius = MOORING_LENGTH;

  int iterations{0};

  gz::sim::Entity buoyLinkEnt{gz::sim::kNullEntity};
  gz::sim::Link buoyLink;
  gz::math::Pose3d buoyLinkPose;

  fixture.
  OnConfigure(
    [&](const gz::sim::Entity & _worldEntity,
    const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager &)
    {
      auto world = gz::sim::World(_worldEntity);

      gz::sim::Entity buoyModelEnt = world.ModelByName(_ecm,
        "MBARI_WEC_ROS");
      EXPECT_NE(gz::sim::kNullEntity, buoyModelEnt);
      auto buoyModel = gz::sim::Model(buoyModelEnt);

      // Get the link on the water surface
      buoyLinkEnt = buoyModel.LinkByName(_ecm, "Buoy");
      EXPECT_NE(gz::sim::kNullEntity, buoyLinkEnt);
      buoyLink = gz::sim::Link(buoyLinkEnt);
      ASSERT_TRUE(buoyLink.Valid(_ecm));
    }).
  OnPreUpdate(
    [&](
      const gz::sim::UpdateInfo &,
      gz::sim::EntityComponentManager & _ecm)
    {
      // Apply wrench to buoy link on water surface to push it away
      gz::math::Vector3d driftForce(1000000, 0, 0);
      gz::math::Vector3d driftTorque(0, 0, 0);
      buoyLink.AddWorldWrench(_ecm, driftForce, driftTorque);
    }).
  OnPostUpdate(
    [&](
      const gz::sim::UpdateInfo &,
      const gz::sim::EntityComponentManager & _ecm)
    {
      iterations++;

      buoyLinkPose = gz::sim::worldPose(buoyLinkEnt, _ecm);

      gz::math::Vector2d anchorXY(anchorPos.X(), anchorPos.Y());
      gz::math::Vector2d buoyXY(buoyLinkPose.Pos().X(), buoyLinkPose.Pos().Y());

      // Expect mooring plugin to apply force on buoy to pull it back toward
      // anchor, such that buoy stays horizontal distance within maxRadius.
      //gzdbg << "iter " << iterations
      //       << ", buoy link pose: " << buoyLinkPose.Pos() << std::endl;
      EXPECT_LT((buoyXY - anchorXY).Length(), maxRadius);
    }).
  Finalize();

  // Run simulation server
  int targetIterations{20000};
  gzdbg << "Applying force to simulate natural buoy drift for "
    << targetIterations << " iterations..."<< std::endl;
  fixture.Server()->Run(true /*blocking*/, targetIterations, false /*paused*/);

  gz::math::Vector2d finalAnchorXY(anchorPos.X(), anchorPos.Y());
  gz::math::Vector2d finalBuoyXY(buoyLinkPose.Pos().X(), buoyLinkPose.Pos().Y());
  gzdbg << "Final buoy link pose: " << buoyLinkPose.Pos()
    << " Final horizontal distance between buoy and anchor: "
    << (finalBuoyXY - finalAnchorXY).Length() << std::endl;

  // Sanity check that the test ran
  EXPECT_EQ(targetIterations, iterations);
  EXPECT_NE(gz::sim::kNullEntity, buoyLinkEnt);
}
