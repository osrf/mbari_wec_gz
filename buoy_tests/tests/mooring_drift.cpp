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

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/TestFixture.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>

#include <ignition/transport/Node.hh>

//////////////////////////////////////////////////
TEST(BuoyTests, MooringDrift)
{
  // Skip debug messages to run faster TODO change to 3 for PR
  ignition::common::Console::SetVerbosity(4);

  // Setup fixture
  ignition::gazebo::ServerConfig config;
  config.SetSdfFile("mbari_wec.sdf");

  ignition::gazebo::TestFixture fixture(config);

  // Expect buoy to stay within a horizontal radius of anchor, the
  // distance calculated from the vertical depth (~80 m) and the
  // hypotenuse that is the length of the mooring line (~100 m).
  // Anchor is at (20, 0, -77.53) as defined in
  // buoy_description/models/mbari_wec_base/model.sdf.em
  const double DEPTH = 80.0;
  const double MOORING_LENGTH = 100.0;
  double maxRadius = sqrt(MOORING_LENGTH * MOORING_LENGTH - DEPTH * DEPTH);

  int iterations{0};

  ignition::gazebo::Entity buoyLinkEnt{ignition::gazebo::kNullEntity};
  ignition::gazebo::Link buoyLink;
  ignition::math::Pose3d buoyLinkPose;

  ignition::gazebo::Entity anchorLinkEnt{ignition::gazebo::kNullEntity};
  ignition::math::Pose3d anchorPose;

  fixture.
  OnConfigure(
    [&](const ignition::gazebo::Entity & _worldEntity,
    const std::shared_ptr<const sdf::Element> &,
    ignition::gazebo::EntityComponentManager & _ecm,
    ignition::gazebo::EventManager &)
    {
      auto world = ignition::gazebo::World(_worldEntity);

      ignition::gazebo::Entity buoyModelEnt = world.ModelByName(_ecm,
        "MBARI_WEC_ROS");
      EXPECT_NE(ignition::gazebo::kNullEntity, buoyModelEnt);
      auto buoyModel = ignition::gazebo::Model(buoyModelEnt);

      // Get the link on the water surface
      buoyLinkEnt = buoyModel.LinkByName(_ecm, "Buoy");
      EXPECT_NE(ignition::gazebo::kNullEntity, buoyLinkEnt);
      buoyLink = ignition::gazebo::Link(buoyLinkEnt);
      ASSERT_TRUE(buoyLink.Valid(_ecm));

      // Get the anchor pose
      anchorLinkEnt = buoyModel.LinkByName(_ecm, "Anchor");
      EXPECT_NE(ignition::gazebo::kNullEntity, anchorLinkEnt);
      anchorPose = ignition::gazebo::worldPose(anchorLinkEnt, _ecm);
    }).
  OnPreUpdate(
    [&](
      const ignition::gazebo::UpdateInfo &,
      ignition::gazebo::EntityComponentManager & _ecm)
    {
      // Apply wrench to buoy link on water surface
      ignition::math::Vector3d force(10000, 0, 0);
      ignition::math::Vector3d torque(0, 0, 0);
      buoyLink.AddWorldWrench(_ecm, force, torque);
    }).
  OnPostUpdate(
    [&](
      const ignition::gazebo::UpdateInfo &,
      const ignition::gazebo::EntityComponentManager & _ecm)
    {
      iterations++;

      buoyLinkPose = ignition::gazebo::worldPose(buoyLinkEnt, _ecm);

      ignition::math::Vector2d anchorXY(anchorPose.Pos().X(), anchorPose.Pos().Y());
      ignition::math::Vector2d buoyXY(buoyLinkPose.Pos().X(), buoyLinkPose.Pos().Y());

      // Expect buoy to stay horizontal distance within maxRadius
      igndbg << "iter " << iterations
             << ", buoy link pose: " << buoyLinkPose.Pos() << std::endl;
      EXPECT_LT((buoyXY - anchorXY).Length(), maxRadius);
    }).
  Finalize();

  // Run simulation server
  int targetIterations{3000};
  fixture.Server()->Run(true /*blocking*/, targetIterations, false /*paused*/);

  igndbg << "Buoy link pose: " << buoyLinkPose.Pos() << std::endl;

  // Sanity check that the test ran
  EXPECT_EQ(targetIterations, iterations);
  EXPECT_NE(ignition::gazebo::kNullEntity, buoyLinkEnt);
}
