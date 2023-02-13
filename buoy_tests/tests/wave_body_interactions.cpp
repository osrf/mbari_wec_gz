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
#include <gz/sim/config.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/TestFixture.hh>


//////////////////////////////////////////////////
TEST(WaveBodyInteractionTests, Motions)
{
  // Maximum verbosity helps with debugging
  gz::common::Console::SetVerbosity(1);

  // Instantiate test fixture. It starts a server and provides hooks that we'll
  // use to inspect the running simulation.

  // Setup fixture
  gz::sim::ServerConfig config;
  config.SetSdfFile("singlefloatingbody.sdf");
  config.SetUpdateRate(0.0);
  gz::sim::TestFixture fixture(config);


  int iterations{0};
  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Entity linkEntity;

  fixture.
  // Use configure callback to get values at startup
  OnConfigure(
    [&model, &linkEntity](const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & /*_sdf*/,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & /*_eventMgr*/)
    {
      std::cout << "In OnConfigure " << std::endl;
      model = gz::sim::Model(_entity);
      linkEntity = model.LinkByName(_ecm, "Buoy");
      if (!_ecm.HasEntity(linkEntity)) {
        ignerr << "Link name Buoy does not exist";
        return;
      }
    }).
  // Use post-update callback to get values at the end of every iteration
  OnPreUpdate(
    [&iterations, &linkEntity](
      const gz::sim::UpdateInfo & _info,
      const gz::sim::EntityComponentManager & _ecm)
    {
//      std::cout << "In OnPreUpdate" << std::endl;
    }).
  // Use post-update callback to get values at the end of every iteration
  OnPostUpdate(
    [&iterations, &linkEntity](
      const gz::sim::UpdateInfo & _info,
      const gz::sim::EntityComponentManager & _ecm)
    {
//      std::cout << "In PostUpdate" << std::endl;
      auto w_Pose_b = gz::sim::worldPose(linkEntity, _ecm);

//      std::cout << w_Pose_b.X() << "  " << w_Pose_b.Y() << "  " << w_Pose_b.Z() << "  "
//                << w_Pose_b.Roll() << "  " << w_Pose_b.Pitch() << "  " << w_Pose_b.Yaw()
//                << std::endl;

      iterations++;
    }).
  // The moment we finalize, the configure callback is called
  Finalize();

  // Setup simulation server, this will call the post-update callbacks.
  // It also calls pre-update and update callbacks if those are being used.
  fixture.Server()->Run(true, 1000, false);

  // Verify that the post update function was called 1000 times
  EXPECT_EQ(1000, iterations);
}
