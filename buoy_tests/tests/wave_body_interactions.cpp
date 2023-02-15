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
#include <boost/numeric/odeint.hpp>

#include <gz/common/Console.hh>
#include <gz/sim/config.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>

#if 0
#include "FreeSurfaceHydrodynamics/FS_Hydrodynamics.hpp"
#include "FreeSurfaceHydrodynamics/LinearIncidentWave.hpp"

/* The rhs of x' = f(x) defined as a class */
class SingleModeMotionRHS {
public:
  FS_HydroDynamics *FloatingBody = NULL;
  double last_accel = 0;
  int mode = 0;

  explicit SingleModeMotionRHS(FS_HydroDynamics *Body) : FloatingBody(Body) {}

  // x[0] = position
  // x[1] = velocity
  void operator()(const std::vector<double> &x, std::vector<double> &dxdt,
                  const double t) {
    Eigen::VectorXd pos(6);
    pos(0) = 0; pos(1) = 0; pos(2) = 0; pos(3) = 0; pos(4) = 0; pos(5) = 0;
    pos(mode) = x[0];
    Eigen::VectorXd vel(6);
    vel(0) = 0; vel(1) = 0; vel(2) = 0; vel(3) = 0; vel(4) = 0; vel(5) = 0;
    vel(mode) = x[1];
    Eigen::VectorXd F_LinDamping(6);
    F_LinDamping = FloatingBody->LinearDampingForce(vel);
    Eigen::VectorXd F_B(6);
    F_B = FloatingBody->BuoyancyForce(pos);
    Eigen::VectorXd F_G(6);
    F_G = FloatingBody->GravityForce(pos);
    Eigen::VectorXd F_R(6);
    Eigen::VectorXd accel(6);
    accel(0) = 0;
    accel(1) = 0;
    accel(2) = 0;
    accel(3) = 0;
    accel(4) = 0;
    accel(5) = 0;
    accel(mode) = last_accel;
    F_R = -FloatingBody->RadiationForce(accel);
    Eigen::VectorXd F_E(6);
    F_E = FloatingBody->ExcitingForce();
    dxdt[0] = x[1];
    double b = 0.1;
    dxdt[1] =
        (F_LinDamping(mode) + F_B(mode) + F_G(mode) + F_R(mode) + F_E(mode)) /
        (FloatingBody->M(mode, mode) +
         FloatingBody->AddedMass(10000.0, mode, mode));
    last_accel = dxdt[1];
  }
};

//[ integrate_observer
struct push_back_state_and_time {
  std::vector<std::vector<double>> &m_states;
  std::vector<double> &m_times;

  push_back_state_and_time(std::vector<std::vector<double>> &states,
                           std::vector<double> &times)
      : m_states(states), m_times(times) {}

  void operator()(const std::vector<double> &x, double t) {
    m_states.push_back(x);
    m_times.push_back(t);
  }
};

#endif
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

//      model = gz::sim::Model(_entity);
//  if (!model.Valid(_ecm)) {
//    ignerr <<
//      "Invalid Model Entity. " <<
//      "Failed to initialize." << std::endl;
//    return;
//  }
//      linkEntity = model.LinkByName(_ecm, "Buoy");

      linkEntity = _ecm.EntityByComponents(gz::sim::components::Name("Buoy"));

      if (!_ecm.HasEntity(linkEntity)) {
        ignerr << "Link name Buoy does not exist";
        return;
      }
      //linkEntity.EnableAccelerationChecks(_ecm, true);
      //linkEntity.EnableVelocityChecks(_ecm, true);
    }).
  // Use post-update callback to get values at the end of every iteration
  OnPreUpdate(
    [&iterations, &linkEntity](
      const gz::sim::UpdateInfo & _info,
      const gz::sim::EntityComponentManager & _ecm)
    {
      auto w_Pose_b = gz::sim::worldPose(linkEntity, _ecm);

      std::cout << iterations << ":  " <<w_Pose_b.X() << "  " << w_Pose_b.Y() << "  " << w_Pose_b.Z() << "  "
                << w_Pose_b.Roll() << "  " << w_Pose_b.Pitch() << "  " << w_Pose_b.Yaw()
                << std::endl;

    }).
  // Use post-update callback to get values at the end of every iteration
  OnPostUpdate(
    [&iterations, &linkEntity](
      const gz::sim::UpdateInfo & _info,
      const gz::sim::EntityComponentManager & _ecm)
    {
//      std::cout << "In PostUpdate" << std::endl;

      iterations++;
    }).
  // The moment we finalize, the configure callback is called
  Finalize();

  // Setup simulation server, this will call the post-update callbacks.
  // It also calls pre-update and update callbacks if those are being used.
  fixture.Server()->Run(true, 3000, false);

  // Verify that the post update function was called 1000 times
  EXPECT_EQ(3000, iterations);
}
