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
#include <Eigen/Dense>
#include <cstdlib>
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
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Gravity.hh>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "FS_Hydrodynamics.hpp"
#include "LinearIncidentWave.hpp"

double last_accel = 0;
/* The rhs of x' = f(x) defined as a class */
class SingleModeMotionRHS
{
public:
  FS_HydroDynamics * FloatingBody = NULL;
  int mode = 0;
  double inf_freq_added_mass = 0;

  explicit SingleModeMotionRHS(FS_HydroDynamics * Body)
  : FloatingBody(Body) {}

  // x[0] = position
  // x[1] = velocity
  void operator()(
    const std::vector<double> & x, std::vector<double> & dxdt,
    const double t)
  {
    Eigen::VectorXd pos(6);
    pos(0) = 0; pos(1) = 0; pos(2) = 0; pos(3) = 0; pos(4) = 0; pos(5) = 0;
    pos(mode) = x[0];
    Eigen::VectorXd vel(6);
    vel(0) = 0; vel(1) = 0; vel(2) = 0; vel(3) = 0; vel(4) = 0; vel(5) = 0;
    vel(mode) = x[1];
    Eigen::VectorXd F_B(6);
    F_B = FloatingBody->BuoyancyForce(pos);
    Eigen::VectorXd F_G(6);
    F_G = FloatingBody->GravityForce(pos);
    Eigen::VectorXd F_R(6);
    Eigen::VectorXd accel(6);
    accel(0) = 0; accel(1) = 0; accel(2) = 0;
    accel(3) = 0; accel(4) = 0; accel(5) = 0;
    accel(mode) = last_accel;
    F_R = -FloatingBody->RadiationForce(accel);
    Eigen::VectorXd F_E(6);
    dxdt[0] = x[1];
    dxdt[1] =
      (F_B(mode) + F_G(mode) + F_R(mode)) /
      (FloatingBody->M(mode, mode) +
      inf_freq_added_mass);
    last_accel = dxdt[1];
  }
};

// [ integrate_observer
struct push_back_state_and_time
{
  std::vector<std::vector<double>> & m_states;
  std::vector<double> & m_times;

  push_back_state_and_time(
    std::vector<std::vector<double>> & states,
    std::vector<double> & times)
  : m_states(states), m_times(times) {}

  void operator()(const std::vector<double> & x, double t)
  {
    m_states.push_back(x);
    m_times.push_back(t);
  }
};

//////////////////////////////////////////////////
TEST(WaveBodyInteractionTests, PitchMotions)
{
  gz::common::Console::SetVerbosity(1);

  // Instantiate test fixture. It starts a server and provides hooks that we'll
  // use to inspect the running simulation.

  double initial_position = .1;  // Radians, about 5.7 degrees

  // Setup fixture
  gz::sim::ServerConfig config;
  config.SetSdfFile("singlefloatingbody_pitch.sdf");
  config.SetUpdateRate(0.0);
  gz::sim::TestFixture fixture(config);


  int iterations{0};
  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Entity linkEntity;

  std::vector<double> GzSimTime;
  std::vector<double> GzSimPitchPos;
  GzSimTime.push_back(0.0);
  GzSimPitchPos.push_back(initial_position);
  double dt;
  double gravity;

  fixture.
  // Use configure callback to get values at startup
  OnConfigure(
    [&model, &linkEntity, &GzSimTime, &GzSimPitchPos, &gravity](const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & /*_sdf*/,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & /*_eventMgr*/)
    {
      gzdbg << "In OnConfigure " << std::endl;
      gz::sim::Entity worldEntity =         // Get world and gravity
      _ecm.EntityByComponents(gz::sim::components::World());
      auto g = _ecm.Component<gz::sim::components::Gravity>(worldEntity);
      gravity = -g->Data()[2];

      linkEntity = _ecm.EntityByComponents(gz::sim::components::Name("Buoy"));

      if (!_ecm.HasEntity(linkEntity)) {
        ignerr << "Link name Buoy does not exist";
        return;
      }
      // linkEntity.EnableAccelerationChecks(_ecm, true);
      // linkEntity.EnableVelocityChecks(_ecm, true);
    }).
  // Use post-update callback to get values at the end of every iteration
  OnPreUpdate(
    [&iterations, &linkEntity, &dt](
      const gz::sim::UpdateInfo & _info,
      const gz::sim::EntityComponentManager & _ecm)
    {
      auto SimTime = std::chrono::duration<double>(_info.simTime).count();
      if (_info.iterations == 1) {  // First iteration, set timestep size.
        dt = std::chrono::duration<double>(_info.dt).count();
      }
    }).
  // Use post-update callback to get values at the end of every iteration
  OnPostUpdate(
    [&iterations, &linkEntity, &GzSimTime, &GzSimPitchPos](
      const gz::sim::UpdateInfo & _info,
      const gz::sim::EntityComponentManager & _ecm)
    {
      auto w_Pose_b = gz::sim::worldPose(linkEntity, _ecm);
      GzSimTime.push_back(iterations + 1);
      GzSimPitchPos.push_back(w_Pose_b.Rot().Pitch());
      iterations++;
    }).
  // The moment we finalize, the configure callback is called
  Finalize();
  // Setup simulation server, this will call the post-update callbacks.
  // It also calls pre-update and update callbacks if those are being used.
  fixture.Server()->Run(true, 1500, false);

  // Compute solution independently for comparison
  const char * modes[6] = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"};
  std::shared_ptr<LinearIncidentWave> Inc = std::make_shared<LinearIncidentWave>();

  double buoy_mass = 2449.75;  // kg
  FS_HydroDynamics BuoyA5;
  BuoyA5.SetGravity(gravity);  // Use gravity that matches what was used by gz-sim

  BuoyA5.SetWaterplane(
    5.47, 1.37,
    1.37);                    // Set area and 2nd moments of area for waterplane
  BuoyA5.SetCOB(
    0, 0,
    -.22);             // Set COB relative to waterplane coordinate system.
  BuoyA5.SetCOG(
    0, 0,
    0.0);             // Set COG at waterplane for this test.
  BuoyA5.SetVolume(2.39);
  BuoyA5.SetMass(buoy_mass);

  std::string HydrodynamicsBaseFilename =
    ament_index_cpp::get_package_share_directory("buoy_description") +
    std::string("/models/mbari_wec_base/hydrodynamic_coeffs/BuoyA5");


  BuoyA5.ReadWAMITData_FD(HydrodynamicsBaseFilename);
  BuoyA5.ReadWAMITData_TD(HydrodynamicsBaseFilename);
  BuoyA5.SetTimestepSize(dt);

  Eigen::Matrix<double, 3, 3> I;
  I << 1430.0, 0, 0, 0, 1430.0, 0, 0, 0, 670.0;
  BuoyA5.SetI(I);
  Eigen::VectorXd b(6);
  b(0) = 300.0;
  b(1) = 300.0;
  b(2) = 900.0;
  b(3) = 400.0;
  b(4) = 400.0;
  b(5) = 100.0;
  BuoyA5.SetDampingCoeffs(b);

  std::vector<double> x(2);
  x[0] = initial_position;  // initial position
  x[1] = 0.0;  // initial velocity

  double t_final = iterations * dt;
  std::vector<std::vector<double>> x_vec;
  std::vector<double> times;
  boost::numeric::odeint::euler<std::vector<double>> stepper;
  SingleModeMotionRHS RHS(&BuoyA5);
  RHS.mode = 4;
  RHS.inf_freq_added_mass = 367.85;

  std::vector<double> ddxdt(2);
  std::vector<double> & _rhs = ddxdt;
  x_vec.push_back(x);  // Set initial conditions
  for (int n = 0; n < GzSimPitchPos.size(); n++) {
    RHS(x_vec[n], _rhs, n * dt);
    std::vector<double> xx(2);
    xx[1] = x_vec[n][1] + _rhs[1] * dt;  // Step velocity
    xx[0] = x_vec[n][0] + xx[1] * dt;   // Step Position
    x_vec.push_back(xx);
  }


  double max_error = 0;
  for (int i = 0; i < GzSimPitchPos.size() - 1; i++) {
    double err = fabs(GzSimPitchPos[i] - (x_vec[i][0]));
    if (err > max_error) {
      max_error = err;
    }
    gzdbg << GzSimTime[i] * dt
          << "    " << GzSimPitchPos[i]
          << "    " << x_vec[i][0]
          << "    " << GzSimPitchPos[i] - (x_vec[i][0]) << std::endl;
  }

  // Verify that the post update function was called the correct number of times and that the
  // error is small.
  EXPECT_EQ(1500, iterations);
  EXPECT_LT(max_error, .015);  // Relatively large error is expected as independent solution
                               // computed here does not model the pitch/surge coupling that
                               // gz sim is including due to a transient at startup that imparts
                               // a little bit of horizontal motion.
                               // A serious problem that leads to a phase difference between the
                               // solutions will still be caught by this error bound
}
