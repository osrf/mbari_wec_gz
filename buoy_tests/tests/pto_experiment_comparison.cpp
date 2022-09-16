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
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/TestFixture.hh>

#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointVelocity.hh"

#include "test_config.hh"
#include "ElectroHydraulicState.hpp"  


using namespace ignition;
using namespace gazebo;
using namespace systems;



TEST(BuoyTests, PTOExperimentComparison)
{
  // Maximum verbosity helps with debugging
  ignition::common::Console::SetVerbosity(4);

  // Instantiate test fixture. It starts a server and provides hooks that we'll
  // use to inspect the running simulation.
  auto sdf_filename = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "worlds", "pto_experiment_comparison_TEST.sdf");
      std::cout << "Loading " << sdf_filename << std::endl;
  ignition::gazebo::TestFixture fixture(sdf_filename);

  int iterations{0};
  ignition::gazebo::Entity jointEntity;
  double foo = 12;
#if 0
//Read Test Data
  int NN;
  {
    //std::ifstream testdata("../TestInputFiles/PositionBasedSineMoves_Scale1.txt");
    std::ifstream testdata("../TestInputFiles/2022.01.28T16.46.31.txt");
    if (testdata.is_open())
    {
      std::string NN_str;
      testdata >> NN_str;
      NN = std::stoi(NN_str.substr(1));
      std::vector<double> seconds(NN);
      std::vector<double> PistonVel(NN);
      std::vector<double> PistonPos(NN);
      std::vector<double> RPM(NN);
      std::vector<double> LowerHydPressure(NN);
      std::vector<double> UpperHydPressure(NN);
      std::vector<double> V_Bus(NN);
      std::vector<double> WindCurr(NN);
      std::vector<double> BattCurr(NN);
      std::vector<double> LoadCurr(NN);
      std::vector<double> PC_Scale(NN);
      std::vector<double> PC_Retract(NN);
      double col1, col2, col3, col4;
      for (int i = 0; i < NN ; i++)
      {
        testdata >> seconds[i] >> PistonPos[i] >>  PistonVel[i] >> RPM[i] >> LowerHydPressure[i] >> UpperHydPressure[i] >> V_Bus[i] >> WindCurr[i] >> BattCurr[i] >> LoadCurr[i];
      }
      testdata.close();
    }
  }
#else
double PrescribedVel = 20; //inches/sec 
#endif
std::cout << "SET UP" << std::endl;
  //jfixture->
  fixture.
  // Use configure callback to get values at startup
  OnConfigure(
    [&jointEntity](const ignition::gazebo::Entity & _worldEntity,
                   const std::shared_ptr<const sdf::Element> &/*_sdf*/,
                   ignition::gazebo::EntityComponentManager & _ecm,
                   ignition::gazebo::EventManager &/*_eventMgr*/)
  {

std::cout << "ONCONFIGURE" << std::endl;
    ignition::gazebo::World world(_worldEntity);

    Model model(world.ModelByName(_ecm, "PTO"));
    jointEntity =   model.JointByName(_ecm, "HydraulicRam");
    std::cout << "jointEntitiy = " << jointEntity << std::endl;
std::cout << "ONCONFIGURE: Got JointEntitty" << std::endl;
  }).
 OnPreUpdate(
    [&jointEntity, &PrescribedVel](
      const ignition::gazebo::UpdateInfo & _info,
      ignition::gazebo::EntityComponentManager & _ecm)
  {


std::cout << "ONPREUPDATE" << std::endl;
    auto SimTime = std::chrono::duration<double>(_info.simTime).count();
    double piston_vel = -0.0254 * PrescribedVel*sin(2*M_PI*SimTime/10); //PrescribedVel(SimTime);
    //Create new component for this entitiy in ECM (if it doesn't already exist)
    auto joint_vel = _ecm.Component<components::JointVelocityCmd>(jointEntity);
    if (joint_vel == nullptr)
      _ecm.CreateComponent(jointEntity, components::JointVelocityCmd({piston_vel}));  //Create this iteration
    else
      *joint_vel = components::JointVelocityCmd({piston_vel});

std::cout << "ONPREUPDATE Set joint_vel = " << piston_vel << std::endl;

  }).
  // Use post-update callback to get values at the end of every iteration
  OnPostUpdate(
    [&iterations, &jointEntity](
      const ignition::gazebo::UpdateInfo & _info,
      const ignition::gazebo::EntityComponentManager & _ecm)
  {
    auto SimTime = std::chrono::duration<double>(_info.simTime).count();
std::cout << "ONPOSTUPDATE:  " << SimTime << std::endl;

    auto PTO_State_comp = _ecm.Component<buoy_gazebo::components::ElectroHydraulicState>(jointEntity);
    if (PTO_State_comp != nullptr)
    {
      auto PTO_State = PTO_State_comp->Data();

      std::cout << SimTime << "  "  << "  " << "    " << PTO_State.rpm << "   " << PTO_State.diff_press << "   "  << PTO_State.voltage << "   "
                << PTO_State.target_a << "  "  << PTO_State.wcurrent << "  "  << PTO_State.bcurrent << std::endl;

    }

    iterations++;
  }).
  Finalize();

  // Setup simulation server, this will call the post-update callbacks.
  // It also calls pre-update and update callbacks if those are being used.
  fixture.Server()->Run(true, 50000, false);

  // Verify that the post update function was called 1000 times
  EXPECT_EQ(50000, iterations);

}
