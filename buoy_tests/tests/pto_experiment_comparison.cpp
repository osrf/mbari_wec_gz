// Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay
// Aquarium Research Institute
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

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/TestFixture.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>

#include <cstring>

#include <splinter_ros/splinter1d.hpp>

#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>

#include <buoy_gazebo/ElectroHydraulicPTO/ElectroHydraulicState.hpp>

#include <gnuplot-iostream.h>


using namespace ignition;
using namespace gazebo;
using namespace systems;


struct TestData {
  const char *names[10] = {"seconds", "PistonPos",        "PistonVel",
                           "RPM",     "LowerHydPressure", "UpperHydPressure",
                           "V_Bus",   "WindCurr",         "BattCurr",
                           "LoadCurr"};

  std::vector<double> seconds;
  std::vector<double> PistonPos;
  std::vector<double> PistonVel;
  std::vector<double> RPM;
  std::vector<double> LowerHydPressure;
  std::vector<double> UpperHydPressure;
  std::vector<double> V_Bus;
  std::vector<double> WindCurr;
  std::vector<double> BattCurr;
  std::vector<double> LoadCurr;

  std::vector<double> operator()(const int data_num) {
    std::vector<double> ret_value;

    if(!strcmp(names[data_num],"seconds"))
       ret_value = seconds;
    if(!strcmp(names[data_num],"PistonPos"))
       ret_value = PistonPos;
    if(!strcmp(names[data_num],"PistonVel"))
       ret_value = PistonVel;
    if(!strcmp(names[data_num],"RPM"))
       ret_value = RPM;
    if(!strcmp(names[data_num],"LowerHydPressure"))
       ret_value = LowerHydPressure;
    if(!strcmp(names[data_num],"UpperHydPressure"))
       ret_value = UpperHydPressure;
    if(!strcmp(names[data_num],"V_Bus"))
       ret_value = V_Bus;
    if(!strcmp(names[data_num],"WindCurr"))
       ret_value = WindCurr;
    if(!strcmp(names[data_num],"BattCurr"))
       ret_value = BattCurr;
    if(!strcmp(names[data_num],"LoadCurr"))
       ret_value = LoadCurr;

    return ret_value;
  }
};

TEST(BuoyTests, PTOExperimentComparison) {
  // Maximum verbosity helps with debugging
  ignition::common::Console::SetVerbosity(4);

  // Instantiate test fixture. It starts a server and provides hooks that we'll
  // use to inspect the running simulation.
  std::string sdf_filename("PTO_TestMachine.sdf");
  std::cout << "Loading " << sdf_filename << std::endl;
  ignition::gazebo::TestFixture fixture(sdf_filename);

  int iterations{0};
  ignition::gazebo::Entity jointEntity;

	std::string buoy_tests_share("");
  try {
    buoy_tests_share = ament_index_cpp::get_package_share_directory("buoy_tests");
  } catch(ament_index_cpp::PackageNotFoundError err) {
  	std::cerr << "Could not find package share" << std::endl;
  }

  std::string inputdata_dirname(common::joinPaths(buoy_tests_share, "test_inputdata"));
  // std::string inputdata_filename = "EXP_2022.01.28T16.46.31.txt";
  std::string inputdata_filename = "2022.01.28T16.46.31.txt";

  bool EXP_Data = !inputdata_filename.compare(
      0, 4, "EXP_"); // Filenames starting with EXP denote raw experimental data,
                    // which will be used to generate test data.

  // Read Test Data
  int NN;
  TestData InputData;
  TestData ResultsData;
  std::ifstream testdata(
      common::joinPaths(inputdata_dirname, inputdata_filename));
  if (testdata.is_open()) {
    double data[10];
    while (testdata >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >>
           data[5] >> data[6] >> data[7] >> data[8] >> data[9]) {
      InputData.seconds.push_back(data[0]);
      InputData.PistonPos.push_back(data[1]);
      InputData.PistonVel.push_back(data[2]);
      InputData.RPM.push_back(data[3]);
      InputData.LowerHydPressure.push_back(data[4]);
      InputData.UpperHydPressure.push_back(data[5]);
      InputData.V_Bus.push_back(data[6]);
      InputData.WindCurr.push_back(data[7]);
      InputData.BattCurr.push_back(data[8]);
      InputData.LoadCurr.push_back(data[9]);
    }
    testdata.close();
  }

//Copy initial condition to solution data
ResultsData.seconds.push_back(InputData.seconds.at(0));
ResultsData.PistonPos.push_back(InputData.PistonVel.at(0));
ResultsData.PistonVel.push_back(InputData.PistonVel.at(0));
ResultsData.RPM.push_back(InputData.RPM.at(0));
ResultsData.LowerHydPressure.push_back(InputData.LowerHydPressure.at(0));
ResultsData.UpperHydPressure.push_back(InputData.UpperHydPressure.at(0));
ResultsData.V_Bus.push_back(InputData.V_Bus.at(0));
ResultsData.WindCurr.push_back(InputData.WindCurr.at(0));
ResultsData.BattCurr.push_back(InputData.BattCurr.at(0));
ResultsData.LoadCurr.push_back(InputData.LoadCurr.at(0));

  // JustInterp::LinearInterpolator<double> PrescribedVel;
  // PrescribedVel.SetData(InputData.seconds.size(), InputData.seconds.data(),
  //                       InputData.PistonVel.data());
  // JustInterp::LinearInterpolator<double> PrescribedPos;
  // PrescribedPos.SetData(InputData.seconds.size(), InputData.seconds.data(),
  //                       InputData.PistonPos.data());

  splinter_ros::Splinter1d PrescribedPos(InputData.seconds, InputData.PistonPos);
  splinter_ros::Splinter1d PrescribedVel(InputData.seconds, InputData.PistonVel);

  fixture
      .
      // Use configure callback to get values at startup
      OnConfigure(
          [&jointEntity](const ignition::gazebo::Entity &_worldEntity,
                         const std::shared_ptr<const sdf::Element> & /*_sdf*/,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager & /*_eventMgr*/) {
            ignition::gazebo::World world(_worldEntity);

            Model model(world.ModelByName(_ecm, "PTO"));
            jointEntity = model.JointByName(_ecm, "HydraulicRam");
          })
      .OnPreUpdate([&jointEntity, &PrescribedVel](
                       const ignition::gazebo::UpdateInfo &_info,
                       ignition::gazebo::EntityComponentManager &_ecm) {
        auto SimTime = std::chrono::duration<double>(_info.simTime).count();
        double piston_vel = -.0254 * PrescribedVel.eval(SimTime);
        // Create new component for this entitiy in ECM (if it doesn't already
        // exist)
        auto joint_vel =
            _ecm.Component<components::JointVelocityCmd>(jointEntity);
        if (joint_vel == nullptr)
          _ecm.CreateComponent(jointEntity,
                               components::JointVelocityCmd(
                                   {piston_vel})); // Create this iteration
        else {
          *joint_vel = components::JointVelocityCmd({piston_vel});
        }
      })
      .
      // Use post-update callback to get values at the end of every iteration
      OnPostUpdate([&iterations, &jointEntity, &ResultsData, &PrescribedPos](
                       const ignition::gazebo::UpdateInfo &_info,
                       const ignition::gazebo::EntityComponentManager &_ecm) {
        auto SimTime = std::chrono::duration<double>(_info.simTime).count();

       //auto prismaticJointPosComp = _ecm.Component<ignition::gazebo::components::JointPosition>(jointEntity);
       //double x = prismaticJointPosComp->Data().at(0);
       double x = .0254 * PrescribedPos.eval(SimTime); //Interpolating from given data b/c having trouble getting joint position from ecm
       auto prismaticJointVelComp = _ecm.Component<ignition::gazebo::components::JointVelocity>(jointEntity);
       double xdot = prismaticJointVelComp->Data().at(0);


        auto PTO_State_comp =
            _ecm.Component<buoy_gazebo::components::ElectroHydraulicState>(
                jointEntity);
        if (PTO_State_comp != nullptr) {
          auto PTO_State = PTO_State_comp->Data();

          ResultsData.seconds.push_back(SimTime);
          ResultsData.PistonPos.push_back(x/.0254);
          ResultsData.PistonVel.push_back(-xdot/.0254);
          ResultsData.RPM.push_back(PTO_State.rpm);
          if (PTO_State.diff_press > 0) // Todo, diff_press isn't meant to be hydraulic forces, maybe
                 // need to add some fields to PTO_State for items of interest
                 // that aren't reported over CAN.
          {
            ResultsData.LowerHydPressure.push_back(-PTO_State.diff_press);
            ResultsData.UpperHydPressure.push_back(0);
          } else {
            ResultsData.LowerHydPressure.push_back(0);
            ResultsData.UpperHydPressure.push_back(-PTO_State.diff_press);
          }
          ResultsData.V_Bus.push_back(PTO_State.voltage);
          ResultsData.WindCurr.push_back(PTO_State.wcurrent);
          ResultsData.BattCurr.push_back(PTO_State.bcurrent);
          ResultsData.LoadCurr.push_back(PTO_State.loaddc);
        }
        iterations++;
      })
      .Finalize();

  // Setup simulation server, this will call the post-update callbacks.
  // It also calls pre-update and update callbacks if those are being used.
  fixture.Server()->Run(true, 50000, false);

  if (EXP_Data) { // Plot data for user to decide if it's valid.
    for (int i = 1; i < 10; i++) {
      Gnuplot gp;
      gp << "set term X11 title  '" << InputData.names[i] << "' Comparison'\n";
      gp << "set grid\n";
      gp << "set xlabel 'time (s)'\n";
      gp << "plot '-' w l title 'EXP " << InputData.names[i]
         << "','-' w l title 'TEST " << InputData.names[i] << "'\n";

      gp.send1d(boost::make_tuple(InputData.seconds, InputData(i)));
      gp.send1d(boost::make_tuple(ResultsData.seconds, ResultsData(i)));
    }

    std::cout << "Please examine plots and determine if they are accepatably correct.  Enter y/n" << std::endl;

  system("stty raw"); // Set terminal to raw mode 
  char key = getchar(); // Wait for single character 
  system("stty cooked"); // Reset terminal to normal "cooked" mode 

    if(key == 'y')
    {
      std::string outputdata_filename = inputdata_filename.substr(4,inputdata_filename.size());
      std::cout << "Writing test results to " << outputdata_filename << " for subsequent use" << std::endl;

  std::ofstream outputdata(
      common::joinPaths(inputdata_dirname, outputdata_filename));
  if (outputdata.is_open()) {
    for(int i = 0; i<ResultsData.seconds.size();i++)
      outputdata << ResultsData.seconds.at(i)  << "  "  
                 << ResultsData.PistonPos.at(i)  << "  "  
                 << ResultsData.PistonVel.at(i)  << "  "  
                 << ResultsData.RPM.at(i)  << "  "  
                 << ResultsData.LowerHydPressure.at(i)  << "  "  
                 << ResultsData.UpperHydPressure.at(i)  << "  "  
                 << ResultsData.V_Bus.at(i)  << "  "  
                 << ResultsData.WindCurr.at(i)  << "  "  
                 << ResultsData.BattCurr.at(i)  << "  "  
                 << ResultsData.LoadCurr.at(i)  <<  std::endl;
    }
    outputdata.close();

    }




  } else // Compare test results to input data and pass test if so.
  {
  }
}
