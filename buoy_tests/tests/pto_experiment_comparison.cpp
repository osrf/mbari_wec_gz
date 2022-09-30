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

#include <gnuplot-iostream.h>
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <buoy_gazebo/ElectroHydraulicPTO/ElectroHydraulicState.hpp>
#include <ignition/common/Console.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/TestFixture.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/config.hh>
#include <splinter_ros/splinter1d.hpp>

#include <cstring>
#include <vector>
#include <string>
#include <memory>

struct TestData
{
  const char * names[14] = {"seconds",
    "PistonPos",
    "PistonVel",
    "RPM",
    "LowerHydPressure",
    "UpperHydPressure",
    "V_Bus",
    "WindCurr",
    "BattCurr",
    "LoadCurr",
    "Scale",
    "Retract",
    "LowerSpringPressure",
    "UpperSpringPressure"};
  const char * units[14] = {"seconds", "inches", "in/sec", "RPM", "psi_g",
    "psi_g", "Volts", "Amps", "Amps", "Amps",
    "", "", "psi_a", "psi_a"};

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
  std::vector<double> Scale;
  std::vector<double> Retract;
  std::vector<double> LowerSpringPressure;
  std::vector<double> UpperSpringPressure;

  std::vector<double> operator()(const int data_num)
  {
    std::vector<double> ret_value;

    if (!strcmp(names[data_num], "seconds")) {
      ret_value = seconds;
    }
    if (!strcmp(names[data_num], "PistonPos")) {
      ret_value = PistonPos;
    }
    if (!strcmp(names[data_num], "PistonVel")) {
      ret_value = PistonVel;
    }
    if (!strcmp(names[data_num], "RPM")) {
      ret_value = RPM;
    }
    if (!strcmp(names[data_num], "LowerHydPressure")) {
      ret_value = LowerHydPressure;
    }
    if (!strcmp(names[data_num], "UpperHydPressure")) {
      ret_value = UpperHydPressure;
    }
    if (!strcmp(names[data_num], "V_Bus")) {
      ret_value = V_Bus;
    }
    if (!strcmp(names[data_num], "WindCurr")) {
      ret_value = WindCurr;
    }
    if (!strcmp(names[data_num], "BattCurr")) {
      ret_value = BattCurr;
    }
    if (!strcmp(names[data_num], "LoadCurr")) {
      ret_value = LoadCurr;
    }

    return ret_value;
  }
};

TEST(BuoyTests, PTOExperimentComparison) {
  // Maximum verbosity helps with debugging
  ignition::common::Console::SetVerbosity(0);

  std::string buoy_tests_share("");
  try {
    buoy_tests_share =
      ament_index_cpp::get_package_share_directory("buoy_tests");
  } catch (ament_index_cpp::PackageNotFoundError err) {
    std::cerr << "Could not find package share" << std::endl;
  }

  // Instantiate test fixture. It starts a server and provides hooks that we'll
  // use to inspect the running simulation.
  std::string sdf_filename("PTO_TestMachine.sdf");
  // std::cout << "Loading " <<
  // ignition::common::joinPaths(buoy_tests_share,"worlds",sdf_filename) << std::endl;
  // ignition::gazebo::TestFixture
  // fixture(ignition::common::joinPaths(buoy_tests_share,"worlds",sdf_filename));
  std::cout << "Loading " << ignition::common::joinPaths(sdf_filename) << std::endl;
  ignition::gazebo::TestFixture fixture(ignition::common::joinPaths(sdf_filename));

  int iterations{0};
  ignition::gazebo::Entity jointEntity;

  std::string inputdata_dirname(
    ignition::common::joinPaths(buoy_tests_share, "test_inputdata"));

  std::string inputdata_filename;
  std::string inputfilelist("TestInputFileList.txt");
  std::ifstream testfilelist(
    ignition::common::joinPaths(inputdata_dirname, inputfilelist));

  if (testfilelist.is_open()) {
    testfilelist >> inputdata_filename;
  } else {
    FAIL() << ignition::common::joinPaths(inputdata_dirname, inputfilelist) <<
      "Not Found";
  }

  bool EXP_Data = !inputdata_filename.compare(
    inputdata_filename.size() - 4, inputdata_filename.size(),
    ".exp");             // Filenames with .exp denote raw experimental data

  // Read Test Data
  int NN;
  TestData InputData;
  TestData ResultsData;
  std::ifstream testdata(
    ignition::common::joinPaths(inputdata_dirname, inputdata_filename));
  std::string header_line;
  if (testdata.is_open()) {
    testdata >> header_line;
    double data[14];
    while (testdata >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >>
      data[5] >> data[6] >> data[7] >> data[8] >> data[9] >> data[10] >>
      data[11] >> data[12] >> data[13])
    {
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
      InputData.Scale.push_back(data[10]);
      InputData.Retract.push_back(data[11]);
      InputData.LowerSpringPressure.push_back(data[12]);
      InputData.UpperSpringPressure.push_back(data[13]);
    }
    testdata.close();
  }

  // Copy initial condition to solution data
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
  ResultsData.Scale.push_back(InputData.Scale.at(0));
  ResultsData.Retract.push_back(InputData.Retract.at(0));
  ResultsData.LowerSpringPressure.push_back(
    InputData.LowerSpringPressure.at(0));
  ResultsData.UpperSpringPressure.push_back(
    InputData.UpperSpringPressure.at(0));

  splinter_ros::Splinter1d PrescribedPos(InputData.seconds,
    InputData.PistonPos);
  splinter_ros::Splinter1d PrescribedVel(InputData.seconds,
    InputData.PistonVel);
  splinter_ros::Splinter1d PrescribedScale(InputData.seconds, InputData.Scale);
  splinter_ros::Splinter1d PrescribedRetract(InputData.seconds,
    InputData.Retract);

  fixture
  .
  // Use configure callback to get values at startup
  OnConfigure(
    [&jointEntity](const ignition::gazebo::Entity & _worldEntity,
    const std::shared_ptr<const sdf::Element> & /*_sdf*/,
    ignition::gazebo::EntityComponentManager & _ecm,
    ignition::gazebo::EventManager & /*_eventMgr*/) {
      ignition::gazebo::World world(_worldEntity);

      ignition::gazebo::Model model(world.ModelByName(_ecm, "PTO"));
      jointEntity = model.JointByName(_ecm, "HydraulicRam");
    })
  .OnPreUpdate(
    [&jointEntity, &PrescribedVel, &PrescribedScale, &PrescribedRetract](
      const ignition::gazebo::UpdateInfo & _info,
      ignition::gazebo::EntityComponentManager & _ecm) {
      auto SimTime = std::chrono::duration<double>(_info.simTime).count();
      double piston_vel = -0.0254 * PrescribedVel.eval(SimTime);
      // Create new component for this entitiy in ECM (if it doesn't
      // already exist)
      auto joint_vel =
      _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(jointEntity);
      if (joint_vel == nullptr) {
        _ecm.CreateComponent(
          jointEntity,
          ignition::gazebo::components::JointVelocityCmd(
            {piston_vel}));                             // Create this iteration
      } else {
        *joint_vel = ignition::gazebo::components::JointVelocityCmd({piston_vel});
      }
    })
  .
  // Use post-update callback to get values at the end of every iteration
  OnPostUpdate(
    [&iterations, &jointEntity, &ResultsData, &PrescribedPos](
      const ignition::gazebo::UpdateInfo & _info,
      const ignition::gazebo::EntityComponentManager & _ecm) {
      auto SimTime = std::chrono::duration<double>(_info.simTime).count();

      // auto prismaticJointPosComp =
      // _ecm.Component<ignition::gazebo::components::JointPosition>(jointEntity);
      // double x = prismaticJointPosComp->Data().at(0);
      double x =
      0.0254 * PrescribedPos.eval(
        SimTime);                         // Interpolating from given data b/c having
                                          // trouble getting joint position from ecm
      auto prismaticJointVelComp =
      _ecm.Component<ignition::gazebo::components::JointVelocity>(
        jointEntity);
      double xdot = prismaticJointVelComp->Data().at(0);

      auto PTO_State_comp =
      _ecm.Component<buoy_gazebo::components::ElectroHydraulicState>(
        jointEntity);
      if (PTO_State_comp != nullptr) {
        auto PTO_State = PTO_State_comp->Data();
        ResultsData.seconds.push_back(SimTime);
        ResultsData.PistonPos.push_back(x / 0.0254);
        ResultsData.PistonVel.push_back(-xdot / 0.0254);
        ResultsData.RPM.push_back(PTO_State.rpm);
        if (PTO_State.diff_press > 0) {
          // Todo, diff_press isn't meant to be hydraulic forces,
          // maybe need to add some fields to PTO_State for items
          // of interest that aren't reported over CAN.
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
        ResultsData.Scale.push_back(0.0);
        ResultsData.Retract.push_back(0.0);
        ResultsData.LowerSpringPressure.push_back(0.0);
        ResultsData.UpperSpringPressure.push_back(0.0);
      }
      iterations++;
    })
  .Finalize();

  // Setup simulation server, this will call the post-update callbacks.
  // It also calls pre-update and update callbacks if those are being used.
  fixture.Server()->Run(
    true, InputData.seconds.back() / 0.01,
    false);                      // Hardcoded timestep that is set in sdf file
                                 // until I figure out how to get access...

  if (EXP_Data) {       // Plot data for user to decide if it's valid.
    for (int i = 1; i < 10; i++) {
      Gnuplot gp;
      gp << "set term X11 title  '" << InputData.names[i] << " Comparison'\n";
      gp << "set grid\n";
      gp << "set xlabel 'time (s)'\n";
      gp << "set ylabel '" << InputData.units[i] << "'\n";
      gp << "plot '-' w l title 'EXP " << InputData.names[i] <<
        "','-' w l title 'TEST " << InputData.names[i] << "'\n";

      gp.send1d(boost::make_tuple(InputData.seconds, InputData(i)));
      gp.send1d(boost::make_tuple(ResultsData.seconds, ResultsData(i)));
    }

    std::cout << "Please examine plots and determine if they are acceptably "
      "correct.  Enter y/n" <<
      std::endl;

    // system("stty raw");    // Set terminal to raw mode
    char key = getchar();             // Wait for single character
    // system("stty cooked"); // Reset terminal to normal "cooked" mode

    if (key == 'y') {
      std::string outputdata_filename =
        inputdata_filename.substr(0, inputdata_filename.size() - 4) + ".tst";

      std::ofstream outputdata(
        ignition::common::joinPaths(inputdata_dirname, outputdata_filename));
      if (outputdata.is_open()) {
        outputdata << header_line << std::endl;
        std::cout << "Writing test results to " << outputdata_filename <<
          std::endl;
        for (int i = 0; i < ResultsData.seconds.size(); i++) {
          outputdata << ResultsData.seconds.at(i) << "  " <<
            ResultsData.PistonPos.at(i) << "  " <<
            ResultsData.PistonVel.at(i) << "  " <<
            ResultsData.RPM.at(i) << "  " <<
            ResultsData.LowerHydPressure.at(i) << "  " <<
            ResultsData.UpperHydPressure.at(i) << "  " <<
            ResultsData.V_Bus.at(i) << "  " <<
            ResultsData.WindCurr.at(i) << "  " <<
            ResultsData.BattCurr.at(i) << "  " <<
            ResultsData.LoadCurr.at(i) << "  " <<
            ResultsData.Scale.at(i) << "  " <<
            ResultsData.Retract.at(i) << "  " <<
            ResultsData.LowerSpringPressure.at(i) << "  " <<
            ResultsData.UpperSpringPressure.at(i) << std::endl;
        }
      }
      outputdata.close();
    }
    FAIL() << "Running in manual mode with " << inputdata_filename <<
      " as input, can't pass this way";
  } else {       // Compare test results to input data and pass test if so.
    double epsilon;
    std::function<bool(const double &, const double &)> comparator =
      [&epsilon](
      const double & left,
      const double & right) {                           // Lambda function to compare 2 doubles
        //  std::cout << i << "  " <<  left << "  " << right << "  "
        //  <<fabs(left-right) << std::endl;
        if (fabs(left - right) < epsilon) {
          return true;
        } else {
          return false;
        }
      };

    epsilon = 1e-2;
    if (!std::equal(
        InputData.PistonVel.begin(), InputData.PistonVel.end(),
        ResultsData.PistonVel.begin(),
        comparator))                     // Clumsy if then else due to apparant lack of
    {                             // ASSERT_TRUE() macro
      FAIL() << "PistonVel Test Failed";
    }

    epsilon = 1.0;             // Accuracy of results in RPM is poor, seems to be due to
                               // sensitive behavior of nonlinear solver near zero RPM.
    if (!std::equal(
        InputData.RPM.begin(), InputData.RPM.end(),
        ResultsData.RPM.begin(), comparator))
    {
      FAIL() << "RPM Test Failed";
    }

    epsilon = 1e-2;
    if (!std::equal(
        InputData.LowerHydPressure.begin(),
        InputData.LowerHydPressure.end(),
        ResultsData.LowerHydPressure.begin(), comparator))
    {
      FAIL() << "LowerHydPressure Test Failed";
    }

    epsilon = 1e-2;
    if (!std::equal(
        InputData.UpperHydPressure.begin(),
        InputData.UpperHydPressure.end(),
        ResultsData.UpperHydPressure.begin(), comparator))
    {
      FAIL() << "UpperHydPressure Test Failed";
    }

    epsilon = 1e-2;
    if (!std::equal(
        InputData.V_Bus.begin(), InputData.V_Bus.end(),
        ResultsData.V_Bus.begin(), comparator))
    {
      FAIL() << "V_Bus Test Failed";
    }

    epsilon = 1e-2;
    if (!std::equal(
        InputData.WindCurr.begin(), InputData.WindCurr.end(),
        ResultsData.WindCurr.begin(), comparator))
    {
      FAIL() << "WindCurr Test Failed";
    }

    epsilon = 1e-2;
    if (!std::equal(
        InputData.BattCurr.begin(), InputData.BattCurr.end(),
        ResultsData.BattCurr.begin(), comparator))
    {
      FAIL() << "BattCurr Test Failed";
    }

    epsilon = 1e-2;
    if (!std::equal(
        InputData.LoadCurr.begin(), InputData.LoadCurr.end(),
        ResultsData.LoadCurr.begin(), comparator))
    {
      FAIL() << "LoadCurr Test Failed";
    }
  }
}
