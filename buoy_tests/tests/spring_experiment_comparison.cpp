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

#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>

#include <buoy_gazebo/PolytropicPneumaticSpring/SpringState.hpp>

#include <gnuplot-iostream.h>

#include <splinter_ros/splinter1d.hpp>


using namespace ignition;
using namespace gazebo;
using namespace systems;


struct TestData {
  const char *names[16] = {"seconds", "PistonPos", "PistonVel",
                           "RPM", "LowerHydPressure", "UpperHydPressure",
                           "V_Bus", "WindCurr", "BattCurr",
                           "LoadCurr", "Scale", "Retract",
                           "LowerSpringPressure", "UpperSpringPressure",
                           "LowerSpringVolume", "UpperSpringVolume"};
  const char *units[16] = {"seconds", "inches", "in/sec", "RPM", "psi_g",
                           "psi_g", "Volts", "Amps", "Amps", "Amps",
                           "", "", "psi_a","psi_a", "cubic m", "cubic m"};

  std::vector<double> seconds;
  std::vector<double> PistonPos;
  std::vector<double> LowerSpringPressure;
  std::vector<double> UpperSpringPressure;
  std::vector<double> LowerSpringVolume;
  std::vector<double> UpperSpringVolume;

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

  std::vector<double> operator()(const int data_num) {
    std::vector<double> ret_value;

    if (!strcmp(names[data_num], "seconds"))
      ret_value = seconds;
    if (!strcmp(names[data_num], "PistonPos"))
      ret_value = PistonPos;
    if (!strcmp(names[data_num], "PistonVel"))
      ret_value = PistonVel;
    if (!strcmp(names[data_num], "RPM"))
      ret_value = RPM;
    if (!strcmp(names[data_num], "LowerHydPressure"))
      ret_value = LowerHydPressure;
    if (!strcmp(names[data_num], "UpperHydPressure"))
      ret_value = UpperHydPressure;
    if (!strcmp(names[data_num], "V_Bus"))
      ret_value = V_Bus;
    if (!strcmp(names[data_num], "WindCurr"))
      ret_value = WindCurr;
    if (!strcmp(names[data_num], "BattCurr"))
      ret_value = BattCurr;
    if (!strcmp(names[data_num], "LoadCurr"))
      ret_value = LoadCurr;
    if (!strcmp(names[data_num], "Scale"))
      ret_value = Scale;
    if (!strcmp(names[data_num], "Retract"))
      ret_value = Retract;
    if (!strcmp(names[data_num], "LowerSpringPressure"))
      ret_value = LowerSpringPressure;
    if (!strcmp(names[data_num], "UpperSpringPressure"))
      ret_value = UpperSpringPressure;
    if (!strcmp(names[data_num], "LowerSpringVolume"))
      ret_value = LowerSpringVolume;
    if (!strcmp(names[data_num], "UpperSpringVolume"))
      ret_value = UpperSpringVolume;

    return ret_value;
  }
};

TEST(BuoyTests, SpringExperimentComparison)
{
  // Maximum verbosity helps with debugging
  ignition::common::Console::SetVerbosity(0);

  // Instantiate test fixture. It starts a server and provides hooks that we'll
  // use to inspect the running simulation.
  std::string sdf_filename("Spring_TestMachine.sdf");
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

  std::string inputdata_filename;
  std::string inputfilelist("TestInputFileList.txt");
  std::ifstream testfilelist(
    common::joinPaths(inputdata_dirname, inputfilelist));

  if (testfilelist.is_open()) {
    testfilelist >> inputdata_filename;
  } else {
    FAIL() << common::joinPaths(inputdata_dirname, inputfilelist)
      << "Not Found";
  }

  bool EXP_Data =
    !inputdata_filename.compare(
      inputdata_filename.size() - 4U, inputdata_filename.size(),
      ".exp"); // Filenames with .exp denote raw experimental data

  // Read Test Data
  int NN;
  TestData InputData;
  TestData ResultsData;
  std::ifstream testdata(
    common::joinPaths(inputdata_dirname, inputdata_filename));
  std::string header_line;
  if (testdata.is_open()) {
    testdata >> header_line;
    if(EXP_Data) {
      double data[14U];
      while (testdata >> data[0U] >> data[1U] >> data[2U] >> data[3U] >> data[4U] >>
        data[5U] >> data[6U] >> data[7U] >> data[8U] >> data[9U] >>
        data[10U] >> data[11U] >> data[12U] >> data[13U])
      {
        InputData.seconds.push_back(data[0U]);
        InputData.PistonPos.push_back(data[1U]);
        InputData.PistonVel.push_back(data[2U]);
        InputData.LowerSpringPressure.push_back(data[12U]);
        InputData.UpperSpringPressure.push_back(data[13U]);
        InputData.LowerSpringVolume.push_back((2.03 - 0.0254* data[1U]) * 0.0115 + 0.0523);
        InputData.UpperSpringVolume.push_back(0.0254 * data[1U] * 0.0127 + 0.0266);
      }
    } else {
      double data[16U];
      while (testdata >> data[0U] >> data[1U] >> data[2U] >> data[3U] >> data[4U] >>
        data[5U] >> data[6U] >> data[7U] >> data[8U] >> data[9U] >>
        data[10U] >> data[11U] >> data[12U] >> data[13U] >> data[14U] >> data[15U])
      {
        InputData.seconds.push_back(data[0U]);
        InputData.PistonPos.push_back(data[1U]);
        InputData.PistonVel.push_back(data[2U]);
        InputData.LowerSpringPressure.push_back(data[12U]);
        InputData.UpperSpringPressure.push_back(data[13U]);
        InputData.LowerSpringVolume.push_back(data[14U]);
        InputData.UpperSpringVolume.push_back(data[15U]);
      }
    }
    testdata.close();
  } else {
    FAIL() << "could not open input file "
      << common::joinPaths(inputdata_dirname, inputdata_filename) << std::endl;
  }

  // Copy initial condition to solution data
  ResultsData.seconds.push_back(InputData.seconds.at(0));
  ResultsData.PistonPos.push_back(InputData.PistonPos.at(0));
  ResultsData.PistonVel.push_back(InputData.PistonVel.at(0));
  ResultsData.LowerSpringPressure.push_back(InputData.LowerSpringPressure.at(0));
  ResultsData.UpperSpringPressure.push_back(InputData.UpperSpringPressure.at(0));
  ResultsData.LowerSpringVolume.push_back(InputData.LowerSpringVolume.at(0));
  ResultsData.UpperSpringVolume.push_back(InputData.UpperSpringVolume.at(0));

  splinter_ros::Splinter1d PrescribedVel(InputData.seconds, InputData.PistonVel);

  fixture
    // Use configure callback to get values at startup
    .OnConfigure(
      [&jointEntity](const ignition::gazebo::Entity &_worldEntity,
        const std::shared_ptr<const sdf::Element> & /*_sdf*/,
        ignition::gazebo::EntityComponentManager &_ecm,
        ignition::gazebo::EventManager & /*_eventMgr*/) {
          ignition::gazebo::World world(_worldEntity);

          Model model(world.ModelByName(_ecm, "PTO"));
          jointEntity = model.JointByName(_ecm, "HydraulicRam");
          _ecm.SetComponentData<components::JointPositionReset>(jointEntity, {2.03 - 0.0254 * 2.909});
        }
    )
    .OnPreUpdate(
      [&jointEntity, &PrescribedVel](
        const ignition::gazebo::UpdateInfo &_info,
        ignition::gazebo::EntityComponentManager &_ecm) {
          auto SimTime = std::chrono::duration<double>(_info.simTime).count();
          double piston_vel = -0.0254 * PrescribedVel.eval(SimTime);
          // Create new component for this entitiy in ECM (if it doesn't already
          // exist)
          auto joint_vel =
            _ecm.Component<components::JointVelocityCmd>(jointEntity);
          if (joint_vel == nullptr) {
            _ecm.CreateComponent(jointEntity,
              components::JointVelocityCmd(
                {piston_vel})); // Create this iteration
          } else {
            *joint_vel = components::JointVelocityCmd({piston_vel});
          }
        }
    )
    // Use post-update callback to get values at the end of every iteration
    .OnPostUpdate([&iterations, &jointEntity, &ResultsData](
      const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm)
    {
      auto SimTime = std::chrono::duration<double>(_info.simTime).count();

      auto prismaticJointVelComp =
        _ecm.Component<ignition::gazebo::components::JointVelocity>(
          jointEntity);
      if(prismaticJointVelComp == nullptr || prismaticJointVelComp->Data().empty()) {
        ResultsData.PistonVel.push_back(0.0);
      } else {
        double xdot = prismaticJointVelComp->Data().at(0);
        ResultsData.PistonVel.push_back(-xdot / 0.0254);
      }

      auto SpringStateComp =
        _ecm.Component<buoy_gazebo::components::SpringState>(
          jointEntity);
      if (SpringStateComp != nullptr) {
        auto SpringState = SpringStateComp->Data();

        ResultsData.seconds.push_back(SimTime);
        ResultsData.PistonPos.push_back(SpringState.range_finder / 0.0254);
        ResultsData.LowerSpringPressure.push_back(SpringState.lower_psi);
        ResultsData.UpperSpringPressure.push_back(SpringState.upper_psi);
        ResultsData.LowerSpringVolume.push_back(
          (2.03 - SpringState.range_finder) * 0.0115 + 0.0523);
        ResultsData.UpperSpringVolume.push_back(SpringState.range_finder * 0.0127 + 0.0266);
      }
      iterations++;
    })
    .Finalize();

  // Setup simulation server, this will call the post-update callbacks.
  // It also calls pre-update and update callbacks if those are being used.
  fixture.Server()->Run(true, InputData.seconds.back() / 0.01,
    false); // Hardcoded timestep that is set in sdf file
            // until I figure out how to get access...


  if (EXP_Data) { // Plot data for user to decide if it's valid.
    std::vector<int> select_time_series{1, 12, 13, 14, 15};
    for (int i = 1; i < 16; i++) {
      if(!std::binary_search(select_time_series.begin(), select_time_series.end(), i)){
        continue;
      }
      Gnuplot gp;
      gp << "set term X11 title  '" << InputData.names[i] << " Comparison'\n";
      gp << "set grid\n";
      gp << "set xlabel 'time (s)'\n";
      gp << "set ylabel '" << InputData.units[i] << "'\n";
      gp << "plot '-' w l title 'EXP " << InputData.names[i]
         << "','-' w l title 'TEST " << InputData.names[i] << "'\n";

      gp.send1d(boost::make_tuple(InputData.seconds, InputData(i)));
      gp.send1d(boost::make_tuple(ResultsData.seconds, ResultsData(i)));
    }

    std::vector<int> select_PV{12, 13};
    for (int i = 1; i < 16; i++) {
      if(!std::binary_search(select_PV.begin(), select_PV.end(), i)){
        continue;
      }
      Gnuplot gp;
      gp << "set term X11 title  '" << InputData.names[i] << " vs " << InputData.names[i+2] << " Comparison'\n";
      gp << "set grid\n";
      gp << "set xlabel '" << InputData.units[i+2] << "'\n";
      gp << "set ylabel '" << InputData.units[i] << "'\n";
      gp << "plot '-' w l title 'EXP "
         << "','-' w l title 'TEST " << "'\n";

      // gp.send1d(boost::make_tuple(InputData.seconds, InputData(i)));
      // gp.send1d(boost::make_tuple(ResultsData.seconds, ResultsData(i)));
      gp.send1d(boost::make_tuple(InputData(i+2), InputData(i)));
      gp.send1d(boost::make_tuple(ResultsData(i+2), ResultsData(i)));
    }

    std::cout << "Please examine plots and determine if they are acceptably "
      "correct.  Enter y/n"
      << std::endl;

    system("stty raw");    // Set terminal to raw mode
    char key = getchar();  // Wait for single character
    system("stty cooked"); // Reset terminal to normal "cooked" mode

    if (key == 'y') {
      std::string outputdata_filename =
        inputdata_filename.substr(0, inputdata_filename.size() - 4) + ".tst";

      std::ofstream outputdata(
        common::joinPaths(inputdata_dirname, outputdata_filename));
      if (outputdata.is_open()) {
        outputdata << header_line << ",LowerSpringVolume(cubic-m),UpperSpringVolume(cubic-m)" << std::endl;
        std::cout << "Writing test results to " << outputdata_filename
          << std::endl;
        for (int i = 0; i < ResultsData.seconds.size(); i++) {
          outputdata << ResultsData.seconds.at(i) << "  "
            << ResultsData.PistonPos.at(i) << "  "
            << ResultsData.PistonVel.at(i) << "  "
            << 0.0 << "  "
            << 0.0 << "  "
            << 0.0 << "  "
            << 0.0 << "  "
            << 0.0 << "  "
            << 0.0 << "  "
            << 0.0 << "  "
            << 0.0 << "  "
            << 0.0 << "  "
            << ResultsData.LowerSpringPressure.at(i) << "  "
            << ResultsData.UpperSpringPressure.at(i) << "  "
            << ResultsData.LowerSpringVolume.at(i) << "  "
            << ResultsData.UpperSpringVolume.at(i) << std::endl;
        }
      }
      outputdata.close();
    }
    FAIL() << "Running in manual mode with " << inputdata_filename
           << " as input, can't pass this way";
  } else {  // Compare test results to input data and pass test if so.

    double epsilon = 1.0;
    std::function<bool(const double &, const double &)> comparator =
      [&epsilon](const double &left, const double &right) { // Lambda function to compare 2 doubles
          //  std::cout << i << "  " <<  left << "  " << right << "  "  <<fabs(left-right) << std::endl;
          if (fabs(left - right) < epsilon) {
            return true;
          } else {
            return false;
          }
        };

    /*
    epsilon = 1e-2;
    if (!std::equal(InputData.PistonPos.begin(), InputData.PistonPos.end(),
      ResultsData.PistonPos.begin(), comparator))
    {
      FAIL() << "PistonPos Test Failed";
    }
    */

    epsilon = 1e-2;
    if (!std::equal(InputData.LowerSpringPressure.begin(), InputData.LowerSpringPressure.end(),
      ResultsData.LowerSpringPressure.begin(), comparator))
    {
      FAIL() << "LowerSpringPressure Test Failed";
    }

    epsilon = 1e-2;
    if (!std::equal(InputData.UpperSpringPressure.begin(), InputData.UpperSpringPressure.end(),
      ResultsData.UpperSpringPressure.begin(), comparator))
    {
      FAIL() << "UpperSpringPressure Test Failed";
    }

    epsilon = 1e-2;
    if (!std::equal(InputData.LowerSpringVolume.begin(), InputData.LowerSpringVolume.end(),
      ResultsData.LowerSpringVolume.begin(), comparator))
    {
      FAIL() << "LowerSpringVolume Test Failed";
    }

    epsilon = 1e-2;
    if (!std::equal(InputData.UpperSpringVolume.begin(), InputData.UpperSpringVolume.end(),
      ResultsData.UpperSpringVolume.begin(), comparator))
    {
      FAIL() << "UpperSpringVolume Test Failed";
    }
  }
}

