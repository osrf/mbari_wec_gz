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

#include <ignition/common/Console.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/TestFixture.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>

#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>

#include <algorithm>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <buoy_gazebo/ElectroHydraulicPTO/ElectroHydraulicState.hpp>
#include <buoy_gazebo/PolytropicPneumaticSpring/SpringState.hpp>

#include <rclcpp/rclcpp.hpp>

#include <splinter_ros/splinter1d.hpp>


const double INCHES_TO_METERS{0.0254};


struct TestData
{
  enum Data
  {
    SECONDS = 0U,
    PISTON_POS = 1U,
    PISTON_VEL = 2U,
    MOTOR_RPM = 3U,
    LOWER_HYD_PRESSURE = 4U,
    UPPER_HYD_PRESSURE = 5U,
    V_BUS = 6U,
    WIND_CURR = 7U,
    BATT_CURR = 8U,
    LOAD_CURR = 9U,
    SCALE = 10U,
    RETRACT = 11U,
    LOWER_SPRING_PRESSURE = 12U,
    UPPER_SPRING_PRESSURE = 13U,
    LOWER_SPRING_VOLUME = 14U,
    UPPER_SPRING_VOLUME = 15U,
    NUM_VALUES = 16U
  };

  const char * names[16U] = {"seconds", "PistonPos", "PistonVel",
    "RPM", "LowerHydPressure", "UpperHydPressure",
    "V_Bus", "WindCurr", "BattCurr",
    "LoadCurr", "Scale", "Retract",
    "LowerSpringPressure", "UpperSpringPressure",
    "LowerSpringVolume", "UpperSpringVolume"};
  const char * units[16U] = {"seconds", "inches", "in/sec", "RPM", "psi_g",
    "psi_g", "Volts", "Amps", "Amps", "Amps",
    "", "", "psi_a", "psi_a", "cubic m", "cubic m"};

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
  std::vector<double> LowerSpringVolume;
  std::vector<double> UpperSpringVolume;

  double get_data_at(const int & data_num, const int & idx)
  {
    double ret_value = 0.0;
    try {
      std::vector<double> data = get_data(data_num);
      ret_value = data.at(idx);
    } catch (const std::out_of_range &) {
      // pass
    }
    return ret_value;
  }

  std::vector<double> get_data(const int & data_num)
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
    if (!strcmp(names[data_num], "Scale")) {
      ret_value = Scale;
    }
    if (!strcmp(names[data_num], "Retract")) {
      ret_value = Retract;
    }
    if (!strcmp(names[data_num], "LowerSpringPressure")) {
      ret_value = LowerSpringPressure;
    }
    if (!strcmp(names[data_num], "UpperSpringPressure")) {
      ret_value = UpperSpringPressure;
    }
    if (!strcmp(names[data_num], "LowerSpringVolume")) {
      ret_value = LowerSpringVolume;
    }
    if (!strcmp(names[data_num], "UpperSpringVolume")) {
      ret_value = UpperSpringVolume;
    }

    return ret_value;
  }
};

class BuoyExperimentComparison : public ::testing::Test
{
protected:
  static TestData InputData;
  static TestData ResultsData;
  static std::string inputdata_filename;
  static std::string header_line;
  static bool manual_comparison;
  static std::shared_ptr<splinter_ros::Splinter1d> PrescribedVel;
  static int argc_;
  static char ** argv_;
  static constexpr double stroke{2.03};
  static constexpr double lower_area{0.0115}, lower_dead_volume{0.0523};
  static constexpr double upper_area{0.0127}, upper_dead_volume{0.0266};
  std::unique_ptr<ignition::gazebo::TestFixture> fixture{nullptr};
  ignition::gazebo::Entity jointEntity{ignition::gazebo::kNullEntity};
  double epsilon{1e-2};
  std::function<bool(const double &, const double &)> comparator;

public:
  static void init(int argc, char * argv[])
  {
    argc_ = argc;
    argv_ = argv;
  }

protected:
  bool CompareData(const int & data_num, const double & _epsilon, const double & timestep)
  {
    epsilon = _epsilon;  // for comparator
    std::vector<double> idata = InputData.get_data(data_num);
    std::vector<double> rdata = ResultsData.get_data(data_num);
    auto mismatch = std::mismatch(idata.begin(), idata.end(), rdata.begin(), comparator);
    if (mismatch.first != idata.end())
    {
      std::cerr << InputData.names[data_num] << " Test Failed after ["
        << timestep * (mismatch.first - idata.begin()) << "] seconds with epsilon ["
        << epsilon << "]" << std::endl;
      return false;
    }
    return true;
  }

  // runs once and is preserved for all `TEST_F`
  static void SetUpTestCase()
  {
    rclcpp::init(argc_, argv_);
    rclcpp::Node::SharedPtr param_node = std::make_shared<rclcpp::Node>("experiment_comparison");
    rclcpp::spin_some(param_node);

    param_node->declare_parameter("inputdata_filename", "");
    inputdata_filename = param_node->get_parameter("inputdata_filename").as_string();
    param_node->declare_parameter("manual_comparison", false);
    manual_comparison = param_node->get_parameter("manual_comparison").as_bool();
    std::cerr << "inputdata_filename: " << inputdata_filename << std::endl;
    std::cerr << "manual_comparison: "
      << std::boolalpha << manual_comparison << std::noboolalpha
      << std::endl;
    rclcpp::shutdown();

    // Read Test Data
    std::ifstream testdata(inputdata_filename);
    if (testdata.is_open()) {
      testdata >> header_line;
      double data[14U];
      while (testdata >> data[TestData::SECONDS] >> data[TestData::PISTON_POS] >>
        data[TestData::PISTON_VEL] >> data[TestData::MOTOR_RPM] >>
        data[TestData::LOWER_HYD_PRESSURE] >> data[TestData::UPPER_HYD_PRESSURE] >>
        data[TestData::V_BUS] >> data[TestData::WIND_CURR] >> data[TestData::BATT_CURR] >>
        data[TestData::LOAD_CURR] >> data[TestData::SCALE] >> data[TestData::RETRACT] >>
        data[TestData::LOWER_SPRING_PRESSURE] >> data[TestData::UPPER_SPRING_PRESSURE])
      {
        InputData.seconds.push_back(data[TestData::SECONDS]);
        InputData.PistonPos.push_back(data[TestData::PISTON_POS]);
        InputData.PistonVel.push_back(data[TestData::PISTON_VEL]);
        InputData.RPM.push_back(data[TestData::MOTOR_RPM]);
        InputData.LowerHydPressure.push_back(data[TestData::LOWER_HYD_PRESSURE]);
        InputData.UpperHydPressure.push_back(data[TestData::UPPER_HYD_PRESSURE]);
        InputData.V_Bus.push_back(data[TestData::V_BUS]);
        InputData.WindCurr.push_back(data[TestData::WIND_CURR]);
        InputData.BattCurr.push_back(data[TestData::BATT_CURR]);
        InputData.LoadCurr.push_back(data[TestData::LOAD_CURR]);
        InputData.Scale.push_back(data[TestData::SCALE]);
        InputData.Retract.push_back(data[TestData::RETRACT]);
        InputData.LowerSpringPressure.push_back(data[TestData::LOWER_SPRING_PRESSURE]);
        InputData.UpperSpringPressure.push_back(data[TestData::UPPER_SPRING_PRESSURE]);
        InputData.LowerSpringVolume.push_back(
          (stroke - INCHES_TO_METERS * data[TestData::PISTON_POS]) *
          lower_area + lower_dead_volume);
        InputData.UpperSpringVolume.push_back(
          INCHES_TO_METERS * data[TestData::PISTON_POS] * upper_area + upper_dead_volume);
      }
      testdata.close();
    } else {
      FAIL() << "could not open input file " << inputdata_filename << std::endl;
    }

    // Copy initial condition to solution data
    ResultsData.seconds.push_back(InputData.seconds.at(0));
    ResultsData.PistonPos.push_back(InputData.PistonPos.at(0));
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
    ResultsData.LowerSpringPressure.push_back(InputData.LowerSpringPressure.at(0));
    ResultsData.UpperSpringPressure.push_back(InputData.UpperSpringPressure.at(0));
    ResultsData.LowerSpringVolume.push_back(InputData.LowerSpringVolume.at(0));
    ResultsData.UpperSpringVolume.push_back(InputData.UpperSpringVolume.at(0));

    PrescribedVel =
      std::make_shared<splinter_ros::Splinter1d>(InputData.seconds, InputData.PistonVel);
  }

  // runs before each `TEST_F`
  void SetUp() override
  {
    comparator =
      // Lambda function to compare 2 doubles
      [&](const double & left, const double & right) {
        if (fabs(left - right) < epsilon) {
          return true;
        } else {
          std::cerr << "failure: " << left << " - " << right << " > " << epsilon << std::endl;
          return false;
        }
      };

    // Skip debug messages to run faster
    ignition::common::Console::SetVerbosity(3);

    // Setup fixture
    ignition::gazebo::ServerConfig config;
    config.SetSdfFile("TestMachine.sdf");
    config.SetUpdateRate(0.0);
    fixture = std::make_unique<ignition::gazebo::TestFixture>(config);
    fixture->
      OnConfigure(
        [&](
          const ignition::gazebo::Entity & _worldEntity,
          const std::shared_ptr<const sdf::Element> &,
          ignition::gazebo::EntityComponentManager & _ecm,
          ignition::gazebo::EventManager &)
        {
          auto world = ignition::gazebo::World(_worldEntity);
          ignition::gazebo::Model pto(world.ModelByName(_ecm, "PTO"));
          jointEntity = pto.JointByName(_ecm, "HydraulicRam");

          EXPECT_NE(ignition::gazebo::kNullEntity, jointEntity);

          std::cerr << "Initializing piston position to ["
            << stroke - INCHES_TO_METERS * InputData.PistonPos.at(0)
            << "] meters (or [" << InputData.PistonPos.at(0) << "] inches)" << std::endl;
          _ecm.SetComponentData<ignition::gazebo::components::JointPositionReset>(
            jointEntity,
            {stroke - INCHES_TO_METERS * InputData.PistonPos.at(0)});
        }).
      OnPreUpdate(
        [&](
          const ignition::gazebo::UpdateInfo & _info,
          ignition::gazebo::EntityComponentManager & _ecm)
        {
          auto SimTime = std::chrono::duration<double>(_info.simTime).count();
          double piston_vel =
            -INCHES_TO_METERS *
            PrescribedVel->eval(SimTime,
              splinter_ros::FILL_VALUE,
              std::vector<double>(2U, 0.0));
          // Create new component for this entitiy in ECM (if it doesn't already
          // exist)
          auto joint_vel =
          _ecm.Component<ignition::gazebo::components::JointVelocityCmd>(jointEntity);
          if (joint_vel == nullptr) {
            _ecm.CreateComponent(
              jointEntity,
              ignition::gazebo::components::JointVelocityCmd(
                {piston_vel}));  // Create this iteration
          } else {
            *joint_vel = ignition::gazebo::components::JointVelocityCmd({piston_vel});
          }
        }).
      OnPostUpdate(
        [&](
          const ignition::gazebo::UpdateInfo & _info,
          const ignition::gazebo::EntityComponentManager & _ecm)
        {
          auto SimTime = std::chrono::duration<double>(_info.simTime).count();

          auto prismaticJointVelComp =
          _ecm.Component<ignition::gazebo::components::JointVelocity>(
            jointEntity);
          if (prismaticJointVelComp == nullptr || prismaticJointVelComp->Data().empty()) {
            ResultsData.PistonVel.push_back(0.0);
          } else {
            double xdot = prismaticJointVelComp->Data().at(0);
            ResultsData.PistonVel.push_back(-xdot / INCHES_TO_METERS);
          }

          auto SpringStateComp =
          _ecm.Component<buoy_gazebo::components::SpringState>(
            jointEntity);
          if (SpringStateComp != nullptr) {
            auto SpringState = SpringStateComp->Data();

            ResultsData.seconds.push_back(SimTime);
            ResultsData.PistonPos.push_back(SpringState.range_finder / INCHES_TO_METERS);
            ResultsData.LowerSpringPressure.push_back(SpringState.lower_psi);
            ResultsData.UpperSpringPressure.push_back(SpringState.upper_psi);
            ResultsData.LowerSpringVolume.push_back(
              (stroke - SpringState.range_finder) * lower_area + lower_dead_volume);
            ResultsData.UpperSpringVolume.push_back(
              SpringState.range_finder * upper_area + upper_dead_volume);
          }

          auto PTO_State_comp =
          _ecm.Component<buoy_gazebo::components::ElectroHydraulicState>(
            jointEntity);
          if (PTO_State_comp != nullptr) {
            auto PTO_State = PTO_State_comp->Data();

            ResultsData.RPM.push_back(PTO_State.rpm);
            // Todo, diff_press isn't meant to be hydraulic forces, maybe
            // need to add some fields to PTO_State for items of interest
            // that aren't reported over CAN.
            if (PTO_State.diff_press > 0.0)
            {
              ResultsData.LowerHydPressure.push_back(-PTO_State.diff_press);
              ResultsData.UpperHydPressure.push_back(0.0);
            } else {
              ResultsData.LowerHydPressure.push_back(0.0);
              ResultsData.UpperHydPressure.push_back(-PTO_State.diff_press);
            }
            ResultsData.V_Bus.push_back(PTO_State.voltage);
            ResultsData.WindCurr.push_back(PTO_State.wcurrent);
            ResultsData.BattCurr.push_back(PTO_State.bcurrent);
            ResultsData.LoadCurr.push_back(PTO_State.loaddc);
          }
        }).
      Finalize();
  }

  // runs after each `TEST_F`
  void TearDown() override
  {
  }

  // runs once after all `TEST_F` have completed
  static void TearDownTestCase()
  {
    if (manual_comparison) {
      std::cout << "Please examine plots and determine if they are acceptably "
        "correct.  Enter y/n" << std::endl;

      system("stty raw");     // Set terminal to raw mode
      char key = getchar();   // Wait for single character
      system("stty cooked");  // Reset terminal to normal "cooked" mode

      if (key == 'y') {
        std::string outputdata_filename =
          inputdata_filename.substr(0U, inputdata_filename.size() - 4U) + ".tst";

        std::ofstream outputdata(outputdata_filename);
        if (outputdata.is_open()) {
          outputdata << header_line << std::endl;
          std::cout << "Writing [" << ResultsData.seconds.size() << "] test results to " << outputdata_filename << std::endl;
          for (size_t idx = 0U; idx < ResultsData.seconds.size(); ++idx) {
            for (size_t jdx = 0U; jdx <= TestData::UPPER_SPRING_PRESSURE; ++jdx) {
              outputdata << ResultsData.get_data_at(jdx, idx) << " ";
            }
            outputdata << std::endl;
          }
        }
        outputdata.close();
      }
      FAIL() << "Running in manual mode with " << inputdata_filename <<
        " as input, can't pass this way";
    }
  }
};

TestData BuoyExperimentComparison::InputData;
TestData BuoyExperimentComparison::ResultsData;
std::string BuoyExperimentComparison::inputdata_filename{""};
std::string BuoyExperimentComparison::header_line{""};
bool BuoyExperimentComparison::manual_comparison{false};
std::shared_ptr<splinter_ros::Splinter1d> BuoyExperimentComparison::PrescribedVel{nullptr};
int BuoyExperimentComparison::argc_;
char ** BuoyExperimentComparison::argv_;


TEST_F(BuoyExperimentComparison, Spring)
{
  // Hardcoded timestep that is set in sdf file
  // until I figure out how to get access...
  double timestep(0.01);
  bool blocking(true), paused(false);
  fixture->Server()->Run(
    blocking, InputData.seconds.back() / timestep,
    paused);


  if (manual_comparison) {  // Plot data for user to decide if it's valid.
    std::vector<size_t> select_time_series{TestData::PISTON_POS,
      TestData::LOWER_SPRING_PRESSURE, TestData::UPPER_SPRING_PRESSURE,
      TestData::LOWER_SPRING_VOLUME, TestData::UPPER_SPRING_VOLUME};
    for (size_t i = 1U; i < TestData::NUM_VALUES; i++) {
      if (!std::binary_search(select_time_series.begin(), select_time_series.end(), i)) {
        continue;
      }
      Gnuplot gp;
      gp << "set term X11 title  '" << InputData.names[i] << " Comparison'\n";
      gp << "set grid\n";
      gp << "set xlabel 'time (s)'\n";
      gp << "set ylabel '" << InputData.units[i] << "'\n";
      gp << "plot '-' w l title 'EXP " << InputData.names[i] <<
        "','-' w l title 'TEST " << InputData.names[i] << "'\n";

      gp.send1d(boost::make_tuple(InputData.seconds, InputData.get_data(i)));
      gp.send1d(boost::make_tuple(ResultsData.seconds, ResultsData.get_data(i)));
    }

    std::vector<size_t> select_PV{TestData::LOWER_SPRING_PRESSURE,
      TestData::UPPER_SPRING_PRESSURE};
    for (size_t i = 1U; i < TestData::NUM_VALUES; i++) {
      if (!std::binary_search(select_PV.begin(), select_PV.end(), i)) {
        continue;
      }
      Gnuplot gp;
      gp << "set term X11 title  '" << InputData.names[i] << " vs " << InputData.names[i + 2U] <<
        " Comparison'\n";
      gp << "set grid\n";
      gp << "set xlabel '" << InputData.units[i + 2U] << "'\n";
      gp << "set ylabel '" << InputData.units[i] << "'\n";
      gp << "plot '-' w l title 'EXP " <<
        "','-' w l title 'TEST " << "'\n";

      gp.send1d(boost::make_tuple(InputData.get_data(i + 2U), InputData.get_data(i)));
      gp.send1d(boost::make_tuple(ResultsData.get_data(i + 2U), ResultsData.get_data(i)));
    }
  } else {  // Compare test results to input data and pass test if so.
    EXPECT_TRUE(CompareData(TestData::PISTON_POS, 1e-2, timestep));
    EXPECT_TRUE(CompareData(TestData::LOWER_SPRING_PRESSURE, 1e-2, timestep));
    EXPECT_TRUE(CompareData(TestData::UPPER_SPRING_PRESSURE, 1e-2, timestep));
    EXPECT_TRUE(CompareData(TestData::LOWER_SPRING_VOLUME, 1e-2, timestep));
    EXPECT_TRUE(CompareData(TestData::UPPER_SPRING_VOLUME, 1e-2, timestep));
  }
}


int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  BuoyExperimentComparison::init(argc, argv);
  return RUN_ALL_TESTS();
}
