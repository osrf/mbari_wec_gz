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

#include <gnuplot-iostream.h>
#include <rclcpp/rclcpp.hpp>

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

#include <splinter_ros/splinter1d.hpp>


#define CompareData(DATA, EPSILON, TIMESTEP) \
    epsilon = EPSILON; \
    auto mismatch_ ## DATA = std::mismatch(InputData.DATA.begin(), InputData.DATA.end(), \
      ResultsData.DATA.begin(), comparator); \
    if (mismatch_ ## DATA.first != InputData.DATA.end()) \
    { \
      FAIL() << #DATA " Test Failed after [" \
      << TIMESTEP * (mismatch_ ## DATA.first - InputData.DATA.begin()) << "] seconds"; \
    }

struct TestData
{
  const char * names[16] = {"seconds", "PistonPos", "PistonVel",
    "RPM", "LowerHydPressure", "UpperHydPressure",
    "V_Bus", "WindCurr", "BattCurr",
    "LoadCurr", "Scale", "Retract",
    "LowerSpringPressure", "UpperSpringPressure",
    "LowerSpringVolume", "UpperSpringVolume"};
  const char * units[16] = {"seconds", "inches", "in/sec", "RPM", "psi_g",
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
  static rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  static std::thread thread_executor_spin_;
  static bool stop_;
  static int argc_;
  static char ** argv_;
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
  // runs once and is preserved for all `TEST_F`
  static void SetUpTestCase()
  {
    rclcpp::init(argc_, argv_);
    rclcpp::Node::SharedPtr param_node = std::make_shared<rclcpp::Node>("experiment_comparison");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(param_node);
    stop_ = false;
    auto spin = [&]()
      {
        while (rclcpp::ok() && !stop_) {
          executor_->spin_once();
        }
      };
    thread_executor_spin_ = std::thread(spin);

    param_node->declare_parameter("inputdata_filename", "");
    inputdata_filename = param_node->get_parameter("inputdata_filename").as_string();
    param_node->declare_parameter("manual_comparison", false);
    manual_comparison = param_node->get_parameter("manual_comparison").as_bool();
    std::cerr << "inputdata_filename: " << inputdata_filename << std::endl;
    std::cerr << "manual_comparison: " << manual_comparison << std::endl;

    // Read Test Data
    std::ifstream testdata(inputdata_filename);
    if (testdata.is_open()) {
      testdata >> header_line;
      double data[14U];
      while (testdata >> data[0U] >> data[1U] >> data[2U] >> data[3U] >> data[4U] >>
        data[5U] >> data[6U] >> data[7U] >> data[8U] >> data[9U] >>
        data[10U] >> data[11U] >> data[12U] >> data[13U])
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
        InputData.LowerSpringVolume.push_back((2.03 - 0.0254 * data[1U]) * 0.0115 + 0.0523);
        InputData.UpperSpringVolume.push_back(0.0254 * data[1U] * 0.0127 + 0.0266);
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
    std::function<bool(const double &, const double &)> comparator =
      // Lambda function to compare 2 doubles
      [&](const double & left, const double & right) {
        if (fabs(left - right) < epsilon) {
          return true;
        } else {
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
          
          _ecm.SetComponentData<ignition::gazebo::components::JointPositionReset>(
            jointEntity,
            {2.03 - 0.0254 * InputData.PistonPos.at(0)});
        }).
      OnPreUpdate(
        [&](
          const ignition::gazebo::UpdateInfo & _info,
          ignition::gazebo::EntityComponentManager & _ecm)
        {
          auto SimTime = std::chrono::duration<double>(_info.simTime).count();
          double piston_vel =
            -0.0254 *
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
    stop_ = true;
    rclcpp::shutdown();
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
          for (size_t i = 0U; i < ResultsData.seconds.size(); i++) {
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
      std::string foo;
      std::cin >> foo;
    }
  }
};

TestData BuoyExperimentComparison::InputData;
TestData BuoyExperimentComparison::ResultsData;
std::string BuoyExperimentComparison::inputdata_filename{""};
std::string BuoyExperimentComparison::header_line{""};
bool BuoyExperimentComparison::manual_comparison{false};
std::shared_ptr<splinter_ros::Splinter1d> BuoyExperimentComparison::PrescribedVel{nullptr};
rclcpp::executors::MultiThreadedExecutor::SharedPtr BuoyExperimentComparison::executor_{nullptr};
std::thread BuoyExperimentComparison::thread_executor_spin_;
bool BuoyExperimentComparison::stop_{false};
int BuoyExperimentComparison::argc_;
char ** BuoyExperimentComparison::argv_;


TEST_F(BuoyExperimentComparison, Spring)
{
      //TODO: TESTS ARE DEPENDENT ON EACH OTHER STORING DATA AND WRITING TO FILE

  // Setup simulation server, this will call the post-update callbacks.
  // It also calls pre-update and update callbacks if those are being used.
  // Hardcoded timestep that is set in sdf file
  // until I figure out how to get access...
  double timestep(0.01);
  bool blocking(true), paused(false);
  fixture->Server()->Run(
    blocking, InputData.seconds.back() / timestep,
    paused);


  if (manual_comparison) {  // Plot data for user to decide if it's valid.
    std::vector<size_t> select_time_series{1U, 12U, 13U, 14U, 15U};
    for (size_t i = 1U; i < 16U; i++) {
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

      gp.send1d(boost::make_tuple(InputData.seconds, InputData(i)));
      gp.send1d(boost::make_tuple(ResultsData.seconds, ResultsData(i)));
    }

    std::vector<size_t> select_PV{12U, 13U};
    for (size_t i = 1U; i < 16U; i++) {
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

      gp.send1d(boost::make_tuple(InputData(i + 2U), InputData(i)));
      gp.send1d(boost::make_tuple(ResultsData(i + 2U), ResultsData(i)));
    }
  } else {  // Compare test results to input data and pass test if so.
    CompareData(PistonPos, 1e-2, timestep);
    CompareData(LowerSpringPressure, 1e-2, timestep);
    CompareData(UpperSpringPressure, 1e-2, timestep);
    CompareData(LowerSpringVolume, 1e-2, timestep);
    CompareData(UpperSpringVolume, 1e-2, timestep);
  }
}


int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  BuoyExperimentComparison::init(argc, argv);
  return RUN_ALL_TESTS();
}
