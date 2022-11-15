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


#include <buoy_gazebo/ElectroHydraulicPTO/WindingCurrentTarget.hpp>
#include <gnuplot-iostream.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <cstring>
#include <iostream>
#include <memory>
#include <vector>

class EHWindTarget : public ::testing::Test
{
protected:
  static bool manual;
  static int argc_;
  static char ** argv_;

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
    rclcpp::Node::SharedPtr param_node = std::make_shared<rclcpp::Node>("eh_windtarget");
    rclcpp::spin_some(param_node);

    param_node->declare_parameter("manual", false);
    manual = param_node->get_parameter("manual").as_bool();
    std::cerr << "manual: " <<
      std::boolalpha << manual << std::noboolalpha <<
      std::endl;
  }

  static void TearDownTestCase()
  {
    if (manual) {
      rclcpp::spin(std::make_shared<rclcpp::Node>("wait_for_exit"));
    }
  }
};
bool EHWindTarget::manual{false};
int EHWindTarget::argc_;
char ** EHWindTarget::argv_;


//////////////////////////////////////////////////
TEST_F(EHWindTarget, GENERATOR_MODE)
{
  double RetractFactor = 1.0;

  WindingCurrentTarget I_Wind;
  for (double ScaleFactor = .6; ScaleFactor <= 1.5; ScaleFactor += .4) {
    for (double RetractFactor = .6; RetractFactor <= 1.1; RetractFactor += .4) {
      for (double BiasCurr = -10.0; BiasCurr < 10.1; BiasCurr += 10.0) {
        I_Wind.bias_override_ = (fabs(BiasCurr) > .1);
        I_Wind.BiasCurrent = BiasCurr;
        I_Wind.ScaleFactor = ScaleFactor;
        I_Wind.RetractFactor = RetractFactor;

        Gnuplot gp;
        std::vector<double> PistonPos = {0, 5, 10, 40, 70, 75, 80};

        if (manual) {
          gp << "set term X11 title  'Gen Mode: " <<
            "Scale = " << std::to_string(ScaleFactor) <<
            "  Retract = " << std::to_string(RetractFactor) <<
            "  BiasCurr = " << std::to_string(BiasCurr) <<
            "'\n";
          gp << "set grid\n";
          gp << "set xlabel 'RPM'\n";
          gp << "set ylabel 'Amps'\n";
          gp << "plot '-' w l title 'PistonPos = " << std::to_string(PistonPos[0]) << " in'" <<
            ",'-' w l title 'PistonPos = " << std::to_string(PistonPos[1]) << " in'" <<
            ",'-' w l title 'PistonPos = " << std::to_string(PistonPos[2]) << " in'" <<
            ",'-' w l title 'PistonPos = " << std::to_string(PistonPos[3]) << " in'" <<
            ",'-' w l title 'PistonPos = " << std::to_string(PistonPos[4]) << " in'" <<
            ",'-' w l title 'PistonPos = " << std::to_string(PistonPos[5]) << " in'" <<
            ",'-' w l title 'PistonPos = " << std::to_string(PistonPos[6]) << " in'" <<
            "\n";
        }

        for (int k = 0; k < PistonPos.size(); k++) {
          I_Wind.RamPosition = PistonPos[k];
          std::vector<double> pts_N;
          std::vector<double> pts_I;
          for (double N = -6500; N <= 6500; N += 250) {                                  // RPM
            double WindCurr = I_Wind(N);
            pts_N.push_back(N);
            pts_I.push_back(WindCurr);
          }

          if (manual) {
            gp.send1d(boost::make_tuple(pts_N, pts_I));
          } else {
            EXPECT_EQ(1.0, 1.0);
            EXPECT_NE(1.0, 0.0);
          }
        }
      }
    }
  }
}

//////////////////////////////////////////////////
TEST_F(EHWindTarget, TORQUE_MODE)
{
  double ScaleFactor = 1.0;
  double RetractFactor = 0.6;

  WindingCurrentTarget I_Wind;

  I_Wind.ScaleFactor = ScaleFactor;
  I_Wind.RetractFactor = RetractFactor;

  if (manual) {
    std::vector<double> PistonPos = {0.0, 5.0, 10.0, 40.0, 70.0, 75.0, 80.0};
    for (int k = 0; k < PistonPos.size(); k++) {
      std::vector<double> pts_N;
      std::vector<double> pts_Icommand;
      std::vector<double> pts_Itarg;
      I_Wind.RamPosition = PistonPos[k];
      for (double I = -36.0; I <= 36.1; I += 2.0) {
        I_Wind.current_override_ = true;
        I_Wind.UserCommandedCurrent = I;
        for (double N = -6500.0; N <= 6500.1; N += 500.0) {  // RPM
          double WindCurr = I_Wind(N);
          pts_N.push_back(N);
          pts_Icommand.push_back(I);
          pts_Itarg.push_back(WindCurr);
        }
      }

      Gnuplot gp;
      gp << "set term X11 title  'Torque Mode: PistonPos = " << std::to_string(PistonPos[k]) <<
        " in'\n";
      gp << "set grid\n";
      gp << "set xlabel 'RPM'\n";
      gp << "set ylabel 'Amps'\n";
      gp << "plot '-' w p title 'Commanded Winding Current' pt 6" <<
        ",'-' w p title 'Target Winding Current' pt 2" <<
        "\n";
      gp.send1d(boost::make_tuple(pts_N, pts_Icommand));
      gp.send1d(boost::make_tuple(pts_N, pts_Itarg));
    }
  } else {
    EXPECT_EQ(1.0, 1.0);
    EXPECT_NE(1.0, 0.0);
  }
}


int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  EHWindTarget::init(argc, argv);  // pass args to rclcpp init
  return RUN_ALL_TESTS();
}
