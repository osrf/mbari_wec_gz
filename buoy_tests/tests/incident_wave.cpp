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

#include <buoy_gazebo/FreeSurfaceHydrodynamics/LinearIncidentWave.hpp>
#include <gnuplot-iostream.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <memory>


class IncidentWaveTarget : public ::testing::Test
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
    rclcpp::Node::SharedPtr param_node = std::make_shared<rclcpp::Node>("indicent_wavetarget");
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
bool IncidentWaveTarget::manual{false};
int IncidentWaveTarget::argc_;
char ** IncidentWaveTarget::argv_;


//////////////////////////////////////////////////
TEST_F(IncidentWaveTarget, MONOCHROME)
{
  LinearIncidentWave Inc;

  // Inc.SetToPiersonMoskowitzSpectrum(1, 0, 300);
  // Inc.SetToPiersonMoskowitzSpectrum(6, 0);
  double A = 1;
  double T = 12;
  double phase = 0 * M_PI / 180;
  double beta = 180 * M_PI / 180;
  Inc.SetToMonoChromatic(A, T, phase, beta);
  double k = pow(2 * M_PI / T, 2) / 9.81;
  std::cout << Inc << std::endl;

  {
    std::vector<double> pts_t;
    std::vector<double> pts_eta, pts_eta_true;
// std::vector<double> pts_etadot, pts_etadot_true;
    double x = 0;
    double y = 0;
    double xx = x * cos(beta) + y * sin(beta);
    for (double t = 0; t < 4 * T; t += .1) {
      pts_t.push_back(t);
      pts_eta.push_back(Inc.eta(x, y, t));
      pts_eta_true.push_back(A * cos(k * xx - 2 * M_PI * t / T + phase));
    }
    Gnuplot gp;
    gp << "set term X11 title  'Incident Wave Elevation at Origin'\n";
    gp << "set grid\n";
    gp << "set xlabel 'time (s)'\n";
    gp << "set ylabel '(m)'\n";
    gp << "plot '-' w l title 'eta'" <<
      ",'-' w l title 'eta\\_true'\n";
    gp.send1d(boost::make_tuple(pts_t, pts_eta));
    gp.send1d(boost::make_tuple(pts_t, pts_eta_true));
  }

  double dt = .05 * T;
  double y = 0;
  for (double t = 0; t <= 3 * dt; t += dt) {
    std::vector<double> pts_x;
    std::vector<double> pts_eta, pts_eta_true;
// std::vector<double> pts_etadot, pts_etadot_true;
    for (double x = -1.5 * 2 * M_PI / k; x < 1.5 * 2 * M_PI / k; x += .1) {
      pts_x.push_back(x);
      pts_eta.push_back(Inc.eta(x, y, t));
      double xx = x * cos(beta) + y * sin(beta);
      pts_eta_true.push_back(A * cos(k * xx - 2 * M_PI * t / T + phase));
    }
    char time[10];
    snprintf(time, sizeof(time), "%.2f", t);
    Gnuplot gp;
    gp << "set term X11 title  'Incident Wave Elevation at t = " << time << " s'\n";
    gp << "set grid\n";
    gp << "set xlabel '(m))'\n";
    gp << "set ylabel '(m)'\n";
    gp << "plot '-' w l title 'eta'" <<
      ",'-' w l title 'eta\\_true'\n";
    gp.send1d(boost::make_tuple(pts_x, pts_eta));
    gp.send1d(boost::make_tuple(pts_x, pts_eta_true));
  }
}


int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  IncidentWaveTarget::init(argc, argv);  // pass args to rclcpp init
  return RUN_ALL_TESTS();
}
