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


#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>

#include <Eigen/Dense>
#include "LinearIncidentWave.hpp"
#include "gnuplot-iostream.h"


int main()
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


  return 0;
}
