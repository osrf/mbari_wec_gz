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
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <limits>

#include "FS_Hydrodynamics.hpp"
#include "LinearIncidentWave.hpp"

int main()
{
  const char * modes[6] = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"};
  LinearIncidentWave Inc;
  LinearIncidentWave & IncRef = Inc;
  double rho = 1025;
  double g = 9.81;
  double buoy_mass = 1400;  // kg
  FS_HydroDynamics BuoyA5(IncRef, 1.0, g, rho);
  BuoyA5.SetWaterplane(5.47, 1.37, 1.37);       // Set area and 2nd moments of area for waterplane
  BuoyA5.SetCOB(0, 0, -.22);       // Set COB relative to waterplane coordinate system.
  BuoyA5.SetCOG(0, 0, -.24);       // Set COG relative to waterplane coordinate system.
  BuoyA5.SetVolume(buoy_mass / rho);
  BuoyA5.ReadWAMITData_FD("HydrodynamicCoeffs/BuoyA5");
  BuoyA5.ReadWAMITData_TD("HydrodynamicCoeffs/BuoyA5");
  BuoyA5.Plot_FD_Coeffs();
  BuoyA5.SetTimestepSize(.01);
  BuoyA5.Plot_TD_Coeffs();

  std::srand((unsigned)time(0));
  double A = .5;       // .5 + ((float)(std::rand() % 20) / 10);
  double Tp = 4;       // 3.0 + (std::rand() % 9);
  double tf = 2.0 * Tp;
  double omega = 2 * M_PI / Tp;
  double phase = 20 * M_PI / 180;
  double dt = 0.01;

#if 1  // Test Radiation Forces
// Note:  This computes the radiation forces for each mode of motion indivdually, so
// MemRadiation is called 6 times per timestep, once with each acceleration set non-zero
// In use it will be called only once per time-step, with all acclerations set.
  {
    int first = 1;
    for (int i = 0; i < 6; i++) {             // i determines mode of motion.
      for (int j = 0; j < 6; j++) {           // j denotes direction of resulting force
        double am = BuoyA5.AddedMass(omega, i, j);
        double dmp = BuoyA5.Damping(omega, i, j);
        double am_inf = BuoyA5.fd_A_inf_freq(i, j);

        std::vector<double> pts_t, pts_F_TD, pts_F_FD;
        std::vector<double> pts_vel, pts_accel;
        double last_accel = 0;
        double F_max = -std::numeric_limits<double>::max();
        double F_min = std::numeric_limits<double>::max();
        BuoyA5.SetTimestepSize(dt);  // Reset timestep to re-initialize storage in BuoyA5 Class
        for (int k = 0; k < tf / dt; k++) {
          double tt = dt * k;
          pts_t.push_back(tt);
          // double pos = A * cos(omega * tt);  // Not needed, but handy to see...
          double vel = -A * omega * sin(omega * tt + phase);
          pts_vel.push_back(vel);
          double accel = -A * pow(omega, 2) * cos(omega * tt + phase);
          pts_accel.push_back(accel);
          Eigen::VectorXd xddot(6);
          for (int n = 0; n < 6; n++) {
            xddot(n) = 0;
          }
          xddot(i) = last_accel;

          Eigen::VectorXd MemForce(6);
          MemForce = BuoyA5.RadiationForce(xddot);
          pts_F_TD.push_back(am_inf * accel + MemForce(j));
          last_accel = accel;
          double FD_Force = am * accel + dmp * vel;
          pts_F_FD.push_back(FD_Force);
          if (FD_Force > F_max) {
            F_max = FD_Force;
          }
          if (FD_Force < F_min) {
            F_min = FD_Force;
          }
        }
        if (first) {
          Gnuplot gp;
          first = 0;
          char Amp[10];
          snprintf(Amp, sizeof(Amp), "%.1f", A);
          char Per[10];
          snprintf(Per, sizeof(Per), "%.1f", Tp);
          gp << "set term X11 title  'A = " << Amp << "m  T = " << Per << "s'\n";
          gp << "set grid\n";
          gp << "set xlabel 'time (s)'\n";
          gp << "plot '-' w l title 'Vel'" <<
            ",'-' w l title 'Accel'\n";
          gp.send1d(boost::make_tuple(pts_t, pts_vel));
          gp.send1d(boost::make_tuple(pts_t, pts_accel));
        }
        if ((F_min < -1) && (F_max > 1)) {                         // Don't plot near-zero forces
          Gnuplot gp;
          char Amp[10];
          snprintf(Amp, sizeof(Amp), "%.1f", A);
          char Per[10];
          snprintf(Per, sizeof(Per), "%.1f", Tp);
          gp << "set term X11 title 'Radiation Forces: " << modes[i] << " Motions, " << modes[j] <<
            " Forces:  A = " << Amp << "m  T = " << Per << "s  \n";
          gp << "set grid\n";
          gp << "set xlabel 'time (s)'\n";
          if (j < 3) {
            gp << "set ylabel 'F (N)'\n";
          } else {
            gp << "set  ylabel 'M (N-m)'\n";
          }
          gp << "plot '-' w l title 'Time-Domain'" <<
            ",'-' w l title 'Freq-Domain'\n";
          gp.send1d(boost::make_tuple(pts_t, pts_F_TD));
          gp.send1d(boost::make_tuple(pts_t, pts_F_FD));
          gp << "set xlabel 'time (s)'\n";
          if (j < 3) {
            gp << "set ylabel 'F (N)'\n";
          } else {
            gp << "set  ylabel 'M (N-m)'\n";
          }
          gp << "set title '" << modes[i] << "(t) = " << std::fixed << std::setprecision(1) <<
            Amp <<
            "cos(2 pi t/" << Per << ")'  \n";
          gp << "replot\n";
        }
      }
    }
  }
#endif

#if 1  // Test Exciting Forces
  {
    Inc.SetToMonoChromatic(2 * A, Tp, phase, 180 * M_PI / 180);

    for (int j = 0; j < 6; j++) {             // j denotes direction of resulting force
      std::complex<double> Chi = BuoyA5.WaveExcitingForceComponents(Inc.m_omega[0], j);
      std::vector<double> pts_t, pts_F_TD, pts_F_FD, pts_eta;
      BuoyA5.SetTimestepSize(dt);
      for (int k = 0; k < tf / dt; k++) {
        double tt = dt * k;
        pts_t.push_back(tt);
        pts_F_FD.push_back(
          Inc.m_A[0] * Chi.real() * cos(
            omega * tt + phase * cos(
              180 * M_PI / 180)) - Inc.m_A[0] * Chi.imag() *
          sin(omega * tt + phase * cos(180 * M_PI / 180)));
        pts_eta.push_back(Inc.eta(0, 0, tt));
        Eigen::VectorXd ExtForce(6);
        ExtForce = BuoyA5.ExcitingForce();
        pts_F_TD.push_back(ExtForce(j));
      }
      Gnuplot gp;
      gp << "set term X11 title  '" << modes[j] << " Exciting Forces'\n";
      gp << "set grid\n";
      gp << "set xlabel 'time (s)'\n";
      if (j < 3) {
        gp << "set ylabel 'F (N)'\n";
      } else {
        gp << "set  ylabel 'M (N-m)'\n";
      }
      gp << "plot '-' w l title 'Time-Domain'" <<
        ",'-' w l title 'Freq-Domain'" <<
        ",'-' w l title 'eta(t)'\n";
      gp.send1d(boost::make_tuple(pts_t, pts_F_TD));
      gp.send1d(boost::make_tuple(pts_t, pts_F_FD));
      gp.send1d(boost::make_tuple(pts_t, pts_eta));
      gp << "set xlabel 'time (s)'\n";
      if (j < 3) {
        gp << "set ylabel 'F (N)'\n";
      } else {
        gp << "set  ylabel 'M (N-m)'\n";
      }
      gp << "replot\n";
    }
  }
#endif

#if 1  // Test Buoyancy Forces
  {
    for (int j = 0; j < 6; j++) {             // j denotes direction of resulting force
      std::vector<double> pts_t, pts_x, pts_F_B;
      for (int k = 0; k < tf / dt; k++) {
        double tt = dt * k;
        pts_t.push_back(tt);
        Eigen::VectorXd x(6);
        x(0) = 0; x(1) = 0; x(2) = 0; x(3) = 0; x(4) = 0; x(5) = 0;
        x(j) = A * cos(omega * tt + phase);
        pts_x.push_back(x(j));
        Eigen::VectorXd BuoyancyForce(6);
        BuoyancyForce = BuoyA5.BuoyancyForce(x);
        pts_F_B.push_back(BuoyancyForce(j));
      }
      Gnuplot gp;
      gp << "set term X11 title  '" << modes[j] << " Buoyancy Forces'\n";
      gp << "set grid\n";
      gp << "set xlabel 'time (s)'\n";
      if (j < 3) {
        gp << "set ylabel 'F (N)'\n";
      } else {
        gp << "set  ylabel 'M (N-m)'\n";
      }
      gp << "plot '-' w l title 'x(t)'" <<
        ",'-' w l title 'Buoyancy Force'\n";
      gp.send1d(boost::make_tuple(pts_t, pts_x));
      gp.send1d(boost::make_tuple(pts_t, pts_F_B));
      gp << "set xlabel 'time (s)'\n";
      if (j < 3) {
        gp << "set ylabel 'F (N)'\n";
      } else {
        gp << "set  ylabel 'M (N-m)'\n";
      }
      gp << "replot\n";
    }
  }
#endif

#if 1  // Test Motions
  {
    BuoyA5.SetMass(buoy_mass);
    Eigen::Matrix<double, 3, 3> I;
    I << 1500, 0, 0,
      0, 1500, 0,
      0, 0, 650;
    BuoyA5.SetI(I);

    std::vector<double> pts_T;
    Eigen::Matrix<std::vector<double>, 6, 1> pts_ChiMod, pts_ChiPh;

    double dt = .1;
    for (double T = dt; T < 24; T += dt) {
      pts_T.push_back(T);
      double w = 2 * M_PI / T;
      auto Chi = BuoyA5.ComplexAmplitude(w);
      for (int j = 0; j < 6; j++) {
        pts_ChiMod(j).push_back(std::abs(Chi(j)));
        pts_ChiPh(j).push_back(std::arg(Chi(j)) * 180 / M_PI);
      }
    }

    for (int j = 0; j < 6; j++) {
      Gnuplot gp1;
      gp1 << "set term X11 title  '" << modes[j] << " RAO Amplitude'\n";
      gp1 << "set grid\n";
      gp1 << "set xlabel 'Wave Period (s)'\n";
      gp1 << "plot '-' w l \n";
      gp1.send1d(boost::make_tuple(pts_T, pts_ChiMod(j)));

      Gnuplot gp2;
      gp2 << "set term X11 title  '" << modes[j] << " RAO Phase Angle (deg)'\n";
      gp2 << "set grid\n";
      gp2 << "set xlabel 'Wave Period (s)'\n";
      gp2 << "plot '-' w l \n";
      gp2.send1d(boost::make_tuple(pts_T, pts_ChiPh(j)));
    }
  }
#endif
}
