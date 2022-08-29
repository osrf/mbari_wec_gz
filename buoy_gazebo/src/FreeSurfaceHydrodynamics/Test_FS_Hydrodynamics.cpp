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
#include <ctime>

#include <Eigen/Dense>
#include "gnuplot-iostream.h"
#include "FS_Hydrodynamics.hpp"

//#include "EtaFunctor.hpp"
#include "LinearIncidentWave.hpp"
//#include <boost/tuple/tuple.hpp>
#include "gnuplot-iostream.h"

int main()
{
  const char *modes[6] = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"};
  LinearIncidentWave Inc;
  LinearIncidentWave &IncRef = Inc;
  FS_HydroDynamics BuoyA5(IncRef, 1.0, 9.81, 1025);
  BuoyA5.SetWaterplane(5.47,1.37,1.37); // Set area and 2nd moments of area for waterplane
  BuoyA5.SetCOB(0,0,-.18); //Set COB relative to waterplane coordinate system.
  BuoyA5.SetVolume(1.75); 
  BuoyA5.ReadWAMITData_FD("HydrodynamicCoeffs/BuoyA5");
  BuoyA5.ReadWAMITData_TD("HydrodynamicCoeffs/BuoyA5");
  BuoyA5.Plot_FD_Coeffs();
  BuoyA5.SetTimestepSize(.01);
  // BuoyA5.Plot_TD_Coeffs();

#if 1 // Test Radiation Forces
  srand((unsigned)time(0));
  double A = .5; //.5 + ((float)(rand() % 20) / 10);
  double T = 8; //3.0 + (rand() % 9);
  double tf = 3 * T;
  double omega = 2 * M_PI / T;
  int first = 1;
  for (int i = 0; i < 6; i++)   // i determines mode of motion.
    for (int j = 0; j < 6; j++) // j denotes direction of resulting force
    {
      double am = BuoyA5.AddedMass(omega, i, j);
      double dmp = BuoyA5.Damping(omega, i,  j);
      double am_inf = BuoyA5.fd_X_inf_freq(i, j);

      std::vector<double> pts_t,pts_F_TD,pts_F_FD;
      std::vector<double> pts_vel,pts_accel;
      double last_accel = 0;
      double F_max = -std::numeric_limits<double>::max();
      double F_min = std::numeric_limits<double>::max();
      for (int k = 0; k < tf / BuoyA5.m_dt; k++)
      {
        double tt = BuoyA5.m_dt * k;
        pts_t.push_back(tt);
        //double pos = A * cos(omega * tt);  //Not needed, but handy to see...
        double vel = -A * omega * sin(omega * tt);
        pts_vel.push_back(vel);
        double accel = -A * pow(omega, 2) * cos(omega * tt);
        pts_accel.push_back(accel);
        Eigen::VectorXd xddot(6);
        for (int n = 0; n < 6; n++)
          xddot(n) = 0;
        xddot(i) = last_accel;

        Eigen::VectorXd MemForce(6);
        MemForce = BuoyA5.RadiationForce(xddot);
        pts_F_TD.push_back(am_inf * accel + MemForce(j));
        last_accel = accel;
        double FD_Force = am * accel + dmp * vel;
        pts_F_FD.push_back(am * accel + dmp * vel); // FD_Force);
        if (FD_Force > F_max)
          F_max = FD_Force;
        if (FD_Force < F_min)
          F_min = FD_Force;
      }
      if(first)
      {
        Gnuplot gp;
        first = 0;
        char Amp[10];
        sprintf(Amp, "%.1f", A);
        char Per[10];
        sprintf(Per, "%.1f", T);
        gp << "set term X11 title  'A = " << Amp << "m  T = " << Per << "s'\n";
        gp << "set grid\n";
        gp << "set xlabel 'time (s)'\n";
        gp << "plot '-' w l title 'Vel'"
           << ",'-' w l title 'Accel'\n";
        gp.send1d(boost::make_tuple(pts_t, pts_vel));
        gp.send1d(boost::make_tuple(pts_t, pts_accel));
      }
      if ((F_min < -1) && (F_max > 1)) // Don't plot near-zero forces
      {
        Gnuplot gp;
        char Amp[10];
        sprintf(Amp, "%.1f", A);
        char Per[10];
        sprintf(Per, "%.1f", T);
        gp << "set term X11 title 'Radiation Forces: " << modes[i] << " Motions, " << modes[j] << " Forces:  A = " << Amp << "m  T = " << Per << "s  \n";
        gp << "set grid\n";
        gp << "set xlabel 'time (s)'\n";
        if (j < 3)
          gp << "set ylabel 'F (N)'\n";
        else
          gp << "set  ylabel 'M (N-m)'\n";
        gp << "plot '-' w l title 'Time-Domain'"
           << ",'-' w l title 'Freq-Domain'\n";
        gp.send1d(boost::make_tuple(pts_t, pts_F_TD));
        gp.send1d(boost::make_tuple(pts_t, pts_F_FD));
        gp << "set xlabel 'time (s)'\n";
        if (j < 3)
          gp << "set ylabel 'F (N)'\n";
        else
          gp << "set  ylabel 'M (N-m)'\n";
        gp << "set title '" << modes[i] << "(t) = " << std::fixed << std::setprecision(1) << Amp << "cos(2 pi t/" << Per << ")'  \n";
        gp << "replot\n";
      }
    }
#endif

#if 0 // Test Exciting Forces
  srand((unsigned)time(0));
  double A = .5 + ((float)(rand() % 20) / 10);
  double T = 3.0 + (rand() % 9);
  double tf = 5 * T;
  double omega = 2 * M_PI / T;

  //Inc.SetToPiersonMoskowitzSpectrum(2*A, 0);
  Inc.SetToMonoChromatic(2*A, T, 0);
  double XiRe, XiIm;

  for (int j = 0; j < 6; j++) // j denotes direction of resulting force
  {
    BuoyA5.WaveExcitingForceComponents(&XiRe, &XiIm, Inc.m_omega[0], j);
    std::vector<double> pts_t, pts_F_TD, pts_F_FD, pts_eta;
    for (int k = 0; k < tf / BuoyA5.m_dt; k++)
    {
      double tt = BuoyA5.m_dt * k;
      pts_t.push_back(tt);
      pts_F_FD.push_back(Inc.m_A[0] * XiRe * cos(omega * tt) - Inc.m_A[0] * XiIm * sin(omega * tt));
      pts_eta.push_back(Inc.eta(0, 0, tt));
      Eigen::VectorXd ExtForce(6);
      ExtForce = BuoyA5.ExcitingForce();
      pts_F_TD.push_back(ExtForce(j));
    }
    Gnuplot gp;
    gp << "set term X11 title  '" << modes[j] << " Exciting Forces'\n";
    gp << "set grid\n";
    gp << "set xlabel 'time (s)'\n";
    if (j < 3)
      gp << "set ylabel 'F (N)'\n";
    else
      gp << "set  ylabel 'M (N-m)'\n";
    gp << "plot '-' w l title 'Time-Domain'"
       << ",'-' w l title 'Freq-Domain'"
       << ",'-' w l title 'eta(t)'\n";
    gp.send1d(boost::make_tuple(pts_t, pts_F_TD));
    gp.send1d(boost::make_tuple(pts_t, pts_F_FD));
    gp.send1d(boost::make_tuple(pts_t, pts_eta));
    gp << "set xlabel 'time (s)'\n";
    if (j < 3)
      gp << "set ylabel 'F (N)'\n";
    else
      gp << "set  ylabel 'M (N-m)'\n";
    gp << "replot\n";
  }
#endif

#if 0 // Test Buoyancy Forces
  srand((unsigned)time(0));
  double A = .5 + ((float)(rand() % 20) / 10);
  double T = 6.0 + (rand() % 9);
  double tf = T;
  double omega = 2 * M_PI / T;


  for (int j = 0; j < 6; j++) // j denotes direction of resulting force
  {
    std::vector<double> pts_t, pts_x,pts_F_B;
    for (int k = 0; k < tf / BuoyA5.m_dt; k++)
    {
      double tt = BuoyA5.m_dt * k;
      pts_t.push_back(tt);
      Eigen::VectorXd x(6);
      x(0) = 0; x(1) = 0; x(2) = 0;x(3) = 0; x(4) = 0; x(5) = 0;
      x(j) = A*cos(omega*tt);
      pts_x.push_back(x(j));
      Eigen::VectorXd BuoyancyForce(6);
      BuoyancyForce = BuoyA5.BuoyancyForce(x);
      std::cout << "x = " << x.transpose() << std::endl;
      std::cout << "F_B = " << BuoyancyForce.transpose() << std::endl << std::endl;
      pts_F_B.push_back(BuoyancyForce(j));
    }
    Gnuplot gp;
    gp << "set term X11 title  '" << modes[j] << " Buoyancy Forces'\n";
    gp << "set grid\n";
    gp << "set xlabel 'time (s)'\n";
    if (j < 3)
      gp << "set ylabel 'F (N)'\n";
    else
      gp << "set  ylabel 'M (N-m)'\n";
    gp << "plot '-' w l title 'x(t)'"
       << ",'-' w l title 'Buoyancy Force'\n";
    gp.send1d(boost::make_tuple(pts_t, pts_x));
    gp.send1d(boost::make_tuple(pts_t, pts_F_B));
    gp << "set xlabel 'time (s)'\n";
    if (j < 3)
      gp << "set ylabel 'F (N)'\n";
    else
      gp << "set  ylabel 'M (N-m)'\n";
    gp << "replot\n";
  }
#endif
}
