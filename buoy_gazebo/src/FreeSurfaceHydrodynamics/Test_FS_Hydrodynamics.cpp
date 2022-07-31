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

#include <Eigen/Dense>
#include "gnuplot-iostream.h"
#include "FS_Hydrodynamics.hpp"

//#include "EtaFunctor.hpp"
#include "LinearIncidentWave.hpp"

int main()
{
    LinearIncidentWave Inc;
    Inc.SetToPiersonMoskowitzSpectrum(1, 0);
    //Inc.SetToMonoChromatic(1,5,0);

    LinearIncidentWave& IncRef = Inc;
  
    FS_HydroDynamics BuoyA5(IncRef,1.0,9.81,1025);

    BuoyA5.ReadWAMITData_FD("HydrodynamicCoeffs/BuoyA5");
    BuoyA5.ReadWAMITData_TD("HydrodynamicCoeffs/BuoyA5");

 
    BuoyA5.Plot_FD_Coeffs();



    BuoyA5.SetTimestepSize(.01);

    BuoyA5.Plot_TD_Coeffs();
/*
std::cout << "m_tau_exc " << BuoyA5.m_tau_exc.transpose() << std::endl;

std::cout << "$$" << BuoyA5.m_IR_sinint(0,0).transpose() << std::endl << std::endl;
std::cout << "$$" << BuoyA5.m_L_rad(0,0).transpose() << std::endl << std::endl;

std::cout << "##" << BuoyA5.m_IR_exc(0).transpose() << std::endl << std::endl;
std::cout << "##" << BuoyA5.m_L_exc(0).transpose() << std::endl << std::endl;

std::cout << BuoyA5.m_IR_exc(0).size() << std::endl;
std::cout << BuoyA5.m_IR_sinint(0,0).size() << std::endl;
std::cout << BuoyA5.m_IR_sinint(1,1).size() << std::endl;
std::cout << BuoyA5.m_IR_exc(0).size() << std::endl << std::endl; 

std::cout << "m_tau_rad " << BuoyA5.m_tau_rad.transpose() << std::endl << std::endl;
std::cout << "m_tau_exc " << BuoyA5.m_tau_exc.transpose() << std::endl << std::endl;
*/
double tf = 50;

#if 0
Eigen::VectorXd xddot(6);
for(int k = 0;k<tf/BuoyA5.m_dt;k++)
{
double tt = BuoyA5.m_dt*k;
double T = 5;
double A = 1;
double pos = A*cos(2*M_PI*tt/T);
double vel = -A*(2*M_PI/T)*sin(2*M_PI*tt/T);
double accel = -A*pow(2*M_PI/T,2)*cos(2*M_PI*tt/T);
double last_accel = -A*pow(2*M_PI/T,2)*cos(2*M_PI*(tt-BuoyA5.m_dt)/T);
xddot(0) = last_accel;
xddot(1) = last_accel;
xddot(2) = last_accel;
xddot(3) = last_accel;
xddot(4) = last_accel;
xddot(5) = last_accel;
std::cout << BuoyA5.m_dt*k << "  "  << pos << "  " << vel << "  " << accel << "  ";
std::cout << BuoyA5.RadiationForce(xddot).transpose() << std::endl;
}


#endif
 
#if 1
for(int k = 0;k<tf/BuoyA5.m_dt;k++)
{
  double t = k*BuoyA5.m_dt;
std::cout << BuoyA5.m_dt*k << "  "  << Inc.eta(0,0,t) << "  " << -5*9.81*1025*Inc.eta(0,0,t) << "  ";
std::cout << BuoyA5.ExcitingForce()(2) << std::endl;
}
#endif
    return 0;
}
