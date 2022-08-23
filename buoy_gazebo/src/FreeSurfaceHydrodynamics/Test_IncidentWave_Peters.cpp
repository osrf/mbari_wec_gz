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

using namespace Eigen;


int main()
{
  LinearIncidentWave Inc;

  //Inc.SetToPiersonMoskowitzSpectrum(1, 0, 300);
  Inc.SetToPiersonMoskowitzSpectrum(6, 0);
  //Inc.SetToMonoChromatic(1, 12, 90*M_PI/180);

//  std::cout << Inc << std::endl;
double eta_max = 0;
double eta_min = 0;
double eta_mean = 0;
double eta_std = 0;
double M = 0;
double Var;

double dt = .1;
double tf = 5000;

for(int k = 0; k < tf/dt; k++)
{
  double t= dt*k;
  double eta = Inc.eta(0,0,t);
  if(eta < eta_min)  
    eta_min = eta;
  if(eta > eta_max)  
    eta_max = eta;
  double eta_mean_last = eta_mean;
  eta_mean = eta_mean_last+(eta-eta_mean_last)/(k+1);

M = M + (eta-eta_mean_last)*(eta-eta_mean);

if(k > 0)
  Var = M/k;
else
  Var = 0;

   std::cout << t << "  " << Inc.eta(0,0,t) << "   "  << eta_min << "  " << eta_max << "   " << eta_mean << "  " << Var << std::endl;
}


  return 0;
}

