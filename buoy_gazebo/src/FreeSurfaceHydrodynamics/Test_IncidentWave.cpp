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
  //Inc.SetToPiersonMoskowitzSpectrum(6, 0);
  Inc.SetToMonoChromatic(1, 12, 90*M_PI/180);

  std::cout << Inc << std::endl;

for(double t = 0; t<600; t+=.1)
   std::cout << t << "  " << Inc.eta(0,0,t) << "   " << Inc.etadot(0,0,t) << "  " << cos(2*M_PI*t/12) << std::endl;

  return 0;
}

