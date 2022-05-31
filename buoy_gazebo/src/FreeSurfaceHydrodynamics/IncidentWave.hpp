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


#ifndef INCIDENTWAVE_HH_
#define INCIDENTWAVE_HH_

/** 
\brief The IncidentWave class is a base class for Incident Wave implementations, which must at least provide eta(x,y,t)
\author Andrew Hamilton
*/

class IncidentWave
{
    public:
    virtual double eta(double x, double y, double t) = 0;
};

#endif