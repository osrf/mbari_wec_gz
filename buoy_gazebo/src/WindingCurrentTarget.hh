/*
 * Copyright (C) 2022 MBARI
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */


#ifndef WINDINGCURRENTTARGET_HH_
#define WINDINGCURRENTTARGET_HH_


#include <string>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <JustInterp/JustInterp.hpp>

//Defines from Controller Firmware, behavior replicated here.
#define TORQUE_CONSTANT 0.438   //0.62 N-m/ARMS  0.428N-m/AMPS Flux Current
#define CURRENT_CMD_RATELIMIT 200 //A/second.  Set to zero to disable feature.
#define TORQUE_CMD_TIMEOUT 2  //Torque Command Timeut, in seconds.   //Set to zero to disable timeout.
#define BIAS_CMD_TIMEOUT 10  //Bias Current Command Timeut, in seconds.    //Set to zero to disable timeout.
#define DEFAULT_SCALE_FACTOR 1.0 //-RPM on Kollemogen is +RPM here and extension
#define MAX_SCALE_FACTOR 1.4
#define MIN_SCALE_FACTOR 0.5
#define DEFAULT_RETRACT_FACTOR 0.6
#define MAX_RETRACT_FACTOR 1.0
#define MIN_RETRACT_FACTOR 0.4     
#define DEFAULT_BIASCURRENT 0.0  //Start with zero bias current
#define MAX_BIASCURRENT 20.0  //Maximum allowable winding bias current Magnitude that can be applied.
#define MAX_WINDCURRENTLIMIT 35 //Winding Current Limit, Amps.  Limit on internal target
#define SC_RANGE_MIN 0  //Inches
#define SC_RANGE_MAX 80  //Inches
#define STOP_RANGE 10  //Inches from SC_RANGE_MIN and SC_RANGE_MAX to increase generator torque
#define MAX_RPM_ADJUSTMENT 5000 //Maximum amount to modify RPM in determining WindingCurrentLimit near ends of stroke.


class WindingCurrentTarget
{
  public: double TorqueConstantNMPerAmp;   // N-m/Amp
  public: double TorqueConstantInLbPerAmp; // in-lb/Amp 
  public: JustInterp::LinearInterpolator<double> DefaultDamping;
  public: double ScaleFactor;
  public: double RetractFactor;
  public: double PistonPos;
  public: double SimTime;
  public: double UserCommandedCurrent;
  private: double UserCurrentSetSimTime;
  private: double UserCommandedCurrentTimeout;
  public: double BiasCurrent;
  private: double BiasCurrentSetSimTime;
  private: double BiasCurrentTimeout;

  /// \brief mutex to protect jointVelCmd
  public: std::mutex UserCommandMutex;


public:
  WindingCurrentTarget(void)
 {     
      std::vector<double> N{0.0, 300.0, 600.0, 1000.0, 1700.0, 4400.0, 6790.0};  //RPM
      std::vector<double> Torque{0.0, 0.0, 0.8, 2.9, 5.6, 9.8, 16.6};  //N-m
      this->DefaultDamping.SetData(N.size(),N.data(),Torque.data());  
      
      //Set Electric Motor Torque Constant
      this->TorqueConstantNMPerAmp = TORQUE_CONSTANT;   // N-m/Amp
      this->TorqueConstantInLbPerAmp = this->TorqueConstantNMPerAmp*8.851; // in-lb/Amp

      this->ScaleFactor = DEFAULT_SCALE_FACTOR;
      this->RetractFactor = DEFAULT_RETRACT_FACTOR;
      this->BiasCurrent = DEFAULT_BIASCURRENT;

      this-> BiasCurrentTimeout = BIAS_CMD_TIMEOUT;
      this-> UserCommandedCurrentTimeout = TORQUE_CMD_TIMEOUT;
}


void SetScaleFactor(double SF)
{
  this->UserCommandMutex.lock();
  if(SF < MIN_SCALE_FACTOR)
    this->ScaleFactor = MIN_SCALE_FACTOR;
  else if(SF > MAX_SCALE_FACTOR)
    this->ScaleFactor = MAX_SCALE_FACTOR;
  else
    this->ScaleFactor = SF;
  this->UserCommandMutex.unlock();
  return;
}

void SetRetractFactor(double RF)
{
  this->UserCommandMutex.lock();
  if(RF < MIN_RETRACT_FACTOR)
    this->RetractFactor = MIN_RETRACT_FACTOR;
  else if(RF > MAX_RETRACT_FACTOR)
    this->RetractFactor = MAX_RETRACT_FACTOR;
  else
    this->RetractFactor = RF;
  this->UserCommandMutex.unlock();
  return;
}


void SetUserCommandedCurrent(double I)
{

  this->UserCommandMutex.lock();
  std::cout << "## Adjusting UserCommanded Current "  << I << std::endl;

  if(I < -MAX_WINDCURRENTLIMIT)
    this->UserCommandedCurrent = -MAX_WINDCURRENTLIMIT;
  else if(I > MAX_WINDCURRENTLIMIT)
    this->UserCommandedCurrent = MAX_WINDCURRENTLIMIT;
  else
    this->UserCommandedCurrent = I;


    this->UserCurrentSetSimTime = this->SimTime;  //Arguably could pass the current sim time in here, but assuming the timeouts are larger than the timestep...
  this->UserCommandMutex.unlock();

  return;
}

void SetBiasCurrent(double BC)
{
  this->UserCommandMutex.lock();
  std::cout << "## Adjusting Bias Current "  << BC << std::endl;

  if(BC < -MAX_BIASCURRENT)
    this->BiasCurrent = -MAX_BIASCURRENT;
  else if(BC > MAX_BIASCURRENT)
    this->BiasCurrent = MAX_BIASCURRENT;
  else
    this->BiasCurrent = BC;

  this->BiasCurrentSetSimTime = this->SimTime;  //Arguably could pass the current sim time in here, but assuming the timeouts are larger than the timestep...
  this->UserCommandMutex.unlock();

  return;
}

double operator()(const double &N) const
    {
      double I;
        if((SimTime-UserCurrentSetSimTime) <  UserCommandedCurrentTimeout)
        {
        //this->UserCommandedCurrentMutex.lock();  //Can't do this b/c this () is const
          I = UserCommandedCurrent; 
        //this->UserCommandedCurrentMutex.unlock();
        }
        else
        {
        I = this->DefaultDamping(fabs(N))*this->ScaleFactor/this->TorqueConstantNMPerAmp;    //TODO.  1.375 makes this match experiment, not sure what is wrong...
        if(N > 0)
          I *= -this->RetractFactor;

        if((SimTime-BiasCurrentSetSimTime) <  BiasCurrentTimeout)
          I += BiasCurrent; 
        }



        return I;
    }
};



#endif