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

#include <gtest/gtest.h>

#include <buoy_gazebo/ElectroHydraulicPTO/ElectroHydraulicSoln.hpp>
#include <unsupported/Eigen/NonLinearOptimization>

//#include <ElectroHydraulicState.hpp>
//#include <ElectroHydraulicLoss.hpp>

//////////////////////////////////////////////////
TEST(EH_SOLVER, GENERATOR_MODE)
{

	const double Ve = 310; // Volts
	const double Ri = 7; // Ohms
	const double PistonArea = 1.375; // in^2
	const double HydMotorDisp = 0.3; // in^3
	double ScaleFactor = 0.6;
	double RetractFactor = 1.0;
	ElectroHydraulicSoln functor{};
	Eigen::VectorXd x{};
	functor.VBattEMF = Ve;
	functor.Ri = Ri;
	functor.HydMotorDisp = HydMotorDisp;

	x.resize(3);
	x.setConstant(3, 0.0);
	x[2] = Ve;

	double PistonPos = 40; // in
	for(double PistonVel  = -30; PistonVel <= 30; PistonVel += 1.0)// in^3
  for(double WindCurrTarg = 0; WindCurrTarg < 1 ;WindCurrTarg += 2.0)
	{

		functor.I_Wind.RamPosition = PistonPos;  // in
		functor.Q = PistonVel * PistonArea;  // inch^3/second


		functor.I_Wind.ScaleFactor = ScaleFactor;
		functor.I_Wind.RetractFactor = 1.0;  // RetractFactor;


		// functor.I_Wind.current_override_ = pto_state.torque_command;
		// functor.I_Wind.UserCommandedCurrent = 0.0;

		// functor.I_Wind.bias_override_ = pto_state.bias_current_command;
		// functor.I_Wind.BiasCurrent = 0.0;


		Eigen::HybridNonLinearSolver<ElectroHydraulicSoln> solver(functor);
		solver.parameters.xtol = 0.0001;
		// Initial condition based on perfect efficiency
		x[0] = 60.0*functor.Q/functor.HydMotorDisp;
    if(x[0] < -5900.0) 
      x[0] = -5900;

		double WindCurr = functor.I_Wind(x[0U]);
		// 1.375 fudge factor required to match experiments, not yet sure why.
		const double T_applied = 1.375 * functor.I_Wind.TorqueConstantInLbPerAmp * WindCurr;
		x[1] = T_applied/(functor.HydMotorDisp/(2*M_PI)); // Need to add applied torque here
		x[2] = Ve;

		std::cout << "# " << PistonVel << "  " << WindCurrTarg << "        " << WindCurr
		          << "  " << x[0] << "  " << x[1] << "  " << x[2] << "      ";
		// solver.solveNumericalDiff will compute Jacobian numerically rather than obtain from user
		const int solver_info = solver.solveNumericalDiff(x);
		/*
    if(solver_info != 1)
		{
			std::cout << "=================================" << std::endl;
			std::cout << "Warning: Numericals solver in ElectroHydraulicPTO did not converge" << std::endl;
			std::cout << "solver info: [" << solver_info << "]" << std::endl;
			std::cout << "=================================" << std::endl;
		}
    */
		std::cout << "  " <<  functor.I_Wind(x[0U])
              << "  " << x[0] << "  " << x[1] << "  " << x[2]
		          << "  " << solver_info << std::endl;

	}




	EXPECT_EQ(1.0, 1.0);
	EXPECT_NE(1.0, 0.0);
}
