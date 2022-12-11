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
#include <boost/numeric/odeint.hpp>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <string>

#include "FS_Hydrodynamics.hpp"
#include "LinearIncidentWave.hpp"

//#include <ament_index_cpp/get_package_share_directory.hpp>

/* The rhs of x' = f(x) defined as a class */
class SingleModeMotionRHS
{
public:
FS_HydroDynamics * FloatingBody = NULL;
double last_accel = 0;
int mode = 0;

explicit SingleModeMotionRHS(FS_HydroDynamics * Body) : FloatingBody(Body) {
}

// x[0] = position
// x[1] = velocity
void operator()(const std::vector<double> & x, std::vector<double> & dxdt, const double t)
{
	Eigen::VectorXd pos(6);
	pos(0) = 0; pos(1) = 0; pos(2) = 0; pos(3) = 0; pos(4) = 0; pos(5) = 0;
	pos(mode) = x[0];
	Eigen::VectorXd vel(6);
	vel(0) = 0; vel(1) = 0; vel(2) = 0; vel(3) = 0; vel(4) = 0; vel(5) = 0;
	vel(mode) = x[1];
	Eigen::VectorXd F_LinDamping(6);
	F_LinDamping = FloatingBody->LinearDampingForce(vel);
	Eigen::VectorXd F_B(6);
	F_B = FloatingBody->BuoyancyForce(pos);
	Eigen::VectorXd F_G(6);
	F_G = FloatingBody->GravityForce(pos);
	Eigen::VectorXd F_R(6);
	Eigen::VectorXd accel(6);
	accel(0) = 0; accel(1) = 0; accel(2) = 0; accel(3) = 0; accel(4) = 0; accel(5) = 0;
	accel(mode) = last_accel;
	F_R = -FloatingBody->RadiationForce(accel);
	Eigen::VectorXd F_E(6);
	F_E = FloatingBody->ExcitingForce();
	dxdt[0] = x[1];
	double b = 0.1;
	dxdt[1] = (F_LinDamping(mode)+F_B(mode) + F_G(mode) + F_R(mode) + F_E(mode)) /
	          (FloatingBody->M(mode, mode) + FloatingBody->AddedMass(10000.0, mode, mode));
	last_accel = dxdt[1];
}
};

//[ integrate_observer
struct push_back_state_and_time
{
	std::vector<std::vector<double> > & m_states;
	std::vector<double> & m_times;

	push_back_state_and_time(std::vector<std::vector<double> > & states, std::vector<double> & times)
		: m_states(states), m_times(times)
	{
	}

	void operator()(const std::vector<double> & x, double t)
	{
		m_states.push_back(x);
		m_times.push_back(t);
	}
};

int main(int argc, char ** argv)
{
	{
		std::string s = "pkill gnuplot_qt";
		int ret = system(s.c_str());
	}

// Defaults
	double A = 1; // .5 + ((float)(std::rand() % 20) / 10);
	double Tp = 5; // 3.0 + (std::rand() % 9);
	double tf = 2.0 * Tp;
	double omega = 2 * M_PI / Tp;
	double phase = 40 * M_PI / 180;
	double dt = 0.005;

	bool PlotCoefficients = false;
	bool TestBuoyancyForces = false;
	bool TestGravityForces = false;
	bool TestRadiationForces = false;
	bool TestExcitingForces = false;
	bool TestMotions = false;
	int c;
	while ((c = getopt(argc, argv, "cbgremh")) != -1) {
		switch (c) {
		case 'c':  PlotCoefficients = true; break;
		case 'b':  TestBuoyancyForces = true; break;
		case 'g':  TestGravityForces = true; break;
		case 'r':  TestRadiationForces = true; break;
		case 'e':  TestExcitingForces = true; break;
		case 'm':  TestMotions = true; break;
		case 'h':
			std::cout << "Usage: Test_FS_Hydrodynamics [-cbgremh]" << std::endl;
			std::cout << "       -c :  Plot Hydrodynamic Coefficients" << std::endl;
			std::cout << "       -b :  Test Bouyancy Forces" << std::endl;
			std::cout << "       -g :  Test Gravity Forces" << std::endl;
			std::cout << "       -r :  Test Radiation Forces" << std::endl;
			std::cout << "       -e :  Test Exciting Forces" << std::endl;
			std::cout << "       -m :  Test Motions" << std::endl;
			std::cout << "       -h :  This help message" << std::endl;
			break;
		}
	}

	const char * modes[6] = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"};
	LinearIncidentWave Inc;
	LinearIncidentWave & IncRef = Inc;
	double rho = 1025;
	double g = 9.81;
	double buoy_mass = 1400; // kg
	FS_HydroDynamics BuoyA5(IncRef, 1.0, g, rho);
	BuoyA5.SetWaterplane(5.47, 1.37, 1.37); // Set area and 2nd moments of area for waterplane
	BuoyA5.SetCOB(0, 0, -.22); // Set COB relative to waterplane coordinate system.
	BuoyA5.SetCOG(0, 0, -.24); // Set COG relative to waterplane coordinate system.
	BuoyA5.SetVolume(buoy_mass / rho);
	BuoyA5.SetMass(buoy_mass);

	std::string HydrodynamicsBaseFilename =
		"../test_hydrodynamic_coeffs/BuoyA5";
//		"/home/hamilton"
//		"/buoy_ws/src/buoy_sim/"
//		"buoy_description/models/mbari_wec_base/hydrodynamic_coeffs/BuoyA5";
	BuoyA5.ReadWAMITData_FD(HydrodynamicsBaseFilename);
	BuoyA5.ReadWAMITData_TD(HydrodynamicsBaseFilename);
	BuoyA5.SetTimestepSize(dt);
	if (PlotCoefficients) {
		BuoyA5.Plot_FD_Coeffs();
		BuoyA5.Plot_TD_Coeffs();
	}
	std::srand((unsigned)time(0));


	std::vector<double> pts_t;
	std::vector<double> pts_pos;
	std::vector<double> pts_vel;
	std::vector<double> pts_accel;
	for (int k = 0; k < tf / dt; k++) {
		double tt = dt * k;
		pts_t.push_back(tt);
		pts_pos.push_back(A * cos(omega * tt + phase));
		pts_vel.push_back(-A * omega * sin(omega * tt + phase));
		pts_accel.push_back(-A * pow(omega, 2) * cos(omega * tt + phase));
	}

	if (TestBuoyancyForces || TestRadiationForces) {
		Gnuplot gp;
		char Amp[10];
		snprintf(Amp, sizeof(Amp), "%.1f", A);
		char Per[10];
		snprintf(Per, sizeof(Per), "%.1f", Tp);
		gp << "set term qt title  'A = " << Amp << "m  T = " << Per << "s'\n";
		gp << "set grid\n";
		gp << "set xlabel 'time (s)'\n";
		gp << "plot '-' w l title 'Vel'" <<
		        ",'-' w l title 'Accel'\n";
		gp.send1d(boost::make_tuple(pts_t, pts_vel));
		gp.send1d(boost::make_tuple(pts_t, pts_accel));
	}


// Test Radiation Forces
// Note:  This computes the radiation forces for each mode of motion indivdually, so
// MemRadiation is called 6 times per timestep, once with each acceleration set non-zero
// In use it will be called only once per time-step, with all acclerations set.
	if (TestRadiationForces) {
		std::cout << "Test Radiation Forces " << std::endl;
		for (int i = 0; i < 6; i++) { // i determines mode of motion.
			for (int j = 0; j < 6; j++) { // j denotes direction of resulting force
				double am = BuoyA5.AddedMass(omega, i, j);
				double dmp = BuoyA5.RadiationDamping(omega, i, j);
				double am_inf = BuoyA5.fd_A_inf_freq(i, j);

				std::vector<double> pts_F_TD, pts_F_FD;
				double last_accel = 0;
				double F_max = -std::numeric_limits<double>::max();
				double F_min = std::numeric_limits<double>::max();
				BuoyA5.SetTimestepSize(dt); // Reset timestep to re-initialize storage in BuoyA5 Class
				for (int k = 0; k < pts_accel.size(); k++) {
					double accel = pts_accel[k];
					Eigen::VectorXd xddot(6);
					for (int n = 0; n < 6; n++) {
						xddot(n) = 0;
					}
					xddot(i) = last_accel;

					Eigen::VectorXd MemForce(6);
					MemForce = BuoyA5.RadiationForce(xddot);
					pts_F_TD.push_back(am_inf * accel + MemForce(j));
					last_accel = accel;
					double FD_Force = am * pts_accel[k] + dmp * pts_vel[k];
					pts_F_FD.push_back(FD_Force);
					if (FD_Force > F_max) {
						F_max = FD_Force;
					}
					if (FD_Force < F_min) {
						F_min = FD_Force;
					}
				}

				if ((F_min < -1) && (F_max > 1)) { // Don't plot near-zero forces
					Gnuplot gp;
					char Amp[10];
					snprintf(Amp, sizeof(Amp), "%.1f", A);
					char Per[10];
					snprintf(Per, sizeof(Per), "%.1f", Tp);
					gp << "set term qt title 'Radiation Forces: " << modes[i] << " Motions, " << modes[j] <<
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

	if (TestExcitingForces) {
		std::cout << "Test Exciting Forces" << std::endl;
		Inc.SetToMonoChromatic(2 * A, Tp, phase, 180 * M_PI / 180);

		for (int j = 0; j < 6; j++) {         // j denotes direction of resulting force
			std::complex<double> Chi = BuoyA5.WaveExcitingForceComponents(Inc.m_omega[0], j);
			std::vector<double> pts_F_TD, pts_F_FD, pts_eta;
			BuoyA5.SetTimestepSize(dt);
			for (int k = 0; k < pts_t.size(); k++) {
				pts_F_FD.push_back(
					Inc.m_A[0] * Chi.real() * cos(
						omega * pts_t[k] + phase * cos(
							180 * M_PI / 180)) - Inc.m_A[0] * Chi.imag() *
					sin(omega * pts_t[k] + phase * cos(180 * M_PI / 180)));
				pts_eta.push_back(Inc.eta(0, 0, pts_t[k]));
				Eigen::VectorXd ExtForce(6);
				ExtForce = BuoyA5.ExcitingForce();
				pts_F_TD.push_back(ExtForce(j));
			}
			Gnuplot gp;
			gp << "set term qt title  '" << modes[j] << " Exciting Forces'\n";
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

	if (TestBuoyancyForces) {
		std::cout << "Computing Buoyancy Forces" << std::endl;
		for (int j = 0; j < 6; j++) {         // j denotes direction of resulting force
			std::vector<double> pts_x, pts_F_B;
			for (int k = 0; k < pts_pos.size(); k++) {
				Eigen::VectorXd x(6);
				x(0) = 0; x(1) = 0; x(2) = 0; x(3) = 0; x(4) = 0; x(5) = 0;
				x(j) = pts_pos[k];
				pts_x.push_back(x(j));
				Eigen::VectorXd BuoyancyForce(6);
				BuoyancyForce = BuoyA5.BuoyancyForce(x);
				pts_F_B.push_back(BuoyancyForce(j));
			}
			Gnuplot gp;
			gp << "set term qt title  '" << modes[j] << " Buoyancy Forces'\n";
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

	if (TestGravityForces) {
		std::cout << "Computing Gravity Forces" << std::endl;
		for (int j = 0; j < 6; j++) {         // j denotes direction of resulting force
			std::vector<double> pts_x, pts_F_G;
			for (int k = 0; k < pts_pos.size(); k++) {
				Eigen::VectorXd x(6);
				x(0) = 0; x(1) = 0; x(2) = 0; x(3) = 0; x(4) = 0; x(5) = 0;
				x(j) = pts_pos[k];
				pts_x.push_back(x(j));
				Eigen::VectorXd GravityForce(6);
				GravityForce = BuoyA5.GravityForce(x);
				pts_F_G.push_back(GravityForce(j));
			}
			Gnuplot gp;
			gp << "set term qt title  '" << modes[j] << " Gravity Forces'\n";
			gp << "set grid\n";
			gp << "set xlabel 'time (s)'\n";
			if (j < 3) {
				gp << "set ylabel 'F (N)'\n";
			} else {
				gp << "set  ylabel 'M (N-m)'\n";
			}
			gp << "plot '-' w l title 'x(t)'" <<
			        ",'-' w l title 'Gravity Force'\n";
			gp.send1d(boost::make_tuple(pts_t, pts_x));
			gp.send1d(boost::make_tuple(pts_t, pts_F_G));
			gp << "set xlabel 'time (s)'\n";
			if (j < 3) {
				gp << "set ylabel 'F (N)'\n";
			} else {
				gp << "set  ylabel 'M (N-m)'\n";
			}
			gp << "replot\n";
		}
	}

	if (TestMotions) {
		std::cout << "Test Motions" << std::endl;
		Eigen::Matrix<double, 3, 3> I;
		I << 1500, 0, 0,
		        0, 1500, 0,
		        0, 0, 650;
		BuoyA5.SetI(I);

		Eigen::VectorXd b(6);
		b(0) = 300.0;
		b(1) = 300.0;
		b(2) = 900.0;
		b(3) = 400.0;
		b(4) = 400.0;
		b(5) = 100.0;
		BuoyA5.SetDampingCoeff(b);


		std::vector<double> pts_T;
		Eigen::Matrix<std::vector<double>, 6, 1> pts_XiMod, pts_XiPh;

		double dT = .01;
		for (double T = dT; T < 24; T += dT) {
			pts_T.push_back(T);
			double w = 2 * M_PI / T;
			auto Xi = BuoyA5.ComplexAmplitude(w);
			for (int j = 0; j < 6; j++) {
				pts_XiMod(j).push_back(std::abs(Xi(j)));
				pts_XiPh(j).push_back(std::arg(Xi(j)) * 180 / M_PI);
			}
		}

		for (int j = 0; j < 6; j++) {
			Gnuplot gp1;
			gp1 << "set term qt title  '" << modes[j] << " RAO Amplitude'\n";
			gp1 << "set grid\n";
			gp1 << "set xlabel 'Wave Period (s)'\n";
			gp1 << "plot '-' w l \n";
			gp1.send1d(boost::make_tuple(pts_T, pts_XiMod(j)));

			Gnuplot gp2;
			gp2 << "set term qt title  '" << modes[j] << " RAO Phase Angle (deg)'\n";
			gp2 << "set grid\n";
			gp2 << "set xlabel 'Wave Period (s)'\n";
			gp2 << "plot '-' w l \n";
			gp2.send1d(boost::make_tuple(pts_T, pts_XiPh(j)));
		}

		Inc.SetToMonoChromatic(A, Tp, phase, 180 * M_PI / 180);
		BuoyA5.SetTimestepSize(dt);

		for(int mode = 0; mode < 6; mode++)
		{
			auto Xi = BuoyA5.ComplexAmplitude(2*M_PI/Tp,mode);

			std::vector<double> x(2);
			x[0] = 0.0; // initial position
			x[1] = 0.0; // initial velocity


			double t_final = 10*Tp;
			// integrate_observ
			std::vector<std::vector<double> > x_vec;
			std::vector<double> times;
			boost::numeric::odeint::euler<std::vector<double> > stepper;
			SingleModeMotionRHS RHS(&BuoyA5);
			RHS.mode = mode;
			int steps = boost::numeric::odeint::integrate_const(
				stepper, RHS,
				x, 0.0, t_final, dt, push_back_state_and_time(x_vec, times) );


			/* output */
			std::vector<double> pts_t, pts_x0, pts_x1;
			std::vector<double> pts_x;
			std::vector<double> pts_eta;
			for (size_t i = 0; i <= steps; i++) {
				pts_t.push_back(times[i]);
				pts_eta.push_back(Inc.eta(0.0,0.0,times[i]));
				pts_x0.push_back(x_vec[i][0]);
				pts_x1.push_back(x_vec[i][1]);
				pts_x.push_back(A*std::abs(Xi)*cos(2*M_PI*times[i]/Tp-phase+std::arg(Xi)));
			}

			Gnuplot gp;
			gp << "set term qt title  '" << modes[mode] <<  " Motion Output'\n";
			gp << "set grid\n";
			gp << "set xlabel 't (s)'\n";
			gp << "plot '-' w l title 'eta'" <<
			        ",'-' w l title 'pos (td)'" <<
			        ",'-' w l title 'pos (fd)'\n";
			gp.send1d(boost::make_tuple(pts_t, pts_eta));
			gp.send1d(boost::make_tuple(pts_t, pts_x0));
			gp.send1d(boost::make_tuple(pts_t, pts_x));
		}
	}
}
