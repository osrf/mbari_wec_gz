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

#include "FS_Hydrodynamics.hpp"
#include <gnuplot-iostream.h>

#include <Eigen/Dense>

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>

#include "IncidentWave.hpp"
#include "interp1d.hpp"
#include "mlinterp.hpp"


const char * modes[6] = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"};

int CountLines(std::string filenm)
{
  int count = 0;
  std::string line;
  std::ifstream ifile(filenm);

  if (ifile.is_open()) {
    while (ifile.peek() != EOF) {
      std::getline(ifile, line);
      count++;
    }
    ifile.close();
  } else {
    std::cout << "ERROR:  " << filenm << " can't be opened" << std::endl;
  }

  return count;
}

FS_HydroDynamics::FS_HydroDynamics(IncidentWave & IncWave)
: _IncWave(IncWave), m_L{1.0}, m_grav{9.81}, m_rho{1025}
{
  this->M.setZero();
  this->c.setZero();
}

FS_HydroDynamics::FS_HydroDynamics(
  IncidentWave & IncWave, double L, double g,
  double rho)
: _IncWave(IncWave), m_L{L}, m_grav{g}, m_rho{rho}
{
  this->M.setZero();
  this->c.setZero();
}

///  \brief  Read frequency domain coefficients from WAMIT.
///
///  .1 files include added mass and damping coefficients
///
///  .3 files include wave-exciting force coefficients
void FS_HydroDynamics::ReadWAMITData_FD(std::string filenm)
{
  int n_lines1 = CountLines(filenm + ".1");

  std::ifstream ifile1(filenm + ".1");
  m_fd_filename = (filenm + ".1 and .3 not found");
  if (ifile1.is_open()) {
    m_fd_filename = (filenm + ".3 not found");
    Eigen::MatrixXd s1(n_lines1, 5);
    for (int n = 0; n < n_lines1; n++) {
      ifile1 >> s1(n, 0) >> s1(n, 1) >> s1(n, 2) >> s1(n, 3) >> s1(n, 4);
    }
    ifile1.close();

    // Determine number of frequencies by looking for first repeat of i,j
    int n_freqs = 0;
    int n = 0;
    while (n < n_lines1) {
      if ((s1(n, 1) == s1(0, 1)) && (s1(n, 2) == s1(0, 2))) {
        n_freqs++;
      }
      n++;
    }
    int coeffPerFreqs = n_lines1 / n_freqs;

    this->fd_am_dmp_tps = Eigen::VectorXd(
      n_freqs - 1);                   // Infinite Frequency coefficents stored independelty,
                                      // assumpting here is all .1 files have an infinite
                                      // frequency field, may or may not be true...
    this->fd_am_dmp_omega = Eigen::VectorXd(n_freqs - 1);
    int nn = 0;
    for (n = 0; n < n_freqs; n++) {
      if (s1(n * coeffPerFreqs, 0) == 0.0) {                   // Infinite frequency case
        for (int k = 0; k < coeffPerFreqs; k++) {
          int i = n * coeffPerFreqs + k;
          this->fd_A_inf_freq.coeffRef(s1(i, 1) - 1, s1(i, 2) - 1) = m_rho * s1(i, 3);
          this->fd_B_inf_freq.coeffRef(s1(i, 1) - 1, s1(i, 2) - 1) =
          0;                               // m_rho * fd_am_dmp_omega(n) * s1(i, 4);
        }
      } else {
        this->fd_am_dmp_tps(nn) = s1(n * coeffPerFreqs, 0);
        if (this->fd_am_dmp_tps(nn) <
          0)                           // Negative period indicates zero frequency
        {
          this->fd_am_dmp_omega(nn) = 0.0;
        } else {
          this->fd_am_dmp_omega(nn) = M_PI * 2 / this->fd_am_dmp_tps(nn);
        }
        Eigen::Matrix<double, 6, 6> A;
        Eigen::Matrix<double, 6, 6> B;
        A.setZero();
        B.setZero();
        for (int k = 0; k < coeffPerFreqs; k++) {
          int i = n * coeffPerFreqs + k;
          A.coeffRef(s1(i, 1) - 1, s1(i, 2) - 1) = m_rho * s1(i, 3);
          B.coeffRef(s1(i, 1) - 1, s1(i, 2) - 1) =
            m_rho * fd_am_dmp_omega(nn) * s1(i, 4);
        }
        this->fd_A.push_back(A);
        this->fd_B.push_back(B);
        nn++;
      }
    }
  } else {
    std::cout << "ERROR:  " << filenm << ".1 can't be opened" << std::endl;
  }

  // Read Wave Exciting Forces
  int n_lines3 = CountLines(filenm + ".3");
  std::ifstream ifile3(filenm + ".3");
  if (ifile3.is_open()) {
    m_fd_filename = filenm;
    Eigen::MatrixXd s3(n_lines3, 7);
    for (int n = 0; n < n_lines3; n++) {
      ifile3 >> s3(n, 0) >> s3(n, 1) >> s3(n, 2) >> s3(n, 3) >> s3(n, 4) >>
      s3(n, 5) >> s3(n, 6);
    }
    ifile3.close();

    // Determine number of frequencies by looking for number of i repeats
    int n_freqs = 0;
    int n = 0;
    while (n < n_lines3) {
      if (s3(n, 2) == s3(0, 2)) {
        n_freqs++;
      }
      n++;
    }
    int coeffPerFreq = n_lines3 / n_freqs;
    this->fd_ext_tps = Eigen::VectorXd(n_freqs);
    this->fd_ext_omega = Eigen::VectorXd(n_freqs);
    this->fd_ext_beta = Eigen::VectorXd(n_freqs);
    auto Mod_Chi = Eigen::VectorXd(n_freqs);
    auto Pha_Chi = Eigen::VectorXd(n_freqs);
    auto Re_Chi = Eigen::VectorXd(n_freqs);
    auto Im_Chi = Eigen::VectorXd(n_freqs);

    for (int k = 0; k < coeffPerFreq; k++) {             // Collect all values for each mode
      for (n = 0; n < n_freqs; n++) {
        int i = n * coeffPerFreq + k;
        Mod_Chi(n) = m_rho * m_grav * s3(i, 3);
        Pha_Chi(n) = s3(i, 4);
        Re_Chi(n) = m_rho * m_grav * s3(i, 5);
        Im_Chi(n) = m_rho * m_grav * s3(i, 6);
        if (k == 0) {                         // Collect tps and beta just once
          this->fd_ext_tps(n) = s3(i, 0);
          this->fd_ext_omega(n) = M_PI * 2 / this->fd_ext_tps(n);
          this->fd_ext_beta(n) = s3(i, 1);
        }
      }
      this->fd_Mod_Chi.push_back(Mod_Chi);                   // Relies on mode ordering being 1->6
      this->fd_Pha_Chi.push_back(Pha_Chi);
      this->fd_Re_Chi.push_back(Re_Chi);
      this->fd_Im_Chi.push_back(Im_Chi);
    }
  } else {
    std::cout << "ERROR:  " << filenm << ".3 can't be opened" << std::endl;
  }
}

/// \brief  Read time-domain coefficients from WAMIT.
///
///  _IR.1 files include radiation force impulse response functions
///
///  _IR.3 files include wave-exciting force impulse response functions
void FS_HydroDynamics::ReadWAMITData_TD(std::string filenm)
{
  int n_lines1 = CountLines(filenm + "_IR.1");

  std::ifstream ifile1(filenm + "_IR.1");
  m_td_filename = (filenm + "_IR.1 and _IR.3 not found");
  if (ifile1.is_open()) {
    m_td_filename = (filenm + "_IR.3 not found");
    Eigen::MatrixXd s1(n_lines1, 5);
    for (int n = 0; n < n_lines1; n++) {
      ifile1 >> s1(n, 0) >> s1(n, 1) >> s1(n, 2) >> s1(n, 3) >> s1(n, 4);
    }
    ifile1.close();

    // Determine number of timesteps by looking for first repeat of i,j
    int n_timesteps = 0;
    for (int n = 0; n < n_lines1; n++) {
      if ((s1(n, 1) == s1(0, 1)) && (s1(n, 2) == s1(0, 2))) {
        n_timesteps++;
      }
    }

    m_tau_rad.resize(n_timesteps);
    for (int n = 0; n < n_timesteps; n++) {
      m_tau_rad(n) = s1(n * n_lines1 / n_timesteps, 0);
    }

    for (int n = 0, k = 0; n < n_lines1; n++) {
      if (m_IR_cosint[s1(n, 1) - 1][s1(n, 2) - 1].size() == 0) {
        m_IR_cosint[s1(n, 1) - 1][s1(n, 2) - 1]
        .resize(n_timesteps);                         // Set vector size once
        m_IR_sinint[s1(n, 1) - 1][s1(n, 2) - 1]
        .resize(n_timesteps);                         // Set vector size once
      }
      if ((s1(n, 1) == s1(0, 1)) && (s1(n, 2) == s1(0, 2))) {
        k++;
      }

      m_IR_cosint[s1(n, 1) - 1][s1(n, 2) - 1](k - 1) =
        m_rho * s1(n, 3);                         // Fill in dataFill in data
      m_IR_sinint[s1(n, 1) - 1][s1(n, 2) - 1](k - 1) =
        m_rho * s1(n, 4);                         // Fill in dataFill in data
    }
  } else {
    std::cout << "ERROR:  " << filenm << "_IR.1 can't be opened" << std::endl;
  }

  int n_lines3 = CountLines(filenm + "_JR.3");

  std::ifstream ifile3(filenm + "_JR.3");
  m_td_filename = (filenm + "_IR.3 not found");
  if (ifile3.is_open()) {
    m_td_filename = (filenm + "_IR.3 not found");
    Eigen::MatrixXd s3(n_lines3, 7);
    for (int n = 0; n < n_lines3; n++) {
      ifile3 >> s3(n, 0) >> s3(n, 1) >> s3(n, 2) >> s3(n, 3) >> s3(n, 4) >>
      s3(n, 5) >> s3(n, 6);
    }
    ifile3.close();

    m_tau_exc.resize(n_lines3);
    for (int k = 0; k < n_lines3; k++) {
      m_tau_exc(k) = s3(k, 0);
    }

    m_dtau_exc = m_tau_exc(1) - m_tau_exc(0);

    for (int j = 0; j < 6; j++) {
      m_IR_exc[j].resize(n_lines3);
      for (int k = 0; k < n_lines3; k++) {
        m_IR_exc[j](k) = m_rho * m_grav * s3(k, j + 1);
      }
    }
  } else {
    std::cout << "ERROR:  " << filenm << "_JR.3 can't be opened" << std::endl;
  }
}

void FS_HydroDynamics::Plot_FD_Coeffs()
{
  {       // Plot Added Mass and Damping Values
    std::vector<double> pts_omega;
    for (int k = 0; k < fd_am_dmp_omega.size(); k++) {
      pts_omega.push_back(fd_am_dmp_omega(k));
    }

    for (int i = 0; i < 6; i++) {
      for (int j = i; j < 6; j++) {
        std::vector<double> pts_am, pts_am_sym;
        std::vector<double> pts_dmp, pts_dmp_sym;
        double am_norm = 0;
        double dmp_norm = 0;
        for (int k = 0; k < fd_am_dmp_tps.size(); k++) {
          pts_am.push_back(fd_A[k](i, j));
          pts_am_sym.push_back(fd_A[k](j, i));
          am_norm += fabs(fd_A[k](i, j)) + fabs(fd_A[k](j, i));
          pts_dmp.push_back(fd_B[k](i, j));
          pts_dmp_sym.push_back(fd_B[k](j, i));
          dmp_norm += fabs(fd_B[k](i, j)) + fabs(fd_B[k](j, i));
        }
        if ((am_norm > 1e-6) || (dmp_norm > 1e-6)) {
          Gnuplot gp;

          if (i == j) {
            gp << "set term X11 title 'Added Mass and Damping (" << i + 1 << "," <<
              j + 1 << ")\n";
            gp << "set multiplot layout 2,1 rowsfirst \n";
            gp << "set grid\n";
            gp << "set xlabel 'rad/sec'\n";
            gp << "set ylabel 'kg'\n";
            gp << "plot '-' u 1:2 with lines title 'am(" << i + 1 << "," <<
              j + 1 << ") inf\\_freq\\_am = " << std::fixed <<
              std::setprecision(2) << this->fd_A_inf_freq(i, j) << "'\n";
            gp.send1d(boost::make_tuple(pts_omega, pts_am));

            gp << "set xlabel 'rad/sec'\n";
            gp << "set ylabel 'kg'\n";
            gp << "plot '-' u 1:2 with lines title 'dmp(" << i + 1 << "," <<
              j + 1 << ") inf\\_freq\\_dmp = " << std::fixed <<
              std::setprecision(2) << this->fd_B_inf_freq(i, j) << "'\n";
            gp.send1d(boost::make_tuple(pts_omega, pts_dmp));
          } else {
            gp << "set term X11 title 'Added Mass and Damping (" << i + 1 << "," <<
              j + 1 << ") and (" << j + 1 << "," << i + 1 << ")\n";
            gp << "set multiplot layout 2,1 rowsfirst \n";
            gp << "set grid\n";
            gp << "set xlabel 'rad/sec'\n";
            gp << "set ylabel 'kg'\n";
            gp << "plot '-' w l title 'am(" << i + 1 << "," << j + 1 <<
              ") inf\\_freq\\_am = " << std::fixed << std::setprecision(2) <<
              this->fd_A_inf_freq(i, j) << "','-' w l title 'am(" << j + 1 <<
              "," << i + 1 << ") inf\\_freq\\_am = " << std::fixed <<
              std::setprecision(2) << this->fd_A_inf_freq(j, i) << "'\n";
            gp.send1d(boost::make_tuple(pts_omega, pts_am));
            gp.send1d(boost::make_tuple(pts_omega, pts_am_sym));

            gp << "set xlabel 'rad/sec'\n";
            gp << "set ylabel 'kg'\n";
            gp << "plot '-' w l title 'dmp(" << i + 1 << "," << j + 1 <<
              ") inf\\_freq\\_dmp = " << std::fixed << std::setprecision(2) <<
              this->fd_B_inf_freq(i, j) << "','-' w l title 'dmp(" << j + 1 <<
              "," << i + 1 << ") inf\\_freq\\_dmp = " << std::fixed <<
              std::setprecision(2) << this->fd_B_inf_freq(j, i) << "'\n";
            gp.send1d(boost::make_tuple(pts_omega, pts_dmp));
            gp.send1d(boost::make_tuple(pts_omega, pts_dmp_sym));
          }
        }
      }
    }
  }
  {       // Plot Wave Exciting Forces
    for (int i = 0; i < 6; i++) {
      std::vector<double> pts_omega;
      std::vector<double> pts_ReChi, pts_ImChi;
      std::vector<double> pts_Mod_Chi, pts_Pha_Chi;
      for (int k = 0; k < fd_ext_omega.size(); k++) {
        pts_omega.push_back(fd_ext_omega(k));
        pts_ReChi.push_back(fd_Re_Chi[i](k));
        pts_ImChi.push_back(fd_Im_Chi[i](k));
        pts_Mod_Chi.push_back(fd_Mod_Chi[i](k));
        pts_Pha_Chi.push_back(fd_Pha_Chi[i](k));
      }
      Gnuplot gp;
      gp << "set term X11 title '" << modes[i] << "Exciting Forces'\n";
      gp << "set multiplot layout 2,1 rowsfirst \n";
      gp << "set grid\n";
      gp << "set xlabel 'rad/sec'\n";
      if (i < 3) {
        gp << "set ylabel 'N/m'\n";
      } else {
        gp << "set ylabel 'N-m/m'\n";
      }
      gp << "plot '-' u 1:2 with lines title 'Re'," <<
        "'-' u 1:2 with lines title 'Im'\n";
      gp.send1d(boost::make_tuple(pts_omega, pts_ReChi));
      gp.send1d(boost::make_tuple(pts_omega, pts_ImChi));

      gp << "set xlabel 'rad/sec'\n";
      if (i < 3) {
        gp << "set ylabel 'N/m'\n";
      } else {
        gp << "set ylabel 'N-m/m'\n";
      }
      gp << "plot '-' u 1:2 with lines title 'Mod'," <<
        "'-' u 1:2 with lines title 'Pha'\n";
      gp.send1d(boost::make_tuple(pts_omega, pts_Mod_Chi));
      gp.send1d(boost::make_tuple(pts_omega, pts_Pha_Chi));
    }
  }
}

void FS_HydroDynamics::Plot_TD_Coeffs()
{
  // Plot Radiation Impulse Response Function
  for (int i = 0; i < 6; i++) {
    for (int j = i; j < 6; j++) {
      std::vector<double> pts_L, pts_L_sym;
      std::vector<double> pts_tau;
      if (m_L_rad[i][j].size() >
        0)                     // Check to see if impulse response functoin is nonzero.
      {
        for (int k = 0; k < m_L_rad[i][j].size(); k++) {
          pts_tau.push_back(m_dt * k);
          pts_L.push_back(m_L_rad[i][j][k]);
          if (i != j) {
            pts_L_sym.push_back(m_L_rad[j][i](k));
          }
        }

        Gnuplot gp;
        if (i == j) {
          gp << "set term X11 title 'Radiation IRF (" << i + 1 << "," << j + 1 <<
            ")\n";
          gp << "set grid\n";
          gp << "plot '-' u 1:2 with lines title 'IRF(" << i + 1 << "," << j + 1 <<
            ")'\n";
          gp.send1d(boost::make_tuple(pts_tau, pts_L));
          gp << "set xlabel 'sec'\n";
          gp << "set ylabel '-'\n";
        } else {
          gp << "set term X11 title 'Radiation IRF (" << i + 1 << "," << j + 1 <<
            ") and (" << j + 1 << "," << i + 1 << ")\n";
          gp << "set grid\n";
          gp << "plot '-' w l title 'IRF(" << i + 1 << "," << j + 1 << ")" <<
            "','-' w l title 'IRF(" << j + 1 << "," << i + 1 << ")'\n";
          gp.send1d(boost::make_tuple(pts_tau, pts_L));
          gp.send1d(boost::make_tuple(pts_tau, pts_L_sym));
          gp << "set xlabel 'sec'\n";
          gp << "set ylabel '-'\n";
        }
      }
    }
  }
  // Plot Exciting Force Impulse Response Functions
  for (int j = 0; j < 6; j++) {
    std::vector<double> pts_Chi;
    std::vector<double> pts_tau;
    for (int k = 0; k < m_L_exc[j].size(); k++) {
      pts_tau.push_back(
        m_dt * (k - m_L_exc[j].size() /
        2));                                 // Have to recreate this as there's
                                             // no real reason to store...
      pts_Chi.push_back(m_L_exc[j](k));
    }

    Gnuplot gp;
    gp << "set term X11 title '" << modes[j] << "Wave Exciting IRF '\n";
    gp << "set grid\n";
    gp << "plot '-' u 1:2 with lines title 'IRF(" << j + 1 << ")'\n";
    gp.send1d(boost::make_tuple(pts_tau, pts_Chi));
    gp << "set xlabel 'sec'\n";
    gp << "set ylabel '-'\n";
  }
}
/// \brief Returns added mass at specified frequency
///  Interpolates from tabulated WAMIT data, omega must be greater than or equal
///  to zero.
double FS_HydroDynamics::AddedMass(double omega, int i, int j)
{
  if (omega < 0) {
    std::cout <<
      "ERROR:  FS_Hydrodyanmics::AddedMass:  omega must be great than zero" <<
      std::endl;
  }

  std::vector<double> xd;
  std::vector<double> yd;
  for (int n = 0; n < fd_A.size(); n++) {
    xd.push_back(this->fd_am_dmp_omega(n));
    yd.push_back(this->fd_A[n](i, j));
  }
  xd.push_back(1e6);       // Approximately Infinite Frequency...
  yd.push_back(fd_A_inf_freq(i, j));
  // Perform the interpolation
  simple_interp::Interp1d LinInterp(xd, yd);  // Called infrequently so re-establish here
  return LinInterp(omega);
}

Eigen::Matrix<double, 6, 6> FS_HydroDynamics::AddedMass(double omega)
{
  Eigen::Matrix<double, 6, 6> A;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      A(i, j) = AddedMass(omega, i, j);
    }
  }

  return A;
}


/// \brief Returns damping at specified frequency for specified mode (i,j)
///  Interpolates from tabulated WAMIT data, omega must be greater than or equal
///  to zero.
double FS_HydroDynamics::Damping(double omega, int i, int j)
{
  if (omega < 0) {
    std::cout <<
      "ERROR:  FS_Hydrodyanmics::Damping:  omega must be great than zero" <<
      std::endl;
  }

  std::vector<double> xd;
  std::vector<double> yd;
  for (int n = 0; n < fd_B.size(); n++) {
    xd.push_back(this->fd_am_dmp_omega(n));
    yd.push_back(this->fd_B[n](i, j));
  }

  xd.push_back(1e6);       // Approximately Infinite Frequency...
  yd.push_back(fd_B_inf_freq(i, j));
  // Perform the interpolation
  simple_interp::Interp1d LinInterp(xd, yd);  // Called infrequently  so re-establish here
  return LinInterp(omega);
}

/// \brief Returns damping at specified frequency, for all modes
///  Interpolates from tabulated WAMIT data, omega must be greater than or equal
///  to zero.
Eigen::Matrix<double, 6, 6> FS_HydroDynamics::Damping(double omega)
{
  Eigen::Matrix<double, 6, 6> B;

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      B(i, j) = Damping(omega, i, j);
    }
  }

  return B;
}


/// \brief Returns Wave Exciting Forces at specified frequency for mode (j)
///  Interpolates from tabulated WAMIT data, omega must be greater than or equal
///  to zero.
std::complex<double> FS_HydroDynamics::WaveExcitingForceComponents(
  double omega, int j)
{
  if (omega < 0) {
    std::cout << "ERROR:  FS_Hydrodyanmics::WaveExcitingForceComponents:  "
      "omega must be great than zero" <<
      std::endl;
  }

  std::vector<double> xd;
  std::vector<double> yRed;
  std::vector<double> yImd;
  for (int n = 0; n < this->fd_Re_Chi[0].size(); n++) {
    xd.push_back(this->fd_ext_omega(n));
    yRed.push_back(this->fd_Re_Chi[j](n));
    yImd.push_back(this->fd_Im_Chi[j](n));
  }

  // Perform the interpolation
  simple_interp::Interp1d LinInterpRe(xd, yRed);  // Called infrequently so re-establish here
  simple_interp::Interp1d LinInterpIm(xd, yImd);  // Called infrequently so re-establish here
  return {LinInterpRe(omega), LinInterpIm(omega)};
}


/// \brief Returns Wave Exciting Forces at specified frequency for all modes
///  Interpolates from tabulated WAMIT data, omega must be greater than or equal
///  to zero.
Eigen::Matrix<std::complex<double>, 6, 1> FS_HydroDynamics::WaveExcitingForceComponents(
  double omega)
{
  Eigen::Matrix<std::complex<double>, 6, 1> Chi;

  for (int j = 0; j < 6; j++) {
    Chi(j) = WaveExcitingForceComponents(omega, j);
  }

  return Chi;
}

void FS_HydroDynamics::Compute_cij(void)
{
  this->c.setZero();
  this->c(2, 2) = m_rho * m_grav * this->S;
  this->c(3, 3) = m_rho * m_grav * (Vol * COB(2) + S22);
  this->c(3, 5) = -m_rho * m_grav * (Vol * COB(0));
  this->c(4, 4) = m_rho * m_grav * (Vol * COB(2) + S11);
  this->c(4, 5) = -m_rho * m_grav * (Vol * COB(1));
}

void FS_HydroDynamics::SetWaterplane(double S, double S11, double S22)
{
  this->S = S;
  this->S11 = S11;
  this->S22 = S22;
  Compute_cij();
}

void FS_HydroDynamics::SetVolume(double V)
{
  this->Vol = V;
  Compute_cij();
}

void FS_HydroDynamics::SetCOB(double x, double y, double z)
{
  this->COB(0) = x;
  this->COB(1) = y;
  this->COB(2) = z;
  Compute_cij();
}

void FS_HydroDynamics::SetCOG(double x, double y, double z)
{
  this->COG(0) = x;
  this->COG(1) = y;
  this->COG(2) = z;
}

void FS_HydroDynamics::SetMass(double m)
{
  this->M(0, 0) = m;
  this->M(1, 1) = m;
  this->M(2, 2) = m;
}

void FS_HydroDynamics::SetI(Eigen::Matrix<double, 3, 3> I)
{
  this->M.block<3, 3>(3, 3) = I;
}


double FS_HydroDynamics::GetTimestepSize()
{
  return this->m_dt;
}

void FS_HydroDynamics::SetTimestepSize(double dt)
{
  m_dt = dt;

  _n_rad_intpts = (m_tau_rad(m_tau_rad.size() - 1) - m_tau_rad(0)) / dt + 1;
  std::vector<double> x_rad;

  for (int k = 0; k < _n_rad_intpts; k++) {
    x_rad.push_back(k * dt);
  }

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      int nd = m_IR_sinint[i][j].size();
      if (nd > 0) {
        m_L_rad[i][j].resize(_n_rad_intpts);
        mlinterp::interp(
          &nd, _n_rad_intpts,
          m_IR_sinint[i][j].data(), m_L_rad[i][j].data(),   // Output Axis
          m_tau_rad.data(), &(x_rad[0]));  // Input axis
      }
    }
  }

  for (int i = 0; i < 6; i++) {
    m_xddot[i].resize(
      STORAGE_MULTIPLIER *
      _n_rad_intpts);                   // Create storage for 5 times the length
  }
  // of the Impulse response function,  code
  // will shift data when this fills.

  _rad_tstep_index = STORAGE_MULTIPLIER * _n_rad_intpts -
    1;                        // Set to fill from last memory spot backwards

  _n_exc_intpts = (m_tau_exc(m_tau_exc.size() - 1) - m_tau_exc(0)) / dt + 1;
  std::vector<double> x_exc;

  for (int k = 0; k < _n_exc_intpts; k++) {
    x_exc.push_back(m_tau_exc(0) + k * dt);
  }

  for (int j = 0; j < 6; j++) {
    if (m_IR_exc[j].size() > 0) {
      m_L_exc[j].resize(_n_exc_intpts);
      int nd = m_IR_exc[j].size();
      mlinterp::interp(
        &nd, _n_exc_intpts, m_IR_exc[j].data(), m_L_exc[j].data(),
        m_tau_exc.data(), &(x_exc[0]));
    }
  }
  _eta0.resize(
    STORAGE_MULTIPLIER *
    _n_exc_intpts);             // Create storage for 5 times the length of the
                                // Impulse response function,  code will shift
                                // data when this fills.

  // Set to fill from last memory spot backwards
  _exc_tstep_index = STORAGE_MULTIPLIER * _n_exc_intpts - 1;
}

Eigen::VectorXd FS_HydroDynamics::ExcitingForce()
{
  Eigen::VectorXd ExctForces(6);

  if (_exc_tstep_index ==
    STORAGE_MULTIPLIER * _n_exc_intpts -
    1)         // Fill in initial values of eta the first time.
  {
    _t_eta = m_tau_exc(0);
    for (; _exc_tstep_index >
      STORAGE_MULTIPLIER * _n_exc_intpts - _n_exc_intpts;
      _exc_tstep_index--)
    {
      _eta0(_exc_tstep_index) = _IncWave.eta(0, 0, _t_eta);
      _t_eta += m_dt;
    }
  }

  // Compute next needed wave-elevation, note this is a bit in the future..
  _eta0(_exc_tstep_index) = _IncWave.eta(0, 0, _t_eta);
  _t_eta += m_dt;

  // Compute convolution integrals, no need to adjust ends in trap rule since
  // integrand there is zero.
  for (int i = 0; i < 6; i++) {
    ExctForces(i) = m_L_exc[i].dot(_eta0.segment(_exc_tstep_index, _n_exc_intpts));
  }

  // Increment timestep index and shift stored xddot data if needed.
  _exc_tstep_index--;
  if (_exc_tstep_index == 0) {       // At end of allocated storage,
    _eta0.tail(_n_exc_intpts) = _eta0.head(_n_exc_intpts);
    _exc_tstep_index = STORAGE_MULTIPLIER * _n_exc_intpts - _n_exc_intpts;
  }

  ExctForces *= m_dt;
  return ExctForces;
}

// Returns force and moment due to gravity in the body frame,
//  as applied at the origin of water-plane.
Eigen::VectorXd FS_HydroDynamics::GravityForce(Eigen::VectorXd x)
{
  Eigen::VectorXd F_G(6);
  F_G.setZero();
  F_G(2) = -this->m_grav * this->M(0, 0);
  F_G(3) = -this->m_grav * this->M(0, 0) * this->COG(1);
  F_G(4) = this->m_grav * this->M(0, 0) * this->COG(0);
  return F_G;
}

// Returns linearized buoyancy force and moment in the body frame,
//  as applied at the origin of water-plane.
Eigen::VectorXd FS_HydroDynamics::BuoyancyForce(Eigen::VectorXd x)
{
  Eigen::VectorXd F_B(6);
  F_B.setZero();
  F_B(2) = this->m_rho * this->m_grav * this->Vol;
  F_B(3) = this->m_rho * this->m_grav * this->Vol * this->COB(1);
  F_B(4) = -this->m_rho * this->m_grav * this->Vol * this->COB(0);
  F_B = F_B - this->c * x;
  return F_B;
}

Eigen::VectorXd FS_HydroDynamics::RadiationForce(Eigen::VectorXd last_xddot)
{
  Eigen::VectorXd RadiationForces(6);
  for (int i = 0; i < 6; i++) {
    RadiationForces(i) = 0.0;             // set to zero.
  }
  // There is no memory contribution at the first timestep, so no
  // need to compute the convolution integrals the first time.
  if (_rad_tstep_index <= STORAGE_MULTIPLIER * _n_rad_intpts - 2) {
    for (int j = 0; j < 6; j++) {
      m_xddot[j](_rad_tstep_index) = 0;  // This is unknown at this timestep,
                                         // but can be set to zero since L_rad(tau = 0) = 0 always
      if (_rad_tstep_index < m_xddot[j].size() - 1) {
        m_xddot[j](_rad_tstep_index + 1) =
          last_xddot(j);                               // Store xddot from last timestep
      }
    }
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6;
        j++)                      // Sum up convolution integrals, no need to adjust ends in trap
      {                   // rule since integrand there is zero.
        if (m_L_rad[i][j].size() > 0) {
          int num_int_pts = m_xddot[j].size() - _rad_tstep_index;
          if (num_int_pts > _n_rad_intpts) {  // Sufficiently into simulation that
                                              // entire IRF can be used.
            num_int_pts = _n_rad_intpts;
          }
          RadiationForces(i) +=
            m_L_rad[i][j]
            .head(num_int_pts)
            .dot(m_xddot[j].segment(_rad_tstep_index, num_int_pts));
        }
      }
    }
  }

  // Decrement timestep index and shift stored xddot data if needed.
  _rad_tstep_index--;
  if (_rad_tstep_index == 0) {       // At beginning of allocated storage.
    for (int j = 0; j < 6; j++) {             // Copy most recent xdddot data to end of
                                              // acceleration storage vector.
      m_xddot[j].tail(_n_rad_intpts) = m_xddot[j].head(_n_rad_intpts);
    }
    _rad_tstep_index = STORAGE_MULTIPLIER * _n_rad_intpts - _n_rad_intpts;
  }
  RadiationForces *= this->m_dt;
  return RadiationForces;
}

Eigen::Matrix<std::complex<double>, 6, 1> FS_HydroDynamics::ComplexAmplitude(double omega)
{
  std::complex<double> i = {0, 1};
  auto A = AddedMass(omega);
  auto B = Damping(omega);
  auto Chi = WaveExcitingForceComponents(omega);
// Eigen::Matrix<std::complex<double>, 6, 6> C;
// C = -pow(omega,2)*(A+this->M) + i*omega*B + this->c;
// return C.inverse()*Chi;
  Eigen::Matrix<std::complex<double>, 6, 1> Xi;
  Xi.setZero();
  Xi(2) = Chi(2) /
    (-pow(omega, 2) * (A(2, 2) + this->M(2, 2)) + i * omega * B(2, 2) + this->c(2, 2));
  return Xi;
}


std::ostream & operator<<(std::ostream & out, const FS_HydroDynamics & f)
{
  std::cout << "# Freq Domain File = " << f.m_fd_filename << ".1" << std::endl;
  std::cout << "# L = " << f.m_L << std::endl;
  std::cout << "# gravity = " << f.m_grav << std::endl;
  std::cout << "# rho = " << f.m_rho << std::endl;
  std::cout << std::endl;
  std::cout << "#tps = " << f.fd_am_dmp_tps.transpose() << std::endl;
  std::cout << "#omega = " << f.fd_am_dmp_omega.transpose() << std::endl;
  std::cout << "#inf freq fd_A = " << std::endl <<
    f.fd_A[0] << std::endl <<
    std::endl;
  std::cout << "#inf freq fd_B = " << std::endl << f.fd_B[0] << std::endl;

  return out;       // return std::ostream so we can chain calls to operator<<
}


void FS_HydroDynamics::operator()(
  const std::vector<double> & x, std::vector<double> & dxdt,
  const double /* t */)
{
  dxdt[0] = x[1];
  dxdt[1] = -x[0] - m_gam * x[1];
}
