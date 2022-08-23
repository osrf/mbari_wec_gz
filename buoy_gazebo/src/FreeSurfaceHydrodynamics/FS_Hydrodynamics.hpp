
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

#include <vector>
#include <Eigen/Dense>
#include "IncidentWave.hpp"

using namespace Eigen;

#define STORAGE_MULTIPLIER 5 // storage space for xddot and eta.  Should be >2, larger takes more memory but there's less data shuffling

class FS_HydroDynamics
{
public:
    //FS_HydroDynamics();
    FS_HydroDynamics(IncidentWave &IncWave);
    FS_HydroDynamics(IncidentWave &IncWave, double L, double g, double rho);
    //void AssignIncidentWave(IncidentWave &IncWave);
    void ReadWAMITData_FD(std::string filenm);
    void ReadWAMITData_TD(std::string filenm);
    void Plot_FD_Coeffs();
    void Plot_TD_Coeffs();
    double AddedMass(double omega, int i, int j);
    double Damping(double omega, int i, int j);

    void SetTimestepSize(double dt);
    void SetWaterplane(double S, double S11, double S22);
    void SetCOB(double x, double y, double z);
    void SetVolume(double V);

    Eigen::VectorXd BuoyancyForce(Eigen::VectorXd x);
    Eigen::VectorXd RadiationForce(Eigen::VectorXd last_xddot);
    Eigen::VectorXd ExcitingForce();

    void WaveExcitingForceComponents(double *XiRe, double *XiIm, double omega, int j);

    friend std::ostream &
    operator<<(std::ostream &out, const FS_HydroDynamics &f);
    std::string m_fd_filename;
    std::string m_td_filename;
    IncidentWave &_IncWave;
    double m_L = 1;
    double m_grav = 9.81;
    double m_rho = 1025;

    // Frequency domain coefficients
    VectorXd fd_am_dmp_tps;
    VectorXd fd_am_dmp_omega;
    std::vector<Eigen::Matrix<double, 6, 6>> fd_X;
    std::vector<Eigen::Matrix<double, 6, 6>> fd_Y;
    Eigen::Matrix<double, 6, 6> fd_X_inf_freq;
    Eigen::Matrix<double, 6, 6> fd_Y_inf_freq;
    Eigen::VectorXd fd_ext_tps;
    Eigen::VectorXd fd_ext_omega;
    Eigen::VectorXd fd_ext_beta;
    std::vector<Eigen::Matrix<double, Dynamic, 1>> fd_Mod_Xi;
    std::vector<Eigen::Matrix<double, Dynamic, 1>> fd_Pha_Xi;
    std::vector<Eigen::Matrix<double, Dynamic, 1>> fd_Re_Xi;
    std::vector<Eigen::Matrix<double, Dynamic, 1>> fd_Im_Xi;

    // Time domain coefficients
    VectorXd m_tau_rad;
    Eigen::Matrix<VectorXd, 6, 6> m_IR_cosint;
    Eigen::Matrix<VectorXd, 6, 6> m_IR_sinint;

    double m_dtau_exc;
    VectorXd m_tau_exc;
    Eigen::Matrix<VectorXd, 1, 6> m_IR_exc;

    // Vectors for numerical integration
    double m_dt = 0;
    Eigen::Matrix<VectorXd, 6, 6> m_L_rad;
    Eigen::Matrix<VectorXd, 1, 6> m_L_exc;

    // Storage for accelerations for each of 6 DOF
    Eigen::Matrix<VectorXd, 1, 6> m_xddot;

    // Storage for wave-elevation at origin;
    VectorXd _eta0;
    
    int _rad_tstep_index = 0;
    int _exc_tstep_index = 0;
    int _n_rad_intpts = 0;
    int _n_exc_intpts = 0;

    // wave-elevation evaluation time;
    double _t_eta;

    /// \brief Buoy WaterPlane Area
    double S;  

    /// \brief Buoy WaterPlane Area Second Moment of Area around x
    double S11;  

    /// \brief Buoy WaterPlane Area Second Moment of Area around y
    double S22;  

    /// \brief Submerged Volume 
    double Vol;

    /// \brief Center of Buoyancy relative to water-plane coordinate system 
    Eigen::Vector3d COB;
};
