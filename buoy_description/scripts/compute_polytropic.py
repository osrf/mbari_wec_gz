#!/usr/bin/python3
# Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scipy.optimize as spo


# Constants
g = 9.81  # m/s^2
R_specific = 0.2968  # mass-specific gas constant for N2
T_ocean = Tenv = 283.15  # K, soak temp for equilibrium
rho = 1025  # density of seawater
V_hc = 0.12  # Displacement of heave cone
m_hc = 820  # mass of heave cone
m_piston = 48  # mass of piston
m = m_hc + m_piston  # total mass

Ap_u = 0.25*np.pi*5.0**2.0
Ap_l = Ap_u - 0.25*np.pi*1.5**2.0
Ap_u *= 0.0254**2.0
Ap_l *= 0.0254**2.0
# Ap_u = 0.0127  # Area of piston in upper chamber
# Ap_l = 0.0115  # Area of piston in lower
Vd_u = 0.0226 # est from cad 0.0172  # 0.015  # 0.0226  # 0.0266  # Dead volume of upper
Vd_l = 0.0463 # est from cad 0.0546  # 0.025  # 0.0463  # 0.0523  # Dead volume of lower

r = 0.5  # coef of heat transfer (Newton's law of cooling)
c_p = 1.04  # for N2


def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx], idx


def compute_n(P, V):
    x = np.log(V[:, np.newaxis])
    X = np.hstack((np.ones_like(x), x))
    y = np.log(P[:, np.newaxis])
    b = np.linalg.lstsq(X, y, rcond=None)[0]
    # print(f'{b=}')
    n = -b[1]
    C = b[0]
    return n, C


def compute_polytropic(PV, target_P):
    P = PV[:, 1]
    V = PV[:, 0]
    inc = np.where(np.diff(V) >= 0)
    dec = np.where(np.diff(V) < 0)
    n1, C1 = compute_n(P[inc], V[inc])
    n2, C2 = compute_n(P[dec], V[dec])
    P0, nearest_idx = find_nearest(P[inc], target_P)
    V0 = V[inc][nearest_idx]
    return float(n1), float(n2), float(P0), float(V0), float(C1), float(C2)


def load_atsea_logs(logs, plot=False):
    # atsea_filenm = ['2022.09.15T12.29.14',
    #                 '2022.09.14T16.27.55',
    #                 '2022.09.06T20.15.25',
    #                 '2022.09.06T19.15.21',
    #                 '2022.09.06T03.14.17',
    #                 '2022.09.05T23.14.01',
    #                 '2022.09.06T17.15.13',
    #                 '2022.09.06T16.15.09',
    #                 '2022.09.11T19.23.19',
    #                 '2022.09.12T13.24.31',
    #                 '2022.09.13T17.26.23',
    #                 '2022.09.16T19.31.18',
    #                 '2022.09.13T20.26.35',
    #                 '2022.09.15T19.29.42']

    df = [pd.read_csv(csv) for csv in logs]
    df = [df_.set_index('Source ID') for df_ in df]
    sc_df = [df_.loc[1] for df_ in df]  # SpringController: Source ID == 1
    spring_data = [df_.loc[(1, [' Timestamp',
                                ' SC Range Finder (in)',
                                ' SC Upper PSI',
                                'SC Lower PSI'])] for df_ in sc_df]
    spring_data = pd.concat(spring_data)
    spring_data.rename(columns={k: v for k, v in zip(spring_data.columns,
                                                     ['t', 'p', 'up', 'lp'])}, inplace=True)
    spring_data['uv'] = None
    spring_data['lv'] = None
    spring_data.p *= 0.0254
    spring_data.up *= 6894.757
    spring_data.lp *= 6894.757
    # spring_data.uv = spring_data.p*0.0127+0.0226  # but maybe 0.0266 does better?
    # spring_data.lv = (2.03 - spring_data.p)*0.0115+0.0463
    spring_data.uv = spring_data.p*Ap_u + Vd_u
    spring_data.lv = (2.03 - spring_data.p)*Ap_l + Vd_l

    fig, ax = None, None
    if plot:
        fig, ax = plt.subplots(2, 1)
        ax[0].plot(spring_data.uv, spring_data.up, label='AtSea')
        ax[1].plot(spring_data.lv, spring_data.lp, label='AtSea')
        # plt.show()

    return spring_data, fig, ax


def set_piston_position(V0_u,  # Volume setpoint from upper chamber polytropic PV curves
                        V0_l,  # Volume setpoint from lower chamber polytropic PV curves
                        P0_u,  # Pressure setpoint from upper PV
                        P0_l,  # Pressure setpoint from lower PV
                        T0_u,  # Tempurature setpoint for upper heat transfer
                        T0_l,  # Tempurature setpoint for lower heat transfer
                        initial_piston_position=0.7, x_mean_pos=0.7):
    ############################
    # Piston Position
    ############################

    # Would like to specify piston mean position and set all other
    # air spring values from that. Start from curve-fitting PV
    # for polytropic index and initial setpoints. Determine piston
    # equilibrium position. Solve for mass shift between chambers
    # that brings the equilibrium to the desired mean piston position.
    # Recompute pressure and volume based on Ideal Gas Law.

    # Balance of forces on piston at equilibrium
    # P_l + rho*g*V = P_u + m*g
    # P_u - P_l = g*(rho*V - m)
    #
    # m_u*R*T   m_l*R*T
    # ------- - ------- = g*(rho*V - m)
    #   V_u       V_l
    #
    #       m_u               m_l            g
    # ---------------- - ---------------- = ---*(rho*V - m) = C
    # Ap_u*x_eq + Vd_u   Ap_l*x_eq + Vd_l   R*T

    C = lambda g, R_specific, T_ocean, rho, V_hc, m: (g / (R_specific*T_ocean))*(rho*V_hc - m)
    x_eq = lambda Ap_u, Ap_l, m_u, m_l, Vd_u, Vd_l, C: -(0.005*(
            math.sqrt(
                (
                    203*Ap_u*Ap_l*C + 100*Ap_u*C*Vd_l + 100*Ap_u*m_l - \
                    100*Ap_l*C*Vd_u + 100*Ap_l*m_u
                )**2 + \
                400*Ap_u*Ap_l*C*(
                    203*Ap_l*C*Vd_u - 203*Ap_l*m_u + 100*C*Vd_u*Vd_l + 100*m_l*Vd_u - 100*m_u*Vd_l
                )
            ) - \
            203*Ap_u*Ap_l*C - 100*Ap_u*C*Vd_l - 100*Ap_u*m_l + 100*Ap_l*C*Vd_u - 100*Ap_l*m_u
        )
    ) / (Ap_u*Ap_l*C)

    # TODO(andermi) unknown why 108.56 fudge factor is necessary
    c = 107.56*C(g, R_specific, T_ocean, rho, V_hc, m)  # RHS of equation above

    ignore_piston_mean_pos = False
    if not ignore_piston_mean_pos:
        m_u = P0_u*V0_u / (R_specific*T0_u)  # mass of N2 in upper, Ideal Gas Law
        m_l = P0_l*V0_l / (R_specific*T0_l)  # mass of N2 in lower, Ideal Gas Law

        # shift mass between upper and lower to bring equilibrium point to desired x_mean_pos
        m_delta_guess = 0.0
        m_delta = spo.fsolve(lambda m_delta: x_mean_pos - x_eq(Ap_u, Ap_l,
                                                               m_u - m_delta,
                                                               m_l + m_delta,
                                                               Vd_u, Vd_l, c), m_delta_guess)[0]
        # recompute setpoints based on new x_mean_pos and new masses
        V0_u = Ap_u*x_mean_pos + Vd_u
        V0_l = Ap_l*(2.03 - x_mean_pos) + Vd_l
        m_u = m_u - m_delta
        m_l = m_l + m_delta
        P0_u = m_u*R_specific*T0_u / V0_u
        P0_l = m_l*R_specific*T0_l / V0_l

    return V0_u, V0_l, P0_u, P0_l


def computeLawOfCoolingForce(V_, T, c, dt, is_upper):
    piston_area = Ap_u if is_upper else Ap_l
    V = V_

    # Newton's Law of Cooling (non-dimensionalized):
    # Tdot = r*(T_env - T(t)) -> T[n] = dt*r*(Tenv - T[n-1]) + T[n-1] (using forward difference)
    dT = dt * r * (Tenv - T)
    T += dT

    # TODO(andermi) find Qdot (rate of heat transfer) from h, A, dT (Qdot = h*A*dT)
    # radius = 0.045;
    # A = (2.0 * piston_area) * radius * x  # TODO(andermi) compute x from V_
    # h = 11.3;  # (W/(m^2*K)) -- Water<->Mild Steel<->Gas
    # Q_rate = h * A * dT

    # Ideal Gas Law: P = (m*R)*T/V
    P = c * T / V

    # F = P*A
    F = P * piston_area

    return F, P, V, T


def computePolytropicForce(V_, P, V, n, c, v, is_upper):
    piston_area = Ap_u if is_upper else Ap_l
    V0, V = V, V_
    P0 = P
    # polytropic relationship: P = P0*(V0/V)^n
    P = P0 * pow(V0 / V, n)
    # Ideal Gas Law: T = P*V/(m*R)
    T = P * V / c

    '''
    # no heat loss if adiabatic
    cp_R = c_p / R
    Q_rate = 0.0
    is_adiabatic = False
    if !is_adiabatic:
        # Rodrigues, M. J. (June 5, 2014). Heat Transfer During the Piston-Cylinder Expansion of a Gas
        # (Master's thesis, Oregon State University).
        # Retrieved from https://ir.library.oregonstate.edu/downloads/ww72bf399
        # heat loss rate for polytropic ideal gas:
        # dQ/dt = (1 - n/gamma)*(c_p/R)*P*A*dx/dt
        # TODO(andermi) A != piston_area... it's the chamber surface area
        r_ = 0.045
        A = (2.0 * piston_area) * r_ * x  # TODO(andermi) compute x from V_
        Q_rate = (1.0 - n / ADIABATIC_INDEX) * cp_R * P * A * v
    '''

    # F = P*A
    F = P * piston_area

    return F, P, V, T  # , Q_rate


def model_thermal(PV, p, dt,
                  P0, V0, n1, n2,
                  T0, vel_dz_u, vel_dz_l,
                  is_upper=True,
                  ax1=None,
                  ax2=None):
    n = n1
    c = P0 * V0 / T0  # m*R_specific
    mass = c / R_specific

    V = V0
    P = P0
    # Ideal Gas Law: T = P*V/(m*R_specific)
    T = P * V / c
    F = 0

    P_data = []
    V_data = []
    T_data = []
    F_data = []
    v = np.diff(p, prepend=np.mean(p))
    for v_, V_ in zip(v, PV[:, 0]):

        if not is_upper:
            v *= -1.0

        if v_ >= vel_dz_u:
            #print(f'vel above: {v_}')
            n = n1
            F, P, V, T = computePolytropicForce(V_, P, V, n, c, v_, is_upper)
        elif v_ <= vel_dz_l:
            #print(f'vel below: {v_}')
            n = n2
            F, P, V, T = computePolytropicForce(V_, P, V, n, c, v_, is_upper)
        else:
            #print(f'cooling: {v_}')
            F, P, V, T = computeLawOfCoolingForce(V_, T, c, dt, is_upper)

        #if is_upper:
        #    F *= -1.0
        P_data.append(P)
        V_data.append(V)
        T_data.append(T)
        F_data.append(F)

    ax1[0 if is_upper else 1].plot(V_data, P_data, label='Sim')
    ax2[0 if is_upper else 1].plot(dt*np.arange(PV[:, 1].shape[0]), PV[:, 1], label='AtSea')
    ax2[0 if is_upper else 1].plot(dt*np.arange(len(P_data[600:])), P_data[600:], label='Sim')

    diff_means = (np.mean(PV[:, 1]) - np.mean(P_data[600:]))/6894.757
    rmse = np.sqrt(np.mean((PV[600:, 1] - np.array(P_data[600:]))**2.0))/6894.757
    ax2[0 if is_upper else 1].set_title(f"{'Upper' if is_upper else 'Lower'} Pressure\n"
                                        f'Diff means (PSI) = {diff_means:.2f}\n'
                                        f'RMSE (PSI) = {rmse:.2f}')

    print('Diff in mean pressure (PSI):', diff_means)
    print('RMSE pressure (PSI):', rmse)

    x = PV[:, 0]
    x -= Vd_u if is_upper else Vd_l
    x /= Ap_u if is_upper else Ap_l

    x_ = np.array(V_data)
    x_ -= Vd_u if is_upper else Vd_l
    x_ /= Ap_u if is_upper else Ap_l

    print('Diff in mean piston (should be zero):', np.mean(x) - np.mean(x_))

    return np.array(F_data)


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Compute polytropic curves for Pneumatic Spring')
    parser.add_argument('atsea_logs', nargs='+')
    parser.add_argument('--plot', action='store_true')
    parser.add_argument('--override', action='store_true')
    parser.add_argument('--polytropic_u', nargs=6, type=float)  # n1_u, n2_u,
                                                                # P0_u, V0_u,
                                                                # C1_u, C2_u
    parser.add_argument('--polytropic_l', nargs=6, type=float)  # n1_l, n2_l
                                                                # P0_l, V0_l,
                                                                # C1_l, C2_l
    parser.add_argument('--model_thermal', action='store_true')
    parser.add_argument('--thermal_params_u', nargs=3, type=float)  # T0
                                                                    # velocity_deadzone_upper
                                                                    # velocity_deadzone_lower
    parser.add_argument('--thermal_params_l', nargs=3, type=float)  # T0
                                                                    # velocity_deadzone_upper
                                                                    # velocity_deadzone_lower
    parser.add_argument('--set_piston_pos', action='store_true')
    # parser.add_argument('--piston_pos', nargs=2, type=float)  # initial_piston_position
    #                                                           # x_mean_pos
    args, unknown = parser.parse_known_args()

    print(f'Loading: {args.atsea_logs}')
    spring_data, fig1, ax1 = load_atsea_logs(args.atsea_logs,
                                             True if args.model_thermal else args.plot)

    if args.override:
        n1_u, n2_u, P0_u, V0_u, C1_u, C2_u = args.polytropic_u
    else:
        n1_u, n2_u, P0_u, V0_u, C1_u, C2_u = compute_polytropic(
                                                 spring_data.loc[(1, ('uv', 'up'))].to_numpy(),
                                                 target_P=410000)
    print('Upper Polytropic Parameters [n1, n2, P0, V0, T0]: ' +
          f'{n1_u:.04f}, {n2_u:.04f}, {P0_u:.00f},' +
          f' {V0_u:.04f}, {args.thermal_params_u[0] if args.model_thermal else Tenv},'
          f' C1={C1_u:.04f} C2={C2_u:.04f}')

    if args.override:
        n1_l, n2_l, P0_l, V0_l, C1_l, C2_l = args.polytropic_l
    else:
        n1_l, n2_l, P0_l, V0_l, C1_l, C2_l = compute_polytropic(
                                                 spring_data.loc[(1, ('lv', 'lp'))].to_numpy(),
                                                 target_P=1212000)
    print('Lower Polytropic Parameters [n1, n2, P0, V0, T0]: ' +
          f'{n1_l:.04f}, {n2_l:.04f}, {P0_l:.00f},' +
          f' {V0_l:.04f}, {args.thermal_params_l[0] if args.model_thermal else Tenv},'
          f' C1={C1_l:.04f} C2={C2_l:.04f}')

    if ax1 is not None:
        inc_u = np.where(np.diff(spring_data.uv) >= 0)
        dec_u = np.where(np.diff(spring_data.uv) < 0)
        inc_l = np.where(np.diff(spring_data.lv) >= 0)
        dec_l = np.where(np.diff(spring_data.lv) < 0)
        ax1[0].plot(spring_data.uv.to_numpy()[inc_u],
                    np.exp(C1_u)/spring_data.uv.to_numpy()[inc_u]**n1_u,
                    label='Polytropic Fit (Increasing Volume)')
        ax1[0].plot(spring_data.uv.to_numpy()[dec_u],
                    np.exp(C2_u)/spring_data.uv.to_numpy()[dec_u]**n2_u,
                    label='Polytropic Fit (Decreasing Volume)')
        ax1[1].plot(spring_data.lv.to_numpy()[inc_l],
                    np.exp(C1_l)/spring_data.lv.to_numpy()[inc_l]**n1_l,
                    label='Polytropic Fit (Increasing Volume)')
        ax1[1].plot(spring_data.lv.to_numpy()[dec_l],
                    np.exp(C2_l)/spring_data.lv.to_numpy()[dec_l]**n2_l,
                    label='Polytropic Fit (Decreasing Volume)')

    if args.set_piston_pos:
        init_pos = x_mean_pos = np.mean(spring_data.p)
        V0_u, V0_l, P0_u, P0_l = set_piston_position(V0_u, V0_l,
                                                     P0_u, P0_l,
                                                     args.thermal_params_u[0],
                                                     args.thermal_params_l[0],
                                                     init_pos, x_mean_pos)

    if args.model_thermal:
        fig2, ax2 = plt.subplots(2, 1)
        dt = np.mean(np.diff(spring_data.t))
        F_u = model_thermal(spring_data.loc[(1, ('uv', 'up'))].to_numpy(),
                           spring_data.p, dt,
                           P0_u, V0_u, n1_u, n2_u,
                           *args.thermal_params_u,
                           is_upper=True, ax1=ax1, ax2=ax2)
        F_l = model_thermal(spring_data.loc[(1, ('lv', 'lp'))].to_numpy(),
                           spring_data.p, dt,
                           P0_l, V0_l, n1_l, n2_l,
                           *args.thermal_params_l,
                           is_upper=False, ax1=ax1, ax2=ax2)

        fig3, ax3 = plt.subplots(1, 1)
        ax3.plot(spring_data.p/0.0254,
                 (Ap_l*spring_data.lp - Ap_u*spring_data.up)*0.2248,
                 label='AtSea')
        ax3.plot(spring_data.p[600:]/0.0254,
                 (F_l[600:] - F_u[600:])*0.2248, '--',
                 label='Sim')
        atsea_stiffness, _ = np.polyfit(spring_data.p/0.0254,
                                        (Ap_l*spring_data.lp - Ap_u*spring_data.up)*0.2248, 1)
        sim_stiffness, _ = np.polyfit(spring_data.p[600:]/0.0254,
                                      (F_l[600:] - F_u[600:])*0.2248, 1)
        print(f'Stiffness: atsea={atsea_stiffness} sim={sim_stiffness}')
        ax3.set_title(f'Stiffness: AtSea={atsea_stiffness:.2f} Sim={sim_stiffness:.2f}')

    if ax1 is not None:
        import os
        for ax1_ in ax1:
            ax1_.legend()
            ax1_.set_xlabel('Volume (m^3)')
            ax1_.set_ylabel('Pressure (Pascals)')
        ax1[0].set_title(f'Upper Pressure vs Volume\n'
                         f'n1={n1_u:.4f} n2={n2_u:.4f} P0={int(round(P0_u))} V0={V0_u:.4f}\n'
                         f'T0={args.thermal_params_u[0]} Tenv={Tenv}'
                         f' vel_dz_u={args.thermal_params_u[1]}'
                         f' vel_dz_l={args.thermal_params_u[2]}')
        ax1[1].set_title(f'Lower Pressure vs Volume\n'
                         f'n1={n1_l:.4f} n2={n2_l:.4f} P0={int(round(P0_l))} V0={V0_l:.4f}\n'
                         f'T0={args.thermal_params_l[0]} Tenv={Tenv}'
                         f' vel_dz_u={args.thermal_params_l[1]}'
                         f' vel_dz_l={args.thermal_params_l[2]}')
        fig1.tight_layout()
        #fig1.savefig(os.path.basename(args.atsea_logs[0])+'.PVPolytropicFit.png',
        #             dpi=fig1.dpi, bbox_inches='tight')
        for ax2_ in ax2:
            ax2_.legend()
            ax2_.set_xlabel('Time from start (sec)')
            ax2_.set_ylabel('Pressure (Pascals)')
        fig2.tight_layout()
        #fig2.savefig(os.path.basename(args.atsea_logs[0])+'.PressureDiff.png',
        #             dpi=fig2.dpi, bbox_inches='tight')
        ax3.set_xlabel('Piston Position (in)')
        ax3.set_ylabel('Force (pound-force)')
        ax3.legend()
        fig3.tight_layout()
        #fig3.savefig(os.path.basename(args.atsea_logs[0])+'.Stiffness.png',
        #             dpi=fig3.dpi, bbox_inches='tight')
        plt.show()


if __name__ == '__main__':
    main()
