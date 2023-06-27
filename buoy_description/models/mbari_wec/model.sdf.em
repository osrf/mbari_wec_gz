<?xml version="1.0" ?>
@{
# Check if scale_factor was passed in via empy
try:
    scale_factor  # 0.5 to 1.4
except NameError:
    scale_factor = 1.0  # not defined so default

# Check if inc_wave_seed was passed in via empy
try:
    inc_wave_seed
except NameError:
    inc_wave_seed = 0  # not defined so default to 0 which changes seed every run

# Check if battery state (battery_soc or battery_emf) was passed in via empy
try:
    battery_soc  # 0.0 to 1.0
    battery_state = lambda : print(f'<BatterySoC>{battery_soc}</BatterySoC>')
except NameError:
    try:
        battery_emf  # 270V to 320V
    except NameError:
        battery_emf = 300.0  # V, neither defined so default
    battery_state = lambda : print(f'<BatteryEMF>{battery_emf}</BatteryEMF>')

#####################
# Wave Spectra
#####################

def monochromatic_spectrum(A=1.0, T=12.0):
    ''' Prints the MonoChromatic <IncWaveSpectrumType> block for the IncidentWave plugin. '''
    print(f'''
      <IncWaveSpectrumType>MonoChromatic</IncWaveSpectrumType>
      <A>{A}</A>
      <T>{T}</T>
''')

def bretschneider_spectrum(Hs=2.0, Tp=13.0):
    ''' Prints the Bretschneider <IncWaveSpectrumType> block for the IncidentWave plugin. '''
    print(f'''
      <IncWaveSpectrumType>Bretschneider</IncWaveSpectrumType>
      <Hs>{Hs}</Hs>
      <Tp>{Tp}</Tp>
''')

def custom_spectrum(f=None, Szz=None):
    ''' Prints the Custom <IncWaveSpectrumType> block for the IncidentWave plugin. '''
    if f is None:
        f = [0.0, 0.2, 0.4, 0.6, 2.0]
    if Szz is None:
        Szz = [0.0, 0.4, 1.0, 1.0, 0.0]
    coefs = \
        ('\n' + 6*' ').join([f'<f{idx}>{fn}</f{idx}> <Szz{idx}>{Szzn}</Szz{idx}>'
            for idx, (fn, Szzn) in enumerate(zip(f, Szz))])
    print(f'''
      <IncWaveSpectrumType>Custom</IncWaveSpectrumType>
      {coefs}
''')

from functools import partial

# Check if inc_wave_spectrum_type was passed in via empy
try:
    inc_wave_spectrum_type
except NameError:
    inc_wave_spectrum_type = 'Bretschneider'  # not defined so default to Bretschneider

if 'MonoChromatic' in inc_wave_spectrum_type:
    try:
        inc_wave_spectrum = partial(monochromatic_spectrum, A, T)
    except NameError:
        inc_wave_spectrum = monochromatic_spectrum  # default
elif 'Bretschneider' in inc_wave_spectrum_type:
    try:
        inc_wave_spectrum = partial(bretschneider_spectrum, Hs, Tp)
    except NameError:
        inc_wave_spectrum = bretschneider_spectrum  # default
elif 'Custom' in inc_wave_spectrum_type:
    try:
        inc_wave_spectrum = partial(custom_spectrum, f, Szz)
    except NameError:
        inc_wave_spectrum = custom_spectrum  # default
else:
    inc_wave_spectrum = lambda : '<!-- No Waves -->'

############################
# Piston Position
############################

# Would like to specify piston mean position and set all other
# air spring values from that. Start from curve-fitting PV
# for polytropic index and initial setpoints. Determine piston
# equilibrium position. Solve for mass shift between chambers
# that brings the equilibrium to the desired mean piston position.
# Recompute pressure and volume based on Ideal Gas Law.

# Check if initial piston position (m) was passed in via empy
try:
    initial_piston_position
except NameError:
    initial_piston_position = 0.7  # m, default;
                                   # measured from fully retracted (same as range_finder)

# Check if x_mean_pos (m, mean piston position) was passed in via empy
try:
    x_mean_pos
except NameError:
    x_mean_pos = 0.7  # m, default; measured from fully retracted (same as range_finder)


import math
import scipy.optimize as spo

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
                203*Ap_u*Ap_l*C + 100*Ap_u*C*Vd_l + 100*Ap_u*m_l - 100*Ap_l*C*Vd_u + 100*Ap_l*m_u
            )**2 + \
            400*Ap_u*Ap_l*C*(
                203*Ap_l*C*Vd_u - 203*Ap_l*m_u + 100*C*Vd_u*Vd_l + 100*m_l*Vd_u - 100*m_u*Vd_l
            )
        ) - \
        203*Ap_u*Ap_l*C - 100*Ap_u*C*Vd_l - 100*Ap_u*m_l + 100*Ap_l*C*Vd_u - 100*Ap_l*m_u
    )
) / (Ap_u*Ap_l*C)

g = 9.81  # m/s^2
R_specific = 0.2968  # mass-specific gas constant for N2
T_ocean = 283.15  # K, soak temp for equilibrium
rho = 1025  # density of seawater
V_hc = 0.12  # Displacement of heave cone
m_hc = 820  # mass of heave cone
m_piston = 48  # mass of piston
m = m_hc + m_piston  # total mass

Ap_u = 0.0127  # Area of piston in upper chamber
Ap_l = 0.0115  # Area of piston in lower
Vd_u = 0.0266  # Dead volume of upper
Vd_l = 0.0523  # Dead volume of lower
# TODO(andermi) unknown why fudge factor is necessary (prev 108.56)
c = 107.56*C(g, R_specific, T_ocean, rho, V_hc, m)  # RHS of equation above

# Check if upper_polytropic_params was passed in via empy
try:
    upper_polytropic_params
except NameError:
    upper_polytropic_params = [1.4309, 1.4367, 422156, 0.0397, 283.15]
(n1_u,  # upper polytropic index for increasing volume
n2_u,  # upper polytropic index for decreasing volume
P0_u,  # Pressure (Pa) setpoint from upper PV
V0_u,  # Volume (m^3) setpoint from upper chamber polytropic PV curves
T0_u  # Temperature (K) setpoint for upper heat transfer
) = upper_polytropic_params

# Check if lower_polytropic_params was passed in via empy
try:
    lower_polytropic_params
except NameError:
    lower_polytropic_params = [1.3771, 1.3755, 1212098, 0.0661, 283.15]
(n1_l,  # lower polytropic index for increasing volume
n2_l,  # lower polytropic index for decreasing volume
P0_l,  # Pressure (Pa) setpoint from lower PV
V0_l,  # Volume (m^3) setpoint from lower chamber polytropic PV curves
T0_l  # Temperature (K) setpoint for lower heat transfer
) = lower_polytropic_params

ignore_piston_mean_pos = True
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
}@

<sdf version="1.8">
  <model name="MBARI_WEC">

    <include merge="true">
      <uri>package://buoy_description/models/mbari_wec_base</uri>
    </include>

    <!-- Electro-Hydraulic PTO Plugin -->
    <plugin filename="ElectroHydraulicPTO" name="buoy_gazebo::ElectroHydraulicPTO">
      <PrismaticJointName>HydraulicRam</PrismaticJointName>
      <PistonArea>1.375</PistonArea>
      <HydMotorDisp>0.30</HydMotorDisp>
      <RotorInertia>1</RotorInertia>
      <ScaleFactor>@(scale_factor)</ScaleFactor>
      @(battery_state())
    </plugin>

    <!-- Upper Polytropic Spring plugin -->
    <plugin filename="PolytropicPneumaticSpring" name="buoy_gazebo::PolytropicPneumaticSpring">
      <JointName>HydraulicRam</JointName>
      <chamber>upper_polytropic</chamber>
      <is_upper>true</is_upper>
      <initial_piston_position>@(initial_piston_position)</initial_piston_position>
      <!-- measure of valve opening cross-section and duration (meter-seconds) -->
      <valve_absement>7.77e-6</valve_absement>
      <pump_absement>4.3e-8</pump_absement>
      <pump_pressure>1.7e+6</pump_pressure>
      <stroke>2.03</stroke>
      <piston_area>@(Ap_u)</piston_area>
      <dead_volume>@(Vd_u)</dead_volume>
      <T0>@(T0_u)</T0>
      <r>0.5</r>  <!-- coef of heat transfer -->
      <R_specific>0.2968</R_specific>
      <c_p>1.04</c_p>
      <hysteresis>true</hysteresis>
      <velocity_deadzone_lower>-0.10</velocity_deadzone_lower>
      <velocity_deadzone_upper>0.05</velocity_deadzone_upper>
      <n1>@(n1_u)</n1>
      <n2>@(n2_u)</n2>
      <V0>@(V0_u)</V0>
      <P0>@(print(f'{P0_u:.00f}', end=''))</P0>
    </plugin>

    <!-- Lower Polytropic Spring plugin -->
    <plugin filename="PolytropicPneumaticSpring" name="buoy_gazebo::PolytropicPneumaticSpring">
      <JointName>HydraulicRam</JointName>
      <chamber>lower_polytropic</chamber>
      <is_upper>false</is_upper>
      <!-- measure of valve opening cross-section and duration (meter-seconds) -->
      <valve_absement>7.77e-6</valve_absement>
      <pump_absement>4.3e-8</pump_absement>
      <pump_pressure>1.7e+6</pump_pressure>
      <stroke>2.03</stroke>
      <piston_area>@(Ap_l)</piston_area>
      <dead_volume>@(Vd_l)</dead_volume>
      <T0>@(T0_l)</T0>
      <r>0.5</r>  <!-- coef of heat transfer -->
      <R_specific>0.2968</R_specific>
      <c_p>1.04</c_p>
      <hysteresis>true</hysteresis>
      <velocity_deadzone_lower>-0.10</velocity_deadzone_lower>
      <velocity_deadzone_upper>0.05</velocity_deadzone_upper>
      <n1>@(n1_l)</n1>
      <n2>@(n2_l)</n2>
      <V0>@(V0_l)</V0>
      <P0>@(print(f'{P0_l:.00f}', end=''))</P0>
    </plugin>

    <plugin filename="WaveBodyInteractions" name="buoy_gazebo::WaveBodyInteractions">  
      <LinkName>Buoy</LinkName>
      <WaterplaneOrigin_x>0</WaterplaneOrigin_x>  <!-- Waterplane origin relative to link origin -->
      <WaterplaneOrigin_y>0</WaterplaneOrigin_y>
      <WaterplaneOrigin_z>2.46</WaterplaneOrigin_z> 
      <COB_x>0.0</COB_x>  <!-- COG relative to waterplance origin -->
      <COB_y>0.0</COB_y>
      <COB_z>-0.18</COB_z> 
      <Vol>1.75</Vol>
      <S>5.47</S>
      <S11>1.37</S11>
      <S22>1.37</S22>
      <hydro_coeffs_uri>package://buoy_description/models/mbari_wec_base/hydrodynamic_coeffs/BuoyA5</hydro_coeffs_uri> 
    </plugin>


    <plugin filename="IncidentWaves" name="buoy_gazebo::IncidentWaves">  
      <IncWaveSeed>@(inc_wave_seed)</IncWaveSeed>
      @(inc_wave_spectrum())
    </plugin>

    <!-- Adding Friction to PTO -->
    <plugin filename="PTOFriction" name="buoy_gazebo::PTOFriction">
      <PrismaticJointName>HydraulicRam</PrismaticJointName>
    </plugin>

  </model>
</sdf>
