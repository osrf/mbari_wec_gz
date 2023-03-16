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
    inc_wave_seed = 42  # not defined so default

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

def custom_spectrum(w=None, Szz=None):
    ''' Prints the Custom <IncWaveSpectrumType> block for the IncidentWave plugin. '''
    if w is None:
        w = [0.0, 0.2, 0.4, 0.6, 2.0]
    if Szz is None:
        Szz = [0.0, 0.4, 1.0, 1.0, 0.0]
    coefs = \
        ('\n' + 6*' ').join([f'<w{idx}>{wn}</w{idx}> <Szz{idx}>{Szzn}</Szz{idx}>'
            for idx, (wn, Szzn) in enumerate(zip(w, Szz))])
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
        inc_wave_spectrum = partial(custom_spectrum, w, Szz)
    except NameError:
        inc_wave_spectrum = custom_spectrum  # default
else:
    inc_wave_spectrum = lambda : '<!-- No Waves -->'

############################
# Mean Piston Position
############################

# Check if x_mean_pos (m, mean piston position) was passed in via empy
try:
    x_mean_pos
except NameError:
    x_mean_pos = 0.7  # m, default; measured from fully retracted (same as range_finder)

import math
import scipy.optimize as spo

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
R_specific = 0.2968
T_ocean = 283.15  # K
rho = 1025
V_hc = 0.12
m_hc = 820
m_piston = 48
m = m_hc + m_piston

Ap_u = 0.0127
Ap_l = 0.0115
Vd_u = 0.0226
Vd_l = 0.0463
# TODO(andermi) unknown why 108.56 fudge factor is necessary
c = 108.56*C(g, R_specific, T_ocean, rho, V_hc, m)

V0_u = 0.0431
V0_l = 0.0574
P0_u = 410152
P0_l = 1212060
T0_u = 263.15
T0_l = 283.15

m_u = P0_u*V0_u / (R_specific*T0_u)
m_l = P0_l*V0_l / (R_specific*T0_l)


m_delta_guess = 0.0
m_delta = spo.fsolve(lambda m_delta: x_mean_pos - x_eq(Ap_u, Ap_l,
                                                       m_u - m_delta,
                                                       m_l + m_delta,
                                                       Vd_u, Vd_l, c), m_delta_guess)[0]
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
      <!-- measure of valve opening cross-section and duration (meter-seconds) -->
      <valve_absement>7.77e-6</valve_absement>
      <pump_absement>4.8e-8</pump_absement>
      <pump_pressure>1.7e+6</pump_pressure>
      <stroke>2.03</stroke>
      <piston_area>@(Ap_u)</piston_area>
      <dead_volume>@(Vd_u)</dead_volume>
      <T0>@(T0_u)</T0>
      <r>0.1</r>  <!-- coef of heat transfer -->
      <R_specific>0.2968</R_specific>
      <c_p>1.04</c_p>
      <hysteresis>true</hysteresis>
      <velocity_deadzone_lower>-0.1</velocity_deadzone_lower>
      <velocity_deadzone_upper>0.05</velocity_deadzone_upper>
      <n1>1.1725</n1>
      <n2>1.2139</n2>
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
      <pump_absement>4.8e-8</pump_absement>
      <pump_pressure>1.7e+6</pump_pressure>
      <stroke>2.03</stroke>
      <piston_area>@(Ap_l)</piston_area>
      <dead_volume>@(Vd_l)</dead_volume>
      <T0>@(T0_l)</T0>
      <r>0.1</r>  <!-- coef of heat transfer -->
      <R_specific>0.2968</R_specific>
      <c_p>1.04</c_p>
      <hysteresis>true</hysteresis>
      <velocity_deadzone_lower>-0.01</velocity_deadzone_lower>
      <velocity_deadzone_upper>0.1</velocity_deadzone_upper>
      <n1>1.1967</n1>
      <n2>1.1944</n2>
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
