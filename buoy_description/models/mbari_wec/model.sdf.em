<?xml version="1.0" ?>
@{
# Check if scale_factor was passed in via empy
try:
    scale_factor
except NameError:
    scale_factor = 1.0  # not defined so default

# Check if inc_wave_seed was passed in via empy
try:
    inc_wave_seed
except NameError:
    inc_wave_seed = 42  # not defined so default

def monochromatic_spectrum(A=1.0, T=12.0):
    ''' Prints the MonoChromatic <IncWaveSpectrumType> block for the IncidentWave plugin. '''
    print(f'''
      <IncWaveSpectrumType>MonoChromatic</IncWaveSpectrumType>
      <A>{A}</A>
      <T>{T}</T>
''')

def bretschneider_spectrum(Hs=3.0, Tp=14.0):
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
    inc_wave_spectrum_type = ''  # not defined so default to no waves; TODO(hamilton) correct?

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
    inc_wave_spectrum = lambda : '<!-- No Waves -->'  # default no waves; TODO(hamilton) correct?

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
    </plugin>

    <!-- Upper Polytropic Spring plugin -->
    <plugin filename="PolytropicPneumaticSpring" name="buoy_gazebo::PolytropicPneumaticSpring">
      <JointName>HydraulicRam</JointName>
      <chamber>upper_polytropic</chamber>
      <is_upper>true</is_upper>
      <!-- measure of valve opening cross-section and duration (meter-seconds) -->
      <valve_absement>7.77e-6</valve_absement>
      <pump_absement>5.1e-8</pump_absement>
      <pump_pressure>1.7e+6</pump_pressure>
      <stroke>2.03</stroke>
      <piston_area>0.0127</piston_area>
      <dead_volume>0.0226</dead_volume>
      <T0>263.15</T0>
      <r>0.1</r>  <!-- coef of heat transfer -->
      <R_specific>0.2968</R_specific>
      <c_p>1.04</c_p>
      <hysteresis>true</hysteresis>
      <velocity_deadzone_lower>-0.1</velocity_deadzone_lower>
      <velocity_deadzone_upper>0.05</velocity_deadzone_upper>
      <n1>1.1725</n1>
      <n2>1.2139</n2>
      <x1>1.6159</x1>
      <x2>1.8578</x2>
      <P1>410152</P1>
      <P2>410190</P2>
    </plugin>

    <!-- Lower Polytropic Spring plugin -->
    <plugin filename="PolytropicPneumaticSpring" name="buoy_gazebo::PolytropicPneumaticSpring">
      <JointName>HydraulicRam</JointName>
      <chamber>lower_polytropic</chamber>
      <is_upper>false</is_upper>
      <!-- measure of valve opening cross-section and duration (meter-seconds) -->
      <valve_absement>7.77e-6</valve_absement>
      <pump_absement>5.1e-8</pump_absement>
      <pump_pressure>1.7e+6</pump_pressure>
      <stroke>2.03</stroke>
      <piston_area>0.0115</piston_area>
      <dead_volume>0.0463</dead_volume>
      <T0>283.15</T0>
      <r>0.1</r>  <!-- coef of heat transfer -->
      <R_specific>0.2968</R_specific>
      <c_p>1.04</c_p>
      <hysteresis>true</hysteresis>
      <velocity_deadzone_lower>-0.01</velocity_deadzone_lower>
      <velocity_deadzone_upper>0.1</velocity_deadzone_upper>
      <n1>1.1967</n1>
      <n2>1.1944</n2>
      <x1>0.9695</x1>
      <x2>1.1850</x2>
      <P1>1212060</P1>
      <P2>1212740</P2>
    </plugin>

    <plugin filename="WaveBodyInteractions" name="buoy_gazebo::WaveBodyInteractions">  
      <LinkName>Buoy</LinkName>
      <WaterplaneOrigin_x>0</WaterplaneOrigin_x>  <!-- Waterplane origin relative to link origin -->
      <WaterplaneOrigin_y>0</WaterplaneOrigin_y>
      <WaterplaneOrigin_z>2.46</WaterplaneOrigin_z> 
      <COB_x>0</COB_x>  <!-- COG relative to waterplance origin -->
      <COB_y>0</COB_y>
      <COB_z>-.18</COB_z> 
      <Vol>1.75</Vol>
      <S>5.47</S>
      <S11>1.37</S11>
      <S22>1.37</S22>
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
