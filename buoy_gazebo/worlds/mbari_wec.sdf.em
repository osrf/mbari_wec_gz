<?xml version="1.0" ?>
@{

# Check if physics_step was passed in via empy
try:
    physics_step
except NameError:
    physics_step = 0.01  # not defined so default

# Check if physics_rtf was passed in via empy
try:
    physics_rtf
except NameError:
    physics_rtf = 1.0  # not defined so default

# Check if lat_lon was passed in via empy
try:
    lat_lon
except NameError:
    lat_lon = [36.743850, -121.876233]  # not defined so default

# Check if initial_buoy_height was passed in via empy
try:
    initial_buoy_height
except NameError:
    initial_buoy_height = 2.8  # not defined so default

}@
<sdf version="1.10">
  <world name="mbari_wec_world">

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>@(lat_lon[0])</latitude_deg>
      <longitude_deg>@(lat_lon[1])</longitude_deg>
      <elevation>0.0</elevation>
      <heading_deg>0.0</heading_deg>
    </spherical_coordinates>

    <physics name="step" type="ignored">
      <max_step_size>@(physics_step)</max_step_size>
      <real_time_factor>@(physics_rtf)</real_time_factor>
    </physics>

    <!-- rely on world plugins from server.config -->

    <scene>
      <sky></sky>
    </scene>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Incident wave surface. IncWaveSurfaceVisual draws it in the GUI from the
         wave components the IncWaveSurface plugin on the buoy serves. -->
    <model name="inc_wave_surface">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <!-- Placeholder: the plugin draws the surface itself -->
          <geometry>
            <box>
              <size>0.001 0.001 0.001</size>
            </box>
          </geometry>
          <transparency>1</transparency>
          <cast_shadows>false</cast_shadows>
          <plugin filename="IncWaveSurfaceVisual" name="buoy_gazebo::IncWaveSurfaceVisual">
            <follow_link>Buoy</follow_link>
            <update_rate>30</update_rate>  <!-- Hz, wall clock -->
            <!-- Short waves above the 0.5 Hz FreeSurfaceHydrodynamics cutoff, visual only:
                 none, tail (continue the spectrum) or wind (wind sea) -->
            <chop>wind</chop>
            <wind_speed>8</wind_speed>  <!-- m/s -->
            <opacity>0.75</opacity>
          </plugin>
        </visual>
      </link>
    </model>

    <model name="MBARI_WEC_ROS">
      <pose relative_to="world">0 0 -@(initial_buoy_height) 0 0 0</pose>

      <include merge="true">
        <uri>package://buoy_description/models/mbari_wec_ros</uri>
      </include>

    </model>

  </world>
</sdf>
