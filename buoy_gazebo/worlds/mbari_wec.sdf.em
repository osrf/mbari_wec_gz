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

}@
<sdf version="1.8">
  <world name="mbari_wec_world">

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>@(lat_lon[0])</latitude_deg>
      <longitude_deg>@(lat_lon[1])</longitude_deg>
      <elevation>0.0</elevation>
    </spherical_coordinates>

    <physics name="step" type="ignored">
      <max_step_size>@(physics_step)</max_step_size>
      <real_time_factor>@(physics_rtf)</real_time_factor>
    </physics>

    <!-- rely on world plugins from server.config -->

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

    <model name="water_plane">
      <static>true</static>
      <link name="link">
        <visual name="water_plane">
          <geometry>
            <plane>
              <size>5 5</size>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
          <material>
            <ambient>0 0 1 0.5</ambient>
            <diffuse>0 0 1 0.5</diffuse>
            <specular>0 0 1 0.5</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="MBARI_WEC_ROS">
      <pose relative_to="world">0 0 -2 0 0 0</pose>

      <include merge="true">
        <uri>package://buoy_description/models/mbari_wec_ros</uri>
      </include>

    </model>

  </world>
</sdf>
