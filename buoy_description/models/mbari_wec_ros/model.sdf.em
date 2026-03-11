<?xml version="1.0" ?>
@{
from functools import partial

def _IncidentWaveHeightPoints(points):
    ''' Prints the points tags for the IncWaveHeight plugin. '''
    points_ = []
    for x, y in points:
        points_.append(f'<xy>{x} {y}</xy>')
    print('\n\t'.join(points_))

# Check if inc_wave_height_points was passed in via empy
try:
    inc_wave_height_points
except NameError:
    inc_wave_height_points = None  # not defined so default

if inc_wave_height_points is not None:
    IncidentWaveHeightPoints = partial(_IncidentWaveHeightPoints, points=inc_wave_height_points)
else:
    IncidentWaveHeightPoints = lambda: None  # no points to print
}
<sdf version="1.10">
  <model name="MBARI_WEC_ROS">

    <include merge="true">
      <uri>package://buoy_description/models/mbari_wec</uri>
    </include>

    <!-- Handle Unimplemented Services -->
    <plugin filename="NoOpController" name="buoy_gazebo::NoOpController">
      <namespace>/</namespace>
      <node_name>noop_controller</node_name>
    </plugin>

    <plugin filename="SpringController" name="buoy_gazebo::SpringController">
      <JointName>HydraulicRam</JointName>
      <namespace>/</namespace>
      <node_name>spring_controller</node_name>
      <topic>spring_data</topic>
      <publish_rate>10</publish_rate>
    </plugin>

    <plugin filename="PowerController" name="buoy_gazebo::PowerController">
      <JointName>HydraulicRam</JointName>
      <namespace>/</namespace>
      <node_name>power_controller</node_name>
      <topic>power_data</topic>
      <publish_rate>10</publish_rate>
    </plugin>

    <plugin filename="BatteryController" name="buoy_gazebo::BatteryController">
      <JointName>HydraulicRam</JointName>
      <namespace>/</namespace>
      <node_name>battery_controller</node_name>
      <topic>battery_data</topic>
      <publish_rate>10</publish_rate>
    </plugin>

    <plugin filename="XBowAHRS" name="buoy_gazebo::XBowAHRS">
      <namespace>/</namespace>
      <node_name>xbow_ahrs</node_name>
      <xb_topic>ahrs_data</xb_topic>
      <imu_topic>xb_imu</imu_topic>
      <publish_rate>10</publish_rate>
    </plugin>

    <plugin filename="TrefoilController" name="buoy_gazebo::TrefoilController">
      <namespace>/</namespace>
      <node_name>trefoil_controller</node_name>
      <tf_topic>trefoil_data</tf_topic>
      <imu_topic>trefoil_imu</imu_topic>
      <mag_topic>trefoil_mag</mag_topic>
      <publish_rate>10</publish_rate>
    </plugin>

    <plugin filename="LatentData" name="buoy_gazebo::LatentDataPublisher">
      <namespace>/</namespace>
      <node_name>latent_data</node_name>
      <ros2_topic>latent_data</ros2_topic>
      <publish_rate>10</publish_rate>
    </plugin>

    <plugin filename="IncWaveHeight" name="buoy_gazebo::IncWaveHeight">
      <namespace>/</namespace>
      <node_name>inc_wave_service</node_name>
      <points use_buoy_origin="true">
        <xy>0.0 0.0</xy>
        @(IncidentWaveHeightPoints())
        <!-- may add multiple xy tags -->
        <!-- <xy>-1.0 0.0</xy> -->
        <!-- <xy>1.0 0.0</xy> -->
        <!-- matches relative positions of example SWIFT data -->
        <!--xy>-125.0 -25.0</xy-->
        <!--xy>-185.0 -115.0</xy-->
        <!--xy>-100.0 -135.0</xy-->
        <!--xy>45.0 -175.0</xy-->
        <!-- "optimal" diamond configuration for wave dir from=225 deg -->
        <!--xy>-185.0 -185.0</xy-->
        <!--xy>-77.0 -77.0</xy-->
        <!--xy>-185.0 -77.0</xy-->
        <!--xy>-77.0 -185.0</xy-->
        <!-- "optimal" diamond configuration for wave dir from=200 deg -->
        <!--xy>-90.0 -246.0</xy-->
        <!--xy>-37.0 -103.0</xy-->
        <!--xy>-135.0 -148.0</xy-->
        <!--xy>8.0 -201.0</xy-->
      </points>
    </plugin>

    <plugin
      filename="gz-sim-joint-state-publisher-system"
      name="gz::sim::systems::JointStatePublisher">
    </plugin>

    <plugin
      filename="gz-sim-pose-publisher-system"
      name="gz::sim::systems::PosePublisher">
      <publish_link_pose>true</publish_link_pose>
      <use_pose_vector_msg>true</use_pose_vector_msg>
      <static_publisher>true</static_publisher>
      <static_update_frequency>1</static_update_frequency>
    </plugin>

    <plugin
      filename="gz-sim-odometry-publisher-system"
      name="gz::sim::systems::OdometryPublisher">
      <dimensions>3</dimensions>
      <odom_frame>MBARI_WEC_ROS/odom</odom_frame>
      <robot_base_frame>MBARI_WEC_ROS</robot_base_frame>
    </plugin>

  </model>
</sdf>
