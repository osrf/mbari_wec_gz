<?xml version="1.0" ?>
@{
from gz.math7 import Cylinderd
from gz.math7 import MassMatrix3d
from gz.math7 import Material
import math

##############
# Parameters #
##############

# PTO
pto_stl_inner_radius = 0.02
pto_gap = 0.02

# Piston
piston_length = 5.08
piston_z_offset = -3.50066

# Tether
tether_radius = 0.025 #0.009525 # Nominal O.D. 0.75 in
tether_density = 3350 # kg/m^3
tether_length = 20.3

num_tether_top_links = 15
tether_top_length = 2.5

num_tether_bottom_links = 5

# TrefoilDoors
trefoil_pose = {'open': 1.047, 'closed': 0.0}
# check if door_state was passed in by empy
try:
    door_state
except NameError:
    door_state = 'closed'  # not defined so default closed

if 'open' in door_state:
    mu_zz = 3000.0  # kg, Heave Added Mass
    heavecone_zWabsW = -3200.0  # kg/m, Heave Quadratic Drag

# Heave cone
heave_total_mass = 817  # kg
trefoil_mass = 20  # kg

########################
# TODO(anyone) mu_zz below is currently unused. Will be used in #115
########################

# check if mu_zz was set by door_state 'open' (or passed in by empy)
try:
    mu_zz
except NameError:
    mu_zz = 10000.0  # kg, not defined so default with doors closed

# check if z_ww was set by door_state 'open' (or passed in by empy)
try:
    heavecone_zWabsW
except NameError:
    heavecone_zWabsW = -3900.0  # kg/m, not defined so default with doors closed

###################
# Computed values #
###################

# Top tether
tether_top_link_length = tether_top_length / num_tether_top_links

tether_top_link_cylinder = Cylinderd(tether_top_link_length, tether_radius)
tether_top_link_cylinder.set_mat(Material(tether_density))
tether_top_link_mm = MassMatrix3d()
tether_top_link_cylinder.mass_matrix(tether_top_link_mm)

# Bottom tether
tether_bottom_length = tether_length - tether_top_length
tether_bottom_link_length = tether_bottom_length / num_tether_bottom_links

tether_bottom_link_cylinder = Cylinderd(tether_bottom_link_length, tether_radius)
tether_bottom_link_cylinder.set_mat(Material(tether_density))
tether_bottom_link_mm = MassMatrix3d()
tether_bottom_link_cylinder.mass_matrix(tether_bottom_link_mm)

def tether_joint_properties():
    """ Prints the <dynamics> and <limit> blocks for tether joints. """
    print("""
        <dynamics>
          <damping>1000.0</damping>
          <friction>500</friction>
          <spring_reference>0</spring_reference>
          <spring_stiffness>1000</spring_stiffness>
        </dynamics>
        <limit>
          <lower>-0.05</lower>
          <upper>0.05</upper>
        </limit>
    """)

def bridal_joint_properties():
    """ Prints the <dynamics> and <limit> blocks for the bridal joint. """
    print("""
        <dynamics>
          <damping>100.0</damping>
          <friction>50</friction>
          <spring_reference>0</spring_reference>
          <spring_stiffness>100</spring_stiffness>
        </dynamics>
        <limit>
          <lower>-1.570796</lower>
          <upper>1.570796</upper>
        </limit>
    """)

# PTO
pto_inner_radius = tether_radius + pto_gap
pto_scale = pto_inner_radius / pto_stl_inner_radius
}@
<sdf version="1.8">
  <model name="MBARI_WEC_BASE">
    <self_collide>true</self_collide>
    <link name="Buoy">
      <pose relative_to="__model__">0 0 0 0 0 0</pose>
      <inertial>
        <pose>0 0 2.13 0 0 0</pose>
        <mass>1400</mass>
        <inertia>
          <ixx>1450.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1450.0</iyy>
          <iyz>0.0</iyz>
          <izz>670.0</izz>
        </inertia>
        <fluid_added_mass>
           <xx>330.0</xx>
           <xy>0.0</xy>
           <xz>0.0</xz>
           <xp>0.0</xp>
           <xq>180.0</xq>
           <xr>0.0</xr>
           <yy>330.0</yy>
           <yz>0.0</yz>
           <yp>-180.0</yp>
           <yq>0.0</yq>
           <yr>0.0</yr>
           <zz>2800.0</zz>
           <zp>0.0</zp>
           <zq>0.0</zq>
           <zr>0.0</zr>
           <pp>430</pp>
           <pq>0.0</pq>
           <pr>0.0</pr>
           <qq>430</qq>
           <qr>0.0</qr>
           <rr>0.0</rr>
       </fluid_added_mass>
      </inertial>
      <visual name="visual_Buoy">
        <geometry>
          <mesh>
            <uri>package://buoy_description/models/mbari_wec_base/meshes/buoy_float.stl</uri>
          </mesh>
        </geometry>
        <!--color-->
        <material>
          <ambient>1.0 1.0 0.0 1</ambient>
          <diffuse>1.0 1.0 0.0 1</diffuse>
          <specular>1.0 1.0 0.0 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <pose>0 0 2.46 0 0 0 </pose>
        <geometry>
          <box>
            <size>2.34 2.34 1</size>
          </box>
        </geometry>
      </collision>
      <sensor name='xbow_imu' type='imu'>
        <topic>Buoy_link/xbow_imu</topic>
        <update_rate>50</update_rate>
        <imu>
          <orientation_reference_frame>
            <localization>ENU</localization>
          </orientation_reference_frame>
        </imu>
        <always_on>1</always_on>
        <visualize>true</visualize>
      </sensor>
    </link>

    <link name="PTO">
      <pose relative_to="Buoy">0 0 0 0 0 0</pose>
      <inertial>
        <pose>0 0 -3.67 0 0 0</pose>
        <mass>600</mass>
        <inertia>
          <ixx>3220.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>3220.0</iyy>
          <iyz>0.0</iyz>
          <izz>10.0</izz>
        </inertia>
        <fluid_added_mass>
           <xx>160.0</xx>
           <xy>0.0</xy>
           <xz>0.0</xz>
           <xp>0.0</xp>
           <xq>0.0</xq>
           <xr>0.0</xr>
           <yy>160.0.0</yy>
           <yz>0.0</yz>
           <yp>0.0</yp>
           <yq>0.0</yq>
           <yr>0.0</yr>
           <zz>0.0</zz>
           <zp>0.0</zp>
           <zq>0.0</zq>
           <zr>0.0</zr>
           <pp>0.0</pp>
           <pq>0.0</pq>
           <pr>0.0</pr>
           <qq>0.0</qq>
           <qr>0.0</qr>
           <rr>0.0</rr>
       </fluid_added_mass>
      </inertial>
      <visual name="visual_PTO">
        <geometry>
          <mesh>
            <uri>package://buoy_description/models/mbari_wec_base/meshes/pto.stl</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>1 1 1 0.9</ambient>
          <diffuse>.2 .2 1 0.9</diffuse>
          <specular>1 1 1 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>package://buoy_description/models/mbari_wec_base/meshes/pto_collision.stl</uri>
            <scale>@(pto_scale) @(pto_scale) 1.0</scale>
          </mesh>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>10</mu>
              <mu2>10</mu2>
            </ode>
          </friction>
          <contact>
            <!-- Collide with top tether -->
            <!--collide_bitmask>0x01</collide_bitmask-->
          </contact>
        </surface>
      </collision>
    </link>

    <link name="Piston">
      <pose relative_to="PTO">0 0 @(piston_z_offset) 0 0 0</pose>
      <inertial>
        <mass>48.0</mass>
        <pose>0 0 -2.58 0 0 0</pose>
        <inertia>
          <!-- TODO(hamilton) Refine values -->
          <ixx>100.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>100.0</iyy>
          <iyz>0.0</iyz>
          <izz>5.0</izz>
        </inertia>
      </inertial>
      <visual name="visual_Piston">
        <geometry>
          <mesh>
            <uri>package://buoy_description/models/mbari_wec_base/meshes/rod_and_piston.stl</uri>
          </mesh>
        </geometry>
        <!--color-->
        <material>
          <ambient>0.0 0.0 1.0 1</ambient>
          <diffuse>0.0 0.0 1.0 1</diffuse>
          <specular>0.0 0.0 1.0 1</specular>
        </material>
      </visual>
    </link>
    <frame name="PistonBottom" attached_to="Piston">
      <pose>0 0 -@(piston_length) 0 0 0</pose>
    </frame>

    <!-- start tether top -->
@[for link_index in range(num_tether_top_links)]@
    <link name="tether_top_@(link_index)">
      <pose relative_to="PistonBottom">0 0 -@((link_index + 0.5) * tether_top_link_length) 0 0 0</pose>
      <inertial>
        <mass>@(tether_top_link_mm.mass())</mass>
        <inertia>
          <ixx>@(tether_top_link_mm.ixx())</ixx>
          <iyy>@(tether_top_link_mm.iyy())</iyy>
          <izz>@(tether_top_link_mm.izz())</izz>
        </inertia>
      </inertial>
      <visual name="visual_tether_top_@(link_index)">
        <geometry>
          <cylinder>
            <radius>@(tether_radius)</radius>
            <length>@(tether_top_link_length)</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.1 0.1 .1 1</ambient>
          <diffuse>0.1 0.1 .1 1</diffuse>
          <specular>0.1 0.1 .1 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>@(tether_radius)</radius>
            <length>@(tether_top_link_length)</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>10</mu>
              <mu2>10</mu2>
            </ode>
          </friction>
          <contact>
            <!-- Collide with PTO -->
            <!--collide_bitmask>0x01</collide_bitmask-->
          </contact>
        </surface>
      </collision>
    </link>

    <joint name="tether_top_joint_@(link_index)" type="universal">
      <pose>0 0 @(tether_top_link_length * 0.5) 0 0 0</pose>
@[if link_index == 0]@
      <parent>Piston</parent>
@[else]@
      <parent>tether_top_@(link_index-1)</parent>
@[end if]@
      <child>tether_top_@(link_index)</child>
      <axis>
        <xyz>1 0 0</xyz>
        @(tether_joint_properties())
      </axis>
      <axis2>
        <xyz>0 1 0</xyz>
        @(tether_joint_properties())
      </axis2>
    </joint>
@[end for]@
    <!-- end tether top -->

    <!-- start tether bottom -->
@[for link_index in range(num_tether_bottom_links)]@
    <link name="tether_bottom_@(link_index)">
      <pose relative_to="PistonBottom">0 0 -@((link_index + 0.5) * tether_bottom_link_length + tether_top_length) 0 0 0</pose>
      <inertial>
        <mass>@(tether_bottom_link_mm.mass())</mass>
        <inertia>
          <ixx>@(tether_bottom_link_mm.ixx())</ixx>
          <iyy>@(tether_bottom_link_mm.iyy())</iyy>
          <izz>@(tether_bottom_link_mm.izz())</izz>
        </inertia>
      </inertial>
      <visual name="visual_tether_bottom_@(link_index)">
        <geometry>
          <cylinder>
            <radius>@(tether_radius)</radius>
            <length>@(tether_bottom_link_length)</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.1 1 .1 1</ambient>
          <diffuse>0.1 1 .1 1</diffuse>
          <specular>0.1 1 .1 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>@(tether_radius)</radius>
            <length>@(tether_bottom_link_length)</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <joint name="tether_bottom_joint_@(link_index)" type="universal">
      <pose>0 0 @(tether_bottom_link_length * 0.5) 0 0 0</pose>
@[if link_index == 0]@
      <parent>tether_top_@(num_tether_top_links-1)</parent>
@[else]@
      <parent>tether_bottom_@(link_index-1)</parent>
@[end if]@
      <child>tether_bottom_@(link_index)</child>
      <axis>
        <xyz>1 0 0</xyz>
        @(tether_joint_properties())
      </axis>
      <axis2>
        <xyz>0 1 0</xyz>
        @(tether_joint_properties())
      </axis2>
    </joint>
@[end for]@

    <joint name="TetherToHeaveCone" type="universal">
      <parent>tether_bottom_@(num_tether_bottom_links-1)</parent>
      <child>HeaveCone</child>
      <axis>
        <xyz>1 0 0</xyz>
        @(tether_joint_properties())
      </axis>
      <axis2>
        <xyz>0 1 0</xyz>
        @(tether_joint_properties())
      </axis2>
    </joint>
    <!-- end tether bottom -->

    <link name="HeaveCone">
      <pose relative_to="PistonBottom">0 0 -@(tether_length) 0 0 0</pose>
      <inertial>
        <pose>0 0 -1.2 0 0 0</pose>
        <mass>@(heave_total_mass - trefoil_mass)</mass>
        <inertia>
          <ixx>340.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>340.0</iyy>
          <iyz>0.0</iyz>
          <izz>600.0</izz>
        </inertia>
        <fluid_added_mass>
           <xx>780.0</xx>
           <xy>0.0</xy>
           <xz>0.0</xz>
           <xp>0.0</xp>
           <xq>0.0</xq>
           <xr>0.0</xr>
           <yy>780.0</yy>
           <yz>0.0</yz>
           <yp>0.0</yp>
           <yq>0.0</yq>
           <yr>0.0</yr>
           <zz>@(mu_zz)</zz>
           <zp>0.0</zp>
           <zq>0.0</zq>
           <zr>0.0</zr>
           <pp>0.1</pp>
           <pq>0.0</pq>
           <pr>0.0</pr>
           <qq>0.1</qq>
           <qr>0.0</qr>
           <rr>0.1</rr>
       </fluid_added_mass>
      </inertial>
      <visual name="visual_HeaveCone">
        <geometry>
          <mesh>
            <uri>package://buoy_description/models/mbari_wec_base/meshes/heave_cone.stl</uri>
          </mesh>
        </geometry>
        <!--color-->
        <material>
          <ambient>0.1 0.1 .1 1</ambient>
          <diffuse>0.1 0.1 .1 1</diffuse>
          <specular>0.1 0.1 .1 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <pose>0 0 -1.21 0 0 0 </pose>
        <geometry>
          <box>
            <size>0.578 0.578 0.5771</size>
          </box>
        </geometry>
      </collision>
    </link>

    <link name="Trefoil">
      <pose relative_to="HeaveCone">0 0 0 0 0 0</pose>
      <inertial>
        <pose>0 0 -1.2 0 0 0</pose>
        <mass>@(trefoil_mass)</mass>
        <inertia>
          <ixx>10</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>10</iyy>
          <iyz>0</iyz>
          <izz>20.0</izz>
        </inertia>
      </inertial>
      <visual name="visual_Trefoil">
        <geometry>
          <mesh>
            <uri>package://buoy_description/models/mbari_wec_base/meshes/trefoil.stl</uri>
          </mesh>
        </geometry>
        <!--color-->
        <material>
          <ambient>0.1 0.1 .1 1</ambient>
          <diffuse>0.1 0.1 .1 1</diffuse>
          <specular>0.1 0.1 .1 1</specular>
        </material>
      </visual>
    </link>

    <joint name="Universal" type="universal">
      <parent>Buoy</parent>
      <child>PTO</child>
      <provide_feedback>1</provide_feedback>
      <pose>0.0 0.0 0.0 0 0 0</pose>
      <axis>
        <xyz>1 0 0</xyz>
        <!-- #@(bridal_joint_properties()) -->
      </axis>
      <axis2>
        <xyz>0 1 0</xyz>
        <!-- #@(bridal_joint_properties()) -->
      </axis2>
      <sensor name="force_torque_sensor" type="force_torque">
        <always_on>true</always_on>
        <update_rate>50</update_rate>
        <visualize>true</visualize>
        <topic>Universal_joint/force_torque</topic>
        <force_torque>
          <frame>sensor</frame>
          <measure_direction>parent_to_child</measure_direction>
        </force_torque>
      </sensor>
    </joint>

    <joint name="HydraulicRam" type="prismatic">
      <parent>PTO</parent>
      <child>Piston</child>
      <pose>0.0 0.0 0.0 0 0 0</pose>
      <axis>
        <limit>
          <lower>0.0</lower>
          <!-- TODO(chapulina) Check why it's only going up to ~1.16-->
          <upper>2.03</upper>
          <effort>1e6</effort>
        </limit>
        <xyz>0.0 0.0 1.0</xyz>
      </axis>
    </joint>

    <joint name="TrefoilDoors" type="fixed">
      <parent>HeaveCone</parent>
      <child>Trefoil</child>
      <provide_feedback>1</provide_feedback>
      <pose>0.0 0.0 0.0 0 0 @(trefoil_pose[door_state])</pose>
    </joint>

    <!-- Viscous Drag for Buoy -->
    <plugin filename="gz-sim-hydrodynamics-system"
      name="gz::sim::systems::Hydrodynamics">
      <link_name>Buoy</link_name>
      <xUabsU>-430</xUabsU>  <!-- Surge Quadratic Drag kg/m -->
      <yVabsV>-430</yVabsV>  <!-- Sway Quadratic Drag kg/m -->
      <zWabsW>-3280</zWabsW>  <!-- Heave Quadratic Drag kg/m -->
      <kPabsP>-880</kPabsP>  <!-- Roll Quadratic Drag kg m^2 -->
      <mQabsQ>-880</mQabsQ>  <!-- Pitch Quadratic Drag kg m^2 -->
      <nRabsR>-50</nRabsR>  <!-- Yaw Quadratic Drag kg m^2 -->
      <disable_coriolis>true</disable_coriolis>
      <disable_added_mass>true</disable_added_mass>
    </plugin>

    <!-- Viscous Drag for PTO -->
    <plugin filename="gz-sim-hydrodynamics-system"
      name="gz::sim::systems::Hydrodynamics">
      <link_name>PTO</link_name>
      <xUabsU>-1140</xUabsU>  <!-- Surge Quadratic Drag kg/m -->
      <yVabsV>-1140</yVabsV>  <!-- Sway Quadratic Drag kg/m -->
      <zWabsW>-50</zWabsW>  <!-- Heave Quadratic Drag kg/m -->
      <kPabsP>-195400</kPabsP>  <!-- Roll Quadratic Drag kg m^2 -->
      <mQabsQ>-195400</mQabsQ>  <!-- Pitch Quadratic Drag kg m^2 -->
      <nRabsR>-50</nRabsR>  <!-- Yaw Quadratic Drag kg m^2 -->
      <disable_coriolis>true</disable_coriolis>
      <disable_added_mass>true</disable_added_mass>
    </plugin>

    <!-- Viscous Drag for Heave Cone -->
    <plugin filename="gz-sim-hydrodynamics-system"
      name="gz::sim::systems::Hydrodynamics">
      <link_name>HeaveCone</link_name>
      <xUabsU>-1580</xUabsU>  <!-- Surge Quadratic Drag kg/m -->
      <yVabsV>-1580</yVabsV>  <!-- Sway Quadratic Drag kg/m -->
<<<<<<< HEAD
      <zWabsW>-3900</zWabsW>  <!-- Vertical Quadratic Drag kg/m: -3200 open, -3900 close -->
=======
      <!-- Vertical Quadratic Drag kg/m: -3200 open, -3900 close -->
      <zWabsW>@(heavecone_zWabsW)</zWabsW>
>>>>>>> main
      <kPabsP>-4620</kPabsP>  <!-- Roll Quadratic Drag kg m^2 -->
      <mQabsQ>-4620</mQabsQ>  <!-- Pitch Quadratic Drag kg m^2 -->
      <nRabsR>-50</nRabsR>  <!-- Yaw Quadratic Drag kg m^2 -->
      <disable_coriolis>true</disable_coriolis>
      <disable_added_mass>true</disable_added_mass>
    </plugin>

  </model>
</sdf>
