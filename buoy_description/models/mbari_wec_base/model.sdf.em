<?xml version="1.0" ?>
@{
from ignition.math import Cylinderd
from ignition.math import MassMatrix3d
from ignition.math import Material
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

# Heave cone
heave_total_mass = 817
trefoil_mass = 10
heave_height = 1.65 # From STL mesh in Blender

# Mooring
seafloor_depth = 80
# TODO(mabelzhang) get actual numbers or check these make sense
mooring_radius = 0.025
mooring_density = 3350 # kg/m^3
mooring_length = seafloor_depth - 33 # Heave cone is at about -30 m

num_mooring_links = 20

# Chain to lie on seafloor
chain_length = 20
num_chain_links = 100  # Short links to allow chain to curve above seafloor

# Anchor
anchor_mass = 180 # 400 lb
anchor_radius = 0.3
anchor_height = 0.1

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

# PTO
pto_inner_radius = tether_radius + pto_gap
pto_scale = pto_inner_radius / pto_stl_inner_radius

# Mooring
mooring_link_length = mooring_length / num_mooring_links
chain_link_length = chain_length / num_chain_links

def mooring_joint_properties():
    """ Prints the <dynamics> and <limit> blocks for mooring joints. """
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
          <ixx>1429</ixx>
          <ixy>6.77</ixy>
          <ixz>4.69</ixz>
          <iyy>670.31</iyy>
          <iyz>30.5</iyz>
          <izz>1476</izz>
        </inertia>
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
        <mass>605</mass>
        <inertia>
          <ixx>3219</ixx>
          <ixy>-0.43</ixy>
          <ixz>-2.56</ixz>
          <iyy>3219</iyy>
          <iyz>3.37</iyz>
          <izz>7.28</izz>
        </inertia>
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
        <mass>48</mass>
        <pose>0 0 -2.57934 0 0 0</pose>
        <inertia>
          <!-- TODO(chapulina) Get real values -->
          <ixx>128</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>128</iyy>
          <iyz>0</iyz>
          <izz>0.0216</izz>
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
        <pose>0 0 -1.25 0 0 0</pose>
        <mass>@(heave_total_mass - trefoil_mass)</mass>
        <inertia>
          <ixx>339.8</ixx>
          <ixy>0.16</ixy>
          <ixz>-0.29</ixz>
          <iyy>343.73</iyy>
          <iyz>0.33</iyz>
          <izz>613.52</izz>
        </inertia>
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
    <frame name="HeaveConeBottom" attached_to="HeaveCone">
      <pose>0 0 -@(heave_height) 0 0 0</pose>
    </frame>

    <link name="Trefoil">
      <pose relative_to="HeaveCone">0 0 0 0 0 0</pose>
      <inertial>
        <pose>0 0 -1.25 0 0 0</pose>
        <!-- TODO(chapulina) Get real values -->
        <mass>@(trefoil_mass)</mass>
        <inertia>
          <ixx>10</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>10</iyy>
          <iyz>0</iyz>
          <izz>19.9</izz>
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
        @(tether_joint_properties())
      </axis>
      <axis2>
        <xyz>0 1 0</xyz>
        @(tether_joint_properties())
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

    <joint name="TrefoilDoors" type="revolute">
      <parent>HeaveCone</parent>
      <child>Trefoil</child>
      <provide_feedback>1</provide_feedback>
      <pose>0.0 0.0 0.0 0 0 0</pose>
      <axis>
        <limit>
          <lower>0.0</lower>
          <upper>1.047</upper>
        </limit>
        <xyz>0.0 0.0 1.0</xyz>
      </axis>
    </joint>

    <!-- begin mooring -->
@[for link_index in range(num_mooring_links)]@
    <link name="mooring_@(link_index)">
      <pose relative_to="HeaveConeBottom">0 0 -@((link_index + 0.5) * mooring_link_length) 0 0 0</pose>
      <!-- TODO(mabelzhang) Get actual numbers for mooring. Now using tether ones -->
      <inertial>
        <mass>@(tether_top_link_mm.mass())</mass>
        <inertia>
          <ixx>@(tether_top_link_mm.ixx())</ixx>
          <iyy>@(tether_top_link_mm.iyy())</iyy>
          <izz>@(tether_top_link_mm.izz())</izz>
        </inertia>
      </inertial>
      <visual name="visual_mooring_@(link_index)">
        <geometry>
          <cylinder>
            <radius>@(mooring_radius)</radius>
            <length>@(mooring_link_length)</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.1 1 1 1</ambient>
          <diffuse>0.1 1 1 1</diffuse>
          <specular>0.1 1 1 1</specular>
        </material>
      </visual>
      <collision name="collision_mooring_@(link_index)">
        <geometry>
          <cylinder>
            <radius>@(mooring_radius)</radius>
            <length>@(mooring_link_length)</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <!-- TODO(mabelzhang) Tune these for seafloor -->
              <mu>10</mu>
              <mu2>10</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>

    <joint name="mooring_joint_@(link_index)" type="universal">
      <pose>0 0 @(mooring_link_length * 0.5) 0 0 0</pose>
@[if link_index == 0]@
      <parent>HeaveConeBottom</parent>
@[else]@
      <parent>mooring_@(link_index-1)</parent>
@[end if]@
      <child>mooring_@(link_index)</child>
      <axis>
        <xyz>1 0 0 </xyz>
        @(mooring_joint_properties())
      </axis>
      <axis2>
        <xyz>0 1 0 </xyz>
        @(mooring_joint_properties())
      </axis2>
    </joint>
@[end for]@

    <frame name="MooringBottom" attached_to="mooring_@(num_mooring_links-1)">
      <pose>0 0 -@(mooring_link_length * 0.5) 0 0 0</pose>
    </frame>
    <!-- end mooring -->

    <!-- begin chain -->
@[for link_index in range(num_chain_links)]@
    <link name="chain_@(link_index)">
      <!-- Horizontal instead of vertical so does not go through seafloor.
           Arbitrarily choose +x for direction. Gravity will naturally drop
           chain to seafloor. -->
      <pose relative_to="MooringBottom" degrees="true">@((link_index + 0.5) * chain_link_length) 0 0 0 90 0</pose>
      <!-- TODO(mabelzhang) Get actual numbers for mooring. Now using tether ones -->
      <inertial>
        <mass>@(tether_top_link_mm.mass())</mass>
        <inertia>
          <ixx>@(tether_top_link_mm.ixx())</ixx>
          <iyy>@(tether_top_link_mm.iyy())</iyy>
          <izz>@(tether_top_link_mm.izz())</izz>
        </inertia>
      </inertial>
      <visual name="visual_chain_@(link_index)">
        <geometry>
          <cylinder>
            <radius>@(mooring_radius)</radius>
            <length>@(chain_link_length)</length>
          </cylinder>
          <!--sphere>
            <radius>@(mooring_radius)</radius>
          </sphere-->
        </geometry>
        <material>
          <ambient>0.1 0.1 1 1</ambient>
          <diffuse>0.1 0.1 1 1</diffuse>
          <specular>0.1 0.1 1 1</specular>
        </material>
      </visual>
      <collision name="collision_chain_@(link_index)">
        <geometry>
          <cylinder>
            <radius>@(mooring_radius)</radius>
            <length>@(chain_link_length)</length>
          </cylinder>
          <!--sphere>
            <radius>@(mooring_radius)</radius>
          </sphere-->
        </geometry>
        <surface>
          <friction>
            <ode>
              <!-- TODO(mabelzhang) Tune these for seafloor -->
              <mu>10</mu>
              <mu2>10</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>

    <joint name="chain_joint_@(link_index)" type="ball">
      <!-- Place the joint at the end of a link, not the middle, to allow chain
           to be in a continuous curve when bent. -->
      <pose>0 0 -@(chain_link_length * 0.5) 0 0 0</pose>
@[if link_index == 0]@
      <parent>MooringBottom</parent>
@[else]@
      <parent>chain_@(link_index-1)</parent>
@[end if]@
      <child>chain_@(link_index)</child>
    </joint>
@[end for]@
    <!-- end chain -->

    <link name="Anchor">
      <pose relative_to="chain_@(num_chain_links-1)" degrees="true">0 0 @(0.5 * (chain_link_length + anchor_height)) 0 0 0</pose>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <!-- TODO(mabelzhang) Get real values -->
        <mass>@(anchor_mass)</mass>
        <inertia>
          <ixx>10</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>10</iyy>
          <iyz>0</iyz>
          <izz>19.9</izz>
        </inertia>
      </inertial>
      <visual name="visual_Anchor">
        <!-- TODO(mabelzhang) Get actual geometry -->
        <geometry>
          <cylinder>
            <radius>@(anchor_radius)</radius>
            <length>@(anchor_height)</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.1 0.1 0.1 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
      <collision name="collision_Anchor">
        <!-- TODO(mabelzhang) Get actual geometry -->
        <geometry>
          <cylinder>
            <radius>@(anchor_radius)</radius>
            <length>@(anchor_height)</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <joint name="ChainToAnchor" type="ball">
      <pose>0 0 -@(0.5* anchor_height) 0 0 0</pose>
      <parent>chain_@(num_chain_links-1)</parent>
      <child>Anchor</child>
    </joint>

  </model>
</sdf>
