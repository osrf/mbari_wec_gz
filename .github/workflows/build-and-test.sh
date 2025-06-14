#!/bin/bash
set -ev

export COLCON_WS=~/ws
export COLCON_WS_SRC=${COLCON_WS}/src
export DEBIAN_FRONTEND=noninteractive
export ROS_PYTHON_VERSION=3

mkdir -p $COLCON_WS_SRC

apt update -qq
apt install -qq -y lsb-release wget curl build-essential

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo apt install /tmp/ros2-apt-source.deb
# curl -s --compressed "https://hamilton8415.github.io/ppa/KEY.gpg" | gpg --dearmor | tee /etc/apt/trusted.gpg.d/ppa.gpg >/dev/null
# curl -s --compressed -o /etc/apt/sources.list.d/my_list_file.list "https://hamilton8415.github.io/ppa/my_list_file.list"
apt update -qq
apt install -y git \
               ros-jazzy-ros-gz \
               python3-colcon-common-extensions \
               python3-rosdep \
               python3-vcstool \
               wget  # \
#               libfshydrodynamics

cd $COLCON_WS_SRC
cp -r $GITHUB_WORKSPACE $COLCON_WS_SRC
wget https://raw.githubusercontent.com/osrf/mbari_wec/andermi/jazzy_harmonic/mbari_wec_all.yaml
vcs import --skip-existing < mbari_wec_all.yaml

rosdep init
rosdep update
rosdep install --from-paths ./ -i -y -r --rosdistro $ROS_DISTRO

git clone -b v1.4.0 --single-branch https://github.com/andermi/FreeSurfaceHydrodynamics.git
cd FreeSurfaceHydrodynamics
touch COLCON_IGNORE
mkdir build
cd build
cmake ..
make
make install
cd $COLCON_WS_SRC

# For rosbag2 test artifacts
apt install -y ros-$ROS_DISTRO-ros2cli ros-$ROS_DISTRO-rosbag2 ros-$ROS_DISTRO-rosbag2-transport

# for cyclonedds rmw implementation
apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp

# Build everything up to buoy_gazebo
source /opt/ros/$ROS_DISTRO/setup.bash
cd $COLCON_WS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --packages-up-to buoy_tests --event-handlers console_direct+
source $COLCON_WS/install/setup.bash

# Test all buoy packages
colcon test --packages-select-regex=buoy --packages-skip=buoy_msgs --event-handlers console_direct+ --retest-until-pass 10
colcon test-result

# Debug specific test
# launch_test $COLCON_WS/install/buoy_tests/share/buoy_tests/launch/pc_bias_damping_ros_feedback_py.launch.py
