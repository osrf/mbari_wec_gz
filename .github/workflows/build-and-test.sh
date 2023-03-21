#!/bin/bash
set -ev

export COLCON_WS=~/ws
export COLCON_WS_SRC=${COLCON_WS}/src
export DEBIAN_FRONTEND=noninteractive
export ROS_PYTHON_VERSION=3

mkdir -p $COLCON_WS_SRC

apt update -qq
apt install -qq -y lsb-release wget curl build-essential

# Garden only has nightlies for now
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-nightly `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-nightly.list
wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
# curl -s --compressed "https://hamilton8415.github.io/ppa/KEY.gpg" | gpg --dearmor | tee /etc/apt/trusted.gpg.d/ppa.gpg >/dev/null
# curl -s --compressed -o /etc/apt/sources.list.d/my_list_file.list "https://hamilton8415.github.io/ppa/my_list_file.list"
apt update -qq
apt install -y git \
               gz-garden \
               python3-colcon-common-extensions \
               python3-rosdep \
               python3-vcstool \
               wget  # \
#               libfshydrodynamics

cd $COLCON_WS_SRC
cp -r $GITHUB_WORKSPACE $COLCON_WS_SRC
wget https://raw.githubusercontent.com/osrf/buoy_entrypoint/main/buoy_all.yaml
vcs import --skip-existing < buoy_all.yaml

rm -rf buoy_examples

rosdep init
rosdep update
rosdep install --from-paths ./ -i -y -r --rosdistro $ROS_DISTRO

git clone https://github.com/hamilton8415/FreeSurfaceHydrodynamics.git
cd FreeSurfaceHydrodynamics
touch COLCON_IGNORE
mkdir build
cd build
cmake ..
make
make install
cd $COLCON_WS_SRC

# For rosbag2 test artifacts
apt install -y ros-humble-ros2cli ros-humble-rosbag2 ros-humble-rosbag2-transport

# for cyclonedds rmw implementation
apt install -y ros-humble-rmw-cyclonedds-cpp

# Build everything up to buoy_gazebo
source /opt/ros/$ROS_DISTRO/setup.bash
cd $COLCON_WS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --packages-up-to buoy_tests --event-handlers console_direct+
source $COLCON_WS/install/setup.bash

# Test all buoy packages
colcon test --packages-select-regex=buoy --packages-skip=buoy_msgs --event-handlers console_direct+ --retest-until-pass 10
colcon test-result
