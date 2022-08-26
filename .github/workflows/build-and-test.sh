#!/bin/bash
set -ev

export COLCON_WS=~/ws
export COLCON_WS_SRC=${COLCON_WS}/src
export DEBIAN_FRONTEND=noninteractive
export ROS_PYTHON_VERSION=3

mkdir -p $COLCON_WS_SRC

apt update -qq
apt install -qq -y lsb-release wget curl build-essential

echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt-get update -qq
apt-get install -y git \
                   python3-colcon-common-extensions \
                   python3-rosdep \
                   python3-vcstool \
                   wget

cd $COLCON_WS_SRC
cp -r $GITHUB_WORKSPACE $COLCON_WS_SRC
wget https://raw.githubusercontent.com/osrf/buoy_entrypoint/main/buoy_all.yaml
vcs import --skip-existing < buoy_all.yaml

rosdep init
rosdep update
rosdep install --from-paths ./ -i -y -r --rosdistro $ROS_DISTRO

# Build everything up to buoy_gazebo
source /opt/ros/$ROS_DISTRO/setup.bash
cd $COLCON_WS
colcon build --packages-up-to buoy_tests --event-handlers console_direct+
source $COLCON_WS/install/setup.bash

# Test all buoy packages
colcon test --packages-select-regex=buoy --packages-skip=buoy_msgs --event-handlers console_direct+
colcon test-result
