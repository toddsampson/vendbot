#!/bin/bash

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu jessie main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install python-pip python-setuptools python-yaml python-distribute python-docutils python-dateutil python-six
sudo pip install rosdep rosinstall_generator wstool rosinstall
sudo rosdep init
rosdep update
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws
rosinstall_generator robot --rosdistro indigo --deps --wet-only --tar > indigo-robot-wet.rosinstall
wstool init src indigo-robot-wet.rosinstall

mkdir ~/ros_catkin_ws/external_src
sudo apt-get install checkinstall cmake
sudo sh -c 'echo "deb-src http://mirrordirector.raspbian.org/raspbian/ testing main contrib non-free rpi" >> /etc/apt/sources.list'
sudo apt-get update

cd ~/ros_catkin_ws/external_src
sudo apt-get build-dep console-bridge
apt-get source -b console-bridge
sudo dpkg -i libconsole-bridge0.2*.deb libconsole-bridge-dev_*.deb

cd ~/ros_catkin_ws/external_src
apt-get source -b lz4
sudo dpkg -i liblz4-*.deb

cd ~/ros_catkin_ws/external_src
sudo apt-get install libboost-filesystem-dev libxml2-dev
wget http://downloads.sourceforge.net/project/collada-dom/Collada%20DOM/Collada%20DOM%202.4/collada-dom-2.4.0.tgz
tar -xzf collada-dom-2.4.0.tgz
cd collada-dom-2.4.0
cmake .
sudo checkinstall make install

cd ~/ros_catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:jessie

sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/indigo

"source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

