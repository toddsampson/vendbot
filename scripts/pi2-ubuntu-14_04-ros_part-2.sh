#!/bin/bash

sudo resize2fs /dev/mmcblk0p2
sudo apt-get install dphys-swapfile
sudo apt-get install linux-firmware
sudo apt-get install openssh-server

sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get -y upgrade
sudo apt-get -y install ros-indigo-robot

sudo apt-get install python-rosdep
sudo rosdep init
rosdep update

echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get -y install python-rosinstall

sudo cp -f lsb-release /etc/lsb-release

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

cd ~/catkin_ws/
catkin_make

source devel/setup.bash


