# Vendbot

Playing with a few ideas with ROS.

## Setting up the app in development
1. Install ROS Indigo Desktop Full Install and setup workspace (run the first few tutorials for a full setup; and add the ~/catkin_ws/devel/setup.bash to your ~/.bashrc): http://wiki.ros.org/indigo/Installation/Ubuntu
2. Install the Arduino IDE: https://www.arduino.cc/en/Main/Software
3. Setup the UDEV rules so Ubuntu can access the Arduino and reboot: `sudo cp udev_rules/51-arduino.rules /etc/udev/rules.d/; sudo gpasswd -a `whoami` dialout; sudo reboot`
4. Install the needed ros components for Vendbot: `sudo apt-get install ros-indigo-camera-calibration ros-indigo-rtabmap-ros ros-indigo-stereo-image-proc ros-indigo-usb-cam ros-indigo-rosserial-arduino ros-indigo-rosserial-python ros-indigo-rosbridge-suite`
5. Source the bash devel setup: `~/catkin_ws/devel/setup.bash`
6. Clone the vendbot repo into the catkin workspace source directory: `cd ~/catkin_ws/src; git clone git@github.com:toddsampson/vendbot.git`
7. Do a catkin build on Vendbot: `cd ~/catkin_ws; catkin_make`
8. Source the bash devel setup one more time: `~/catkin_ws/devel/setup.bash`
9. Run the launch file: `roslaunch vendbot mobile_base.launch`
