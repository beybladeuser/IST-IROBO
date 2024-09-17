source ./turtlebot3_launch_real_setup.sh
ssh user@$TURTLEBOT3_IP
roscore
#in another terminal
ssh user@$TURTLEBOT3_IP
sudo apt-get install ntpdate #optional
sudo ntpdate ntp.ubuntu.com #optional
roslaunch turtlebot3_bringup turtlebot3_robot.launch

sudo shutdown now