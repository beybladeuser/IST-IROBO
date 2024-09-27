sudo apt update
sudo apt upgrade -y
sudo apt install build-essential curl git wget -y
sudo apt install net-tools -y
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full -y
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update
sudo apt-get install ros-noetic-turtlebot3-msgs -y
#sudo apt-get install ros-noetic-turtlebot3-gazebo -y
sudo apt-get install ros-noetic-turtlebot3 -y
sudo apt-get install ros-noetic-robot-localization -y
sudo apt-get install ros-noetic-dynamixel-sdk -y
sudo apt-get install ros-noetic-teleop-twist-keyboard -y
sudo apt-get install ros-noetic-gmapping -y
sudo apt-get install ros-noetic-navigation -y

git submodule init
git submodule update
cd $(pwd)/catkin_ws
catkin_make
