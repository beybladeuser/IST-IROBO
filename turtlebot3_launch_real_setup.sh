
export TURTLEBOT3_NUMBER=15
export TURTLEBOT3_NAME=waffle5 #[TurtleBot3 363636 on the stick], can be waffle2 or waffle5
export TURTLEBOT3_MODEL=waffle_pi 
#export TURTLEBOT3_IP=192.168.28.[11...15] #[TurtleBot IP] can be .12 or .15
export TURTLEBOT3_IP=192.168.28.$TURTLEBOT3_NUMBER
#export TURTLEBOT3_NUMBER=[11...15] #[Last numbers of the TurtleBot3 IP] can be 12 or 15
#export ROS_MASTER_URI=http://192.168.28.[11...15]:11311 #[TurtleBot3 IP] can be .12 or .15
export ROS_MASTER_URI=http://192.168.28.$TURTLEBOT3_NUMBER:11311
#export ROS_HOSTNAME=192.168.[27/28].XXX #[lab computer / laptop IP] make ifconfig and check wlan (just because you are using wifi in the lab)
export ROS_HOSTNAME=192.168.28.124
export ROS_IP=$ROS_HOSTNAME #[lab computer / laptop IP] make ifconfig and check wlan (just because you are using wifi in the lab)