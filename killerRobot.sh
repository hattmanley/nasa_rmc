#!/bin/bash
clear
echo -e "\e[34;1m Make sure you and the robot are connected to the same Wifi"
echo -e "and that the IP address are as expected.  ctrl+c to stop!!!\e[0m"
echo ""
secs=$((10))
while [ $secs -gt 0 ]; do

   echo -ne "\e[31;1mYou have $secs seconds to plug in the controller!!!\0\e[0m\r"
   sleep 1
   : $((secs--))
done
clear
sixpair
rossetup
echo "Running roscore on the robot."
xterm -e ssh bepis@192.168.1.2 "roscore &"
echo "Startign the joy_node"
xterm -e rosrun joy joy_node
source /home/matt/catkin_ws/devel/setup.bash &
echo "Starting ps3 controller teleop driver"
xterm -e rosrun ps3_controller ps3TeleopSimple.py
echo "Starting motor contorl on the robot."
xterm -e ssh bepis@192.168.1.2 "rosrun serial_motor_control piMotorControlSerial.py &"
echo "Starting rosserial on the robot."
xterm -e ssh bepis@192.168.1.2 "rosrun rosserial_python serial_node.py &"
echo -e "\e[31m Go fuck 'em up.\e[0m"
