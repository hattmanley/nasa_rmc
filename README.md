# CU Boulder NASA RMC
## Steps to get the code working (in progress)
Pair the PS3 controller with the remote machine, i.e. your laptop.
Assuming you have all the proper librarires installed...
```
sixpair
```

Get the Raspberry Pi's ip address, you may need to connect a monitor and keyboard to do this if you don't already know it.  Run ifconfig and look for the inet address.  I will use 192.168.0.29 as a placeholder, replace with what you find in this step.
```
ifconfig
```

ssh into the raspberry pi in a new terminal (ip will probably change)
```
ssh -X pi@192.168.0.29
```

Set the Raspberry Pi's enviroment variable ROS_IP to the ip address of the Pi. This needs to be done in each new terminal you open.  Consider adding this line to your .bashrc on your pi

```
export ROS_IP=192.168.0.29
````

Set the Raspberry Pi's enviroment varialbe ROS_MASTER_URI as follows.  This needs to be done in each new terminal you open.  Consider adding this line to your .bashrc on your pi.
```
export ROS_MASTER_URI=http://192.168.0.29:11311/
```

Do the same on the remote machine, also with the Pi's ip address.  This needs to be done in each new terminal you open.  Consider adding this line to your .bashrc on your remote machine
```
export ROS_MASTER_URI=http://192.168.0.29:11311/
```

Set the remote machine's ROS_IP to the ip address of the remote machine.  Use ifconfig to find inet address again.  You will need to do this for each new terminal you open.  Consider adding the second line to your .bashrc on your remote machine
```
ifconfig
export ROS_IP=192.168.0.13
```

Now you should be able to start the ROS_MASTER.  In the Pi ssh terminal, start
the ROS_MASTER
```
roscore
```

Open up a new terminal on your remote machine.  Assuming your catkin_ws is in your home directory, run the following command.  Again, you will have to do this a lot so consider adding it to .bashrc or creating an alias.
Note that you will need to open new terminals for all of these.
```
source ~/catkin_ws/devel/setup.bashrc
```

Now start your first node, the PS3 Teleop Node.
```
rosrun joy joy_node
```

Now start the node to turn controller commands into velocity/tool commands
```
rosrun ps3_controller ps3TeleopSimple.py
rosrun ps3_controller ps3TeleopTool.py
```

Now start the motor control node on the pi.  Make sure you run this from a pi ssh terminal.
```
rosrun motor_control piMotorControl.py
```

And in a new pi ssh terminal

```
rosrun tool_control piToolControl.py
```

You might be good now.  Probably not.



## visp_auto_tracker notes
Change values in /opt/ros/kinetic/share/visp_auto_tracker/models/pattern.cfg for the specific QR Code.

# detector-type

The following detectors are supported

detector-type= zbar: uses libzbar to detect QRcodes
detector-type= dmtx: uses libdmtx to detect flashcodes

# flashcode-coordinates

3D-coordinates in meters of the box delimiting the pattern (QRcode or flashcode).

# inner-coordinates

3D-coordinates in meters of the white box containing the pattern.

#outer-coordinates

3D-coordinates in meters of the black box containing the pattern.

# ad-hoc-recovery

When set (tracker-type= 1) this parameter activates the tracking lost detection and recovery using flashcode-coordinates, inner-coordinates and outer-coordinates point coordinates.

## Best of luck...

