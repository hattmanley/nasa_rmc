# CU Boulder NASA RMC
##Steps to get the code working (in progress)
Pair the PS3 controller with the remote machine, i.e. your laptop.

Get the Raspberry Pi's ip address, you may need to connect a monitor and keyboard to do this if you don't already know it.  Run
```
ifconfig
```

ssh into the raspberry pi in a new terminal (ip will probably change)
```
ssh -X pi@192.168.0.29
```

Set the Raspberry Pi's enviroment variable ROS_IP to the ip address of the Pi

```
export ROS_IP=192.168.0.29
