#!/usr/bin/env python
import serial #make sure you install pyserial
import rospy
import time
import struct
import sys
from serial.tools import list_ports
from geometry_msgs.msg import Twist

class TeensySerial:
    def __init__(self, baudrate):
        self.teensy_port = self.getTeensyPort()
        self.teensy = serial.Serial(self.teensy_port[0], baudrate)

    def getTeensyPort(self):
        """Discover where is Teensy."""
        ports_avaiable = list(list_ports.comports())
        teensy_port = tuple()
        for port in ports_avaiable:
            if port[1].startswith("Teensy"):
                teensy_port = port
        if teensy_port:
            return teensy_port

    def close(self):
        if self.teensy.isOpen():
            self.teensy.close()




motorScalar=100 #change this to adjust what magnitude the serial commands are
linearScalar=100
rotateScalar=100


def callback(data):
    twist = Twist()
    twist.linear.x = data.linear.x
    twist.angular.z = data.angular.z

    twist.linear.z=data.linear.z
    twist.linear.y=data.linear.y
    twist.angular.x=data.angular.x
    twist.angular.y=data.angular.y

    pub.publish(twist) #publishing recieved data for sanity

    #initialize values to zero
    xDir = 0
    thetaDir = 0
    linearOutput=0
    rotateOutput=0

    if twist.linear.x < 0:
        #left stick pulled back from ps3TeleopSimple.py
        print "Going Backwards"
        xDir = twist.linear.x
    elif twist.linear.x > 0:
        #left stick pushed forward from ps3TeleopSimple.py
        print "Going Forwards"
        xDir = twist.linear.x
    if twist.angular.z > 0:
        #right stick left from ps3TeleopSimple.py
        print "Turning Left"
        thetaDir = twist.angular.z
    elif twist.angular.z < 0:
        #right stick right from ps3TeleopSimple.py
        thetaDir = twist.angular.z
        print "Turning Right"
    # if twist.linear.x == 0 and twist.angular.z == 0:
    #     thetaDir=0
    #     xDir=0
    #     print "Stopping"
    if twist.angular.y:
        print "rotating shovel up"
        rotateOutput = 1*rotateScalar
    elif twist.angular.x:
        print "rotating shovel down"
        rotateOutput = -1*rotateScalar
    if twist.linear.z:
        print "moving linear up"
        linearOutput=1*linearScalar
    elif twist.linear.y:
        print "moving linear down"
        linearOutput = -1*linearScalar
#Motor List:
    # frontLeft = 0
    # frontRight = 1
    # backLeft = 2
    # backRight = 3
	# linearShovel = 4
	# angularShovel = 5

	#if we want to go forward, left-CCW, right-CW
	#turning left, left motors rotate slower/reverse
	#turning right, right motors rotate slower/reverse

	#lets do some math to get pretty driving.  We don't want
	#motors to suddenly reverse when we want to turn.  They
	#should probably just go a little slower.



    if xDir == 0:
        leftOutput = thetaDir*motorScalar
        rightOutput = -leftOutput
    if thetaDir == 0:
        leftOutput = xDir*motorScalar
        rightOutput = leftOutput
    if thetaDir > 0 and abs(xDir) > 0:
	#left
        rightOutput = xDir*motorScalar
        leftOutput = rightOutput*(1-abs(thetaDir))
    elif thetaDir < 0 and abs(xDir) > 0:
        #right
        leftOutput = xDir*motorScalar
        rightOutput = leftOutput*(1-abs(thetaDir))
    try:
        ser.write(struct.pack('<BBB',leftOutput,rightOutput,leftOutput,rightOutput,linearOutput,rotateOutput)) #sending code
    except:
        print ""
    print leftOutput,",",rightOutput,",",linearOutput,",",rotateOutput

def start():
    global pub
    pub = rospy.Publisher('/robot/recieved_cmd',Twist,queue_size=10)
    rospy.Subscriber('/robot/cmd_vel',Twist,callback)
    rospy.Subscriber('robot/tool_control',Twist,)
    rospy.init_node('motor_control')
    print "spinning"
    rospy.spin()


if __name__ == "__main__":
    print "Detecting Teensy..."
    try:
        ser = TeensySerial(115200)
        print "Connected to: %s" % teensy.teensy_port[1]
        teensy.close()
    except TypeError:
        print "Cannot find a Teensy board connected..."
    print "started"
    start()
