#!/usr/bin/env python
from __future__ import division
import time
import Adafruit_PCA9685
import rospy
from geometry_msgs.msg import Twist


pwm = Adafruit_PCA9685.PCA9685()

#change the frequency here
freq = 100

#function to convert length to frequency
def lengthToFreq(length,freq):
        return abs(length*freq*4096/1000) #for testing with leds
        #return length*freq*4096/1000 #for actual applications
def cmdToLength(cmdIn):
        return abs(cmdIn) #for testing with leds
        #return cmdIn/2+1.5 #for actual applications

def callback(data):
    twist = Twist()
    twist.linear.x = data.linear.x
    twist.angular.z = data.angular.z
    pub.publish(twist) #publishing recieved data for sanity
    xDir = 0
    thetaDir = 0
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
    if twist.linear.x == 0 and twist.angular.z == 0:
        thetaDir=0
        xDir=0
        print "Stopping"

#Motor List:
    frontLeft = 0
    frontRight = 1
    backLeft = 2
    backRight = 3
	######the below motors should not be controlled here.#########
	#linearShovel = 4
	#angularShovel = 5

	#if we want to go forward, left-CCW, right-CW
	#turning left, left motors rotate slower/reverse
	#turning right, right motors rotate slower/reverse

	#lets do some math to get pretty driving.  We don't want
	#motors to suddenly reverse when we want to turn.  They
	#should probably just go a little slower.

    if xDir == 0:
        leftLength = cmdToLength(thetaDir)
        rightLength = -leftLength
    if thetaDir == 0:
        leftLength = cmdToLength(xDir)
        rightLength = leftLength
    if thetaDir > 0 and abs(xDir) > 0:
	#left
        rightLength = cmdToLength(xDir)
        leftLength = cmdToLength(rightLength*(1-thetaDir))
    elif thetaDir < 0 and abs(xDir) > 0:
        #right
        leftLength = cmdToLength(xDir)
        rightLength = cmdToLength(leftLength*(1-thetaDir))

    pwm.set_pwm(frontLeft, 0, int(lengthToFreq(leftLength,freq)))
    pwm.set_pwm(frontRight, 0, int(lengthToFreq(rightLength,freq)))
    pwm.set_pwm(backLeft, 0, int(lengthToFreq(leftLength,freq)))
    pwm.set_pwm(backRight, 0, int(lengthToFreq(rightLength,freq)))

def start():
    global pub
    pub = rospy.Publisher('/robot/recieved_cmd',Twist,queue_size=10)
    rospy.Subscriber('/robot/cmd_vel',Twist,callback)
    rospy.init_node('motor_control')
    print "spinning"
    rospy.spin()
# print('Lighting LED on channel 0, press Ctrl-C to quit...')
# while True:
#     n = input("Enter a percentage: ")
#     if n < 0 or n > 100:
# 	print("You're fucking dumb")
#     else:
# 	n = int(n)
# 	xx = n/100
# 	print("Setting load to: ",xx,"%")
#         pwm.set_pwm(3, 0, int(lengthToFreq(xx,freq)))
# time.sleep(1)

#pwm.set_pwm_freq(freq) #set pwm frequency to 100 hz

if __name__ == "__main__":
    print "started"
    pwm.set_pwm_freq(freq)
    print "pwm freq set"
    start()
