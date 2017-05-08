#!/usr/bin/env python
from __future__ import division
import time
import Adafruit_PCA9685
import rospy
from geometry_msgs.msg import Twist


pwm = Adafruit_PCA9685.PCA9685()

#change the frequency here
freq = 100
maxLoad = 2
minLoad = 1
#function to convert length to frequency
def lengthToFreq(length,freq):
	#return abs(length*freq*4096/1000) #for testing with leds
	return length*freq*4096/1000 #for actual applications

def callback(data):
    twist = Twist()
    twist.linear.x = data.linear.x
    twist.linear.y = data.linear.y
    twist.angular.x = data.angular.x
    twist.angular.y = data.angular.y
    pub.publish(twist) #publishing recieved data for sanity

    if twist.linear.x:
        linear = maxLoad
    elif twist.linear.y:
        linear = minLoad
    if twist.angular.x:
        angle = maxLoad
    elif twist.angular.y:
        angle = minLoad


	#Motor List:
	linearShovel = 4
	angularShovel = 5
    ######the below motors should not be controlled here.#########
	# frontLeft = 0
	# frontRight = 1
	# backLeft = 2
	# backRight = 3


    pwm.set_pwm(linearShovel, 0, int(lengthToFreq(linear,freq)))
	pwm.set_pwm(angularShovel, 0, int(lengthToFreq(angle,freq)))

def start():
    global pub
    pub = rospy.Publisher('/robot/recieved_tool_cmd',Twist,queue_size=10)
    rospy.Subscriber('/robot/tool_control',Twist,callback)
    rospy.init_node('tool_control')
    print "spinning"
    rospy.spin()

if __name__ == "__main__":
        print "started"
        pwm.set_pwm_freq(freq)
        print "pwm freq set"
        start()
