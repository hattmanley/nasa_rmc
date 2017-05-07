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
	return length*freq*4096/1000

def callback(data):
    twist = Twist()
    twist.linear.x = data.linear.x
    twist.angular.z = data.angular.z
    pub.publish(twist) #publishing recieved data for sanity
    if twist.linear.x < 0:
        print "Going Backwards"
    elif twist.linear.x > 0:
        print "Going Forwards"
    if twist.angular.z > 0:
        print "Turning Left"
    elif twist.angular.z < 0:
        print "Turning Right"
    if twist.linear.x == 0 and twist.angular.z == 0:
        print "Stopping"

def start():
    global pub
    pub = rospy.Publisher('/robot/recieved_cmd',Twist,queue_size=10)
    rospy.Subscriber('/robot/cmd_vel',Twist,callback)
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

pwm.set_pwm_freq(freq) #set pwm frequency to 100 hz
