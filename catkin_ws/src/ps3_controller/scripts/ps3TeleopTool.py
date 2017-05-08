#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

#button assignments
BUTTON_SELECT           =0
BUTTON_STICK_LEFT       =1
BUTTON_STICK_RIGHT      =2
BUTTON_START            =3
BUTTON_CROSS_UP         =4
BUTTON_CROSS_RIGHT      =5
BUTTON_CROSS_DOWN       =6
BUTTON_CROSS_LEFT       =7
BUTTON_REAR_LEFT_2      =8
BUTTON_REAR_RIGHT_2     =9
BUTTON_REAR_LEFT_1      =10
BUTTON_REAR_RIGHT_1     =11
BUTTON_ACTION_TRIANGLE  =12
BUTTON_ACTION_CIRCLE    =13
BUTTON_ACTION_CROSS     =14
BUTTON_ACTION_SQUARE    =15
BUTTON_PAIRING          =16

AXIS_STICK_LEFT_LEFTWARDS   =0
AXIS_STICK_LEFT_UPWARDS     =1
AXIS_STICK_RIGHT_LEFTWARDS  =2
AXIS_STICK_RIGHT_UPWARDS    =3
AXIS_BUTTON_CROSS_UP        =4
AXIS_BUTTON_CROSS_RIGHT     =5
AXIS_BUTTON_CROSS_DOWN      =6
AXIS_BUTTON_CROSS_LEFT      =7
AXIS_BUTTON_REAR_LEFT_2     =8
AXIS_BUTTON_REAR_RIGHT_2    =9
AXIS_BUTTON_REAR_LEFT_1     =10
AXIS_BUTTON_REAR_RIGHT_1    =11
AXIS_BUTTON_ACTION_TRIANGLE =12
AXIS_BUTTON_ACTION_CIRCLE   =13
AXIS_BUTTON_ACTION_CROSS    =14
AXIS_BUTTON_ACTION_SQUARE   =15
AXIS_ACCELEROMETER_LEFT     =16
AXIS_ACCELEROMETER_FORWARD  =17
AXIS_ACCELEROMETER_UP       =18
AXIS_GYRO_YAW               =19


def callback(data):
    #because I'm lazy, we're goign to use geometry messages for tool stuff
    twist = Twist()
    #linear up
    twist.linear.x = data.buttons[BUTTON_REAR_LEFT_1]
    #linear down
    twist.linear.y = data.buttons[BUTTON_REAR_LEFT_2]
    #rotate dump
    twist.angular.x = data.buttons[BUTTON_REAR_RIGHT_2]
    #rotate scoop
    twist.angular.y=data.buttons[BUTTON_REAR_RIGHT_1]
    pub.publish(twist)

def start():
    global pub
    pub = rospy.Publisher('robot/tool_control',Twist,queue_size=10)
    rospy.Subscriber("joy",Joy,callback)
    rospy.init_node('Joy2RobotTool')
    print "spinning"
    rospy.spin()

if __name__ == '__main__':
    start()
