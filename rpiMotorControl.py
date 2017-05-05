import RPi.GPIO as GPIO
from time import sleep
import rospy

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "Recieved data %s",data.buttons)

def motorControl():
	rospy.init_node('motorControl',anoymous=True)
	rospy.Subscriber('joy_node',buttons,callback)

	rospy.spin()

if __name__ == '__main__':
	motorControl()




# GPIO.setmode(GPIO.BOARD)

# Motor1A = 16
# Motor1B = 18
# Motor1E = 22

# GPIO.setup(Motor1A,GPIO.OUT)
# GPIO.setup(Motor1B,GPIO.OUT)
# GPIO.setup(Motor1E,GPIO.OUT)

# print "Turning motor on"
# GPIO.output(Motor1A,GPIO.HIGH)
# GPIO.output(Motor1B,GPIO.LOW)
# GPIO.output(Motor1E,GPIO.HIGH)

# sleep(2)

# print "Stopping motor"
# GPIO.output(Motor1E,GPIO.LOW)

# GPIO.cleanup()
