#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <inttypes.h>

int8_t MIDVAL = 0;
int8_t RANGE = 127;
ros::Publisher motor_cmd_pub;
int8_t cmd[8] = {MIDVAL,MIDVAL,MIDVAL,MIDVAL,MIDVAL,MIDVAL,MIDVAL,MIDVAL};
uint8_t limiters[4];

void press(const sensor_msgs::Joy::ConstPtr& msg)
{
	cmd[0] = MIDVAL;

	if (msg->buttons[4] != 0){
		if (limiters[1] == 0)
			cmd[1] = MIDVAL + RANGE;
		else
			cmd[1] = MIDVAL;
	}
	else if (msg->axes[2] != 1.0)
	{
		if (limiters[0] == 0)
			cmd[1] = MIDVAL - RANGE;
		else
			cmd[1] = MIDVAL;
	}
	else
		cmd[1] = MIDVAL;

	cmd[2] = MIDVAL;
	if (msg->buttons[5] != 0)
		cmd[3] = MIDVAL + RANGE;
	else if (msg->axes[5] != 1.0)
		cmd[3] = MIDVAL - RANGE;
	else
		cmd[3] = MIDVAL;
	if (msg->buttons[2] != 0)
		cmd[4] = MIDVAL - RANGE;
	else if (msg->buttons[3] != 0)
		cmd[4] = MIDVAL + RANGE;
	else
		cmd[4] = MIDVAL;
	cmd[5] = MIDVAL;

	if (msg->axes[7] == 1.0)
	{
		if (limiters[2] == 0)
			cmd[6] = MIDVAL - RANGE;
		else
			cmd[6] = MIDVAL;
	}
	else if (msg->axes[7] == -1.0)
	{
		if (limiters[3] == 0)
			cmd[6] = MIDVAL + RANGE;
		else
			cmd[6] = MIDVAL;
	}
	else
		cmd[6] = MIDVAL;
	cmd[7] = MIDVAL;
	std_msgs::Int8MultiArray cmd2;
	std::vector<int8_t> v(cmd, cmd + sizeof cmd / sizeof cmd[0]);
	cmd2.data = v;
	motor_cmd_pub.publish(cmd2);
}
void drive(const geometry_msgs::Twist& msg)
{
	double left = -(msg.linear.x - msg.angular.z);
	double right = (msg.linear.x + msg.angular.z);
	cmd[0] = right*RANGE + MIDVAL;
	cmd[1] = MIDVAL;
	cmd[2] = left*RANGE + MIDVAL;;
	cmd[3] = MIDVAL;
	cmd[4] = MIDVAL;
	cmd[5] = right*RANGE + MIDVAL;
	cmd[6] = MIDVAL;
	cmd[7] = left*RANGE + MIDVAL;
	std_msgs::Int8MultiArray cmd2;
	std::vector<int8_t> v(cmd, cmd + sizeof cmd / sizeof cmd[0]);
	cmd2.data = v;
	motor_cmd_pub.publish(cmd2);
}

void limit(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
	for (int i = 0; i < 4; ++ i)
		limiters[i] = msg->data[i];
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"robot_driver");

	ros::NodeHandle n;

	ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1000, drive);
	ros::Subscriber joy_sub = n.subscribe("joy",1000,press);
	motor_cmd_pub = n.advertise<std_msgs::Int8MultiArray>("raw_motor", 50);

	ros::spin();

	return 0;
} 
