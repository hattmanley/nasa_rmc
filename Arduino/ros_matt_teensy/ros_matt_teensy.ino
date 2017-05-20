#include "AltEncoder.h"
#include "ros.h"
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

const int pwm_zero_throttle = 1535;
const float pwm_throttle_scaler = 5.12;
bool failsafe=0;

void callback(const geometry_msgs::Twist& data) { //set targets to command, should be triggered by recieving command through serial in main loop
  rpmControl(2, data.linear.x, 0, 0);
  rpmControl(3, data.linear.y, 1, 1);
  rpmControl(4, data.linear.z, 2, 2);
  rpmControl(5, data.angular.x, 3, 3);
}

ros::Subscriber<geometry_msgs::Twist> sub("robot/motor_control_serial", &callback );

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  //nh.advertise(fuckyou);
  //define endstop pins
  pinMode(33, INPUT); //rotation bottom
  pinMode(34, INPUT); //rotation top
  pinMode(35, INPUT); //linear bottom
  pinMode(36, INPUT); //linear top

  int pwmfreq = 250;   //250hz = 4ms each
  analogWriteFrequency(2, pwmfreq);//drive1
  analogWriteFrequency(3, pwmfreq);//drive2
  analogWriteFrequency(4, pwmfreq);//drive3
  analogWriteFrequency(5, pwmfreq);//drive4
  analogWriteFrequency(6, pwmfreq);//linear
  analogWriteFrequency(7, pwmfreq);//rotation
  analogWriteResolution(12); //0 - 4095, at 250hz, 4ms is split into 4096 parts, 1~2ms is from 1023-2047, 1537 being the center
}

void loop() {
  nh.spinOnce();
  nh.subscribe(sub);
  delay(1);
} //end main loop



void rpmControl(int pin, float throttle, int enc, int rpm) { //throttle is from -100 to 100
  throttle = pwm_zero_throttle + constrain(throttle, -100, 100) * pwm_throttle_scaler; //convert to analogwrite value
  if (failsafe == 1) {
    analogWrite(pin, pwm_zero_throttle);//dont send it
  }
  else {
    analogWrite(pin, throttle);
  }
}
