#include "AltEncoder.h"
using namespace AltEncoder;

#include <ros.h>


#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>

#include <std_msgs/Float32.h>


ros::NodeHandle  nh;          //create a node handle
std_msgs::Float32 message;
ros::Publisher teensypub("teensypub", &message);

IntervalTimer sendrostimer;






//IntervalTimer rpmMeasure;

Encoder* encoderList[] =
{ //            A  B
  new Encoder( 8, 9),
  new Encoder( 10, 11),
  new Encoder( 24, 25),
  new Encoder( 26, 27),
  new Encoder(28, 29),
  new Encoder(30, 31),
  nullptr
};

const int pwm_zero_throttle = 1535;
const float pwm_throttle_scaler = 5.12;     //used for converting throttle to usable 250hz pwm




void setup()
{
  nh.initNode();

  nh.advertise(teensypub);
  
  sendrostimer.begin(messagesend, 1000000); //do helloros every 1,000,000 micro seconds


  Controller::begin(encoderList, 25/*µs*/);   // choose a sampling period which is at least a factor of two smaller than the shortest time between two encoder signal edges.


  //int pwmfreq = 250;   //250hz = 4ms each
  analogWriteFrequency(2, 250);
  analogWriteFrequency(3, 250);
  analogWriteFrequency(4, 250);
  analogWriteFrequency(5, 250);
  analogWriteFrequency(6, 250);
  analogWriteFrequency(7, 250);
  analogWriteResolution(12); //0 - 4095, at 250hz, 4ms is split into 4096 parts, 1~2ms is from 1023-2047, 1537 being the center






}



void loop() {

  escA(2, 0);

} //end main loop



void messagesend(void) {
  //int vect1[] = {1, 2, 3, 4, 5, 6};

  message.data=1;
  
  teensypub.publish( &message );
  nh.spinOnce();
}




//call this function to set throttle on a drive motor esc
void escA(int pin, float throttle) { //throttle is from -100 to 100
  throttle = pwm_zero_throttle + constrain(throttle, -100, 100) * pwm_throttle_scaler; //convert to analogwrite value
  analogWrite(pin, throttle);//send it
}

//call this function to set throttle on positional motor esc
void escB(int pin, float throttle, bool esup, bool esdown) { //throttle is from -100 to 100, input endstops 
  if (esup) {
    throttle = pwm_zero_throttle + constrain(throttle, -100, 0) * pwm_throttle_scaler; //if at top, only move down
  }
  else if (esdown) {
    throttle = pwm_zero_throttle + constrain(throttle, 0, 100) * pwm_throttle_scaler; //if at bottom, only move up
  }
  else {
    throttle = pwm_zero_throttle + constrain(throttle, -100, 100) * pwm_throttle_scaler; //if inbetween its gucci
  }
  analogWrite(pin, throttle);  //send it
}

