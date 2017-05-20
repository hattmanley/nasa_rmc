#include "AltEncoder.h"
#include "ros.h"
#include <geometry_msgs/Twist.h>
using namespace AltEncoder;
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

ros::NodeHandle nh;

float inputcommandarray[] = {0, 0, 0, 0, 0, 0};
IntervalTimer fakecommandtimer; //fake read in

long lastcommand = 0; //used as a check for failsafe
bool failsafe = 1; //start with failsafe activatd

float targets[] = {0, 0, 0, 0, 0, 0};
IntervalTimer pidtimer;

long calibrations[] = {0, 0};
long positions[] = {0, 0};
long ranges[] = {80000, 8000}; //NEED TO DETERMINE THE RANGES OF MOTION, Also need to determine motor direction (one gaurnteed wrong)





IntervalTimer rpmtimer;
int lastcount[] = {0, 0, 0, 0, 0, 0};
float rpms[] = {0, 0, 0, 0, 0, 0};
int rpmcount[5][6] = {
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0}
};

const int pwm_zero_throttle = 1535;
const float pwm_throttle_scaler = 5.12;

void takecommand(const geometry_msgs::Twist& robotShit) { //set targets to command, should be triggered by recieving command through serial in main loop
  //Serial.write("working");
//  targets[0]=robotShit.linear.x;
//  targets[1]=robotShit.linear.y;
//  targets[2]=robotShit.linear.z;
//  targets[3]=robotShit.angular.x;
//  targets[4]=robotShit.angular.y;
//  targets[5]=robotShit.angular.z;
  targets[0]=30;
  targets[1]=30;
  targets[2]=30;
  targets[3]=robotShit.angular.x;
  targets[4]=robotShit.angular.y;
  targets[5]=robotShit.angular.z;
  lastcommand = millis();
}

ros::Subscriber<geometry_msgs::Twist> sub("robot/motor_control_serial", takecommand );
geometry_msgs::Twist twisty;
ros::Publisher fuckyou("robot/serial_out",&twisty);

void setup()
{
  //Serial.begin(9600);
  //Serial.write("started");
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(fuckyou);
  //define endstop pins
  pinMode(33, INPUT); //rotation bottom
  pinMode(34, INPUT); //rotation top
  pinMode(35, INPUT); //linear bottom
  pinMode(36, INPUT); //linear top


  //fakecommandtimer.begin(takecommand, 10000000); //take the fake command every 10 seconds

  rpmtimer.begin(rpmmeasure, 4000); //measure rpm 250 times a second

  pidtimer.begin(controlloop, 10000); //go through control loop 100 times second


  Controller::begin(encoderList, 30/*µs*/);  //33khz // choose a sampling period which is at least a factor of two smaller than the shortest time between two encoder signal edges.

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

//Serial.write("loop");
targets[0]=0;
nh.spinOnce();
delay(1);

} //end main loop




void rpmmeasure(void) { //average last 5 updates 250 times a second

  for (int i = 4; i > 0; i--) { //count down from 5th position, this lets array data be shifted down one row, leaves top row unchanged
    for (int j = 0; j < 6; j++) { //go through all 6 encoder counts
      rpmcount[i][j] = rpmcount[i - 1][j]; //shift the data
    }
  }

  for (int f = 0; f < 6; f++) { //replace first row with new data, set lastcount to the current count
    rpmcount[0][f] = ((encoderList[f]->counter) - lastcount[f]) * 3.6; //rough change into actual rpm
    lastcount[f] = (encoderList[f]->counter);
  }

  for (int k = 0; k < 6; k++) {
    rpms[k] = (rpmcount[0][k] + rpmcount[1][k] + rpmcount[2][k] + rpmcount[3][k] + rpmcount[4][k]) / 5.00;
    // encoderList[k]->counter;
  }
}








void controlloop (void) {

  long checktime = millis() - lastcommand;//check if message has been recieved in last 0.5 seconds to activate failsafe
  if (checktime < 250) {
    failsafe = 0;
  }
  else {
    failsafe = 1;
  }

  rpmControl(2, targets[0], 0, 0);
  rpmControl(3, targets[1], 1, 1);
  rpmControl(4, targets[2], 2, 2);
  rpmControl(5, targets[3], 3, 3);


  positionControl(6, targets[4], 35, 36, 4, 0);//linear
  positionControl(7, targets[5], 33, 34, 5, 1);//shovel

}



void rpmControl(int pin, float rpmtarget, int enc, int rpm) { //throttle is from -100 to 100

  float throttle = rpmtarget;

  escA(pin, throttle);
}



void positionControl(int pin, int postarget, int esdown, int esup, int enc, int cal) {


  bool up = digitalRead(esup);
  bool down = digitalRead(esdown);

  if (up == true) {
    calibrations[cal] = (encoderList[enc]->counter);   //if at top of range (hit top stop) set calibration to current encoder count, this means position will be zero at the top.
  }
  if (down == true) {
    calibrations[cal] = (encoderList[enc]->counter) - ranges[cal]; //if at bottom of range (hit bot stop) set calibration to current encoder count - range. this means the position is set to the range downward with the top being zero
  }

  positions[cal] = (encoderList[enc]->counter) - calibrations[cal];

  int dft1 = postarget - positions[cal];
  float  throttle =  - dft1 / 7.0;

  escB(pin, throttle, up, down);
}


//call this function to set throttle on a drive motor esc
void escA(int pin, float throttle) { //throttle is from -100 to 100
  throttle = pwm_zero_throttle + constrain(throttle, -100, 100) * pwm_throttle_scaler; //convert to analogwrite value

  if (failsafe == 0) {
    analogWrite(pin, throttle);//send it
  }
  else {
   // analogWrite(pin, pwm_zero_throttle);
      analogWrite(pin, throttle);
  }
}





//call this function to set throttle on positional motor esc
void escB(int pin, float throttle, bool esup, bool esdown) { //throttle is from -100 to 100, input endstops and whether the motor is reversed

  esup = true;
  esdown = false;

  if (esup == true)
  {
    throttle = constrain(throttle, -100, 0); //if at top, only move down
  }
  if (esdown == true)
  {
    throttle = constrain(throttle, 0, 100); //if at bottom, only move up
  }



  if (failsafe == 0) {
    throttle = pwm_zero_throttle + constrain(throttle, -100, 100) * pwm_throttle_scaler;
    analogWrite(pin, throttle);  //send it
  } else {
    analogWrite(pin, pwm_zero_throttle);  //send it
  }

}

