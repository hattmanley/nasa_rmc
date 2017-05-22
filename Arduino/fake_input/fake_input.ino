#include "AltEncoder.h"
using namespace AltEncoder;

//================================INIT ENCODER OBJECT===========================
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


//================================GLOBALS=======================================
//MIGHT WANT TO CHANGE:
float motorMax=50;  //change to adjust maximum drivetrain output!!!!!!!!!!!
int timeout = 1000;
long ranges[] = {10000, 500}; //NEED TO DETERMINE THE RANGES OF MOTION

//Probably don't need to change:
ros::NodeHandle nh;
const int pwm_zero_throttle = 1535;
const float pwm_throttle_scaler = 5.12;
unsigned long lastcommand = 0; //used as a check for failsafe--CLT
volatile bool failsafe = 1; //start with failsafe activated...
volatile float targets[] = {0, 0, 0, 0, 0 ,0};
volatile long calibrations[] = {0, 0};
//use volatile for shared variables!
long positions[] = {0, 0};


//=====MOTORS=============
int frontLeft = 0;
int frontRight = 1;
int backLeft = 2;
int backRight = 3;
int linear = 4;
int rotation = 5;


//=====MOTOR PINS=========
int frontLeftPin = 2;
int frontRightPin = 3;
int backLeftPin = 4;
int backRightPin = 5;
int linearPin = 6;
int rotationPin = 7;


//=====ENDSTOP PINS=======
int rBottom = 33;
int rTop = 34;
int lBottom = 35;
int lTop = 36;

//================================DEFINE INTERVAL TIMERS========================
IntervalTimer pidtimer; //used w/ controlloop

//================================ROS===========================================
void callbackVel(const geometry_msgs::Twist& data) {
  noInterrupts();
  targets[0]=data.linear.x;
  targets[1]=data.linear.y;
  targets[2]=data.linear.z;
  targets[3]=data.angular.x;
  interrupts();
  //adding interrupt blockers so that we don't try to read variables whilst
  //writing them.  This can lead to some nasty-ass shit.
  lastcommand = millis(); //remember millis returns an unsigned long.
  //millis checks how long the program has been running.  Pretty dope, eh?
}

ros::Subscriber<geometry_msgs::Twist> cmd_sub("robot/motor_control_serial", &callbackVel );
//this makes the teensy subscribe to the topic robot/motor_control_serial and sends
//the twist message into the function callbackVel.  piMotorControlSerial.py is
//the program that sends the message.

void setup(){
  //================================SERIAL======================================
  //Serial.begin(9600);

  //================================ROS=========================================
  nh.initNode(); //initialize the node
  nh.subscribe(cmd_sub); //declare cmd_sub as a subscriber

  //================================ENDSTOP PINS================================
  //declare that the endstop pins are inputting data to the teensy.
  pinMode(rBottom, INPUT); //rotation bottom
  pinMode(rTop, INPUT); //rotation top
  pinMode(lBottom, INPUT); //linear bottom
  pinMode(lTop, INPUT); //linear top


  //================================STARTING INTERVAL TIMERS====================
  //this runs the control loop a specified number of times per second.  Be aware
  //that it interupts the running processes and can change variables.  This could
  //cause some weird memory issues, so it could be a location of bugs.
  pidtimer.begin(controlloop, 10000); //go through control loop 100 times second



  //================================ENCODER SAMPLING============================
  Controller::begin(encoderList, 30/*µs*/);  //33khz
  // choose a sampling period which is at least a factor of two smaller than the
  //shortest time between two encoder signal edges.

  //================================SETTING UP MOTORS===========================
  int pwmfreq = 250;   //250hz = 4ms each
  analogWriteFrequency(frontLeftPin, pwmfreq);//drive1
  analogWriteFrequency(frontRightPin, pwmfreq);//drive2
  analogWriteFrequency(backLeftPin, pwmfreq);//drive3
  analogWriteFrequency(backRightPin, pwmfreq);//drive4
  analogWriteFrequency(linearPin, pwmfreq);//linear
  analogWriteFrequency(rotationPin, pwmfreq);//rotation
  analogWriteResolution(12);
  //0 - 4095, at 250hz, 4ms is split into 4096 parts, 1~2ms is from 1023-2047,
  //1537 being the center
}

//==================================MAIN========================================
void loop() {
  //================================DEBUG PRINTS================================
  //Serial.println(positions[1]);
  //Serial.println(lastcommand);
  //Serial.println(millis() - lastcommand);
  //Serial.println("failsafe:");
  // Serial.println(failsafe);

  //================================ROS=========================================
  nh.spinOnce();
  nh.subscribe(cmd_sub);

  delay(1);
}
//==============================END MAIN========================================

void controlloop (void) {
  unsigned long checktime = millis() - lastcommand;
  //millis() returns how long the prgram has been running
  //lastcommand is the last time that a command was recieved, relative to the start
  //of the prgram.
  //millis returns unsigned long
  //in the very off chance that we have a strange timing error, we will get overflow
  //and this will result in some strange shit.
  //check if message has been recieved in last xx seconds to activate failsafe
  if (checktime < timeout) {
    failsafe = 0;
  }
  else {
    failsafe = 1;
  }

  // HERE FOR REFERENCE. DEFINED ABOVE.
  // int frontLeft = 0;
  // int frontRight = 1;
  // int backLeft = 2;
  // int backRight = 3;
  // int linear = 4;
  // int rotation = 5;
  motorWrite(frontLeftPin, targets[frontLeft]);
  motorWrite(frontRightPin, targets[frontRight]);
  motorWrite(backLeftPin, targets[backLeft]);
  motorWrite(backRightPin, targets[backRight]);
//toolControl(int pin, int postarget, int esdown, int esup, int enc, int cal)
  toolControl(linearPin, targets[linear], lBottom, lTop, 4, 0);
  toolControl(rotationPin, targets[rotation], rBottom, rTop, 5, 1);
}

void motorWrite(int pin, float throttle){
  //used to convert throttle in percentage
  throttle = pwm_zero_throttle + constrain(throttle, -motorMax, motorMax) * pwm_throttle_scaler; //convert to analogwrite value
  if (failsafe == 0) {
    analogWrite(pin, throttle);
    //send it for the boys
  }
  else {
    analogWrite(pin, pwm_zero_throttle);
    //don't send it for the boys.
  }
}

void calibrate(bool up,bool down,int enc,int cal){
  if (up == true){
    calibrations[cal] = encoderList[enc]->counter;
  }
  if (down == true){
    calibrations[cal] = encoderList[enc]->counter - ranges[cal];
  }
}

void toolControl(int pin, int postarget, int esdown, int esup, int enc, bool cal){
  //WE'RE GOING TO NEED TO DO SOME TESTING TO GET THIS DIALED IN.
  bool up = digitalRead(esup);
  bool down = digitalRead(esdown);
  calibrate(up,down,enc,cal);
  positions[cal] = (encoderList[enc]->counter) - calibrations[cal];
  int difference = postarget - positions[cal];
  if (!cal){
    float throttle = -difference / 5.0;
    //linear case
  }
  else{
    float throttle = difference / 7.0;
    //rotation case
  }
  esc(pin,throttle,up,down);
}

void esc(int pin, float throttle, bool esup, bool esdown){
  if (esup == true){
    throttle = constrain(throttle, -motorMax, 0); //if at top, only move down
  }
  if (esdown == true){
    throttle = constrain(throttle, 0, motorMax); //if at bottom, only move up
  }
  motorWrite(pin,throttle);
}







































//===============================JUNK YARD======================================

//IntervalTimer rpmtimer;

// fakecommandtimer.begin(callbackVel, 1000000); //take the fake command every 10 seconds

// IntervalTimer fakecommandtimer; //fake read in


// rpmtimer.begin(rpmmeasure, 4000); //measure rpm 250 times a second

// int lastcount[] = {0, 0, 0, 0, 0, 0};

// float rpms[] = {0, 0, 0, 0, 0, 0};

// int rpmcount[5][6] = {
//   {0, 0, 0, 0, 0, 0},
//   {0, 0, 0, 0, 0, 0},
//   {0, 0, 0, 0, 0, 0},
//   {0, 0, 0, 0, 0, 0},
//   {0, 0, 0, 0, 0, 0}
// };

// void rpmmeasure(void) { //average last 5 updates 250 times a second
//
//   for (int i = 4; i > 0; i--) { //count down from 5th position, this lets array data be shifted down one row, leaves top row unchanged
//     for (int j = 0; j < 6; j++) { //go through all 6 encoder counts
//       rpmcount[i][j] = rpmcount[i - 1][j]; //shift the data
//     }
//   }
//
//   for (int f = 0; f < 6; f++) { //replace first row with new data, set lastcount to the current count
//     rpmcount[0][f] = ((encoderList[f]->counter) - lastcount[f]) * 3.6; //rough change into actual rpm
//     lastcount[f] = (encoderList[f]->counter);
//   }
//
//   for (int k = 0; k < 6; k++) {
//     rpms[k] = (rpmcount[0][k] + rpmcount[1][k] + rpmcount[2][k] + rpmcount[3][k] + rpmcount[4][k]) / 5.00;
//     // encoderList[k]->counter;
//   }
// }

//call this function to set throttle on positional motor esc
// void escB(int pin, float throttle, bool esup, bool esdown) { //throttle is from -100 to 100, input endstops and whether the motor is reversed
//   if (esup == true){
//     throttle = constrain(throttle, -motorMax, 0); //if at top, only move down
//   }
//   if (esdown == true){
//     throttle = constrain(throttle, 0, motorMax); //if at bottom, only move up
//   }
//   motorWrite(pin,throttle);
// }
//
//
// void escC(int pin, float throttle, bool esup, bool esdown) {
//   //throttle is from -100 to 100, input endstops and whether the motor is reversed
//   if (esup == true){
//     throttle = constrain(throttle, 0, motorMax);
//     //if at top, only move down
//   }
//   if (esdown == true){
//     throttle = constrain(throttle, -motorMax, 0);
//     //if at bottom, only move up
//   }
//   motorWrite(pin,throttle);
// }

// void linearControl(int pin, int postarget, int esdown, int esup, int enc, int cal) {
//   bool up = digitalRead(esup);
//   bool down = digitalRead(esdown);
//   calibrate(up,down,enc,cal);
//   positions[cal] = (encoderList[enc]->counter) - calibrations[cal];
//   int dft1 = postarget - positions[cal];
//   float  throttle =  -dft1 / 5.0;
//   escB(pin, throttle, up, down);
// }
//
// void rotationControl(int pin, int postarget, int esdown, int esup, int enc, int cal) {
//   bool up = digitalRead(esup);
//   bool down = digitalRead(esdown);
//   calibrate(up,down,enc,cal);
//   positions[cal] = (encoderList[enc]->counter) - calibrations[cal];
//   int dft1 = postarget - positions[cal];
//   float  throttle = dft1 / 7.0;
//   escC(pin, throttle, up, down);
// }

//==============================================================================
