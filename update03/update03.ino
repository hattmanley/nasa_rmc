#include "AltEncoder.h"
#include "ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
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
float motorMax=60;  //change to adjust maximum drivetrain output!!!!!!!!!!!
int timeout = 1000;
long ranges[] = {13900, 11500}; //NEED TO DETERMINE THE RANGES OF MOTION
//float deadzone = 1535*0.05;
float safezone = 4000;
float linearsafe = 150;

//Probably don't need to change:
ros::NodeHandle nh;
const int pwm_zero_throttle = 1535;
const float pwm_throttle_scaler = 5.12;
unsigned long lastcommand = 0; //used as a check for failsafe--CLT
bool failsafe = 0; //start with failsafe deactivated...
float targets[] = {0, 0, 0, 0, 0, 0};
long calibrations[] = {0, 0};
long positions[] = {0, 0};
bool calibrated[]={false,false};

//===DIG STUFF=============
bool sequence_dig_active = false;
long digstarttime = 0;
long currenttime = 0;
long digTimeDifference = 0;


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
int rBottom = 36;
int rTop = 35;
int lBottom = 33;
int lTop = 34;

//=====
int tmpl = targets[4];
int tmpr = targets[5];

//================================DEFINE INTERVAL TIMERS========================
IntervalTimer pidtimer; //used w/ controlloop

//================================ROS===========================================
void callbackVel(const geometry_msgs::Twist& data) {
  if(data.linear.z==1 && calibrated[0]==true && calibrated[1]==true){
    digstarttime = millis();
    sequence_dig_active = true;
  }
  if(data.angular.x){
    sequence_dig_active = false;
    targets[frontLeft]=0;
    targets[frontRight]=0;
    targets[backLeft]=0;
    targets[backRight]=0;
    targets[linear] = positions[0];
    targets[rotation] = positions[1];
  }
 if(!sequence_dig_active){
  //command drivetrain speed
  targets[frontLeft]=data.linear.x;//front left
  targets[frontRight]=data.linear.y;
  targets[backLeft]=data.linear.x;
  targets[backRight]=data.linear.y;

  tmpl = targets[linear] + data.angular.y;
  tmpr = targets[rotation] + data.angular.z;

  if (data.angular.y == 0){
    tmpl = positions[0];
  }
  else{
    if (tmpl > ranges[0]+200){
      tmpl = ranges[0];
    }
    else if (tmpl < 0){
      tmpl = -100;
    }
  }
  if (data.angular.z == 0){
    tmpr = positions[1];
  }
  else{
    if (tmpr > ranges[1]+200){
      tmpr = ranges[1];
    }
    else if (tmpr < 0){
      tmpr = -100;
    }
  }
  targets[linear] = tmpl;
  targets[rotation] = tmpr;
  }//end if dig seq
  lastcommand = millis(); //remember millis returns an unsigned long.
  //millis checks how long the program has been running.  Pretty dope, eh?
}

ros::Subscriber<geometry_msgs::Twist> cmd_sub("robot/motor_control_serial", &callbackVel );
//this makes the teensy subscribe to the topic robot/motor_control_serial and sends
//the twist message into the function callbackVel.  piMotorControlSerial.py is
//the program that sends the message.

void setup(){
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

  //================================ROS=========================================
  nh.spinOnce();
  nh.subscribe(cmd_sub);

  //================================Dig Sequence================================
  currenttime = millis();
  digTimeDifference = currenttime-digstarttime;

  int stepStartTime[] = {0, //step0 start
                         3000,      //0-1
                         4500,      //1-2
                         7000,      //2-3
                         8000,      //3-4
                         10000,     //4-5
                         11000,     //5-6
                         15000};    //6-7

  //this is just adding 500 ms to each start time so we are sure the main loop will execute it.
  int stepEndTime[]= {stepStartTime[0]+500,
                      stepStartTime[1]+500,
                      stepStartTime[2]+500,
                      stepStartTime[3]+500,
                      stepStartTime[4]+500,
                      stepStartTime[5]+500,
                      stepStartTime[6]+500,
                      stepStartTime[7]+500};
  
  //STEP 0 ----------------------------------------------------------------------------------
  if (sequence_dig_active==true){
    if(((digTimeDifference)>stepStartTime[0]) && ((digTimeDifference)<stepEndTime[0])){
     lastcommand = millis();
      targets[0] = 0;
      targets[1] = 0;
      targets[2] = 0;
      targets[3] = 0;
      targets[4] = 6000;
      targets[5] = 8000;
    }
//STEP 1 ----------------------------------------------------------------------------------
    if(((digTimeDifference)>stepStartTime[1]) && ((digTimeDifference)<stepEndTime[1])){
      lastcommand = millis();
      targets[0] = 0;
      targets[1] = 0;
      targets[2] = 0;
      targets[3] = 0;
      targets[4] = 6000;
      targets[5] = 14000;
    }
//STEP 2 ----------------------------------------------------------------------------------
    if(((digTimeDifference)>stepStartTime[2]) && ((digTimeDifference)<stepEndTime[2])){
      lastcommand = millis();
      targets[0] = 0;
      targets[1] = 0;
      targets[2] = 0;
      targets[3] = 0;
      targets[4] = 11000;
      targets[5] = 14000;
    }
//STEP 3 ----------------------------------------------------------------------------------
    if(((digTimeDifference)>stepStartTime[3]) && ((digTimeDifference)<stepEndTime[3])){
      lastcommand = millis();
      targets[0] = 0;
      targets[1] = 0;
      targets[2] = 0;
      targets[3] = 0;
      targets[4] = 11000;
      targets[5] = 10900;
    }
//STEP 4 ----------------------------------------------------------------------------------
    if(((digTimeDifference)>stepStartTime[4]) && ((digTimeDifference)<stepEndTime[4])){
      lastcommand = millis();
      targets[0] = 0;
      targets[1] = 0;
      targets[2] = 0;
      targets[3] = 0;
      targets[4] = 13500;
      targets[5] = 10900;
    }
//STEP 5 ----------------------------------------------------------------------------------
    if(((digTimeDifference)>stepStartTime[5]) && ((digTimeDifference)<stepEndTime[5])){
      lastcommand = millis();
      targets[0] = 0;
      targets[1] = 0;
      targets[2] = -35;
      targets[3] = 35;
      targets[4] = 13500;
      targets[5] = 10900;
    }
//STEP 6 ----------------------------------------------------------------------------------
    if(((digTimeDifference)>stepStartTime[6]) && ((digTimeDifference)<stepEndTime[6])){
      lastcommand = millis();
      targets[0] = 0;
      targets[1] = 0;
      targets[2] = -20;
      targets[3] = 20;
      targets[4] = 13500;
      targets[5] = 4000;
    }
//STEP 7 ----------------------------------------------------------------------------------
    if(((digTimeDifference)>stepStartTime[7]) && ((digTimeDifference)<stepEndTime[7])){
      lastcommand = millis();
      targets[0] = 0;
      targets[1] = 0;
      targets[2] = 0;
      targets[3] = 0;
      targets[4] = 1000;
      targets[5] = 4000;
    }
//  //STEP 7 ----------------------------------------------------------------------------------
//  if(((digTimeDifference)>stepStartTime[6]) && ((digTimeDifference)<stepEndTime[6])){
//    lastcommand = millis();
//    targets[0] = 0;
//    targets[1] = 0;
//    targets[2] = 0;
//    targets[3] = 0;
//    targets[4] = 1000;
//    targets[5] = 5000;
//  }
    if ((digTimeDifference)>17500){
      sequence_dig_active=false;
    }
  }//end dig sequence

  delay(10);
}
//==============================END MAIN========================================

void controlloop (void) {
  long checktime = millis() - lastcommand;
  if (checktime < timeout) {
    failsafe = 0;
  }
  else {
    failsafe = 1;
  }

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
//  if (abs(pwm_zero_throttle - throttle) < deadzone){
    //analogWrite(pin, pwm_zero_throttle);
  //}//commanding zero throttle
  if (failsafe == 0) {
    analogWrite(pin, throttle);
    //send it for the boys
  }
  else {
    analogWrite(pin, pwm_zero_throttle);
    //don't send it for the boys.
  }
}

void calibratea(bool up,bool down,int enc,int cal){
  if (up == true){
    calibrations[cal] = encoderList[enc]->counter;
    calibrated[cal]=true;
    //Serial.println(encoderList[5]->counter);
  }
  if (down == true){
    calibrations[cal] = encoderList[enc]->counter - ranges[cal];
    calibrated[cal]=true;
    //Serial.println(encoderList[5]->counter);
  }
}

void toolControl(int pin, int postarget, int esdown, int esup, int enc, bool cal){
  //WE'RE GOING TO NEED TO DO SOME TESTING TO GET THIS DIALED IN.
  bool up = digitalRead(esup);
  bool down = digitalRead(esdown);
  float throttle = 0;
  calibratea(up,down,enc,cal);
  positions[cal] = (encoderList[enc]->counter) - calibrations[cal];
  int difference = postarget - positions[cal];

  //scales down throttle as it gets close to position
  if (!cal){
    throttle = -difference / 7.5;
    //linear case
    }
  else{
    throttle = difference/7.0 ;
    //rotation case
  }

  if (up){
    if(!cal){
      throttle = constrain(throttle, -motorMax, 0); //if at top, only move down
    }
    else{
      throttle = constrain(throttle, 0, motorMax); //if at top, only move down
    }
  }
  if(down){
    if(!cal){
      throttle = constrain(throttle, 0, motorMax); //if at bottom, only move up
    }
    else{
      throttle = constrain(throttle, -motorMax, 0); //if at bottom, only move down
    }
  }
  if(positions[cal] <= safezone && cal==1 && positions[0] >= linearsafe){
    throttle = constrain(throttle,0,motorMax);
  }
  if(positions[cal] <= linearsafe && cal==0 && positions[1] <= safezone){
    throttle = constrain(throttle,0,motorMax);
  }
  motorWrite(pin,throttle);
}































//===============================JUNK YARD======================================


// void esc(int pin, float throttle, bool esup, bool esdown){
//   if (esup == true){
//     throttle = constrain(throttle, -motorMax, 0); //if at top, only move down
//   }
//   if (esdown == true){
//     throttle = constrain(throttle, 0, motorMax); //if at bottom, only move up
//   }
//   motorWrite(pin,throttle);
// }

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
//void startCalibrate(){
//  while(0){
//    targets[4] = -10000;
//  }
//  targets[4] = 0;
//  while(0){
//    targets[5] = -10000;
//  }
//  targets[5] = 0;
//  delay(1000);
//  store();
//}
//
//void store(){
//  targets[5] = 5000;
//  delay(3000);
//  targets[4] = 6500;
//}
//
//void scoop(){
//  targets[5] = 10000;
//  delay(5000);
//  targets[4] = 10000;
//  delay(3000);
//  targets[0] = 30;
//  targets[1] = 30;
//  targets[2] = 30;
//  targets[3] = 30;
//  delay(1500);
//  store();
//}
//
//void dump(){
//  targets[4] = 0;
//  delay(2000);
//  targets[5] = 0;
//}
//==============================================================================
