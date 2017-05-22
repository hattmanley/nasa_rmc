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

bool failsafe = 1;
const int pwm_zero_throttle = 1535;
const float pwm_throttle_scaler = 5.12;

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

void setup(){
  Serial.begin(9600);
  //================================ENDSTOP PINS================================
  pinMode(rBottom, INPUT); //rotation bottom
  pinMode(rTop, INPUT); //rotation top
  pinMode(lBottom, INPUT); //linear bottom
  pinMode(lTop, INPUT); //linear top

  //================================ENCODER SAMPLING============================
  Controller::begin(encoderList, 30/*µs*/);  //33khz // choose a sampling period
                                                     //which is at least a factor
                                                     //of two smaller than the
                                                     //shortest time between two
                                                     //encoder signal edges.
}


void loop(){
  if (Serial.available() > 0) {
                // read the incoming byte:
                motorCommand = Serial.read();
                motorCommand = constrain(motorCommand,-100,100);
                // say what you got:
                Serial.print("I received: ");
                Serial.println(incomingByte, DEC);
                Serial.print("Send it for the boys");
                motorWrite(frontLeftPin,motorCommand);
        }
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
