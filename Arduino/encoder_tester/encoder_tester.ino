#include "AltEncoder.h"
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


const int pwm_zero_throttle = 1535;
const float pwm_throttle_scaler = 5.12;
bool failsafe = 0;
void setup() {
  // put your setup code here, to run once:
  Controller::begin(encoderList, 30/*µs*/);  //33khz // choose a sampling period which is at least a factor of two smaller than the shortest time between two encoder signal edges.
  int pwmfreq = 250;   //250hz = 4ms each
  analogWriteFrequency(2, pwmfreq);//drive1
  analogWriteFrequency(3, pwmfreq);//drive2
  analogWriteFrequency(4, pwmfreq);//drive3
  analogWriteFrequency(5, pwmfreq);//drive4
  analogWriteFrequency(6, pwmfreq);//linear
  analogWriteFrequency(7, pwmfreq);//rotation
  analogWriteResolution(12); //0 - 4095, at 250hz, 4ms is split into 4096 parts, 1~2ms is from 1023-2047, 1537 being the center
  rpmControl(7,0,0,0);
  Serial.begin(9600);
  Serial.write("started");
}

void loop() {
  Serial.println(String(encoderList[5]->counter));
  delay(2000);
}


void rpmControl(int pin, float throttle, int enc, int rpm) { //throttle is from -100 to 100
  throttle = pwm_zero_throttle + constrain(throttle, -100, 100) * pwm_throttle_scaler; //convert to analogwrite value
  if (failsafe == 1) {
    analogWrite(pin, pwm_zero_throttle);//dont send it
  }
  else {
    analogWrite(pin, throttle);
  }
}

