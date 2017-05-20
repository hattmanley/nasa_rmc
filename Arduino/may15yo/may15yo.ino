#include "AltEncoder.h"
using namespace AltEncoder;



// Define a list of as much encoders you need
// Parameters are pin numbers for
// phase A and phase B

Encoder* encoderList[] =
{ //            A  B
  new Encoder( 8, 9),
  new Encoder( 10, 11),
  new Encoder( 24, 25),
  new Encoder( 26,27),
  new Encoder(28,29),
  new Encoder(30,31),
  nullptr
};

IntervalTimer rpmMeasure;

int i = 1;
const int pwm_zero_throttle = 1535;
const float pwm_throttle_scaler = 5.12;

void setup()
{
  Serial.begin(0);
  Controller::begin(encoderList, 25/*µs*/);   // choose a sampling period which is at least a factor of two smaller than the shortest time between two encoder signal edges.

  int pwmfreq = 250;   //250hz = 4ms each
  analogWriteFrequency(2, 250);
  analogWriteFrequency(3, 250);
  analogWriteFrequency(4, 250);
  analogWriteFrequency(5, 250);
  analogWriteFrequency(6, 250);
  analogWriteFrequency(7, 250);
  analogWriteResolution(12); //0 - 4095, at 250hz, 4ms is split into 4096 parts, 1~2ms is from 1023-2047, 1537 being the center
}








void loop()





{

    int target1 = 10000;


  int currenttime = millis();

  if (currenttime % 8 == 0){
    if (i == 0){
      Serial.println(encoderList[0]->counter);
      Serial.println(millis());
      Serial.println();
      i = 1;



//int dft1 = target1 - (encoderList[0]->counter);
//float  testthrottle=100 - dft1/50;
//escA(7,testthrottle);

      
    }
  }
  else{
    i = 0;
  }




escA(2,0);




} //end main loop






//call this function to set throttle on an esc

void escA(int pin,float throttle) { //throttle is from -100 to 100
throttle = pwm_zero_throttle + constrain(throttle, -100, 100)*pwm_throttle_scaler;
analogWrite(pin, throttle);
 }
