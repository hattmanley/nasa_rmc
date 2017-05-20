#include "AltEncoder.h"

using namespace AltEncoder;

// Define a list of as much encoders you need
// Parameters are pin numbers for 
// phase A and phase B

Encoder* encoderList[] =
{//            A  B
  new Encoder( 8, 9),
 // new Encoder( 5, 6),
 // new Encoder( 7, 8),
 // new Encoder( 9,10),
 // new Encoder(11,12),
 // new Encoder(14,15),
  nullptr
};

void setup() 
{
  Serial.begin(0);
  Controller::begin(encoderList, 25/*µs*/);   // choose a sampling period which is at least a factor of two smaller than the shortest time between two encoder signal edges.



  int pwmfreq = 250;   //250hz = 4ms each
analogWriteFrequency(2, pwmfreq);
analogWriteFrequency(3, pwmfreq);
analogWriteFrequency(4, pwmfreq);
analogWriteFrequency(5, pwmfreq);
analogWriteFrequency(6, pwmfreq);
analogWriteFrequency(7, pwmfreq);
analogWriteResolution(12); //0 - 4095, at 250hz, 4ms is split into 4096 parts, 1~2ms is from 1023-2047, 1537 being the center

}                                            
void loop() 
{
   float testpwm = 1537;
  //for(int i = 0; i< 6; i++) 
 // {
    Serial.println(encoderList[0]->counter);
 // }
  Serial.println();


int target1 = 10000;

int dft1= target1-(encoderList[0]->counter);

 testpwm = 1537 - dft1/5;
 
if (testpwm < 1023){
testpwm = 1023;
}


if (testpwm> 2047){
testpwm = 2047;
}
analogWrite(2,testpwm);
analogWrite(3,testpwm);
analogWrite(4,testpwm);
analogWrite(5,testpwm);
analogWrite(6,testpwm);
analogWrite(7,testpwm);


}
