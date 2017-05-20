


#include "AltEncoder.h"
using namespace AltEncoder;
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



#include <ros.h>
#include <std_msgs/Float32.h>
ros::NodeHandle  nh;          //create a node handle
std_msgs::Float32 pmessage;
ros::Publisher teensypub("teensy", &pmessage);
IntervalTimer sendrostimer;



IntervalTimer rpmtimer;
int lastcount[]={0,0,0,0,0,0};
float rpms[]={0,0,0,0,0,0};
int rpmcount[5][6]={
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0}
};



const int pwm_zero_throttle = 1535;
const float pwm_throttle_scaler = 5.12;



void setup()
{

  
 nh.initNode();
 nh.advertise(teensypub);
 sendrostimer.begin(sendmessage,1000000); //do sendmessage every 1,000,000 micro seconds

 
rpmtimer.begin(rpmmeasure,4000); //measure rpm 250 times a second
  
  
Controller::begin(encoderList, 25/*µs*/);   // choose a sampling period which is at least a factor of two smaller than the shortest time between two encoder signal edges.



  int pwmfreq = 250;   //250hz = 4ms each
  analogWriteFrequency(2, pwmfreq);//drive1
  analogWriteFrequency(3, pwmfreq);//drive2
  analogWriteFrequency(4, pwmfreq);//drive3
  analogWriteFrequency(5, pwmfreq);//drive4
  analogWriteFrequency(6, pwmfreq);//linear
  analogWriteFrequency(7, pwmfreq);//rotation
  analogWriteResolution(12); //0 - 4095, at 250hz, 4ms is split into 4096 parts, 1~2ms is from 1023-2047, 1537 being the center



  
}




void loop(){

//int dft1 = target1 - (encoderList[0]->counter);
//float  testthrottle=100 - dft1/50;
//escA(7,testthrottle);

      
escA(2,0);

} //end main loop

void sendmessage(void){
 // pmessage.data = encoderList[0]->counter;
 pmessage.data = rpms[0];
  teensypub.publish( &pmessage );
  nh.spinOnce();
}




void rpmmeasure(void){  //average last 5 updates 250 times a second

for(int i = 4; i>0; i--){  //count down from 5th position, this lets array data be shifted down one row, leaves top row unchanged
  for(int j=0; j<6; j++){    //go through all 6 encoder counts
    rpmcount[i][j]=rpmcount[i-1][j]; //shift the data
  }
}

for(int f=0; f<6; f++){  //replace first row with new data, set lastcount to the current count
rpmcount[0][f]=(encoderList[f]->counter)-lastcount[f];
lastcount[f]=(encoderList[f]->counter);
}

for(int k = 0; k<6; k++){
  rpms[k]=(rpmcount[0][k]+rpmcount[1][k]+rpmcount[2][k]+rpmcount[3][k]+rpmcount[4][k])/5.00;
 // encoderList[k]->counter;
}
}






//call this function to set throttle on a drive motor esc
void escA(int pin,float throttle) { //throttle is from -100 to 100 
throttle = pwm_zero_throttle + constrain(throttle, -100, 100)*pwm_throttle_scaler;//convert to analogwrite value
analogWrite(pin, throttle);//send it
 }





//call this function to set throttle on positional motor esc
void escB(int pin,float throttle, bool esup, bool esdown) { //throttle is from -100 to 100, input endstops and whether the motor is reversed
  

if (esup){
throttle = pwm_zero_throttle + constrain(throttle, -100, 0)*pwm_throttle_scaler;  //if at top, only move down
}
else if (esdown){
throttle = pwm_zero_throttle + constrain(throttle, 0, 100)*pwm_throttle_scaler;   //if at bottom, only move up
}
else{
throttle = pwm_zero_throttle + constrain(throttle, -100, 100)*pwm_throttle_scaler; //if inbetween its gucci
}


analogWrite(pin, throttle);  //send it
 }
 
