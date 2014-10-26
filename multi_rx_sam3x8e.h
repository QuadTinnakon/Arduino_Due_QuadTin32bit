//#include "multi_rx2560.h"
//project_Quad 32 bit Arduino Due
//1. stabilized quadrotor 
//by: tinnakon kheowree 
//0860540582
//tinnakon_za@hotmail.com
//tinnakonza@gmail.com

#define MIDRC 1500
#define MINCHECK 1100
#define MAXCHECK 1900

int CH_THR = 1000;
int CH_AIL = 1500;
int CH_ELE = 1500;
int CH_RUD = 1500;
float CH_AILf = 1500;
float CH_ELEf = 1500;
float CH_RUDf = 1500;
int CH_AIL_Cal = 1500;
int CH_ELE_Cal = 1500;
int CH_RUD_Cal = 1500;
int AUX_1 = 1000;
//int AUX_2 = 1000;

//RX PIN assignment inside the port //for PORTK
//SET YOUR PINS! TO MATCH RECIEVER CHANNELS
#define CHAN1PIN 62 // RECIEVER 1 PPM
//#define CHAN2PIN 63 // RECIEVER 2
//#define CHAN3PIN 64 // RECIEVER 3
//#define CHAN4PIN 65 // RECIEVER 4 
//#define CHAN5PIN 61 // RECIEVER 5
//#define CHAN6PIN 67 //not used at the moment
//#define CHAN7PIN 50 //not used at the moment
//#define CHAN8PIN 51 //not used at the moment

#define ROLL       0
#define PITCH      1
#define YAW        3
#define THROTTLE   2
#define AUX1       5
//#define AUX2       4
//#define CAMPITCH   6
//#define CAMROLL    7      

//volatile unsigned long pwmLast[8];
volatile unsigned long last = 0;
uint8_t chan1 = 0;
volatile uint16_t rcValue[8] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]

// Arduino Due RX
void pwmHandler(int ch, int pin) {//PPM
/*
  unsigned long timeST = micros();
  if (digitalRead(pin)) {//pin HIGH
    pwmLast[ch] = timeST;
  } else {//pin LOW
    timeST = (timeST - pwmLast[ch]); // / 42;
    if (timeST>900 && timeST<2200) {
      rcValue[ch] = timeST;
    }
  }
  */
  ///PPM/////////////////
    unsigned long now,diff;  
    now = micros();
    diff = now - last;
    last = now;
    if(diff>3000) chan1 = 0;
    else {
      if(900<diff && diff<2200 && chan1<8) {   //Only if the signal is between these values it is valid, otherwise the failsafe counter should move up
        rcValue[chan1] = diff;
      }
    chan1++;
  }
  ///////////////////////
}
void ch1Handler() { pwmHandler(0, CHAN1PIN); }
//void ch2Handler() { pwmHandler(1, CHAN2PIN); }
//void ch3Handler() { pwmHandler(2, CHAN3PIN); }
//void ch4Handler() { pwmHandler(3, CHAN4PIN); }
//void ch5Handler() { pwmHandler(4, CHAN5PIN); }

void configureReceiver() {
    for (uint8_t chan = 0; chan < 8; chan++){
      for (uint8_t a = 0; a < 4; a++){
        rcValue[a] = 1500;
      }
    }
   attachInterrupt(CHAN1PIN,ch1Handler,RISING);//RISING  FALLING CHANGE
   //attachInterrupt(CHAN2PIN,ch2Handler,CHANGE);
   //attachInterrupt(CHAN3PIN,ch3Handler,CHANGE);
   //attachInterrupt(CHAN4PIN,ch4Handler,CHANGE);
   //attachInterrupt(CHAN5PIN,ch5Handler,CHANGE);
}

void computeRC() {
    CH_THR = rcValue[THROTTLE];
    CH_AIL = rcValue[ROLL];
    CH_ELE = rcValue[PITCH];
    CH_RUD = rcValue[YAW];
    AUX_1 = rcValue[AUX1];
    //AUX_2 = rcValue[AUX2];
    CH_AILf = CH_AILf + (CH_AIL - CH_AILf)*0.02/tarremote;
    CH_ELEf = CH_ELEf + (CH_ELE - CH_ELEf)*0.02/tarremote;
    CH_RUDf = CH_RUDf + (CH_RUD - CH_RUDf)*0.02/tarremote;
}
//By tinnakon
  void RC_Calibrate(){
  Serial.print("RC_Calibrate");Serial.println("\t");
  for (int i = 0; i < 10; i++) {
    computeRC();
    delay(20);
  }
  CH_AIL_Cal = CH_AIL;
  CH_ELE_Cal = CH_ELE;
  CH_RUD_Cal = CH_RUD;
    Serial.print(CH_AIL_Cal);Serial.print("\t");//-0.13
    Serial.print(CH_ELE_Cal);Serial.print("\t");//-0.10
    Serial.print(CH_RUD_Cal);Serial.println("\t");//0.03 
}
