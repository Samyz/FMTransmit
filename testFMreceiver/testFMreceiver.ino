/*
 * Simple Program to set a Frequency and read Statistics 
 * 
 * Licence: GNU GPL
 * 
 * by big12boy 2017
 */

#include <TEA5767.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_ADS1015.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
#define r_slope 30

Adafruit_MCP4725 dac;

//#define baud 250
#define defaultFreq 1700
#define f0min 1800
#define f0max 2200
#define f1min 1200
#define f1max 1450
#define f2min 900
#define f2max 1100
#define f3min 700
#define f3max 900
int delay0, delay1, delay2, delay3;
char inData[30];
int keep = 0;
int sum = 0;
int max = 0;
int min = 0;
int prev = 0;
int output = -1;
int count = 0;
int nub = 0;
int check = false, first = true;
uint8_t dis=0;
unsigned long times;
unsigned long oooo;
int eiei = 3950;
TEA5767 radio = TEA5767();

float frequency = 105.8;//98.2;//102.8 //Enter your own Frequency
long baud = 115200; //Enter your own Baudrate

long lmillis = 0;

void setup() {
  //Setup Serial and I2C
  Serial.begin(baud);
  Wire.begin();
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  //Set Frequency to start with
  radio.setFrequency(frequency);
}

void loop() {
  int tmp = analogRead(A0);
  //Serial.println(tmp);
  if(tmp - prev> r_slope && check == false && tmp > 420){
    max = 420;
    check = true;
  }
  if(tmp>max){
    max=tmp;
  }
  //Serial.println(tmp);
  if(first && tmp > 450 && check == true){
    times = micros();
    /*
    if (nub >= 1)
    {
      if (micros() - oooo > 5000)
      {
        nub = 0;
        nub=0;
        dis=0;
        keep=0;
      }
    }
    oooo = micros();*/
    first = false;
    count = 0;
  }
  if(max-tmp>r_slope){
    //Serial.println(tmp);
    if(check){
      if(490<max&&max<1024){
        count++;
        //Serial.println(count);
        //delayMicroseconds(300);
      }
    }
    check = false;
    }
    if(micros()-times > eiei&&!first){
      
      //Serial.print(nub);
      //Serial.println(count);
      if(count<2)
      {
        count = 0;
        first = true;
        dis = 0;
        nub = 0;
      }
      else{
        times = micros();
        nub++;
        keep = count - 2;
        //Serial.println(keep);
        dis>>=2;
        dis |= (keep<<6) ;
        //Serial.println(dis,BIN);
        //Serial.println();
        count=0;
        if(nub==4){
          Serial.print((char)dis);
          nub=0;
          dis=0;
          keep=0;
          first=true;
          delayMicroseconds(4800);
        }
      }
    }
    prev=tmp;
}
