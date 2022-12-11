//Robot Sensors/Actuators Final Project
//James Kaluna, Shayan Sadiq, Andrew Palacio

//Arduino code for the peripheral 

//This code must accomplish the following:
//Receive the potentiometer signals from the controller and control DC motor speed
//Receive inputs from the IMU/GPS and calculate velocity. 
//Send the velocity readings back to the controller

#include <SPI.h>
#include <RH_RF69.h>
/************ Radio Setup ***************/
// Change to 434.0 MHz or other frequency,
// must match the other transceiver's freq!
#define RF69_FREQ 910.0 //change this!!!!
#define RFM69_INT 2 //
#define RFM69_CS 7 //
#define RFM69_RST 8 //
// Instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// pins for H Bridge (assuming EN1 is left motor and EN2 is right motor)
const int input1 = 10;
const int input2 = 9;
const int EN1 = 11;

const int input3 = 8;
const int input4 = 7;
const int EN2 = 6;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(input1, OUTPUT); //inputs 1 and 2 control motor direction. 
  pinMode(input2, OUTPUT);
  pinMode(EN1, OUTPUT); //pwm signal that controls motor speed
  pinMode(input3, OUTPUT); //inputs 3 and 4 control motor direction. 
  pinMode(input4, OUTPUT);
  pinMode(EN2, OUTPUT); //pwm signal that controls motor speed
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  delay(100);
  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  if (!rf69.init()) {
    Serial.println("RFM69 radio RX init failed");
  }
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }
  rf69.setTxPower(20, true); // range from 14-20 for power, 2nd arg must be true for 69HCW
}

void loop() {
  // put your main code here, to run repeatedly:
    
 int left;
 int right;
 int FB;
  int enableSignalOne;
  int enableSignalTwo;
 unsigned long startTime = millis();
 while (1) {
   if (rf69.available()) {
     uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
     uint8_t len = sizeof(buf);
     if (rf69.recv(buf, &len)) {
       buf[len] = 0;
       left = buf[0];
       right = buf[1];
       FB = buf[2];
       if (FB > 0) {
         digitalWrite(input1, HIGH);
          digitalWrite(input2, LOW);
          digitalWrite(input3, HIGH);
        digitalWrite(input4, LOW);      
        enableSignalOne = left + abs(FB);
        enableSignalTwo = right + abs(FB);
       } // push motor forwards if forward-back signal is positive
       if (FB <= 0) {
         digitalWrite(input1, HIGH);
          digitalWrite(input2, LOW);
          digitalWrite(input3, HIGH);
        digitalWrite(input4, LOW);    
        enableSignalOne = left + abs(FB);
        enableSignalTwo = right + abs(FB);
       } // push motor backwards if forward-back signal is negative
       analogWrite(EN1, enableSignalOne);
       analogWrite(EN2, enableSignalTwo);
       break;
     }
   }
 }
}
