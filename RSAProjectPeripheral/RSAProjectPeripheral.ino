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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
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

  //We have to reverse engineer Dr. Kraemer's code here
  int x;
  unsigned long startTime = millis();
  while (1) {
    if (rf69.available()) {
      uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rf69.recv(buf, &len)) {
        buf[len] = 0;
        x = buf[0];
        Serial.println(x);
        break;
      }
    }
  }
    
//  int x;
//  int y;
//  unsigned long startTime = millis();
//  while (1) {
//    if (rf69.available()) {
//      uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
//      uint8_t len = sizeof(buf);
//      if (rf69.recv(buf, &len)) {
//        buf[len] = 0;
//        x = buf[0];
//        y = buf[1];
//        Serial.print(x);
//        Serial.print("      ");
//        Serial.println(y);
//        break;
//      }
//    }
//  }
}
