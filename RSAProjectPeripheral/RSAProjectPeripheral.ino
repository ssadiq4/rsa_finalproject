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
#define RF69_FREQ 915.0  //change this!!!!
#define RFM69_INT 2      //
#define RFM69_CS 10      //
#define RFM69_RST 9      //
// Instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

//this code is going to calculate the angular velocity from the left motor encoders and display it on the LED
volatile bool encoderA = false;  //these two booleans will determine whether the two encoder signals are HIGH or LOW.
volatile bool encoderB = false;
volatile long counter = 0;     //will track the relative position of encoder A
volatile float radius = 3.5;   //radius of wheel in cm
volatile float deltaT;         //time elapsed between every half cycle
volatile float startTime = 0;  //time in which cycle begins
volatile float omega;          //angular velocity in rad/s
volatile float velocity = 1;       //cm/s
volatile float distance = 1;   //m

// pins for H Bridge (assuming EN1 is left motor and EN2 is right motor)
const int input14 = 8;
const int input23 = 4;
const char EN1 = A1;
const char EN2 = A5;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  attachInterrupt(1, encoderChange, CHANGE);  //this interrupt is triggered when encoder A changes to HIGH or LOW.
  // attachInterrupt(1, encoderChangeB, CHANGE); //This interrupt is triggered when encoder B changes to HIGH or LOW.

  pinMode(input14, OUTPUT);  //inputs control motor direction.
  pinMode(input23, OUTPUT);
  pinMode(EN1, OUTPUT);  //pwm signal that controls motor speed
  pinMode(EN2, OUTPUT);  //pwm signal that controls motor speed
  pinMode(RFM69_RST, OUTPUT);
  // manual reset
  digitalWrite(RFM69_RST, LOW);
  delay(100);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  if (!rf69.init()) {
    Serial.println("RFM69 radio RX init failed");
  }
  else {
    Serial.println("The initialization was successful");
  }
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
}

void loop() {
  // put your main code here, to run repeatedly:
  int left = 0;
  int right = 0;
  int FB = 0;
  int enableSignalOne;
  int enableSignalTwo;
  unsigned long startTime = micros();
  while (micros() - startTime < 500) {
  char radiopacket[2] = {velocity, distance};
  //Serial.println("sending");
  rf69.send((uint8_t *)radiopacket, sizeof(radiopacket));
  rf69.waitPacketSent();
  }; 
  while (1) {
//    Serial.println("in loop");
    if (rf69.available()) {
      uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rf69.recv(buf, &len)) {
        buf[len] = 0;
        left = buf[0];
        right = buf[1];
        FB = buf[2];
//        Serial.print(left);
//        Serial.print("      ");
//        Serial.print(right);
//        Serial.print("      ");
//        Serial.println(FB);
        if (FB > 0) {
          digitalWrite(input14, HIGH);
          digitalWrite(input23, LOW);
        }  // push motor forwards if forward-back signal is positive
        if (FB <= 0) {
          digitalWrite(input14, LOW);
          digitalWrite(input23, HIGH);
        }  // push motor backwards if forward-back signal is negative
        enableSignalOne = abs(FB) - left;
        enableSignalTwo = abs(FB) - right;
        analogWrite(EN1, enableSignalOne);
        analogWrite(EN2, enableSignalTwo);
        break;
      }
    }
    if (abs(micros() - startTime) > 250000) {break;}
  }
  if (abs(micros() - startTime) > 250000) {
    velocity = 0;
  }
  char reception;

  if ((velocity > 0) & (distance > 0)) {
  //these lines transmit the velocity and distance measurements back to the controller.
  char radiopacket[2] = {velocity, distance};
  //Serial.println("sending");
  rf69.send((uint8_t *)radiopacket, sizeof(radiopacket));
  //Serial.println(radiopacket);
  rf69.waitPacketSent();
  }
  // if (rf69.available()) {
  //     uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  //     uint8_t len = sizeof(buf);
  //     if (rf69.recv(buf, &len)) {
  //       buf[len] = 0;
  //       reception = buf[0]; 
  //       Serial.println(reception);
  //       if (reception != NULL) {
  //         break;
  //       }
  //       else {continue;}
  //       reception = NULL;
  //     }
  //   }
  // }

}

void encoderChange() {
  deltaT = micros() - startTime;
  startTime = micros();
  omega = 3.14159 / 120 / (deltaT / 1000000);
  velocity = omega * radius;

  if (digitalRead(3) == HIGH) {
    counter++;
  }
  // if (encoderA != encoderB) { //checks whether both encoders are HIGH or LOW.
  //   counter++; //if the encoder signals are not equal, then the motor is spinning in a certain direction. counter increases.

  // }
  // else { //if the encoder signals are equal, the motor is spinning in the opposite direction. counter decreases.
  //   counter--;

  // }
  encoderA = !encoderA;                              //changes the boolean to match the measured signal change
  distance = counter * 3.14 * (radius / 100) / 240;  //calculates distance the car has travelled
}
