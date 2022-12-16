//Robot Sensors/Actuators Final Project
//James Kaluna, Shayan Sadiq, Andrew Palacio
//Arduino code for the peripheral Arduino

/*
  This code must accomplish the following:
  -Receive the potentiometer signals from the controller and use it to create bytes for left/right motor speed
  -Output motor speed data to H-bridge for DC motor contro
  -Receive inputs from the encoder and calculate velocity and distance.
  -Send the velocity readings back to the controller
*/

// include libraries for RF transceiver
#include <SPI.h>
#include <RH_RF69.h>

// initialize RF transceiver:
#define RFM69_INT 2
#define RFM69_CS 10
#define RFM69_RST 9
RH_RF69 rf69(RFM69_CS, RFM69_INT); //creates transceiver object

// create global variables for use in ISR that will calculate motor velocity/distance travelled
volatile long counter = 0;     //will track the relative position of encoder A
const float radius = 3.5;   //radius of wheel in cm
volatile float deltaT;         //time elapsed between every half cycle
volatile float startTime = 0;  //time in which cycle begins
volatile float omega;          //angular velocity in rad/s
volatile float velocity = 0;   //velocity in cm/s
volatile float distance = 0;   //distance travelled in m

// pins for H Bridge (input 14 powers input 1 and 4 pins, input 14 powers input 2 and 3 pins, EN1 is enable 1 for left motor and EN2 is enable 2 for right motor)
const int input14 = 8;
const int input23 = 4;
const char EN1 = A0;
const char EN2 = A5;

void setup() {
  // initialize Serial Monitor
  Serial.begin(9600);

  //attach ISR to digital pin 3, triggered when encoder signal changes from HIGH to LOW or vice versa
  attachInterrupt(1, encoderChange, CHANGE);

  // initialize H-bridge output pins
  pinMode(input14, OUTPUT);  //controls motor direction
  pinMode(input23, OUTPUT);  //controls motor direction
  pinMode(EN1, OUTPUT);  //pwm signal that controls left motor speed
  pinMode(EN2, OUTPUT);  //pwm signal that controls right motor speed

  // initialize and manually reset RF transceiver
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  delay(100);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  // display message if RF transceiver initialization was successeful or not
  if (!rf69.init()) {
    Serial.println("RFM69 radio RX init failed");
  }
  else {
    Serial.println("The initialization was successful");
  }
  if (!rf69.setFrequency(915)) {
    Serial.println("setFrequency failed");
  }

  //set RF power
  rf69.setTxPower(20, true);
}

void loop() {
  // initialize variables for controlling motor speed
  int left = 0; //left motor modifier (0 is center of joystick, 128 is furthest left)
  int right = 0; //right motor modifier (0 is center of joystick, 128 is furthest right)
  int FB = 0; //forward/back speed (-255 to 255)
  int enableSignalOne; //signal output to EN1 pin
  int enableSignalTwo; //signal output to EN2 pin
  unsigned long startTime = micros();
  // while loop to wait for transmission from controller Arduino
  while (1) {
    // series of if statements for when peripheral receives message from Arduino
    if (rf69.available()) {
      uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rf69.recv(buf, &len)) {
        // read message from controller and store in variables
        buf[len] = 0;
        left = buf[0];
        right = buf[1];
        FB = buf[2];

        // if F/B signal is positive, power input pins to move motors forward
        if (FB > 0) {
          digitalWrite(input14, HIGH);
          digitalWrite(input23, LOW);
        }

        // if F/B signal is negative, power input pins to move motors backward
        if (FB <= 0) {
          digitalWrite(input14, LOW);
          digitalWrite(input23, HIGH);
        }

        // enable signal is equal to forward/back amplitude - left/right modifier (higher modifier means more turning in that direction, so we want motor to go slower)
        enableSignalOne = abs(FB) - left;
        enableSignalTwo = abs(FB) - right;
        //send enable signal to respective enable piuns
        analogWrite(EN1, enableSignalOne);
        analogWrite(EN2, enableSignalTwo);

        // once message is received, break from while loop
        break;
      }
    }
  }
  // if car is not moving for 250 milliseconds, reset velocity to 0 cm/s.
  if (abs(micros() - startTime) > 250000) {
    velocity = 0;
  }

  // transmit velocity and distance data to controller Arduino continuously for 10 milliseconds
  while (micros() - startTime < 10000) {
    char radiopacket[2] = {velocity, distance};
    rf69.send((uint8_t *)radiopacket, sizeof(radiopacket));
    rf69.waitPacketSent();
  };




}

//ISR that detects change in encoder signal and calculates velocity and distance
void encoderChange() {
  // calculate change in time since last ISR activation
  deltaT = micros() - startTime;
  startTime = micros();

  // calculate angular velocity, then convert to linear velocity
  omega = 3.14159 / 120 / (deltaT / 1000000);
  velocity = omega * radius;

  // if a full cycle has completed, increase counter variable 
  if (digitalRead(3) == HIGH) {
    counter++;
  }
  distance = counter * 3.14 * (radius / 100) / 240;  //calculates distance the car has travelled
}
