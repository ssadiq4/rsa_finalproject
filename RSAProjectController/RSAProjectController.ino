//Robot Sensors/Actuators Final Project
//James Kaluna, Shayan Sadiq, Andrew Palacio

//Controller Code

//This code will need to accomplish the following tasks:
//Read the two potentiometer knobs from the joystick and convert them into bytes
//Use the RF transceiver to send the bytes to the peripheral
//Receive speed data from the peripheral and display it on the OLED

#include <SPI.h>
#include <RH_RF69.h> //RF transceiver library

const int CS_pin = 7;
const int INT_pin = 2;
const int RST_pin = 8;

RH_RF69 rfTransceiver(CS_pin, INT_pin); //creates transceiver object

//Joystick: 
int buttonPin = 10;
//x steering potentiometer is A2
//y throttle potentiometer is A3
//both potentiometers are neutral at 2.5V

//both of these are 1023 analog signals divided by 4 to make a byte
int LR = 0; //byte signal of steering potentiometer
int FB = 0; //byte signal of throttle
int left;
int right;

void setup() {
  // put your setup code here, to run once:
  pinMode(RST_pin, OUTPUT);
  digitalWrite(RST_pin, LOW); //initializing RST pin

  //Reset the transceiver
  digitalWrite(RST_pin, HIGH);
  delay(10);
  digitalWrite(RST_pin, LOW);
  delay(10);

  rfTransceiver.init(); //initializes the transceiver

  //verify that the transceiver initialization was successful:
  Serial.begin(9600);
  if (rfTransceiver.init() == true) {
    Serial.print("The initialization was successful");
  }
  else {
    Serial.print("The initialization failed");
  }

  //Set the transceiver frequency
  rfTransceiver.setFrequency(915);
  rfTransceiver.setTxPower(20, true);
  
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(buttonPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Read the A2 and A3 pins and convert them to bytes
  LR = analogRead(A2)/4; //voltage signal ranging from 0 to 255
  FB = analogRead(A3)/4; 
  FB = FB - 127;
  if (LR < 128) {
    left = LR;
    right = 0;
  }
  if (LR >= 128) {
    left = 0;
    right = abs(LR-255);
  }

  //these lines transmit the data. We're gonna try 1 character for now.
  char radiopacket[3] = {left, right, FB};
  rfTransceiver.send((uint8_t *)radiopacket, sizeof(radiopacket));
  rfTransceiver.waitPacketSent();
  delay(25);
  
}
