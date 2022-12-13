//Robot Sensors/Actuators Final Project
//James Kaluna, Shayan Sadiq, Andrew Palacio

//Controller Code

//This code will need to accomplish the following tasks:
//Read the two potentiometer knobs from the joystick and convert them into bytes
//Use the RF transceiver to send the bytes to the peripheral
//Receive speed data from the peripheral and display it on the OLED

#include <SPI.h>
#include <RH_RF69.h> //RF transceiver library
#include <Wire.h> 
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels 
#define SCREEN_HEIGHT 64 // OLED display height, in pixels 

// Declaration for SSD1306 display connected using software SPI:
//double check all of your pinouts!
#define OLED_PICO   6
#define OLED_CLK    7 
#define OLED_DC     4 
#define OLED_CS     3 
#define OLED_RESET  5 
Adafruit_SSD1306 display(OLED_PICO, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS); 

float velo;
float dist;

const int CS_pin = 10;
const int INT_pin = 2;
const int RST_pin = 9;

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

  // initialize display and set text style and cursor
  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.setTextColor(WHITE); 
  display.setCursor(10, 10);
  display.setTextSize(2); 

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

  //we must figure out how to time the transmitting and receiving of signals. 
  if (rf69.available()) {
     uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
     uint8_t len = sizeof(buf);
     if (rf69.recv(buf, &len)) {
       buf[len] = 0;
       velo = buf[0];
       dist = buf[1];
        // clear display and reset cursor
        display.clearDisplay();
        display.setCursor(0, 0);

        // send the pulse distance to the OLED and display it
        display.print("Total Distance Traveled: ");
        display.print(dist);
        display.println(" m");

        display.setCursor(30, 30);
        display.print("Velocity: ");
        display.print(velo);
        display.println(" cm/s");
        display.display();
        
        // wait half a second before making another reading
        //delay(500);
      }   
    }

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
