//Robot Sensors/Actuators Final Project
//James Kaluna, Shayan Sadiq, Andrew Palacio
//Arduino code for the controller Arduino

/*
This code will need to accomplish the following tasks:
-Read the two potentiometer knobs from the joystick and convert them into bytes
-Use the RF transceiver to send the bytes to the peripheral
-Receive velocity/distance data from the peripheral and display it on the OLED
*/

// including libraries for serial communication, RF transceiver, and OLED display
#include <SPI.h>
#include <RH_RF69.h> //RF transceiver library
#include <Wire.h>
#include <Adafruit_SSD1306.h>

// initializing SSD1306 display connected using software SPI:
#define SCREEN_WIDTH 128 // OLED display width, in pixels 
#define SCREEN_HEIGHT 64 // OLED display height, in pixels 
#define OLED_PICO   6
#define OLED_CLK    7
#define OLED_DC     4
#define OLED_CS     3
#define OLED_RESET  5
Adafruit_SSD1306 display(OLED_PICO, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS); //creates display object


// initializing RF transceiver:
const int CS_pin = 10;
const int INT_pin = 2;
const int RST_pin = 9;
RH_RF69 rfTransceiver(CS_pin, INT_pin); //creates transceiver object


void setup() {

  // initialize display and set text style and cursor
  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(10, 10);
  display.setTextSize(1);

  pinMode(RST_pin, OUTPUT); //initializing RST pin for transceiver

  // Manually reset the transceiver
  digitalWrite(RST_pin, LOW);
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


  //x steering potentiometer is A2
  //y throttle potentiometer is A3
  //both potentiometers are neutral at 2.5V
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
}

void loop() {

  //read the potentiometer signals from the A2 and A3 pins and convert them to bytes
  int LR = analogRead(A2) / 4; // LR stands for left/right magnitude
  int FB = analogRead(A3) / 4; // FB stands for forward/backwards magnitude
  FB = (FB - 127) * 2; // center FB signal at 0 and scale up to -255 - 255 (integer math means this is not going to be exactly symmetrical)
  int left; //modifier for left motor (minimum (center of joystick) is 0, maximum is 128);
  int right; //modifier for right motor (minimum (center of joystick) is 0, maximum is 128);

  // if LR >= 128, user is pushing joystick left --> use signal as left modifier
  if (LR >= 128) {
    left = LR - 128;
    right = 0;
  }
  // if LR < 128, user is pushing joystick right --> use signal as right modifier
  if (LR < 128) {
    left = 0;
    right = 128 - LR;
  }

  // constrain FB in case integer math above causes 256 value
  FB = constrain(FB, -255, 255);

  // create character array with left modifier, right modifier, and FB amplitude
  char radiopacket[3] = {left, right, FB};

  // send character array using RF transceiver and wait until sent
  rfTransceiver.send((uint8_t *)radiopacket, sizeof(radiopacket));
  rfTransceiver.waitPacketSent();

  unsigned long startTime = millis();
  // create while loop to wait to receive velocity/distance data from peripheral Arduino
  while (1) {
    // if transceiver receives message, unpack message into velocity and distance variables
    if (rfTransceiver.available()) {
      uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rfTransceiver.recv(buf, &len)) {
        buf[len] = 0;
        float velo = buf[0];
        float dist = buf[1];
        // clear display and reset cursor/text size
        display.clearDisplay();
        display.setCursor(0, 0);
        display.setTextSize(1);
        // send the velocity/distance data to the OLED and display it
        display.print("Total Dist.: ");
        display.print(dist);
        display.println(" m");
        display.setCursor(0, 15);
        display.print("Velocity: ");
        display.setTextSize(2);
        display.print(velo);
        display.setCursor(0, 25);
        display.setTextSize(1);
        display.println("(cm/s)");
        display.display();
        // exit while loop if data message was received and displayed properly
        break;

        // if controller is in the while loop for more than 100 ms, it's because peripheral Arduino is also waiting in a while loop to receive
        // if that's the case, we want to break the loop somewhere and continue to control the car, so this if statement exits the while loop after 100 ms
        // and sends joystick data to peripheral Arduino to keep crosstalk active
        if (millis() - startTime > 100) {
          break;
        }
      }
    }
  }
}
