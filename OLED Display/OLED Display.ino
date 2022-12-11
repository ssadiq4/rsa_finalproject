//receiving velocity/distance signal from encoder and reading it on OLED display

#include <SPI.h> 
#include <Wire.h> 
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels 
#define SCREEN_HEIGHT 64 // OLED display height, in pixels 
// Declaration for SSD1306 display connected using software SPI:
#define OLED_PICO   5
#define OLED_CLK    6 
#define OLED_DC     3 
#define OLED_CS     2 
#define OLED_RESET  4 
Adafruit_SSD1306 display(OLED_PICO, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS); 

float velo;
float dist;

void setup() {

  // initialize display and set text style and cursor
  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.setTextColor(WHITE); 
  display.setCursor(10, 10);
  display.setTextSize(2); 

}

void loop() {
  while (1) {
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
        display.println(" cm");

        display.setCursor(30, 30);
        display.print("Velocity: ");
        display.print(velo);
        display.println(" meters per hour");
        display.display();
        
        // wait half a second before making another reading
        delay(500);
      }   
    }
  }

}

