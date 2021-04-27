#include <Adafruit_iCap_OV7670.h>

OV7670_pins pins = {
  .enable = -1, // Also called PWDN, or set to -1 and tie to GND
  .reset  = 14, // Cam reset, or set to -1 and tie to 3.3V
  .xclk   = 13, // MCU clock out / cam clock in
  .pclk   = 10, // Cam clock out / MCU clock in
  .vsync  = 11, // Also called DEN1
  .hsync  = 12, // Also called DEN2
  .data   = {2, 3, 4, 5, 6, 7, 8, 9}, // Camera parallel data out
  .sda    = 20, // I2C data
  .scl    = 21, // I2C clock
};
Adafruit_iCap_OV7670 cam(pins, Wire);

void setup() {
  Serial.begin(115200);
  //while(!Serial);

  // pass video attributes to begin (size, fps, color mode)
  cam.begin();
}

void loop() {
}
