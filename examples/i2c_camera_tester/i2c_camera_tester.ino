// This runs on a Metro Mini and pings OV7670-equipped Pico board
// running the cameratest_i2c_OV7670_Pico sketch.
// (Both boards run concurrently, talking over I2C)

#include <Wire.h>

#define CAM_ADDR 0x55

uint8_t camBuf[10];

void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("HOST BOARD STARTED");

  Wire.begin();

  // Start camera, check response

  // TO DO: as this gets librarified, will want all these constants
  // accessible to the host-side code, but without all the hardware-
  // dependent bits (e.g. camera DMA and stuff) because host side needs
  // to run on smaller boards (e.g. Metro Mini in testing). And/or
  // convenience functions will wrap around a lot of this wordy stuff...

  camBuf[0] = 0x10; // Start camera
  camBuf[1] = 0;    // ICAP_COLOR_RGB565
  camBuf[2] = 2;    // OV7670_SIZE_DIV4
  camBuf[3] = 30;   // FPS
  Wire.beginTransmission(CAM_ADDR);
  Wire.write(camBuf, 4);
  Wire.endTransmission();

  camBuf[0] = 0x20; // Request status
  Wire.beginTransmission(CAM_ADDR);
  Wire.write(camBuf, 1);
  Wire.endTransmission();
  Wire.requestFrom(CAM_ADDR, 4);
  if (Wire.available() >= 4) {
    Serial.print("Status: ");
    for(int i=0; i<4; i++) Serial.print(Wire.read(), HEX);
    Serial.println();
  }

  // Poll the PID and VER registers to see if camera's working

  camBuf[0] = 0x30; // Read register
  camBuf[1] = 0x0A; // OV7670_REG_PID, should return 0x76
  Wire.beginTransmission(CAM_ADDR);
  Wire.write(camBuf, 2);
  Wire.endTransmission();
  Wire.requestFrom(CAM_ADDR, 1);
  if (Wire.available() >= 1) {
    Serial.print("PID: ");
    Serial.println(Wire.read(), HEX); // Expecting 0x76
  }

  camBuf[0] = 0x30; // Read register
  camBuf[1] = 0x0B; // OV7670_REG_VER, should return 0x73
  Wire.beginTransmission(CAM_ADDR);
  Wire.write(camBuf, 2);
  Wire.endTransmission();
  Wire.requestFrom(CAM_ADDR, 1);
  if (Wire.available() >= 1) {
    Serial.print("VER: ");
    Serial.println(Wire.read(), HEX); // Expecting 0x73
  }
}

void loop() {
}
