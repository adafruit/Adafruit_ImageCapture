// This runs on a Metro Mini and pings OV7670-equipped Pico board
// running the cameratest_i2c_OV7670_Pico sketch.
// (Both boards run concurrently, talking over I2C)

/*
NOTES TO SELF:
Start librarifying these I2C transactions, because they're
gonna get awkward FAST. Prob 2 headers, one for cam I2C host &
peripheral (enumerates things like the connand bytes), and one
specifically for host (to encapsulate the ugly code below into
easy functions).

Consider changing the cam lib so PWM can be independently
initialized before starting up the whole camera capture deal.
This would allow registers to be read & written (camera needs
clock input via PWM to run its own I2C) and might allow things
like auto camera make & model detection later.
*/

#include <Wire.h>
#include "Adafruit_iCap_I2C_host.h"

#define CAM_ADDR 0x55

uint8_t camBuf[10];

void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("HOST BOARD STARTED");

  Wire.begin();
  Wire.setClock(100000);

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

  // Wait for camera ready
  // (to do: should add timeout here)
  int status = 0;
  uint32_t startTime = millis();
  do {
    camBuf[0] = 0x11; // Poll camera ready state
    delay(100); // Don't hit it too fast, else trouble
    Wire.beginTransmission(CAM_ADDR);
    Wire.write(camBuf, 1);
    Wire.endTransmission();
    Wire.requestFrom(CAM_ADDR, 1);
    if (Wire.available() >= 1) {
      status = Wire.read();
    }
  } while((status < 2) && ((millis() - startTime) < 3000));
  if(status < 2) {
    Serial.println("Camera failed to start");
    for(;;);
  }

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
