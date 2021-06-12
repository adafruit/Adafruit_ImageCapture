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
#include "Adafruit_iCap_OV7670.h"

Adafruit_iCap_peripheral cam; // Remote camera on I2C

void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("HOST BOARD STARTED");

  cam.begin();

  // Start camera, check response
  int status = cam.cameraStart(ICAP_COLOR_RGB565, OV7670_SIZE_DIV4, 30.0);
  if(status != ICAP_STATUS_OK) {
    Serial.println("Camera failed to start");
    Serial.print("Status: ");
    Serial.println(cam.status(), HEX);
    for(;;);
  }

  // Poll the PID and VER registers to see if camera's working

  Serial.print("PID: ");
  Serial.println(cam.readRegister(OV7670_REG_PID), HEX); // Expecting 0x76
  Serial.print("VER: ");
  Serial.println(cam.readRegister(OV7670_REG_VER), HEX); // Expecting 0x73
}

void loop() {
#if 0
// This part isn't working yet
  uint32_t bytes = cam.capture();
  Serial.print("Expecting ");
  Serial.print(bytes);
  Serial.println(" from camera");

  while(bytes > 0) {
    uint8_t *data = cam.getData(255);
    for (int i=0; i<255; i++) {
      Serial.print(data[i], HEX);
      Serial.write(' ');
    }
    Serial.println();
    bytes -= 255;
  }

  cam.resume();
#endif
}
