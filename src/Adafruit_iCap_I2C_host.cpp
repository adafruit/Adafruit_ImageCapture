/*!
 * @file Adafruit_iCap_I2C.h
 *
 * This is documentation for Adafruit's Image Capture library for Arduino,
 * providing drivers for image sensors such as OV7670 and OV2640.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Phil "PaintYourDragon" Burgess for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 */

#include "Adafruit_iCap_I2C_host.h"
#include <Arduino.h>

Adafruit_iCap_peripheral::Adafruit_iCap_peripheral(uint8_t addr, TwoWire *w,
                                                   uint32_t s) :
                          i2c_address(addr), wire(w), i2c_speed(s) {
}

Adafruit_iCap_peripheral::~Adafruit_iCap_peripheral() {
}

void Adafruit_iCap_peripheral::begin(void) {
  wire->begin();
  wire->setClock(i2c_speed);
}

int Adafruit_iCap_peripheral::cameraStart(uint8_t mode, uint8_t size,
                                          float fps, uint32_t timeout_ms) {
  i2c_buffer[0] = ICAP_CMD_START;
  i2c_buffer[1] = mode;
  i2c_buffer[2] = size;
  i2c_buffer[3] = (int)fps;
  wire->beginTransmission(i2c_address);
  wire->write(i2c_buffer, 4);
  wire->endTransmission();

  // Wait for camera ready
  int status = 0;
  uint32_t startTime = millis();
  do {
    i2c_buffer[0] = ICAP_CMD_READY; // Poll camera ready state
    delay(100); // Don't hit it too fast, else trouble
    wire->beginTransmission(i2c_address);
    wire->write(i2c_buffer, 1);
    wire->endTransmission();
    wire->requestFrom(i2c_address, (uint8_t)1);
    status = (wire->available() >= 1) ? wire->read() : 0;
  } while((status < 2) && ((millis() - startTime) < timeout_ms));

  return (status < 2) ? ICAP_STATUS_ERR_TIMEOUT : ICAP_STATUS_OK;
}

int Adafruit_iCap_peripheral::status(void) {
  i2c_buffer[0] = ICAP_CMD_STATUS; // Request status
  wire->beginTransmission(i2c_address);
  wire->write(i2c_buffer, 1);
  wire->endTransmission();
  wire->requestFrom(i2c_address, (uint8_t)4);
  if (wire->available() >= 4) {
    for(int i=0; i<4; i++) i2c_buffer[i] = wire->read();
  }
  return (i2c_buffer[0] | (i2c_buffer[1] << 8) | (i2c_buffer[2] << 16) |
          (i2c_buffer[3] << 24));

}

int Adafruit_iCap_peripheral::readRegister(uint8_t reg) {
  i2c_buffer[0] = ICAP_CMD_READ_REG; // Read register
  i2c_buffer[1] = reg;
  wire->beginTransmission(i2c_address);
  wire->write(i2c_buffer, 2);
  wire->endTransmission();
  wire->requestFrom(i2c_address, (uint8_t)1);
  return (wire->available() >= 1) ? wire->read() : -1;
}
