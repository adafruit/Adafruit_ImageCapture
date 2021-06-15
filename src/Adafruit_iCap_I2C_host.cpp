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

  _width = 640 >> (int)size;
  _height = 480 >> (int)size;

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

void Adafruit_iCap_peripheral::writeRegister(uint8_t reg, uint8_t val) {
  i2c_buffer[0] = ICAP_CMD_WRITE_REG;
  i2c_buffer[1] = reg;
  i2c_buffer[2] = val;
  wire->beginTransmission(i2c_address);
  wire->write(i2c_buffer, 3);
  wire->endTransmission();
}

uint32_t Adafruit_iCap_peripheral::capture() {
  i2c_buffer[0] = ICAP_CMD_CAPTURE;
  wire->beginTransmission(i2c_address);
  wire->write(i2c_buffer, 1);
  wire->endTransmission();
  //delay(100);
  wire->requestFrom(i2c_address, (uint8_t)4);
  if (wire->available() >= 4) {
    for(int i=0; i<4; i++) i2c_buffer[i] = wire->read();
  }
  return ((uint32_t)i2c_buffer[0] | ((uint32_t)i2c_buffer[1] << 8) |
          ((uint32_t)i2c_buffer[2] << 16) | ((uint32_t)i2c_buffer[3] << 24));
}

uint8_t *Adafruit_iCap_peripheral::getData(uint8_t len) {
  len = min(len, BUFFER_LENGTH - 1); // Limit data to I2C buffer
  i2c_buffer[0] = ICAP_CMD_GET_DATA; // Plz send...
  i2c_buffer[1] = len;               // ...up to this many
  wire->beginTransmission(i2c_address);
  wire->write(i2c_buffer, 2);
  wire->endTransmission();
  //delay(100);
// Was hanging here.
// Issue is that host can't request more bytes than remain --
// the I2C transfer can't "clip" the transfer size after the request
// (i.e. if host requests 30 bytes, can't send 20, it HANGS).
// One option might be to pad the transfer with extra bytes to
// fill the requested size. Other would be for an initial negotiation
// phase where the two devices decide on a maximum I2C transfer size
// that both can work with.
  wire->requestFrom(i2c_address, len + 1); // Length byte + data
  if (wire->available() >= 1) {
    len = wire->read();                // Length byte
    len = min(len, wire->available()); // Remaining bytes
    i2c_buffer[0] = len;
    for(int i=1; i<=len; i++) {
      i2c_buffer[i] = wire->read();
    }
  }
  return i2c_buffer;
}

void Adafruit_iCap_peripheral::resume() {
  i2c_buffer[0] = ICAP_CMD_RESUME;
  wire->beginTransmission(i2c_address);
  wire->write(i2c_buffer, 1);
  wire->endTransmission();
}
