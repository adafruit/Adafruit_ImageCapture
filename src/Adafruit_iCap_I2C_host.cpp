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
                          i2cAddress(addr), wire(w), i2cSpeed(s) {
}

Adafruit_iCap_peripheral::~Adafruit_iCap_peripheral() {
}

// Read 'len' bytes into i2cBuf, limited to i2cBuf size and pending I2C
// bytes available. Returns number of bytes actually read.
int Adafruit_iCap_peripheral::i2cRead(int len) {
  // Limit read to buffer size and to available incoming bytes
  if (len > sizeof i2cBuf) len = sizeof i2cBuf;
  int i = wire->available();
  if (len > i) len = i;
  for (i=0; i<len; i++) {
    i2cBuf[i] = wire->read();
  }
  return len;
}

void Adafruit_iCap_peripheral::begin(void) {
  wire->begin();
  wire->setClock(i2cSpeed);

  // Negotiate max I2C transfer size between host and camera peripheral.
  // i2cMaxLen was previously initialized to a small value known valid for
  // AVR etc., but might get upgraded on successful negotiation. Peripheral
  // performs similar operation, so both have same value at end.
  i2cBuf[0] = ICAP_CMD_BUFSIZ;              // Plz tell your I2C limit,
  i2cBuf[1] =  sizeof i2cBuf        & 0xFF; // here's mine (32 bits)
  i2cBuf[2] = (sizeof i2cBuf >>  8) & 0xFF;
  i2cBuf[3] = (sizeof i2cBuf >> 16) & 0xFF;
  i2cBuf[4] = (sizeof i2cBuf >> 24) & 0xFF;
  wire->beginTransmission(i2cAddress);
  wire->write(i2cBuf, 5);
  wire->endTransmission();
  wire->requestFrom(i2cAddress, (uint8_t)4);
  if (i2cRead(4) == 4) { // 32-bit response
    uint32_t response = (uint32_t)i2cBuf[0]        |
                       ((uint32_t)i2cBuf[1] <<  8) |
                       ((uint32_t)i2cBuf[2] << 16) |
                       ((uint32_t)i2cBuf[3] << 24);
    i2cMaxLen = min(sizeof i2cBuf, response); // Use smaller of both
  }
}

int Adafruit_iCap_peripheral::cameraStart(uint8_t mode, uint8_t size,
                                          float fps, uint32_t timeout_ms) {
  // Issue camera start command
  i2cBuf[0] = ICAP_CMD_START;
  i2cBuf[1] = mode;
  i2cBuf[2] = size;
  i2cBuf[3] = (int)fps;
  wire->beginTransmission(i2cAddress);
  wire->write(i2cBuf, 4);
  wire->endTransmission();

  // Poll until camera ready or timeout has elapsed
  uint32_t startTime = millis();
  do {
    delay(100); // Don't hit it too fast, else trouble
    if (getState() == CAM_ON) {
      _width = 640 >> (int)size;
      _height = 480 >> (int)size;
      return ICAP_STATUS_OK;
    }
  } while((millis() - startTime) < timeout_ms);

  _width = 0;
  _height = 0;
  return ICAP_STATUS_ERR_TIMEOUT;
}

iCap_state Adafruit_iCap_peripheral::getState(void) {
  i2cBuf[0] = ICAP_CMD_STATE;
  wire->beginTransmission(i2cAddress);
  wire->write(i2cBuf, 1);
  wire->endTransmission();
  wire->requestFrom(i2cAddress, (uint8_t)1);
  return (wire->available() >= 1) ? (iCap_state)wire->read() : CAM_UNKNOWN;
}

int Adafruit_iCap_peripheral::getReturnValue(void) {
  i2cBuf[0] = ICAP_CMD_RETURN; // Request return value
  wire->beginTransmission(i2cAddress);
  wire->write(i2cBuf, 1);
  wire->endTransmission();
  wire->requestFrom(i2cAddress, (uint8_t)4);
  int response = 0;
  if (i2cRead(4) == 4) { // 32-bit response
    response = (uint32_t)i2cBuf[0] | ((uint32_t)i2cBuf[1] << 8) |
      ((uint32_t)i2cBuf[2] << 16) | ((uint32_t)i2cBuf[3] << 24);
  }
  return response;

}

int Adafruit_iCap_peripheral::readRegister(uint8_t reg) {
  i2cBuf[0] = ICAP_CMD_READ_REG; // Read register
  i2cBuf[1] = reg;
  wire->beginTransmission(i2cAddress);
  wire->write(i2cBuf, 2);
  wire->endTransmission();
  wire->requestFrom(i2cAddress, (uint8_t)1);
  return (wire->available() >= 1) ? wire->read() : -1;
}

void Adafruit_iCap_peripheral::writeRegister(uint8_t reg, uint8_t val) {
  i2cBuf[0] = ICAP_CMD_WRITE_REG;
  i2cBuf[1] = reg;
  i2cBuf[2] = 1; // Single register for now
  i2cBuf[3] = val;
  wire->beginTransmission(i2cAddress);
  wire->write(i2cBuf, 4);
  wire->endTransmission();
}

uint32_t Adafruit_iCap_peripheral::capture() {
  i2cBuf[0] = ICAP_CMD_CAPTURE;
  wire->beginTransmission(i2cAddress);
  wire->write(i2cBuf, 1);
  wire->endTransmission();
  delay(100);
  wire->requestFrom(i2cAddress, (uint8_t)4);
  uint32_t response = 0;
  if (i2cRead(4) == 4) { // 32-bit response
    response = (uint32_t)i2cBuf[0] | ((uint32_t)i2cBuf[1] << 8) |
      ((uint32_t)i2cBuf[2] << 16) | ((uint32_t)i2cBuf[3] << 24);
  }
Serial.print("response = ");
Serial.println(response);
Serial.println("Polling...");

  // Start polling state until ready
  uint32_t startTime = millis();
  uint32_t timeout_ms = 3000;
  uint32_t status = 0;
  do {
    i2cBuf[0] = ICAP_CMD_STATE; // Poll camera state
    delay(100); // Don't hit it too fast, else trouble
    wire->beginTransmission(i2cAddress);
    wire->write(i2cBuf, 1);
    wire->endTransmission();
    wire->requestFrom(i2cAddress, (uint8_t)1);
    status = (wire->available() >= 1) ? wire->read() : 0;
  } while((status != CAM_PAUSED) && ((millis() - startTime) < timeout_ms));
  // TO DO: make this return 0 on timeout or other issue
Serial.println("OK");
// I think there's a race condition where periph is already returning
// data because 'capturing' var is set too soon (needs to wait until
// byte count has been transmitted to host).

  return response;
}

// This is awful similar to i2cRead()
uint8_t *Adafruit_iCap_peripheral::getData(int len) {
  wire->requestFrom(i2cAddress, len);
  int avail = wire->available();
  if (avail >= 1) {
    len = min(len, avail);
    for(int i=0; i<len; i++) {
      i2cBuf[i] = wire->read();
    }
  }
  return i2cBuf;
}

void Adafruit_iCap_peripheral::resume() {
  i2cBuf[0] = ICAP_CMD_RESUME;
  wire->beginTransmission(i2cAddress);
  wire->write(i2cBuf, 1);
  wire->endTransmission();
}
