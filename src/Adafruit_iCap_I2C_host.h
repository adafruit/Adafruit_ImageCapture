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

#pragma once

#include <stdint.h>
#include <Wire.h>
#include "Adafruit_iCap_I2C.h"
#if !defined(BUFFER_LENGTH)
#define BUFFER_LENGTH 256
#endif

class Adafruit_iCap_peripheral {
public:
  Adafruit_iCap_peripheral(uint8_t addr = ICAP_DEFAULT_ADDRESS,
                           TwoWire *w = &Wire, uint32_t s = 100000UL);
  ~Adafruit_iCap_peripheral(); // Destructor
  void begin();
  int cameraStart(uint8_t mode, uint8_t size, float fps,
                  uint32_t timeout_ms = 3000);
  int status();
  int readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t val);
  uint32_t capture();
  uint8_t *getData(uint8_t len = 255);
  void resume();
  /*!
    @brief   Get image width of camera's current resolution setting.
    @return  Width in pixels.
  */
  uint16_t width(void) { return _width; }

  /*!
    @brief   Get image height of camera's current resolution setting.
    @return  Height in pixels.
  */
  uint16_t height(void) { return _height; }

protected:
  TwoWire *wire;                     //< Pointer to I2C periph (e.g. &Wire)
  uint32_t i2c_speed;                //< I2C data rate, bps (e.g. 100000)
  uint8_t i2c_buffer[BUFFER_LENGTH]; //< TX/RX buffer, size from Wire lib
  uint8_t i2c_address;               //< I2C peripheral address
  uint16_t _width = 0;               ///< Current settings width in pixels
  uint16_t _height = 0;              ///< Current settings height in pixels
  iCap_colorspace colorspace;        ///< Colorspace passed to begin()
};

