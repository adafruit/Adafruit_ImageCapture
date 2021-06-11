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
#include "Adafruit_iCap_I2C.h"

class Adafruit_iCap_peripheral {
public:
  Adafruit_iCap_peripheral();
  ~Adafruit_iCap_peripheral(); // Destructor
  void begin(uint8_t address = ICAP_DEFAULT_ADDRESS);
protected:
  uint8_t i2c_address;
};

