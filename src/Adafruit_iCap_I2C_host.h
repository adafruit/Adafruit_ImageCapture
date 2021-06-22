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
                           TwoWire *w = &Wire, uint32_t s = 400000UL);
  ~Adafruit_iCap_peripheral(); // Destructor

  void begin(void);
  void begin(uint8_t size, uint8_t space, float fps = 30.0,
             uint32_t timeout_ms = 3000);
  int getReturnValue();
  iCap_state getState();
  uint8_t *getData(int len);
  int config(uint8_t size, uint8_t space, float fps = 30.0,
             uint32_t timeout_ms = 3000);
  int readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t val);
  uint32_t capture();
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

  /*!
    @brief   Get maximum I2C transfer size that was negotiated between
             host and peripheral during begin().
    @return  Maximum single transfer size, in bytes.
  */
  uint32_t maxTransferSize(void) { return i2cMaxLen; }

  /*!
    @brief   Get address of I2C receive buffer. When capturing image,
             calls to i2cRead() store data here, which can then be passed
             to display or other code.
    @return  Pointer to receive buffer (BUFFER_LENGTH bytes).
  */
  uint8_t *getBuffer(void) { return i2cBuf; };

  /*!
    @brief   Read data from I2C peripheral camera, up to a maximum length
             (actual read may be less, if it exceeds the device's I2C buffer
             or the available amount of data waiting). Host code can use
             this when reading image from the camera (internally it's used
             for general I2C receive operations).
    @param   len  Number of bytes to read (should not exceed host or remote
                  I2C buffer size).
    @return  Number of bytes actually read.
  */
  int i2cRead(int len);

  // Write data to I2C peripheral camera from library's transfer buffer.
  int i2cWrite(int len);

protected:
  TwoWire *wire;                 //< Pointer to I2C periph (e.g. &Wire)
  uint32_t i2cSpeed;             //< I2C data rate, bps (e.g. 100000)
  uint8_t i2cAddress;            //< I2C peripheral address
  uint8_t i2cBuf[BUFFER_LENGTH]; //< TX/RX buffer, size from Wire lib
  uint32_t i2cMaxLen = 32;       //< Actual negotiated I2C xfer limit
  uint16_t _width = 0;           //< Current settings width in pixels
  uint16_t _height = 0;          //< Current settings height in pixels
  iCap_colorspace colorspace;    //< Colorspace passed to begin()
};

