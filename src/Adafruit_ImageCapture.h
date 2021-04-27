/*!
 * @file Adafruit_ImageCapture.h
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

#include <stdint.h> // uint types, etc.

// Must include ALL arch headers here. Each has #ifdefs to avoid mayhem.
#include "arch/rp2040.h"
#include "arch/samd51.h"

/** Status codes returned by some functions */
typedef enum {
  ICAP_STATUS_OK = 0,         ///< Success
  ICAP_STATUS_ERR_MALLOC,     ///< malloc() call failed
  ICAP_STATUS_ERR_PERIPHERAL, ///< Peripheral (e.g. timer) not found
} ICAP_status;

/** Supported color formats */
typedef enum {
  ICAP_COLOR_RGB = 0, ///< RGB565 big-endian
  ICAP_COLOR_YUV,     ///< YUV/YCbCr 4:2:2 big-endian
} ICAP_colorspace;

/*!
    @brief  Class encapsulating common image sensor functionality.
*/
class Adafruit_ImageCapture {
public:
  /*!
    @brief  Constructor for Adafruit_ImageCapture class.
  */
  Adafruit_ImageCapture();
  ~Adafruit_ImageCapture(); // Destructor

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

private:
  uint16_t *buffer[2];  ///< Camera buffer(s) allocated by lib
  uint32_t buffer_size; ///< Size of camera buffer, in bytes
  uint16_t _width;      ///< Current settings width in pixels
  uint16_t _height;     ///< Current settings height in pixels
#if 0
  OV7670_pins pins;          ///< Camera physical connections
  OV7670_arch arch;          ///< Architecture-specific peripheral info
  OV7670_colorspace space;   ///< RGB or YUV colorspace
  const uint8_t i2c_address; ///< I2C address
  const bool arch_defaults;  ///< If set, ignore arch struct, use defaults
  uint8_t active_buffer;     ///< If double buffering, which index is capturing
#endif
};
