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

#include "image_ops.h"
#include <stdint.h> // uint types, etc.
#include <stdlib.h> // NULL, etc.

/** Status codes returned by some functions */
typedef enum {
  ICAP_STATUS_OK = 0,         ///< Success
  ICAP_STATUS_ERR_MALLOC,     ///< malloc() call failed
  ICAP_STATUS_ERR_PERIPHERAL, ///< Peripheral (e.g. timer) not found
} iCap_status;

// Must include ALL arch headers here. Each has #ifdefs to avoid mayhem.
// Do this here, after the iCap_status typedef, as functions declared in
// these headers may rely on that.
#include "arch/rp2040.h"
#include "arch/samd51.h"

/** Supported color formats */
typedef enum {
  ICAP_COLOR_RGB565 = 0, ///< RGB565 big-endian
  ICAP_COLOR_YUV,        ///< YUV/YCbCr 4:2:2 big-endian
} iCap_colorspace;

/** Buffer reallocation behaviors when changing captured image size */
typedef enum {
  ICAP_REALLOC_NONE = 0, ///< No realloc, error if new size > current buffer
  ICAP_REALLOC_CHANGE,   ///< Reallocate image buffer if size changes
  ICAP_REALLOC_LARGER,   ///< Realloc only if new size is larger
} iCap_realloc;

/*!
    @brief  Class encapsulating common image sensor functionality.
*/
class Adafruit_ImageCapture {
public:
  /*!
    @brief  Constructor for Adafruit_ImageCapture class.
    @param  arch  Pointer to structure containing architecture-specific
                  settings. For example, on SAMD51, this structure
                  includes a pointer to a timer peripheral's base address,
                  used to generate the xclk signal. The structure is
                  always of type iCap_arch, but the specific elements
                  within will vary with each supported architecture.
                  Can be NULL if not used/needed.
  */
  Adafruit_ImageCapture(iCap_arch *arch = NULL);
  ~Adafruit_ImageCapture(); // Destructor

  /*!
    @brief   Allocate and initialize resources behind an
             Adafruit_ImageCapture instance.
    @return  Status code. ICAP_STATUS_OK on successful init.
  */
  iCap_status begin(void);

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
    @brief   Get address of image buffer being used by camera.
    @return  uint16_t pointer to image data in RGB565 format.
  */
  uint16_t *getBuffer(void) { return buffer[0]; }

protected:
  uint16_t *buffer[2] = {NULL, NULL}; ///< Camera buffer(s) allocated by lib
  uint32_t buffer_size = 0;           ///< Size of camera buffer, in bytes
  uint16_t _width = 0;                ///< Current settings width in pixels
  uint16_t _height = 0;               ///< Current settings height in pixels
  iCap_arch *arch = NULL;             ///< Device-specific data, if needed
#if 0
  uint8_t active_buffer;     ///< If double buffering, which index is capturing
#endif
};
