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

#include <stdlib.h> // NULL, etc.
#include <stdint.h> // uint types, etc.

/** Status codes returned by some functions */
typedef enum {
  ICAP_STATUS_OK = 0,         ///< Success
  ICAP_STATUS_ERR_MALLOC,     ///< malloc() call failed
  ICAP_STATUS_ERR_PERIPHERAL, ///< Peripheral (e.g. timer) not found
} ICAP_status;

// Must include ALL arch headers here. Each has #ifdefs to avoid mayhem.
// Do this here, after the ICAP_status typedef, as functions declared in
// these headers may rely on that.
#include "arch/rp2040.h"
#include "arch/samd51.h"

/** Supported color formats */
typedef enum {
  ICAP_COLOR_RGB565 = 0, ///< RGB565 big-endian
  ICAP_COLOR_YUV,        ///< YUV/YCbCr 4:2:2 big-endian
} ICAP_colorspace;

/** Buffer reallocation behaviors when changing captured image size */
typedef enum {
  ICAP_REALLOC_NONE = 0, ///< No realloc, error if new size > current buffer
  ICAP_REALLOC_CHANGE,   ///< Reallocate image buffer if size changes
  ICAP_REALLOC_LARGER,   ///< Realloc only if new size is larger
} ICAP_realloc;

/*!
    @brief  Class encapsulating common image sensor functionality.
*/
class Adafruit_ImageCapture {
public:
  /*!
    @brief  Constructor for Adafruit_ImageCapture class.
  */
  Adafruit_ImageCapture(iCap_arch *arch = NULL);
  ~Adafruit_ImageCapture(); // Destructor

  ICAP_status begin(void);

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

// Non-class functions

// Convert Y (brightness) component YUV image in RAM to RGB565 big-
// endian format for preview on TFT display. Data is overwritten in-place,
// Y is truncated and UV elements are lost. No practical use outside TFT
// preview. If you need actual grayscale 0-255 data, just access the low
// byte of each 16-bit YUV pixel.
void iCam_Y2RGB565(uint16_t *ptr, uint32_t len);
