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
#include <stdlib.h> // NULL, etc.

/** Status codes returned by some functions */
typedef enum {
  ICAP_STATUS_OK = 0,         ///< Success
  ICAP_STATUS_ERR_MALLOC,     ///< malloc() or realloc() call failed
  ICAP_STATUS_ERR_PERIPHERAL, ///< Peripheral (e.g. timer) not found
  ICAP_STATUS_ERR_PINS,       ///< Pin config doesn't align with peripheral(s)
  ICAP_STATUS_ERR_TIMEOUT,    ///< Function didn't complete in expected time
} iCap_status;

// Must include ALL arch headers here (each has #ifdef checks for specific
// architectures). Do this here, after the iCap_status typedef, as functions
// declared in these headers may rely on that.
#include "arch/rp2040.h"
#include "arch/samd51.h"

// If ICAP_XCLK_HZ got defined in one of the arch headers, that indicates
// the host device can directly interface with a camera, and the whole
// Adafruit_ImageCapture class is available. If not (e.g. AVR, SAMD21),
// then only I2C peripheral camera support is provided (via
// Adafruit_iCap_I2C_host.*), though some of the enums and defines in
// these other headers may still be useful (and not likely to induce
// bloat if they're not referenced).
#if defined(ICAP_XCLK_HZ)
#define ICAP_FULL_SUPPORT ///< Local device and remote I2C supported
#endif

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

#if defined(ICAP_FULL_SUPPORT)

/*!
    @brief  Class encapsulating common image sensor functionality.
*/
class Adafruit_ImageCapture {
public:
  /*!
    @brief  Constructor for Adafruit_ImageCapture class. This constructor
            is never invoked directly by user code. Instead, a subclass is
            used, which implicitly calls this constructor...hence no
            defaults here.
    @param  arch      Pointer to structure containing architecture-specific
                      settings. For example, on SAMD51, this structure
                      includes a pointer to a timer peripheral's base
                      address, used to generate the xclk signal. Structure
                      is always of type iCap_arch, but the specific elements
                      within will vary with each architecture. Can be NULL
                      if not used/needed.
    @param  pbuf      Preallocated buffer for captured pixel data, or NULL
                      for library to allocate as needed when a camera
                      resolution is selected.
    @param  pbufsize  Size of passed-in buffer (or 0 if NULL).
  */
  Adafruit_ImageCapture(iCap_arch *arch, uint16_t *pbuf, uint32_t pbufsize);
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

  /*!
    @brief   Get address of image buffer being used by camera.
    @return  uint16_t pointer to image data in RGB565 format.
  */
  uint16_t *getBuffer(void) { return buffer[0]; }

  /*!
    @brief  Produces a negative image. This is a postprocessing effect,
            not in-camera, and must be applied to frame(s) manually.
            Image in memory will be overwritten.
  */
  void image_negative(void);

  /*!
    @brief  Decimate an image to only it's min/max values (ostensibly
            "black and white," but works on color channels separately
            so that's not strictly the case). This is a postprocessing
            effect, not in-camera, and must be applied to frame(s) manually.
            Image in memory will be overwritten.
    @param  threshold  Threshold level, 0-255; pixel brightnesses at or
                       above this level are set to the maximum, below this
                       level are set to the minimum. Input value is scaled
                       to accommodate the lower color fidelity of the RGB
                       colorspace -- use 0 to 255, not 0 to 31 or 63.
  */
  void image_threshold(uint8_t threshold = 128);

  /*!
    @brief  Decimate an image to a limited number of brightness levels or
            steps. This is a postprocessing effect, not in-camera, and must
            be applied to frame(s) manually. Image in memory will be
            overwritten.
    @param  levels  Number of brightness levels -- 2 to 32 for RGB
                    colorspace, 2 to 255 for YUV.
  */
  void image_posterize(uint8_t levels = 4);

  /*!
    @brief  Mosaic or "shower door effect," downsamples an image into
            rectangular tiles, each tile's color being the average of all
            source image pixels within that tile's area. This is a
            postprocessing effect, not in-camera, and must be applied to
            frame(s) manually. Image in memory will be overwritten. If
            image size does not divide equally by tile size, fractional
            tiles will always be along the right and/or bottom edge(s);
            top left corner is always a full tile.
            YUV colorspace is not currently supported.
    @param  tile_width   Tile width in pixels (1 to 255)
    @param  tile_height  Tile height in pixels (1 to 255)
  */
  void image_mosaic(uint8_t tile_width = 8, uint8_t tile_height = 8);

  /*!
    @brief  3x3 pixel median filter, reduces visual noise in image.
            This is a postprocessing effect, not in-camera, and must be
            applied to frame(s) manually. Image in memory will be
            overwritten. YUV colorspace is not currently supported.
  */
  void image_median(void);

  /*!
    @brief  Edge detection filter.
            This is a postprocessing effect, not in-camera, and must be
            applied to frame(s) manually. Image in memory will be
            overwritten. YUV colorspace is not currently supported.
    @param  sensitivity  Smaller value = more sensitive to edge changes.
  */
  void image_edges(uint8_t sensitivity = 7);

  /*!
    @brief  Convert Y (brightness) component YUV image in RAM to RGB565
            big-endian format for preview on TFT display. Camera buffer is
            overwritten in-place, Y is truncated and UV elements are lost.
            No practical use outside TFT preview. If you need actual
            grayscale 0-255 data, just access the low byte of each 16-bit
            YUV pixel.
  */
  void Y2RGB565(void);

protected:
  uint16_t *pixbuf[3];        ///< Frame pointers (up to 3) into pixel buffer
  uint32_t pixbuf_size = 0;   ///< Full size of pixbuf, in bytes
  uint8_t bufmode;            ///< 1-3 = single-, double-, triple-buffered
  bool pixbuf_allocable;      ///< Internally allocated vs static buffer
  uint16_t _width = 0;        ///< Current settings width in pixels
  uint16_t _height = 0;       ///< Current settings height in pixels
  iCap_arch *arch = NULL;     ///< Device-specific data, if needed
  iCap_colorspace colorspace; ///< Colorspace passed to begin()

  iCap_status setSize(uint16_t width, uint16_t height, uint8_t nbuf=1,
                      iCap_realloc allo=ICAP_REALLOC_CHANGE);
};

#endif // end ICAP_FULL_SUPPORT
