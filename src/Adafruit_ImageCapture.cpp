/*!
 * @file Adafruit_ImageCapture.cpp
 *
 * @mainpage Adafruit Image Capture Library
 *
 * @section intro_sec Introduction
 *
 * This is documentation for Adafruit's Image Capture library for Arduino,
 * providing drivers for image sensors such as OV7670 and OV2640.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on
 * <a href="https://github.com/adafruit/Adafruit_ZeroDMA">
 * Adafruit_ZeroDMA</a> being present on your system. Please make sure you
 * have installed the latest version before using this library.
 *
 * @section author Author
 *
 * Written by Phil "PaintYourDragon" Burgess for Adafruit Industries.
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 */

#include <Adafruit_ImageCapture.h>

Adafruit_ImageCapture::Adafruit_ImageCapture(iCap_arch *arch) : arch(arch) {}

Adafruit_ImageCapture::~Adafruit_ImageCapture() {}

// Reformat YUV gray component to RGB565 for TFT preview.
// Big-endian in and out.
void iCam_Y2RGB565(uint16_t *ptr, uint32_t len) {
  while (len--) {
    uint8_t y = *ptr & 0xFF; // Y (brightness) component of YUV
    uint16_t rgb = ((y >> 3) * 0x801) | ((y & 0xFC) << 3); // to RGB565
    *ptr++ = __builtin_bswap16(rgb); // Big-endianify RGB565 for TFT
  }
}
