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

#define ICAP_DEFAULT_ADDRESS 0x55 //< I2C peripheral address

/** I2C command bytes passed from host to peripheral, MUST remain in sync */
typedef enum {
  ICAP_CMD_ID = 0,    ///< Identify make/model of camera
  ICAP_CMD_START,     ///< Start camera capture (continuous in background)
  ICAP_CMD_READY,     ///< Poll whether camera is initialized & ready
  ICAP_CMD_STATUS,    ///< Get status of last camera function call
  ICAP_CMD_READ_REG,  ///< Read register from camera
  ICAP_CMD_WRITE_REG, ///< Write register(s) to camera
  ICAP_CMD_CAPTURE,   ///< Buffer one frame; background capture stops
  ICAP_CMD_GET_DATA,  ///< Retrieve captured data
  ICAP_CMD_RESUME,    ///< Resume background capture
} iCap_i2c_command;
