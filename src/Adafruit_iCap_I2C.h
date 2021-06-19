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

#include "Adafruit_ImageCapture.h"

#define ICAP_DEFAULT_ADDRESS 0x55 //< I2C peripheral address

/** I2C command bytes passed from host to peripheral, MUST remain in sync */
typedef enum {
  ICAP_CMD_BUFSIZ = 0, ///< Negotiate maximum I2C transfer size
  ICAP_CMD_ID,         ///< Identify make/model of camera
  ICAP_CMD_STATE,      ///< Poll current camera state
  ICAP_CMD_READ_REG,   ///< Read register from camera
  ICAP_CMD_WRITE_REG,  ///< Write register(s) to camera
  ICAP_CMD_SETUP,      ///< Specify capture parameters
  ICAP_CMD_CAPTURE,    ///< Pause camera, hold one frame for transfer
  ICAP_CMD_RESUME,     ///< Resume background capture after transfer
  ICAP_CMD_RETURN,     ///< Get return val of last periph-side camera func call
} iCap_i2c_command;

/** Camera state used in I2C operation */
typedef enum {     // Camera states:
  CAM_UNKNOWN = 0, // Camera State could not be polled
  CAM_OFF,         // Camera not yet initialized
  CAM_ON,          // Camera is "on" and running
  CAM_REQ_CONFIG,  // Request to start or change camera configuration
  CAM_REQ_PAUSE,   // Request to pause camera DMA for still
  CAM_PAUSED,      // Camera currently paused
  CAM_REQ_RESUME,  // Request to resume camera DMA (goes to CAM_ON)
} iCap_state;

