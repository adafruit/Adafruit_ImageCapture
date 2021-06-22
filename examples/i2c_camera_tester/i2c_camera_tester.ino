// This runs on a Metro Mini and pings OV7670-equipped Pico board
// running the cameratest_i2c_OV7670_Pico sketch.
// (Both boards run concurrently, talking over I2C)

#include <Wire.h>
#include "Adafruit_iCap_I2C_host.h"
#include "Adafruit_iCap_OV7670.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

#if !defined(BUFFER_LENGTH) // AVR defines this, otherwise...
#define BUFFER_LENGTH 256   // Max I2C transfer size
#endif

Adafruit_iCap_peripheral cam; // Remote camera on I2C

#define TFT_CS   4
#define TFT_DC   5
#define TFT_RST -1

Adafruit_ST7789 tft(&SPI, TFT_CS, TFT_DC, TFT_RST);

void setup() {
  Serial.begin(115200);
  //while(!Serial);
  Serial.println("HOST BOARD STARTED");

  tft.init(240, 240);
  tft.fillScreen(0);
  tft.println("I2C HOST");
  tft.setRotation(3);

  // Initialize I2C connection to camera, negotiate max transfer size
  // and initialize hardware.
  cam.begin();
  delay(1000);

  // Poll the PID and VER registers to see if camera's working

  Serial.print("PID: ");
  Serial.println(cam.readRegister(OV7670_REG_PID), HEX); // Expecting 0x76
  Serial.print("VER: ");
  Serial.println(cam.readRegister(OV7670_REG_VER), HEX); // Expecting 0x73
}

uint16_t pixelbuf[BUFFER_LENGTH / 2 + 1]; // +1 for possible half-pixel mid transfer
uint8_t *pbuf8 = (uint8_t *)pixelbuf;
int      bytesinbuf = 0;
uint8_t *i2cbuf = (uint8_t *)cam.getBuffer();

OV7670_size sizes[] = { OV7670_SIZE_DIV4, OV7670_SIZE_DIV8, OV7670_SIZE_DIV16 };
uint8_t size_index = 0;

void loop() {

  // Erase rect at prior size before resizing camera
  tft.fillRect((tft.width() - cam.width()) / 2,
               (tft.height() - cam.height()) / 2,
               cam.width(), cam.height(), 0);

// Need reconfig command
  int status = cam.config(sizes[size_index], ICAP_COLOR_RGB565, 30.0);
  if (++size_index >= (sizeof sizes / sizeof sizes[0])) size_index = 0;

  int32_t bytes = cam.capture();
  Serial.print("Expecting ");
  Serial.print(bytes);
  Serial.print(" bytes from camera, maxTransferSize is ");
  Serial.println(cam.maxTransferSize());

  tft.startWrite();
  tft.setAddrWindow((tft.width() - cam.width()) / 2,
                    (tft.height() - cam.height()) / 2,
                    cam.width(), cam.height());

  while(bytes > 0) {
    int len = min(bytes, cam.maxTransferSize());
    len = cam.i2cRead(len);
    // Add new bytes to pixelbuf
    memcpy(&pbuf8[bytesinbuf], i2cbuf, len);
    bytesinbuf += len;
    int pixelsThisPass = bytesinbuf / 2; // 16 bit pixels (any trailing byte is ignored)
    tft.writePixels(pixelbuf, pixelsThisPass, false, true);
    if (bytesinbuf & 1) {               // Trailing byte present?
      pbuf8[0] = pbuf8[bytesinbuf - 1]; // Move it to beginning
      bytesinbuf = 1;
    } else {
      bytesinbuf = 0;
    }
    bytes -= len;
  }
  if (bytesinbuf >= 2) {
    tft.writePixels(pixelbuf, bytesinbuf / 2, false, true);
  }
  tft.endWrite();   // Close out prior transfer

  cam.resume();

  delay(1000);
}
