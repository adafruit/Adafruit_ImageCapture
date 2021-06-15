#if defined(ARDUINO_ARCH_RP2040)
/*
OV7670 + Pico RP2040 acting as an I2C peripheral.
No display; I2C host supplies that.
Works with companion i2c_camera_tester sketch.

HARDWARE REQUIRED:
- Raspberry Pi Pico RP2040
- OV7670 camera
- 10K pullups on SDA+SCL pins
*/

#include <Adafruit_ST7789.h>
#include <Wire.h>
#include <Adafruit_iCap_OV7670.h> // Camera library
#include <Adafruit_iCap_I2C.h>
#if !defined(BUFFER_LENGTH)
#define BUFFER_LENGTH 256
#endif

#define TFT_CS  17 // Near SPI0 at south end of board
#define TFT_DC  16
#define TFT_RST -1 // Connect to MCU reset

Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);

// CAMERA CONFIG -----------------------------------------------------------

// Set up arch and pins structures for Pico RP2040.
iCap_arch arch = {
  .pio = pio0,   // Which PIO peripheral to use (pio0 or pio1)
  .bswap = true, // Capture in big-endian order, can go straight to TFT
  // Other elements are set by the library at runtime and should not be
  // specified by user code (will be overwritten).
};
OV7670_pins pins = {
  .enable = -1, // Also called PWDN, or set to -1 and tie to GND
  .reset  = 14, // Cam reset, or set to -1 and tie to 3.3V
  .xclk   = 13, // MCU clock out / cam clock in
  .pclk   = 10, // Cam clock out / MCU clock in
  .vsync  = 11, // Also called DEN1
  .hsync  = 12, // Also called DEN2
  .data   = {2, 3, 4, 5, 6, 7, 8, 9}, // Camera parallel data out
  .sda    = 20, // I2C data
  .scl    = 21, // I2C clock
};

#define CAM_I2C Wire               // I2C to camera
#define CAM_SIZE OV7670_SIZE_DIV4  // QQVGA (160x120 pixels)
#define CAM_MODE ICAP_COLOR_RGB565 // RGB plz

Adafruit_iCap_OV7670 cam(pins, CAM_I2C, &arch);

uint32_t status; // Return value of last camera func call

// I2C PERIPH CONFIG -------------------------------------------------------

#define PERIPH_SDA 26
#define PERIPH_SCL 27
#define PERIPH_ADDR ICAP_DEFAULT_ADDRESS
TwoWire *periphI2C = &Wire1;

// I2C CALLBACKS -----------------------------------------------------------

volatile uint8_t *reqAddr = NULL; // Pointer to data that requestCallback() will send
volatile int      reqLen = 0;     // Length of data "
volatile uint8_t  camState = 0;   // 0 = not started, 1 = plz start in loop(), 2 = started
volatile uint8_t  camBuf[500];

void requestCallback() {
  Serial.printf("requestCallback(), reqAddr=%08x, reqLen=%d\n", (uint32_t)reqAddr, reqLen);
  if (reqAddr && reqLen) {
    periphI2C->write((uint8_t *)reqAddr, reqLen);
  }
  // Reset both vars anyway in case a 0-length request accidentally happened
  reqAddr = NULL; // Clear out vars to indicate it was sent
  reqLen = 0;
}

// Read 'len' bytes from I2C into 'addr' buf
int readInto(uint8_t *addr, int len) {
  Serial.printf("Expecting %d bytes, %d are available\n", len, periphI2C->available());
  int i = 0;
  if (periphI2C->available() >= len) {
    for (; i<len; i++) {
      addr[i] = periphI2C->read();
    }
  }
  return i; // bytes read
}

volatile uint32_t capturedBytesRemaining = 0;
volatile uint8_t *capturedImagePtr = NULL;

void receiveCallback(int howMany) {
  Serial.printf("receiveCallback() howMany: %d, available(): %d\n", howMany, periphI2C->available());
  if (!howMany) return;

  int cmd = periphI2C->read(); // 1st byte should be command
  Serial.printf("Received command %02X\n", cmd);

  switch (cmd) {

    case ICAP_CMD_START: // Start camera
      Serial.printf("Start camera %02X %02X %02X\n", camBuf[0], camBuf[1], camBuf[2]);
      if (!camState) {
        // For now we'll treat mode, size and framerate as byte values.
        // That's OK for the former, but might want fractional rates later.
        if (readInto((uint8_t *)camBuf, 3) == 3) {
          // Have confirmed expected bytes are arriving (0, 2, 30)
          // The camera is NOT actually started here in this function,
          // as that causes lockup. Rather than try to track down in
          // cam lib (maybe something needs to be volatile there),
          // it's simpler to use a state variable here to trigger
          // camera startup in loop(). Host-side code should use the
          // polling command (below) to determine when the camera is
          // ready to accept further commands.
          camState = 1; // Plz start camera in loop()
        }
      }
      break;

    case ICAP_CMD_READY: // Poll camera ready state
      // Host-side code SHOULD NOT DO ANY OTHER CAM COMMANDS
      // until the return state is >1! This code should maybe
      // check for that.
      Serial.println("Return camera ready state");
      reqAddr = &camState; // Set up pointer & len for
      reqLen = 1;          // subsequent requestCallback()
      break;

    case ICAP_CMD_STATUS: // Return last status (will be followed by a requestCallback())
      Serial.println("Return last status");
      reqAddr = (uint8_t *)&status;            // Set up pointer & len for
      reqLen = 4;                              // subsequent requestCallback()
      break;

    case ICAP_CMD_READ_REG: // Read register (will be followed by a requestCallback())
      Serial.println("Read camera register");
      if ((readInto((uint8_t *)camBuf, 1) == 1) && camState > 1) {  // Register to read
        Serial.printf("Register = %02x\n", camBuf[0]);
        camBuf[0] = cam.readRegister(camBuf[0]); // Read from cam, put in buf
        Serial.printf("Value = %02x\n", camBuf[0]);
        reqAddr = camBuf;                        // Set up pointer & len for
        reqLen = 1;                              // subsequent requestCallback()
      }
      break;

    case ICAP_CMD_WRITE_REG: // Write register(s)
      Serial.println("Write camera register(s)");
      // First reg, length
      if ((readInto((uint8_t *)camBuf, 2) == 2) && camState > 1) {
        uint8_t reg = camBuf[0];
        uint8_t len = camBuf[1];
        Serial.printf("%d bytes starting at %02x\n", len, reg);
        if (readInto((uint8_t *)camBuf, len) == len) {
          for(int i=0; i<len; i++) {
            cam.writeRegister(reg + i, camBuf[i]);
          }
        }
      }
      break;

    case ICAP_CMD_CAPTURE: // Capture frame (will be followed by a requestCallback())
      if(camState > 1) {
        Serial.println("Capturing");
//        cam.suspend(); // Pause camera DMA, hold buffer steady to avoid tearing
// Could this be a thing where I have to do the suspend() in loop()?
        capturedImagePtr = (uint8_t *)cam.getBuffer();
        capturedBytesRemaining = cam.width() * cam.height() * 2;
        camBuf[0] = capturedBytesRemaining & 0xFF;
        camBuf[1] = (capturedBytesRemaining >> 8) & 0xFF;
        camBuf[2] = (capturedBytesRemaining >> 16) & 0xFF;
        camBuf[3] = (capturedBytesRemaining >> 24) & 0xFF;
        reqAddr = camBuf;                        // Set up pointer & len for
        reqLen = 4;                              // subsequent requestCallback()
camState = 3; // Plz suspend in loop()
      }
      // Will this require polling cam state on the host end?
      break;

    case ICAP_CMD_GET_DATA: // Request part of last-captured image data (followed by requestCallback())
      if ((readInto((uint8_t *)camBuf, 1) == 1) && camState > 1) {
        uint8_t len = camBuf[0];           // What host requested
        len = min(len, BUFFER_LENGTH - 1); // Limit it to our buffer size
        Serial.printf("Requested %d bytes of image, %d remain\n", len, capturedBytesRemaining);
        uint8_t bytesThisPass = min(capturedBytesRemaining, len);
        if (bytesThisPass)
          memcpy((void *)(&camBuf[1]), (void *)capturedImagePtr, bytesThisPass);
        camBuf[0] = bytesThisPass;
        reqAddr = camBuf;
        reqLen = bytesThisPass + 1;
        capturedBytesRemaining -= bytesThisPass;
        capturedImagePtr += bytesThisPass;
      }
      break;

    case ICAP_CMD_RESUME: // Resume camera
      camState = 5;
//      cam.resume(); // Resume DMA into camera buffer
      break;
  }

  // Purge any I2C residue
  int cruft = periphI2C->available();
  while (cruft-- > 0) {
    (void)periphI2C->read();
  }
}


// SETUP - RUNS ONCE ON STARTUP --------------------------------------------

void setup() {
  Serial.begin(9600);
  while(!Serial);
  delay(1000);
  Serial.println("CAMERA PERIPH BOARD STARTED");

  pinMode(25, OUTPUT); // LED
  digitalWrite(25, LOW);

  tft.init(240, 240);
  tft.setSPISpeed(48000000);
  tft.fillScreen(ST77XX_BLACK);
  tft.println("Howdy");
  tft.setRotation(3);

  // Pico acts as an I2C peripheral on a second bus
  // (since first is tied up with camera)
  periphI2C->setSDA(PERIPH_SDA);
  periphI2C->setSCL(PERIPH_SCL);
  periphI2C->begin(PERIPH_ADDR);
  periphI2C->setClock(100000);
  periphI2C->onRequest(requestCallback);
  periphI2C->onReceive(receiveCallback);
}

// MAIN LOOP - RUNS REPEATEDLY UNTIL RESET OR POWER OFF --------------------

void loop() {

  // Cam can't be started in I2C callback, probably interrupt-related,
  // so a small state var thing is done. Host-side code should poll
  // camera readyness before issuing any further requests.
  if (camState == 1) {
    Serial.println("STARTING CAMERA");
    status = cam.begin((iCap_colorspace)camBuf[0], (OV7670_size)camBuf[1], (float)camBuf[2]);
    if (status == ICAP_STATUS_OK) {
      Serial.println("CAMERA IS OK");
      //cam.test_pattern(OV7670_TEST_PATTERN_COLOR_BAR);
      delay(1000); // Allow exposure and PIO sync and whatnot
      camState = 2;
    } else {
      Serial.println("CAMERA FAIL");
      camState = 0;
    }
  } else if (camState == 3) {
    cam.suspend(); // Pause camera DMA, hold buffer steady to avoid tearing
    camState = 4; // Is suspended
  } else if (camState == 5) {
    cam.resume();
    camState = 2; // Is running
  }

  digitalWrite(25, !((millis() >> 7) & 7)); // LED heartbeat

  if (camState >= 2) {
    tft.endWrite();   // Close out prior transfer
    tft.startWrite(); // and start a fresh one (required)
    // Address window centers QQVGA image on screen. NO CLIPPING IS
    // PERFORMED, it is assumed here that the camera image is equal
    // or smaller than the screen.
    tft.setAddrWindow((tft.width() - cam.width()) / 2,
                      (tft.height() - cam.height()) / 2,
                      cam.width(), cam.height());
    tft.writePixels(cam.getBuffer(), cam.width() * cam.height(), false, true);
  }
}

#else
// Empty code to make this pass CI for now
void setup() {}
void loop() {}
#endif // ARDUINO_ARCH_RP2040
