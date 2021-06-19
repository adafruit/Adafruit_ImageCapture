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

/*
Declare image buffer as global var in this code, NOT the capture library.
Provide a way to pass it in.
There's a 'bufsiz' arg in begin() but that's about specifying how much
the lib should allocate, not how much the calling code already offers.
*/


#include <Wire.h>
#include <Adafruit_iCap_OV7670.h>
#include <Adafruit_iCap_I2C.h>
#if !defined(BUFFER_LENGTH)
#define BUFFER_LENGTH 256 // Max I2C transfer size
#endif

#define TFT_PREVIEW // Enable this line for viewfinder

#if defined(TFT_PREVIEW)
#include <Adafruit_ST7789.h>

#define TFT_CS  17 // Near SPI0 at south end of board
#define TFT_DC  16
#define TFT_RST -1 // Connect to MCU reset

Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);
#endif // TFT_PREVIEW

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

uint16_t pixelBuf[320 * 240]; // 150KB
Adafruit_iCap_OV7670 cam(pins, &arch, pixelBuf, sizeof pixelBuf, CAM_I2C);

// Calling camera library functions from I2C callbacks (interrupts) is
// bad news -- perhaps something in Pico peripherals or DMA, not sure.
// Workaround is a camera state machine in the loop() function that
// interacts with the callbacks via the camState variable. This is also
// used by I2C host code to poll the current camera state to avoid
// requesting data too soon.
volatile iCap_state camState = CAM_OFF;

uint32_t status; // Return value of last camera func call

// I2C PERIPH CONFIG -------------------------------------------------------

#define PERIPH_SDA 26
#define PERIPH_SCL 27
#define PERIPH_ADDR ICAP_DEFAULT_ADDRESS
TwoWire *periphI2C = &Wire1;

// I2C CALLBACKS -----------------------------------------------------------

volatile uint8_t  i2cBuf[BUFFER_LENGTH];
volatile int      i2cReqLen = 0;     // # i2cBuf bytes for requestCallback() to send
volatile uint32_t i2cMaxLen = 32;    // May be upgraded on negotiation
volatile uint8_t *capturedImagePtr = NULL;
volatile uint32_t capturedBytesRemaining = 0;
volatile uint8_t  capturing = false; // When true, sending image data, not i2cBuf
volatile bool     tripWire = false;  // Race condition weeds, see comments later

// I2C request callback; length of data i(i2cReqLen) s anticipated in I2CRecvCallback
void i2cReqCallback() {
  Serial.printf("i2cReqCallback(), i2cReqLen=%d\n", i2cReqLen);
  if (i2cReqLen > 0) {
    if (capturing) {
      // Send chunk of image data
      periphI2C->write((uint8_t *)capturedImagePtr, i2cReqLen);
      capturedBytesRemaining -= i2cReqLen;
      if (capturedBytesRemaining > 0) {
        capturedImagePtr += i2cReqLen;
        i2cReqLen = min(capturedBytesRemaining, i2cMaxLen);
      } else {
        capturing = false;
        camState = CAM_REQ_RESUME;
        i2cReqLen = 0;
      }
    } else {
      // Send i2cBuf data
      periphI2C->write((uint8_t *)i2cBuf, i2cReqLen);
      i2cReqLen = 0; // Reset length to avoid double invocation problems
    }
  }
  // If this is the first request callback after polling switched to
  // camera pause, set the 'capturing' flag true...next request will
  // then start issuing image data.
  if (tripWire == true) {
    i2cReqLen = min(capturedBytesRemaining, i2cMaxLen);
    capturing = true;
    tripWire = false;
  }
}

// Read 'len' bytes from I2C into i2cBuf
int i2cRead(int len) {
  Serial.printf("i2cRead(), expecting %d bytes, ", len);
  if (len > sizeof i2cBuf) len = sizeof i2cBuf; // Don't exceed i2cBuf size
  int i = periphI2C->available();
  if (len > i) len = i;                         // Don't exceed pending data
  Serial.printf("%d are available\n", i);
  for (i=0; i<len; i++) {
    i2cBuf[i] = periphI2C->read();
  }
  return i; // Number of bytes actually read
}

void i2cRecvCallback(int len) {
  Serial.printf("i2cRecvCallback(), expecting %d bytes", len);
  if (!len) {
    Serial.println();
    return;
  }
  int avail = periphI2C->available();
  len = min(len, avail);
  Serial.printf(", %d are available\n", len);

  int cmd = periphI2C->read(); // 1st byte should be command
  Serial.printf("Received command %02X\n", cmd);

  //if (capturing) return; // Ignore received byte

  switch (cmd) {

    case ICAP_CMD_BUFSIZ: // Negotiate I2C max transfer (subsequent requestCallback())
      Serial.println("Buffer size negotiation...");
      if (i2cRead(4) == 4) { // Host's I2C transfer limit
        uint32_t hostBufferLen = (uint32_t)i2cBuf[0]        |
                                ((uint32_t)i2cBuf[1] <<  8) |
                                ((uint32_t)i2cBuf[2] << 16) |
                                ((uint32_t)i2cBuf[3] << 24);
        i2cMaxLen = min(sizeof i2cBuf, hostBufferLen);
        Serial.printf("%d bytes\n", i2cMaxLen);
        i2cBuf[0] =  i2cMaxLen        & 0xFF;
        i2cBuf[1] = (i2cMaxLen  >> 8) & 0xFF;
        i2cBuf[2] = (i2cMaxLen >> 16) & 0xFF;
        i2cBuf[3] = (i2cMaxLen >> 24) & 0xFF;
        i2cReqLen = 4;
      } else {
        Serial.println("arguments missing; request ignored");
      }
      break;

    case ICAP_CMD_ID: // Identify camera model (subsequent requestCallback())
      Serial.println("Identify camera (0)");
      i2cBuf[0] = 0; // OV7670
      i2cReqLen = 1;
      break;

    case ICAP_CMD_STATE: // Poll camera state (subsequent requestCallback())
      Serial.printf("Poll state (%02x)\n", (int)camState);
      i2cBuf[0] = camState;
      i2cReqLen = 1;
      // If camState just changed to CAM_PAUSED, then the next request
      // AFTER this one will be the first image data. So, can't set the
      // 'capturing' flag just yet...that happens at the end of the
      // request callback if the 'tripWire' flag is set (could instead
      // make 'capturing' a 3-state var...it's states within states).
//      if (camState == CAM_PAUSED) tripWire = true;
      break;

    case ICAP_CMD_READ_REG: // Read register (subsequent requestCallback())
      Serial.print("Read camera register...");
      if (i2cRead(1) == 1) {                       // Register to read
        if (camState >= CAM_ON) {                  // Only possible w/camera running
          Serial.printf("%02x = ", i2cBuf[0]);     // Register
          i2cBuf[0] = cam.readRegister(i2cBuf[0]); // Read from cam, put in buf
          Serial.printf("%02x\n", i2cBuf[0]);      // Value
          i2cReqLen = 1;
        } else {
          Serial.println("camera not started; request ignored");
        }
      } else {
        Serial.println("argument missing; request ignored");
      }
      break;

    case ICAP_CMD_WRITE_REG: // Write register(s)
      Serial.print("Write camera register(s)...");
      if ((i2cRead(2) == 2)) {   // First, length
        uint8_t len = i2cBuf[1]; // Max len 255
        if (camState >= CAM_ON) {
          uint8_t reg = i2cBuf[0];
          if (i2cRead(len) == len) {
            Serial.printf("%d byte(s) starting at %02x\n", len, reg);
            for(int i=0; i<len; i++) {
              cam.writeRegister(reg + i, i2cBuf[i]);
            }
          } else {
            Serial.println("arguments missing; request ignored");
          }
        } else {
          Serial.println("camera not started; request ignored");
        }
      } else {
        Serial.println("arguments missing; request ignored");
      }
      break;

    case ICAP_CMD_SETUP: // Specify capture parameters
      Serial.print("Start camera...");
      // Only start camera if currently in OFF state.
      // Read I2C bytes regardless to keep in sync.
      if (i2cRead(3) == 3) {
        Serial.printf("[%02X %02X %02X]", i2cBuf[0], i2cBuf[1], i2cBuf[2]);
        if (camState == CAM_OFF) {
          Serial.println("...requested");
          camState = CAM_REQ_CONFIG; // Plz start camera in loop()
        } else {
          Serial.println("...already running; request ignored");
        }
      } else {
        Serial.println("arguments missing; request ignored");
      }
      break;


// Host code MUST poll state until camera is paused
    case ICAP_CMD_CAPTURE: // Capture frame (subsequent requestCallback()s)
      Serial.print("Capture...");
      if ((camState == CAM_ON) || (camState == CAM_PAUSED)) {
        Serial.println("started");
        camState = CAM_REQ_PAUSE;
        capturedImagePtr = (uint8_t *)cam.getBuffer();
        capturedBytesRemaining = cam.width() * cam.height() * 2;
        // Tell host how many bytes to expect
        i2cBuf[0] =  capturedBytesRemaining        & 0xFF;
        i2cBuf[1] = (capturedBytesRemaining >>  8) & 0xFF;
        i2cBuf[2] = (capturedBytesRemaining >> 16) & 0xFF;
        i2cBuf[3] = (capturedBytesRemaining >> 24) & 0xFF;
        i2cReqLen = 4;
        // Once camera is paused, subsequent requests return image data
tripWire = true;
      } else {
        Serial.println("camera not running; request ignored");
      }
      break;

    case ICAP_CMD_RESUME: // Un-pause camera, resume normal background capture
      camState = CAM_REQ_RESUME;
      break;

    case ICAP_CMD_RETURN: // Poll last function return value (subsequent requestCallback())
      // This is super rare and probably not needed except for weird debugging.
      Serial.printf("Return last camera function call status (%d)\n", status);
      i2cBuf[0] = (uint8_t)status;
      i2cReqLen = 1;
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

#if defined(TFT_PREVIEW)
  tft.init(240, 240);
  tft.setSPISpeed(48000000);
  tft.fillScreen(ST77XX_BLACK);
  tft.println("I2C PERIPH");
  tft.setRotation(3);
#endif // TFT_PREVIEW

  cam.begin(); // Init camera-related peripherals, but don't capture yet
// (Alternately, could init this at a default size if using
// the viewfinder preview)

  // Pico acts as an I2C peripheral on a second bus
  // (since first is tied up with camera)
  periphI2C->setSDA(PERIPH_SDA);
  periphI2C->setSCL(PERIPH_SCL);
  periphI2C->begin(PERIPH_ADDR);
  periphI2C->setClock(400000UL);
  periphI2C->onRequest(i2cReqCallback);
  periphI2C->onReceive(i2cRecvCallback);
}

// MAIN LOOP - RUNS REPEATEDLY UNTIL RESET OR POWER OFF --------------------

void loop() {

  switch (camState) { // Only the "REQ" states need to be handled here


// Change this. Camera is already started, just changing settings now.
    case CAM_REQ_CONFIG:
      // For now we'll treat mode, size and framerate as byte values.
      // That's OK for the former, but might want fractional rates later.
      status = cam.config((OV7670_size)i2cBuf[0], (iCap_colorspace)i2cBuf[1], 
                         (float)i2cBuf[2]);
      if (status == ICAP_STATUS_OK) {
        Serial.println("OK");
        //cam.test_pattern(OV7670_TEST_PATTERN_COLOR_BAR);
        delay(500); // Allow exposure and PIO sync and whatnot
        camState = CAM_ON;
      } else {
        Serial.println("FAIL");
        camState = CAM_OFF;
      }
      break;
    case CAM_REQ_PAUSE:
      Serial.print("PAUSING CAMERA...");
      cam.suspend();
      Serial.println("OK");
      camState = CAM_PAUSED;
      break;
    case CAM_REQ_RESUME:
      Serial.print("RESUMING CAMERA...");
      cam.resume();
      capturing = false;
      Serial.println("OK");
      camState = CAM_ON;
      break;
  }

  digitalWrite(25, !((millis() >> 7) & 7)); // LED heartbeat

#if defined(TFT_PREVIEW)
  if (camState == CAM_ON) {
    tft.startWrite();
    // Address window centers QQVGA image on screen. NO CLIPPING IS
    // PERFORMED, it is assumed here that the camera image is equal
    // or smaller than the screen.
    tft.setAddrWindow((tft.width() - cam.width()) / 2,
                      (tft.height() - cam.height()) / 2,
                      cam.width(), cam.height());
    tft.writePixels(cam.getBuffer(), cam.width() * cam.height(), true, true);
    tft.endWrite();
  }
#endif // TFT_PREVIEW
}

#else
// Empty code to make this pass CI for now
void setup() {}
void loop() {}
#endif // ARDUINO_ARCH_RP2040
