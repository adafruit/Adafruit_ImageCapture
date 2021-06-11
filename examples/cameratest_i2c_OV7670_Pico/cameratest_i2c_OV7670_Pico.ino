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

#include <Wire.h>
#include <Adafruit_iCap_OV7670.h> // Camera library

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
volatile uint8_t  camBuf[10];

void requestCallback() {
  Serial.printf("requestCallback(), reqAddr=%08x, reqLen=%d\n", (uint32_t)reqAddr, reqLen);
  if (reqAddr && reqLen) {
    periphI2C->write((uint8_t *)reqAddr, reqLen);
    reqAddr = NULL; // Clear out vars to indicate it was sent
    reqLen = 0;
  }
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
      break;

    case ICAP_CMD_CAPTURE: // Capture frame
      // Pause the camera DMA - hold buffer steady to avoid tearing
      //cam.suspend();
      break;

    case ICAP_CMD_GET_DATA: // Request frame data
      // Probably will have start & len here, with xfer broken into small pieces
      break;

    case ICAP_CMD_RESUME: // Resume camera
      //cam.resume(); // Resume DMA into camera buffer
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
    status = cam.begin((iCap_colorspace)camBuf[0], (OV7670_size)camBuf[1], (float)camBuf[3]);
    if (status == ICAP_STATUS_OK) {
      Serial.println("CAMERA IS OK");
      camState = 2;
    } else {
      Serial.println("CAMERA FAIL");
      camState = 0;
    }
  }

  digitalWrite(25, !((millis() >> 7) & 7)); // LED heartbeat
}

#else
// Empty code to make this pass CI for now
void setup() {}
void loop() {}
#endif // ARDUINO_ARCH_RP2040
