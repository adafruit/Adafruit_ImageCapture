#if defined(ARDUINO_ARCH_RP2040)
/*
Pico RP2040 acting as an I2C peripheral.
NO DISPLAY and NO CAMERA attached,
just trying bare "act as I2C peripheral" test.
Works with companion i2c_camera_tester sketch.

NO USB Serial prints in callback functions

HARDWARE REQUIRED:
- Raspberry Pi Pico RP2040
- 10K pullups on SDA+SCL pins
*/

#include <Wire.h>

// If we were using a camera, 'status' would hold the last-returned
// camera function response. It's just a fake uint32_t here so we
// have something to pass back to i2c_camera_tester.
uint32_t status;

// I2C PERIPH CONFIG -------------------------------------------------------

#define PERIPH_SDA 26
#define PERIPH_SCL 27
#define PERIPH_ADDR 0x55
TwoWire *periphI2C = &Wire1;

// I2C CALLBACKS -----------------------------------------------------------

uint8_t *reqAddr = NULL; // Pointer to data that requestCallback() will send
int      reqLen = 0;     // Length of data "

void requestCallback() {
  if (reqAddr && reqLen) {
    periphI2C->write(reqAddr, reqLen);
  }
}

bool camStarted = false;
uint8_t camBuf[10];

// Read 'len' bytes from I2C into 'addr' buf
int readInto(uint8_t *addr, int len) {
  int i = 0;
  if (periphI2C->available() >= len) {
    for (; i<len; i++) {
      addr[i] = periphI2C->read();
    }
  }
  return i; // bytes read
}

void receiveCallback(int howMany) {
  if (!howMany) return;

  int cmd = periphI2C->read(); // 1st byte should be command

  switch (cmd) {

    case 0x10: // Start camera
      if (!camStarted) {
        if (readInto(camBuf, 3) == 3) { // Next 3 bytes are mode, size and FPS
          status = 42; // Fake status value for testing
          camStarted = true;
        }
      }
      break;

    case 0x20: // Return last status (will be followed by a requestCallback())
      reqAddr = (uint8_t *)&status;            // Set up pointer & len for
      reqLen = 4;                              // subsequent requestCallback()
      break;

    case 0x30: // Read register (will be followed by a requestCallback())
      if ((readInto(camBuf, 1) == 1) && camStarted) {  // Register to read
        camBuf[0] = 42; // Fake register value for testing
        reqAddr = camBuf;                        // Set up pointer & len for
        reqLen = 1;                              // subsequent requestCallback()
      }
      break;

    case 0x40: // Capture frame
      break;

    case 0x50: // Request frame data
      // Probably will have start & len here, with xfer broken into small pieces
      break;

    case 0x60: // Resume camera
      break;

    case 0x70: // Pass I2C sequence to camera
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
  // Nothing to do here, everything's I2C callback-based

  Serial.println("."); // Heartbeat
  delay(500);
}


#else
// Empty code to make this pass CI for now
void setup() {}
void loop() {}
#endif // ARDUINO_ARCH_RP2040
