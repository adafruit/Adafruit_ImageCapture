// Okay, a do-over of camera-as-I2C-peripheral (which wasn't working).
// Starting with the Pico-as-peripheral (no camera) code...
// Then moving in small pieces of the cam-as-periph code (currently
// #if 0'd out at the bottom of this file) and seeing where it fails,
// or if the new periph code (w volatiles, etc.) fixed things.
// UPDATE: declaring the camera structs & objects is OK.
// Problem occurs when camera is started. Have verified that the
// arguments (mode, size & framerate) are OK.
// UPDATE 2: problem appears to be with starting the camera inside the
// I2C callback. A kludgey fix that seems to work is to have a volatile
// state variable that instead starts the camera in loop() when requested
// by the I2C callback. Alternative would require finding out why cam lib
// doesn't allow start from callback...probably something needs to be
// volatile or who-knows-what.

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
#include <Adafruit_iCap_OV7670.h> // Camera library

static uint8_t camState = 0; // Cam not started

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

volatile uint8_t *reqAddr = NULL; // Pointer to data that requestCallback() will send
volatile int      reqLen = 0;     // Length of data "
volatile bool     camStarted = false;
volatile uint8_t  camBuf[10];

void requestCallback() {
  if (reqAddr && reqLen) {
    periphI2C->write((uint8_t *)reqAddr, reqLen);
    reqAddr = NULL; // Clear out vars to indicate it was sent
    reqLen = 0;
  }
}

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

#if 1
// "Real" camera start - DOES NOT WORK
    case 0x10: // Start camera
      if (!camStarted) {
Serial.println("Hey");
        // For now we'll treat mode, size and framerate as byte values.
        // That's OK for the former, but might want fractional rates later.
        if (readInto((uint8_t *)camBuf, 3) == 3) {
          // Have confirmed expected bytes are arriving (0, 2, 30)
camState = 1; // Plz start camera in loop()
//         status = cam.begin((iCap_colorspace)camBuf[0], (OV7670_size)camBuf[1], (float)camBuf[3]);
//         if (status == ICAP_STATUS_OK)
//           camStarted = true;
        }
      }
      break;
#else
// "Dummy" camera start (no camera, just sets fake status) - DOES WORK
    case 0x10: // Start camera
      if (!camStarted) {
        if (readInto((uint8_t *)camBuf, 3) == 3) { // Next 3 bytes are mode, size and FPS
          status = 42; // Fake status value for testing
          camStarted = true;
        }
      }
      break;
#endif

    case 0x20: // Return last status (will be followed by a requestCallback())
      reqAddr = (uint8_t *)&status;            // Set up pointer & len for
      reqLen = 4;                              // subsequent requestCallback()
      break;

    case 0x30: // Read register (will be followed by a requestCallback())
#if 1
// "Real" register read from camera
      if ((readInto((uint8_t *)camBuf, 1) == 1) && camStarted) {  // Register to read
        camBuf[0] = cam.readRegister(camBuf[0]); // Read from cam, put in buf
        reqAddr = camBuf;                        // Set up pointer & len for
        reqLen = 1;                              // subsequent requestCallback()
      }
#else
// "Dummy" register read (sends a fake response value)
      if ((readInto((uint8_t *)camBuf, 1) == 1) && camStarted) {  // Register to read
        camBuf[0] = 42; // Fake register value for testing
        reqAddr = camBuf;                        // Set up pointer & len for
        reqLen = 1;                              // subsequent requestCallback()
      }
#endif
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

// EXPERIMENT: start camera before I2C commands
//  status = cam.begin((iCap_colorspace)0, (OV7670_size)2, (float)30);
//  if (status == ICAP_STATUS_OK)
//    camStarted = true;
}

// MAIN LOOP - RUNS REPEATEDLY UNTIL RESET OR POWER OFF --------------------

void loop() {
  // Nothing to do here, everything's I2C callback-based

  if (camState == 1) {
    Serial.println("STARTING CAMERA");
    status = cam.begin((iCap_colorspace)camBuf[0], (OV7670_size)camBuf[1], (float)camBuf[3]);
    if (status == ICAP_STATUS_OK) {
      Serial.println("CAMERA IS OK");
      camStarted = true;
      camState = 2;
    } else {
      Serial.println("CAMERA FAIL");
      camState = 0;
    }
  }

  Serial.println("."); // Heartbeat
  delay(500);
}

#else
// Empty code to make this pass CI for now
void setup() {}
void loop() {}
#endif // ARDUINO_ARCH_RP2040




// See notes at top
#if 0 // TEST TEST TEST TEST TEST TEST

/*
OV7670 + Pico RP2040 acting as an I2C peripheral.
No display; I2C host supplies that.
Works with companion i2c_camera_tester sketch.

HARDWARE REQUIRED:
- Raspberry Pi Pico RP2040
- OV7670 camera
- 10K pullups on SDA+SCL pins
*/

//iCap_status status; // Return value of last camera func call
uint32_t status; // Return value of last camera func call

// I2C CALLBACKS -----------------------------------------------------------

void requestCallback() {
  Serial.printf("requestCallback(), reqAddr=%08x, reqLen=%d\n", (uint32_t)reqAddr, reqLen);
  if (reqAddr && reqLen) {
    periphI2C->write(reqAddr, reqLen);
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
  return i;
}

void receiveCallback(int howMany) {
  Serial.printf("receiveCallback(), %d bytes\n", howMany);
  // Why does this hang here? Makes no sense.
  // I’m getting a correct "howMany" value (e.g. 4 bytes)
  // and then it freezes. Address is not printed.
  // Heartbeat in loop() function stops. Total lockup?
  Serial.flush(); delay(100); Serial.flush();
  Serial.printf("periphI2C address is %08X\n", (uint32_t)periphI2C);
  Serial.flush(); delay(100); Serial.flush();
  Serial.printf("howMany: %d, available(): %d\n", howMany, periphI2C->available());
  Serial.flush(); delay(100); Serial.flush();
  if (!howMany) return;

  int cmd = periphI2C->read();
  Serial.printf("Received command %02X\n", cmd);

  switch (cmd) {

    case 0x10: // Start camera
      if (!camStarted) {
        // For now we'll treat mode, size and framerate as byte values.
        // That's OK for the former, but might want fractional rates later.
        if (readInto(camBuf, 3) == 3) {
          Serial.printf("Start camera %02X %02X %02X\n", camBuf[0], camBuf[1], camBuf[2]);
          status = cam.begin((iCap_colorspace)camBuf[0], (OV7670_size)camBuf[1], (float)camBuf[3]);
          if (status == ICAP_STATUS_OK)
            camStarted = true;
        }
      }
      break;

    case 0x20: // Return last status (will be followed by a requestCallback())
      Serial.println("Return last status");
      reqAddr = (uint8_t *)&status;            // Set up pointer & len for
      reqLen = 4;                              // subsequent requestCallback()
      break;

    case 0x30: // Read register (will be followed by a requestCallback())
      Serial.println("Read camera register");
      if ((readInto(camBuf, 1) == 1) && camStarted) {  // Register to read
        Serial.printf("Register = %02x\n", camBuf[0]);
        camBuf[0] = cam.readRegister(camBuf[0]); // Read from cam, put in buf
        Serial.printf("Value = %02x\n", camBuf[0]);
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

#if 0

  // Stuff from old cameratest that'll get incorporated into
  // various command cases above.

  uint8_t pid = cam.readRegister(OV7670_REG_PID); // Should be 0x76
  uint8_t ver = cam.readRegister(OV7670_REG_VER); // Should be 0x73
  Serial.println(pid, HEX);
  Serial.println(ver, HEX);

  gpio_xor_mask(1 << 25); // Toggle LED each frame


  // Pause the camera DMA - hold buffer steady to avoid tearing
  //cam.suspend();

  //cam.resume(); // Resume DMA into camera buffer

#endif

// MAIN LOOP - RUNS REPEATEDLY UNTIL RESET OR POWER OFF --------------------

void loop() {
  // Nothing to do here, everything's I2C callback-based

  Serial.println("."); // Heartbeat
  delay(500);
}

#endif // TEST TEST TEST TEST TEST
