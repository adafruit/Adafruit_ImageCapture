#if defined(ARDUINO_ARCH_RP2040)
/*
OV7670 + Pico RP2040 acting as an I2C peripheral.
No display; I2C host supplies that.

HARDWARE REQUIRED:
- Raspberry Pi Pico RP2040
- OV7670 camera
- 10K pullups on SDA+SCL pins
*/

#include <Wire.h>                 // I2C comm to camera
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

#define CAM_I2C Wire
#define CAM_SIZE OV7670_SIZE_DIV4  // QQVGA (160x120 pixels)
#define CAM_MODE ICAP_COLOR_RGB565 // RGB plz

Adafruit_iCap_OV7670 cam(pins, CAM_I2C, &arch);

//iCap_status status; // Return value of last camera func call
uint32_t status; // Return value of last camera func call

// I2C PERIPH CONFIG -------------------------------------------------------

#define PERIPH_SDA 26
#define PERIPH_SCL 27
#define PERIPH_I2C Wire1
#define PERIPH_ADDR 0x55
TwoWire *periphI2C = &Wire1;

// I2C CALLBACKS -----------------------------------------------------------

uint8_t *reqAddr = NULL; // Pointer to data that requestEvent() will send
int      reqLen = 0;     // Length of data "

void requestEvent() {
  Serial.printf("requestEvent() callback, reqAddr=%08x, reqLen=%d\n", (uint32_t)reqAddr, reqLen);
  if (reqAddr && reqLen) {
    periphI2C->write(reqAddr, reqLen);
  }
}

bool camStarted = false;
uint8_t camBuf[10];

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

void receiveEvent(int howMany) {
  Serial.printf("receiveEvent() callback, %d bytes\n", howMany);
  Serial.printf("howMany: %d, available(): %d\n", howMany, periphI2C->available());
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

    case 0x20: // Return last status (will be followed by a requestEvent())
      Serial.println("Return last status");
      reqAddr = (uint8_t *)&status;            // Set up pointer & len for
      reqLen = 4;                              // subsequent requestEvent()
      break;

    case 0x30: // Read register (will be followed by a requestEvent())
      Serial.println("Read camera register");
      if ((readInto(camBuf, 1) == 1) && camStarted) {  // Register to read
        Serial.printf("Register = %02x\n", camBuf[0]);
        camBuf[0] = cam.readRegister(camBuf[0]); // Read from cam, put in buf
        Serial.printf("Value = %02x\n", camBuf[0]);
        reqAddr = camBuf;                        // Set up pointer & len for
        reqLen = 1;                              // subsequent requestEvent()
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
  periphI2C->onRequest(requestEvent);
  periphI2C->onReceive(receiveEvent);
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
}


#else
// Empty code to make this pass CI for now
void setup() {}
void loop() {}
#endif // ARDUINO_ARCH_RP2040
