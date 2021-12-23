#if defined(ARDUINO_ARCH_RP2040)
/*
Adafruit_ImageCapture example for Pico RP2040 + ST7789 240x240 display.
Simple camera test - input is shown on display.

HARDWARE REQUIRED:
- Raspberry Pi Pico RP2040
- Adafruit 240x240 TFT display
- OV5640 camera
- 10K pullups on SDA+SCL pins
*/

#include <Wire.h>                 // I2C comm to camera
#include <Adafruit_iCap_OV5640.h> // Camera library
#include <Adafruit_ST7789.h>      // Hardware-specific library for ST7789

// CAMERA CONFIG -----------------------------------------------------------

// Set up arch and pins structures for Pico RP2040.
iCap_arch arch = {
  .pio = pio0,   // Which PIO peripheral to use (pio0 or pio1)
  .bswap = true, // Capture in big-endian order, can go straight to TFT
  // Other elements are set by the library at runtime and should not be
  // specified by user code (will be overwritten).
};
OV5640_pins pins = {
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
#define CAM_SIZE OV5640_SIZE_DIV1
#define CAM_MODE ICAP_COLORSPACE_RGB565 // RGB plz

//Adafruit_iCap_OV5640 cam(pins, &arch, CAM_I2C);
// Alloc buffer for me, 1 MHz I2C, no delay
Adafruit_iCap_OV5640 cam(pins, &arch, CAM_I2C, NULL, 0, OV5640_ADDR, 1000000, 0);

// DISPLAY CONFIG ----------------------------------------------------------

#define TFT_CS  17 // Near SPI0 at south end of board
#define TFT_DC  16
#define TFT_RST -1 // Connect to MCU reset

Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);

// SETUP - RUNS ONCE ON STARTUP --------------------------------------------

void setup() {
  Serial.begin(9600);
  while(!Serial);
  delay(1000);
  Serial.println("Hello! Camera Test.");

  pinMode(25, OUTPUT);
  digitalWrite(25, LOW);

  // These are currently RP2040 Philhower-specific
  SPI.setSCK(18); // SPI0 (for display)
  SPI.setTX(19);

  tft.init(240, 240);
  tft.setSPISpeed(48000000);
  tft.fillScreen(ST77XX_BLACK);
  tft.println("Howdy");
  tft.setRotation(2);
  // Once started, the camera continually fills a frame buffer
  // automagically; no need to request a frame.
  iCap_status status = cam.begin(CAM_SIZE, CAM_MODE, 30.0);
  if (status != ICAP_STATUS_OK) {
    Serial.println("Camera begin() fail");
    for(;;);
  }

  uint16_t chip_id = cam.readRegister16x16(OV5640_REG_CHIP_ID_HIGH);
  Serial.println(chip_id, HEX); // Should be 0x5640
}

// MAIN LOOP - RUNS REPEATEDLY UNTIL RESET OR POWER OFF --------------------

// TFT setAddrWindow() involves a lot of context switching that can slow
// things down a bit, so we don't do it on every frame. Instead, it's only
// set periodically, and we just keep writing data to the same area of the
// screen (it wraps around automatically). We do need an OCCASIONAL
// setAddrWindow() in case SPI glitches, as this syncs things up to a
// known region of the screen again.
#define KEYFRAME 30        // Number of frames between setAddrWindow commands
uint16_t frame = KEYFRAME; // Force 1st frame as keyframe

void loop() {

  gpio_xor_mask(1 << 25); // Toggle LED each frame

  // This was for empirically testing window settings in src/arch/ov7670.c.
  // Your code doesn't need this. Just keeping around for future reference.
  if(Serial.available()) {
    uint32_t vstart = Serial.parseInt();
    uint32_t hstart = Serial.parseInt();
    uint32_t edge_offset = Serial.parseInt();
    uint32_t pclk_delay = Serial.parseInt();
    while(Serial.read() >= 0); // Delete line ending or other cruft
//    cam.frameControl(CAM_SIZE, vstart, hstart,
//                     edge_offset, pclk_delay);
  }

  if (++frame >= KEYFRAME) { // Time to sync up a fresh address window?
    frame = 0;

    tft.endWrite();   // Close out prior transfer
    tft.startWrite(); // and start a fresh one (required)
    // Address window centers QQVGA image on screen. NO CLIPPING IS
    // PERFORMED, it is assumed here that the camera image is equal
    // or smaller than the screen.
    tft.setAddrWindow((tft.width() - cam.width()) / 2,
                      (tft.height() - cam.height()) / 2,
                      cam.width(), cam.height());
  }

  // Pause the camera DMA - hold buffer steady to avoid tearing
  //cam.suspend();

  if (CAM_MODE == ICAP_COLORSPACE_YUV) {
    cam.Y2RGB565(); // Convert grayscale for TFT preview
  }

  // Blocking, little-endian source data
  tft.writePixels(cam.getBuffer(), cam.width() * cam.height(), true, false);

  //cam.resume(); // Resume DMA into camera buffer
}
#else
// Empty code to make this pass CI for now
void setup() {}
void loop() {}
#endif // ARDUINO_ARCH_RP2040
