#include <Arduino.h>
#include <Adafruit_iCap_OV7670.h>

Adafruit_iCap_OV7670::Adafruit_iCap_OV7670(OV7670_pins &pins,
                                           TwoWire &twi, iCap_arch *arch,
                                           uint8_t addr)
    : Adafruit_iCap_parallel((iCap_parallel_pins *)&pins, (TwoWire *)&twi,
                             arch, addr, 100000, 1000) {}

Adafruit_iCap_OV7670::~Adafruit_iCap_OV7670() {}

// CAMERA STARTUP ----------------------------------------------------------

static const iCap_parallel_config
    OV7670_init[] =
        {
            // OV7670 camera initialization after reset
            {OV7670_REG_TSLB, OV7670_TSLB_YLAST},    // No auto window
            {OV7670_REG_COM10, OV7670_COM10_VS_NEG}, // -VSYNC (req by SAMD PCC)
            {OV7670_REG_SLOP, 0x20},
            {OV7670_REG_GAM_BASE, 0x1C},
            {OV7670_REG_GAM_BASE + 1, 0x28},
            {OV7670_REG_GAM_BASE + 2, 0x3C},
            {OV7670_REG_GAM_BASE + 3, 0x55},
            {OV7670_REG_GAM_BASE + 4, 0x68},
            {OV7670_REG_GAM_BASE + 5, 0x76},
            {OV7670_REG_GAM_BASE + 6, 0x80},
            {OV7670_REG_GAM_BASE + 7, 0x88},
            {OV7670_REG_GAM_BASE + 8, 0x8F},
            {OV7670_REG_GAM_BASE + 9, 0x96},
            {OV7670_REG_GAM_BASE + 10, 0xA3},
            {OV7670_REG_GAM_BASE + 11, 0xAF},
            {OV7670_REG_GAM_BASE + 12, 0xC4},
            {OV7670_REG_GAM_BASE + 13, 0xD7},
            {OV7670_REG_GAM_BASE + 14, 0xE8},
            {OV7670_REG_COM8,
             OV7670_COM8_FASTAEC | OV7670_COM8_AECSTEP | OV7670_COM8_BANDING},
            {OV7670_REG_GAIN, 0x00},
            {OV7670_COM2_SSLEEP, 0x00},
            {OV7670_REG_COM4, 0x00},
            {OV7670_REG_COM9, 0x20}, // Max AGC value
            {OV7670_REG_BD50MAX, 0x05},
            {OV7670_REG_BD60MAX, 0x07},
            {OV7670_REG_AEW, 0x75},
            {OV7670_REG_AEB, 0x63},
            {OV7670_REG_VPT, 0xA5},
            {OV7670_REG_HAECC1, 0x78},
            {OV7670_REG_HAECC2, 0x68},
            {0xA1, 0x03},              // Reserved register?
            {OV7670_REG_HAECC3, 0xDF}, // Histogram-based AEC/AGC setup
            {OV7670_REG_HAECC4, 0xDF},
            {OV7670_REG_HAECC5, 0xF0},
            {OV7670_REG_HAECC6, 0x90},
            {OV7670_REG_HAECC7, 0x94},
            {OV7670_REG_COM8, OV7670_COM8_FASTAEC | OV7670_COM8_AECSTEP |
                                  OV7670_COM8_BANDING | OV7670_COM8_AGC |
                                  OV7670_COM8_AEC},
            {OV7670_REG_COM5, 0x61},
            {OV7670_REG_COM6, 0x4B},
            {0x16, 0x02},            // Reserved register?
            {OV7670_REG_MVFP, 0x07}, // 0x07,
            {OV7670_REG_ADCCTR1, 0x02},
            {OV7670_REG_ADCCTR2, 0x91},
            {0x29, 0x07}, // Reserved register?
            {OV7670_REG_CHLF, 0x0B},
            {0x35, 0x0B}, // Reserved register?
            {OV7670_REG_ADC, 0x1D},
            {OV7670_REG_ACOM, 0x71},
            {OV7670_REG_OFON, 0x2A},
            {OV7670_REG_COM12, 0x78},
            {0x4D, 0x40}, // Reserved register?
            {0x4E, 0x20}, // Reserved register?
            {OV7670_REG_GFIX, 0x5D},
            {OV7670_REG_REG74, 0x19},
            {0x8D, 0x4F}, // Reserved register?
            {0x8E, 0x00}, // Reserved register?
            {0x8F, 0x00}, // Reserved register?
            {0x90, 0x00}, // Reserved register?
            {0x91, 0x00}, // Reserved register?
            {OV7670_REG_DM_LNL, 0x00},
            {0x96, 0x00}, // Reserved register?
            {0x9A, 0x80}, // Reserved register?
            {0xB0, 0x84}, // Reserved register?
            {OV7670_REG_ABLC1, 0x0C},
            {0xB2, 0x0E}, // Reserved register?
            {OV7670_REG_THL_ST, 0x82},
            {0xB8, 0x0A}, // Reserved register?
            {OV7670_REG_AWBC1, 0x14},
            {OV7670_REG_AWBC2, 0xF0},
            {OV7670_REG_AWBC3, 0x34},
            {OV7670_REG_AWBC4, 0x58},
            {OV7670_REG_AWBC5, 0x28},
            {OV7670_REG_AWBC6, 0x3A},
            {0x59, 0x88}, // Reserved register?
            {0x5A, 0x88}, // Reserved register?
            {0x5B, 0x44}, // Reserved register?
            {0x5C, 0x67}, // Reserved register?
            {0x5D, 0x49}, // Reserved register?
            {0x5E, 0x0E}, // Reserved register?
            {OV7670_REG_LCC3, 0x04},
            {OV7670_REG_LCC4, 0x20},
            {OV7670_REG_LCC5, 0x05},
            {OV7670_REG_LCC6, 0x04},
            {OV7670_REG_LCC7, 0x08},
            {OV7670_REG_AWBCTR3, 0x0A},
            {OV7670_REG_AWBCTR2, 0x55},
            {OV7670_REG_MTX1, 0x80},
            {OV7670_REG_MTX2, 0x80},
            {OV7670_REG_MTX3, 0x00},
            {OV7670_REG_MTX4, 0x22},
            {OV7670_REG_MTX5, 0x5E},
            {OV7670_REG_MTX6, 0x80}, // 0x40?
            {OV7670_REG_AWBCTR1, 0x11},
            {OV7670_REG_AWBCTR0, 0x9F}, // Or use 0x9E for advance AWB
            {OV7670_REG_BRIGHT, 0x00},
            {OV7670_REG_CONTRAS, 0x40},
            {OV7670_REG_CONTRAS_CENTER, 0x80}}, // 0x40?
    OV7670_rgb[] =
        {
            // Manual output format, RGB, use RGB565 and full 0-255 output range
            {OV7670_REG_COM7, OV7670_COM7_RGB},
            {OV7670_REG_RGB444, 0},
            {OV7670_REG_COM15, OV7670_COM15_RGB565 | OV7670_COM15_R00FF}},
    OV7670_yuv[] =
        {
            // Manual output format, YUV, use full output range
            {OV7670_REG_COM7, OV7670_COM7_YUV},
            {OV7670_REG_COM15, OV7670_COM15_R00FF}};

ICAP_status Adafruit_iCap_OV7670::begin(ICAP_colorspace colorspace,
                    OV7670_size size, float fps, uint32_t bufsiz) {

  ICAP_status status;

// _width and _height need to be known before starting PCC
// also, buffer prealloc (or is that in ImageCapture?).
// setSize can do the size & alloc stuff, but tries to issue I2C
// commands, which isn't valid until after begin here.
  status = setSize(size, ICAP_REALLOC_CHANGE);
  if(status != ICAP_STATUS_OK) {
    return status;
  }

  // Initialize memory, peripherals for parallel+I2C camera:
  status = Adafruit_iCap_parallel::begin();
  if(status != ICAP_STATUS_OK) {
    return status;
  }

  // ENABLE AND/OR RESET CAMERA --------------------------------------------

  if (pins.enable >= 0) { // Enable pin defined?
    pinMode(pins.enable, OUTPUT);
    digitalWrite(pins.enable, 0); // PWDN low (enable)
  }

  // Unsure of camera startup time from beginning of input clock.
  // Let's guess it's similar to tS:REG (300 ms) from datasheet.
  delayMicroseconds(300000);

  if (pins.reset >= 0) { // Hard reset pin defined?
    pinMode(pins.reset, OUTPUT);
    digitalWrite(pins.reset, LOW);
    delay(1);
    digitalWrite(pins.reset, HIGH);
  } else { // Soft reset, doesn't seem reliable, might just need more delay?
    writeRegister(OV7670_REG_COM7, OV7670_COM7_RESET);
  }
  delay(1); // Datasheet: tS:RESET = 1 ms

  (void)setFPS(fps); // Timing
  if (colorspace == ICAP_COLOR_RGB565) {
    writeList(OV7670_rgb, sizeof OV7670_rgb / sizeof OV7670_rgb[0]);
  } else {
    writeList(OV7670_yuv, sizeof OV7670_yuv / sizeof OV7670_yuv[0]);
  }
  // Init main camera settings
  writeList(OV7670_init, sizeof OV7670_init / sizeof OV7670_init[0]);
  setSize(size); // Frame size

  delayMicroseconds(300000); // tS:REG = 300 ms (settling time = 10 frames)

  resume(); // Start DMA cycle

  return ICAP_STATUS_OK;
}

// MISCELLANY AND CAMERA CONFIG FUNCTIONS ----------------------------------

// Configure camera frame rate. Actual resulting frame rate (returned) may
// be different depending on available clock frequencies. Result will only
// exceed input if necessary for minimum supported rate, but this is very
// rare, typically below 1 fps. In all other cases, result will be equal
// or less than the requested rate, up to a maximum of 30 fps (the "or less"
// is because requested fps may be based on other host hardware timing
// constraints (e.g. screen) and rounding up to a closer-but-higher frame
// rate would be problematic). There is no hardcoded set of fixed frame
// rates because it varies with architecture, depending on OV7670_XCLK_HZ.
// If platform is NULL, no registers are set, a fps request/return can be
// evaluated without reconfiguring the camera, or without it even started.

float Adafruit_iCap_OV7670::setFPS(float fps) {

  // Pixel clock (PCLK), which determines overall frame rate, is a
  // function of XCLK input frequency (OV7670_XCLK_HZ), a PLL multiplier
  // and then an integer division factor (1-32). These are the available
  // OV7670 PLL ratios:
  static const uint8_t pll_ratio[] = {1, 4, 6, 8};
  const uint8_t num_plls = sizeof pll_ratio / sizeof pll_ratio[0];

  // Constrain frame rate to upper and lower limits
  fps = (fps > 30) ? 30 : fps;               // Max 30 FPS
  float pclk_target = fps * 4000000.0 / 5.0; // Ideal PCLK Hz for target FPS
  uint32_t pclk_min = OV7670_XCLK_HZ / 32;   // Min PCLK determines min FPS
  if (pclk_target < (float)pclk_min) {       // If PCLK target is below limit
    writeRegister(OV7670_REG_DBLV, 0);       //  1:1 PLL
    writeRegister(OV7670_REG_CLKRC, 31);     //  1/32 div
    return (float)(pclk_min * 5 / 4000000);  //  Return min frame rate
  }

  // Find nearest available FPS without going over. This is done in a
  // brute-force manner, testing all 127 PLL-up by divide-down permutations
  // and tracking the best fit. I’m certain there are shortcuts but was
  // having trouble with my math, might revisit later. It's not a huge
  // bottleneck...MCUs are fast now, many cases are quickly discarded, and
  // this operation is usually done only once on startup (the I2C transfers
  // probably take longer).

  uint8_t best_pll = 0;    // Index (not value) of best PLL match
  uint8_t best_div = 1;    // Value of best division factor match
  float best_delta = 30.0; // Best requested vs actual FPS (init to "way off")

  for (uint8_t p = 0; p < num_plls; p++) {
    uint32_t xclk_pll = OV7670_XCLK_HZ * pll_ratio[p]; // PLL'd freq
    uint8_t first_div = p ? 2 : 1; // Min div is 1 for PLL 1:1, else 2
    for (uint8_t div = first_div; div <= 32; div++) {
      uint32_t pclk_result = xclk_pll / div; // PCLK-up-down permutation
      if (pclk_result > pclk_target) {       // Exceeds target?
        continue;                            //  Skip it
      }
      float fps_result = (float)pclk_result * 5.0 / 4000000.0;
      float delta = fps - fps_result; // How far off?
      if (delta < best_delta) {       // Best match yet?
        best_delta = delta;           //  Save delta,
        best_pll = p;                 //  pll and
        best_div = div;               //  div for later use
      }
    }
  }

  // Set up DBLV and CLKRC registers with best PLL and div values
  if (pll_ratio[best_pll] == best_div) { // If PLL and div are same (1:1)
    // Bypass PLL, use external clock directly
    writeRegister(OV7670_REG_DBLV, 0);
    writeRegister(OV7670_REG_CLKRC, 0x40);
  } else {
    // Set DBLV[7:6] for PLL, CLKRC[5:0] for div-1 (1-32 stored as 0-31)
    writeRegister(OV7670_REG_DBLV, best_pll << 6);
    writeRegister(OV7670_REG_CLKRC, best_div - 1);
  }

  return fps - best_delta; // Return actual frame rate
}

ICAP_status Adafruit_iCap_OV7670::setSize(OV7670_size size, ICAP_realloc allo) {
  uint16_t new_width = 640 >> (int)size;
  uint16_t new_height = 480 >> (int)size;
  uint32_t new_buffer_size = new_width * new_height * sizeof(uint16_t);
  bool ra = false;

  switch (allo) {
  case ICAP_REALLOC_NONE:
    if (new_buffer_size > buffer_size) { // New size won't fit
      // Don't realloc. Keep current camera settings, return error.
      return ICAP_STATUS_ERR_MALLOC;
    }
    break;
  case ICAP_REALLOC_CHANGE:
    ra = (new_buffer_size != buffer_size); // Realloc on size change
    break;
  case ICAP_REALLOC_LARGER:
    ra = (new_buffer_size > buffer_size); // Realloc on size increase
    break;
  }

  if (ra) { // Reallocate?
    uint16_t *new_buffer = (uint16_t *)realloc(buffer, new_buffer_size);
    if (new_buffer == NULL) { // FAIL
      _width = _height = buffer_size = 0;
      buffer[0] = NULL;
      // Calling code had better poll width(), height() or getBuffer() in
      // this case so it knows the camera buffer is gone, doesn't attempt
      // to read camera data into unknown RAM.
      return ICAP_STATUS_ERR_MALLOC;
    }
    buffer_size = new_buffer_size;
  }

  _width = new_width;
  _height = new_height;

  if (i2c_started) {
    // Array of five window settings, index of each (0-4) aligns with the five
    // OV7670_size enumeration values. If enum changes, list must change!
    static struct {
      uint8_t vstart;
      uint8_t hstart;
      uint8_t edge_offset;
      uint8_t pclk_delay;
    } window[] = {
        // Window settings were tediously determined empirically.
        // I hope there's a formula for this, if a do-over is needed.
        {9, 162, 2, 2},  // SIZE_DIV1  640x480 VGA
        {10, 174, 0, 2}, // SIZE_DIV2  320x240 QVGA
        {11, 186, 2, 2}, // SIZE_DIV4  160x120 QQVGA
        {12, 210, 0, 2}, // SIZE_DIV8  80x60   ...
        {15, 252, 3, 2}, // SIZE_DIV16 40x30
    };

    frameControl(size, window[size].vstart, window[size].hstart,
                 window[size].edge_offset, window[size].pclk_delay);
  }

  return ICAP_STATUS_OK;
}

// Sets up PCLK dividers and sets H/V start/stop window. Rather than
// rolling this into OV7670_set_size(), it's kept separate so test code
// can experiment with different settings to find ideal defaults.
void Adafruit_iCap_OV7670::frameControl(OV7670_size size, uint8_t vstart,
  uint16_t hstart, uint8_t edge_offset, uint8_t pclk_delay) {
  uint8_t value;

  // Enable downsampling if sub-VGA, and zoom if 1:16 scale
  value = (size > OV7670_SIZE_DIV1) ? OV7670_COM3_DCWEN : 0;
  if (size == OV7670_SIZE_DIV16)
    value |= OV7670_COM3_SCALEEN;
  writeRegister(OV7670_REG_COM3, value);

  // Enable PCLK division if sub-VGA 2,4,8,16 = 0x19,1A,1B,1C
  value = (size > OV7670_SIZE_DIV1) ? (0x18 + size) : 0;
  writeRegister(OV7670_REG_COM14, value);

  // Horiz/vert downsample ratio, 1:8 max (H,V are always equal for now)
  value = (size <= OV7670_SIZE_DIV8) ? size : OV7670_SIZE_DIV8;
  writeRegister(OV7670_REG_SCALING_DCWCTR, value * 0x11);

  // Pixel clock divider if sub-VGA
  value = (size > OV7670_SIZE_DIV1) ? (0xF0 + size) : 0x08;
  writeRegister(OV7670_REG_SCALING_PCLK_DIV, value);

  // Apply 0.5 digital zoom at 1:16 size (others are downsample only)
  value = (size == OV7670_SIZE_DIV16) ? 0x40 : 0x20; // 0.5, 1.0
  // Read current SCALING_XSC and SCALING_YSC register values because
  // test pattern settings are also stored in those registers and we
  // don't want to corrupt anything there.
  uint8_t xsc = readRegister(OV7670_REG_SCALING_XSC);
  uint8_t ysc = readRegister(OV7670_REG_SCALING_YSC);
  xsc = (xsc & 0x80) | value; // Modify only scaling bits (not test pattern)
  ysc = (ysc & 0x80) | value;
  // Write modified result back to SCALING_XSC and SCALING_YSC
  writeRegister(OV7670_REG_SCALING_XSC, xsc);
  writeRegister(OV7670_REG_SCALING_YSC, ysc);

  // Window size is scattered across multiple registers.
  // Horiz/vert stops can be automatically calc'd from starts.
  uint16_t vstop = vstart + 480;
  uint16_t hstop = (hstart + 640) % 784;
  writeRegister(OV7670_REG_HSTART, hstart >> 3);
  writeRegister(OV7670_REG_HSTOP, hstop >> 3);
  writeRegister(OV7670_REG_HREF, (edge_offset << 6) | ((hstop & 0b111) << 3) |
                                 (hstart & 0b111));
  writeRegister(OV7670_REG_VSTART, vstart >> 2);
  writeRegister(OV7670_REG_VSTOP, vstop >> 2);
  writeRegister(OV7670_REG_VREF, ((vstop & 0b11) << 2) | (vstart & 0b11));

  writeRegister(OV7670_REG_SCALING_PCLK_DELAY, pclk_delay);
}








#if 0
OV7670_status OV7670_begin(OV7670_host *host, OV7670_colorspace colorspace,
                           OV7670_size size, float fps) {
  OV7670_status status;

  // I2C must already be set up and running (@ 100 KHz) in calling code

  // Do device-specific (but platform-agnostic) setup. e.g. on SAMD this
  // function will fiddle registers to start a timer for XCLK output and
  // enable the parallel capture peripheral.
  status = OV7670_arch_begin(host);
  if (status != OV7670_STATUS_OK) {
    return status;
  }


}
#endif
