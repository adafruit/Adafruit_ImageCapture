#include <Adafruit_iCap_OV5640.h>
#include <Arduino.h>

#if defined(ICAP_FULL_SUPPORT)

Adafruit_iCap_OV5640::Adafruit_iCap_OV5640(OV5640_pins &pins, iCap_arch *arch,
                                           TwoWire &twi, uint16_t *pbuf,
                                           uint32_t pbufsize, uint8_t addr,
                                           uint32_t speed, uint32_t delay_us)
    : Adafruit_iCap_parallel((iCap_parallel_pins *)&pins, arch, pbuf, pbufsize,
                             (TwoWire *)&twi, addr, speed, delay_us) {}

Adafruit_iCap_OV5640::~Adafruit_iCap_OV5640() {}

// CAMERA STARTUP ----------------------------------------------------------

static const iCap_parallel_config16x8
    OV5640_init[] = {
        // OV5640 camera initialization after reset
        //{OV5640_REG_SYSTEM_CTROL0, 0x40}, // Power down
        {OV5640_REG_SCCB_SYSTEM_CTRL1, 0x02}, // Enable PLL
        {OV5640_REG_PAD_OUTPUT_ENABLE01, 0xFF}, // Output enable
        {OV5640_REG_PAD_OUTPUT_ENABLE02, 0xFF}, // all clocks & GPIO
        {OV5640_REG_PAD_CONTROL00, 0xC2}, // 4X drive, FREX enable
        {OV5640_REG_POLARITY_CTRL00, 0x20}, // PCLK +active, VSYNC -active
        {OV5640_REG_JPG_MODE_SELECT, 0x02}, // JPEG mode 2
{ OV5640_REG_ISP_CONTROL01, 0x83}, // turn color matrix, awb and SDE
        {OV5640_REG_SYSTEM_RESET00, 0x00}, // Enable all blocks
        {OV5640_REG_SYSTEM_RESET02, 0x1C}, // Reset JFIFO, SFIFO, JPG
        {OV5640_REG_CLOCK_ENABLE00, 0xFF}, // Enable various clocks
        {OV5640_REG_CLOCK_ENABLE02, 0xC3}, // and more (except JPEG)
        {OV5640_REG_ISP_CONTROL00, 0xA7}, // LENC, GMA, B+W pix cancel, etc
        {OV5640_REG_ISP_CONTROL01, 0xA3}, // +digital FX, scale, matrix, AWB
        {OV5640_REG_ISP_CONTROL03, 0x08}, // ? debug bit
        {0x370C, 0x02}, // Unknown "debug" registers,
        {0x3634, 0x40}, // but necessary
        // AEC/AGC
        {OV5640_REG_AEC_MAX_EXPO_60_HI, 0x03}, // Exposure limit 60 Hz
        {OV5640_REG_AEC_MAX_EXPO_60_LO, 0xD8},
        {OV5640_REG_B50_STEP_HI, 0x01},
        {OV5640_REG_B50_STEP_LO, 0x27},
        {OV5640_REG_B60_STEP_HI, 0x00},
        {OV5640_REG_B60_STEP_LO, 0xF6},
        {OV5640_REG_AEC_CTRL0D, 0x04},
        {OV5640_REG_AEC_CTRL0E, 0x03},
        {OV5640_REG_AEC_CTRL0F, 0x30}, // Stable range limits
        {OV5640_REG_AEC_CTRL10, 0x28},
        {OV5640_REG_AEC_CTRL11, 0x60}, // Fast zone high limit
        {OV5640_REG_AEC_CTRL13, 0x43},
        {OV5640_REG_AEC_MAX_EXPO_50_HI, 0x03}, // Exposire limit 50 Hz
        {OV5640_REG_AEC_MAX_EXPO_50_LO, 0xD8},
        {OV5640_REG_AEC_GAIN_CEIL_HI, 0x00}, // Gain ceiling
        {OV5640_REG_AEC_GAIN_CEIL_LO, 0xF8},
        {OV5640_REG_AEC_CTRL1B, 0x30}, // Stable range limits
        {OV5640_REG_AEC_CTRL1E, 0x26},
        {OV5640_REG_AEC_CTRL1F, 0x14}, // Fast zone low limit
        {0x3600, 0x08}, // Unknown VCM debug registers
        {0x3601, 0x33},
        {OV5640_REG_5060HZ_CTRL01, 0xA4}, // 50/60 Hz control
        {OV5640_REG_5060HZ_CTRL04, 0x28},
        {OV5640_REG_5060HZ_CTRL05, 0x98},
        {OV5640_REG_LIGHT_METER1_HI, 0x00},
        {OV5640_REG_LIGHT_METER1_LO, 0x08},
        {OV5640_REG_LIGHT_METER2_HI, 0x00},
        {OV5640_REG_LIGHT_METER2_LO, 0x1C},
        {OV5640_REG_SAMPLE_NUMBER_HI, 0x9C},
        {OV5640_REG_SAMPLE_NUMBER_LO, 0x40},
        {OV5640_REG_VFIFO_CTRL0C, 0x22}, // Disable JPEG footer
        {OV5640_REG_BLC_CTRL01, 0x02}, // BLC
        {OV5640_REG_BLC_CTRL04, 0x02},
        {OV5640_REG_AWB_CONTROL00, 0xFF}, // AWB
        {OV5640_REG_AWB_CONTROL01, 0xF2},
        {OV5640_REG_AWB_CONTROL02, 0x00},
        {OV5640_REG_AWB_CONTROL03, 0x14},
        {OV5640_REG_AWB_CONTROL04, 0x25},
        {OV5640_REG_AWB_CONTROL05, 0x24},
        {OV5640_REG_AWB_CONTROL_ADVANCED, 0x09},
        {0x5187, 0x09}, // AWB mystery debug registers
        {0x5188, 0x09},
        {0x5189, 0x75},
        {0x518A, 0x54},
        {0x518B, 0xE0},
        {0x518C, 0xB2},
        {0x518D, 0x42},
        {0x518E, 0x3D},
        {0x518F, 0x56},
        {0x5190, 0x46},
        {OV5640_REG_AWB_CONTROL17, 0xF8},
        {OV5640_REG_AWB_CONTROL18, 0x04},
        {OV5640_REG_AWB_CONTROL19, 0x70},
        {OV5640_REG_AWB_CONTROL20, 0xF0},
        {OV5640_REG_AWB_CONTROL21, 0xF0},
        {OV5640_REG_AWB_CONTROL22, 0x03},
        {OV5640_REG_AWB_CONTROL23, 0x01},
        {0x5198, 0x04}, // AWB mystery debug registers
        {0x5199, 0x12},
        {0x519A, 0x04},
        {0x519B, 0x00},
        {0x519C, 0x06},
        {0x519D, 0x82},
        {OV5640_REG_AWB_CONTROL30, 0x38},
#if 0
        {0x5381, 0x1E}, // Color matrix (saturation)
        {0x5382, 0x5B},
        {0x5383, 0x08},
        {0x5384, 0x0A},
        {0x5385, 0x7E},
        {0x5386, 0x88},
        {0x5387, 0x7C},
        {0x5388, 0x6C},
        {0x5389, 0x10},
        {0x538A, 0x01},
        {0x538B, 0x98},
#else
        {0x5381, 0x1D}, // Saturation 0 from CircuitPython code
        {0x5382, 0x60},
        {0x5383, 0x03},
        {0x5384, 0x0C},
        {0x5385, 0x78},
        {0x5386, 0x84},
        {0x5387, 0x7D},
        {0x5388, 0x6B},
        {0x5389, 0x12},
        {0x538A, 0x01},
        {0x538B, 0x98},
#endif
        {0x5300, 0x10}, // CIP control (sharpness)
        {0x5301, 0x10},
        {0x5302, 0x18},
        {0x5303, 0x19},
        {0x5304, 0x10},
        {0x5305, 0x10},
        {0x5306, 0x08}, // Denoise
        {0x5307, 0x16},
        {0x5308, 0x40},
        {0x5309, 0x10}, // Sharpness
        {0x530A, 0x10}, // Sharpness
        {0x530B, 0x04}, // Sharpness
        {0x530C, 0x06}, // Sharpness
        {0x5480, 0x01}, // Gamma
        {0x5481, 0x00},
        {0x5482, 0x1E},
        {0x5483, 0x3B},
        {0x5484, 0x58},
        {0x5485, 0x66},
        {0x5486, 0x71},
        {0x5487, 0x7D},
        {0x5488, 0x83},
        {0x5489, 0x8F},
        {0x548A, 0x98},
        {0x548B, 0xA6},
        {0x548C, 0xB8},
        {0x548D, 0xCA},
        {0x548E, 0xD7},
        {0x548F, 0xE3},
        {0x5490, 0x1D},
{0x3406, 0x00}, // Auto white balance (from CircuitPython code)
{0x3400, 0x04}, // R/G/B gains from same
{0x3401, 0x00},
{0x3402, 0x04},
{0x3403, 0x00},
{0x3404, 0x04},
{0x3405, 0x00},
        // Special Digital Effects (SDE) (UV adjust)
        {0x5580, 0x06}, // enable brightness and contrast
        {0x5583, 0x40}, // special_effect
        {0x5584, 0x10}, // special_effect
        {0x5586, 0x20}, // contrast +0
        {0x5585, 0x00}, // more contrast +0
        {0x5587, 0x00}, // brightness
        {0x5588, 0x00}, // brightness
        {0x5589, 0x10},
        {0x558A, 0x00},
        {0x558B, 0xF8},
        {0x501D, 0x40},                   // enable manual contrast offset
//        {OV5640_REG_SYSTEM_CTROL0, 0x02}, // Mystery debug bit, power on?
        {OV5640_REG_SYSTEM_CTROL0, 0x00}, // Software power on
        {OV5640_REG_5060HZ_CTRL00, 0x04}, // 50 Hz
        },
    OV5640_rgb[] = {
        {OV5640_REG_FORMAT_MUX_CONTROL, 0x01}, // RGB
        {OV5640_REG_FORMAT_CONTROL00, 0x6F}},  // RGB565 (RGB little-endian)
//Want one of these for CONTROL00 reg:
//0x61: RRRRRGGG GGGBBBBB {r[4:0], g[5:3]}, {g[2:0], b[4:0]} big-endian
//0x6F: GGGBBBBB RRRRRGGG {g[2:0], b[4:0]}, {r[4:0], g[5:3]} little-endian
    OV5640_yuv[] = {
        {OV5640_REG_FORMAT_MUX_CONTROL, 0x00}, // YUV422
        {OV5640_REG_FORMAT_CONTROL00, 0x30}},  // YUYV
    OV5640_gray[] = {
        {OV5640_REG_FORMAT_MUX_CONTROL, 0x00}, // YUV422
        {OV5640_REG_FORMAT_CONTROL00, 0x10}};  // YYYY

iCap_status Adafruit_iCap_OV5640::begin(void) {
  iCap_status status;

  // Initialize peripherals for parallel+I2C camera:
  status = Adafruit_iCap_parallel::begin();
  if (status != ICAP_STATUS_OK) {
    return status;
  }

  // ENABLE AND/OR RESET CAMERA --------------------------------------------
  // This is done here (rather than Adafruit_iCap_parallel) because pin
  // polarity, reset times, etc. might vary among cameras.

  if (pins.enable >= 0) { // Enable pin defined?
    pinMode(pins.enable, OUTPUT);
    digitalWrite(pins.enable, 0); // PWDN low (enable)
  }

  if (pins.reset >= 0) { // Hard reset pin defined?
    pinMode(pins.reset, OUTPUT);
    digitalWrite(pins.reset, LOW);
    delayMicroseconds(1000); // From datasheet power-up sequence 2.7
    digitalWrite(pins.reset, HIGH);
  } else { // Soft reset
    writeRegister16x8(OV5640_REG_SYSTEM_CTROL0, 0x80);
  }
  delay(20); // From datasheet power-up sequence 2.7

  // Init main camera settings
  writeList(OV5640_init, sizeof OV5640_init / sizeof OV5640_init[0]);

  // Further initialization for specific colorspaces, frame sizes, timing,
  // etc. are done in other functions.

  return ICAP_STATUS_OK;
}

iCap_status Adafruit_iCap_OV5640::begin(OV5640_size size, iCap_colorspace space,
                                        float fps, uint8_t nbuf) {
  iCap_status status = begin();
  if (status == ICAP_STATUS_OK) {
    status = config(size, space, fps, nbuf);
  }

  return status;
}

// CAMERA CONFIG FUNCTIONS AND MISCELLANY ----------------------------------

iCap_status Adafruit_iCap_OV5640::config(OV5640_size size,
                                         iCap_colorspace space, float fps,
                                         uint8_t nbuf, iCap_realloc allo) {
// For now, just to get a pic up, let's rig all the register init
// for a known fixed size. We'll use QQVGA because that's what the
// prior 2640/7670 code was doing. 160x120 pixels (4:3 aspect)
  uint16_t width = 160;
  uint16_t height = 120;
  suspend();
  iCap_status status = bufferConfig(width, height, space, nbuf, allo);
  if (status == ICAP_STATUS_OK) {

// This is all from the _set_size_and_colorspace() func in CircuitPython
// Here's the ratio table values for 4:3
//  mw,   mh,sx,sy,   ex,   ey, ox, oy,   tx,   ty
//2560, 1920, 0, 0, 2623, 1951, 32, 16, 2844, 1968

//self._binning = (width <= max_width // 2) and (height <= max_height // 2)
//self._scale = not (
//    (width == max_width and height == max_height)
//    or (width == max_width // 2 and height == max_height // 2)
//)
// binning is true because small, scale is true because not max size

    // Because binning true (use 0 for no binning)
    writeRegister16x8(OV5640_REG_TIMING_TC_REG20, 1);
    writeRegister16x8(OV5640_REG_TIMING_TC_REG21, 1);
    writeRegister16x8(0x4514, 0xAA);
    writeRegister16x8(0x4520, 0x0B);
    writeRegister16x8(OV5640_REG_TIMING_X_INC, 0x31);
    writeRegister16x8(OV5640_REG_TIMING_Y_INC, 0x31);

    //self._write_addr_reg(_X_ADDR_ST_H, start_x, start_y)
    //self._write_addr_reg(_X_ADDR_END_H, end_x, end_y)
    //self._write_addr_reg(_X_OUTPUT_SIZE_H, width, height)
    writeRegister16x16(OV5640_REG_TIMING_HS_HI, 0);
    writeRegister16x16(OV5640_REG_TIMING_VS_HI, 0);
    writeRegister16x16(OV5640_REG_TIMING_HW_HI, 2623);
    writeRegister16x16(OV5640_REG_TIMING_VH_HI, 1951);
    writeRegister16x16(OV5640_REG_TIMING_DVPHO_HI, 160);
    writeRegister16x16(OV5640_REG_TIMING_DVPVO_HI, 120);

    //self._write_addr_reg(_X_TOTAL_SIZE_H, 2060, total_y // 2)
    //self._write_addr_reg(_X_OFFSET_H, offset_x // 2, offset_y // 2)
    // Binning is true
    writeRegister16x16(OV5640_REG_TIMING_HTS_HI, 2060);
    writeRegister16x16(OV5640_REG_TIMING_VTS_HI, 1968 / 2);
    writeRegister16x16(OV5640_REG_TIMING_HOFFSET_HI, 32 / 2);
    writeRegister16x16(OV5640_REG_TIMING_VOFFSET_HI, 16 / 2);

    //uint8_t x = readRegister16x8(OV5640_REG_ISP_CONTROL01);
    //x |= 0x20; // Enable scale
    //writeRegister16x8(OV5640_REG_ISP_CONTROL01, x);
// Let's try just scale and AWB enable for now:
    writeRegister16x8(OV5640_REG_ISP_CONTROL01, 0x21);

    // Set up PLL (hardcoded for 160x120 case from Python code)
    //self._set_pll(False, 32, 1, 1, False, 1, True, 4)
    // bypass = False, multiplier = 32, sys_div = 1, pre_div = 1,
    // root_2x = False, pclk_root_div = 1, pclk_manual = True, pclk_div = 4
    writeRegister16x8(0x3039, 0);
    writeRegister16x8(0x3034, 0x1A);
    writeRegister16x8(0x3035, 1 | (1 << 4));
    writeRegister16x8(0x3036, 32);
    writeRegister16x8(0x3037, 1);
    writeRegister16x8(0x3108, (1 << 4) | 0x06);
    writeRegister16x8(0x3824, 4);
    writeRegister16x8(0x460C, 0x22);
    writeRegister16x8(0x3103, 0x13);
    // Getting a 20 MHz PCLK out from 12.5 MHz in, good

    setColorspace(space); // Select RGB/YUV/Grayscale

    delay(1000 / 30); // Settling time

#if 0
// This is old 7670-specific stuff, will update for 5640 later.
    setColorspace(space); // Select RGB/YUV/Grayscale
    fps = setFPS(fps);    // Frame timing
    frameControl(size, window[size].vstart, window[size].hstart,
                 window[size].edge_offset, window[size].pclk_delay);
    if (fps > 0.0) {
      delayMicroseconds((int)(10000000.0 / fps)); // 10 frame settling time
    }
#endif
// Note: if grayscale, this will change (1 byte/pixel)
    dma_change(pixbuf[0], _width * _height);
    resume(); // Start DMA cycle
  } else {
    // Stop cam
  }

  return status;
}

void Adafruit_iCap_OV5640::setColorspace(iCap_colorspace space) {
  switch (space) {
    case ICAP_COLORSPACE_RGB565:
      writeList(OV5640_rgb, sizeof OV5640_rgb / sizeof OV5640_rgb[0]);
      break;
    case ICAP_COLORSPACE_YUV:
      writeList(OV5640_yuv, sizeof OV5640_yuv / sizeof OV5640_yuv[0]);
      break;
    case ICAP_COLORSPACE_GRAYSCALE:
      writeList(OV5640_gray, sizeof OV5640_gray / sizeof OV5640_gray[0]);
      break;
  }
}

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

float Adafruit_iCap_OV5640::setFPS(float fps) {

// 5640 PLL control is in registers 0x3034-3039 SC_PLL_CONTRL*
// PLL multiplier can be any integer for 4-127 and any even
// integer for 128-252
// PLL root divider is bypass or /2
// PLL pre-divider is 1,2,3,4,6,8
// See also reg 3108

#if 0
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
#else
  return 0;
#endif
}

// Sets up PCLK dividers and sets H/V start/stop window. Rather than
// rolling this into OV7670_set_size(), it's kept separate so test code
// can experiment with different settings to find ideal defaults.
void Adafruit_iCap_OV5640::frameControl(OV5640_size size, uint8_t vstart,
                                        uint16_t hstart, uint8_t edge_offset,
                                        uint8_t pclk_delay) {
#if 0
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
  writeRegister(OV7670_REG_HREF,
                (edge_offset << 6) | ((hstop & 0b111) << 3) | (hstart & 0b111));
  writeRegister(OV7670_REG_VSTART, vstart >> 2);
  writeRegister(OV7670_REG_VSTOP, vstop >> 2);
  writeRegister(OV7670_REG_VREF, ((vstop & 0b11) << 2) | (vstart & 0b11));

  writeRegister(OV7670_REG_SCALING_PCLK_DELAY, pclk_delay);
}

// Select one of the camera's night modes (or disable).
// Trades off frame rate for less grainy images in low light.
// Note: seems that frame rate is somewhat automatic despite
//       the requested setting, i.e. if 1:8 is selected, might
//       still get normal frame rate or something higher than
//       1:8, if the scene lighting permits. Also the setting
//       seems to 'stick' in some cases when trying to turn
//       this off. Might want to try always having night mode
//       enabled but using 1:1 frame setting as 'off'.
void Adafruit_iCap_OV7670::night(OV7670_night_mode night) {
  // Table of bit patterns for the different supported night modes.
  // There's a "same frame rate" option for OV7670 night mode but it
  // doesn't seem to do anything useful and can be skipped over.
  static const uint8_t night_bits[] = {0b00000000, 0b10100000, 0b11000000,
                                       0b11100000};
  // Read current COM11 register setting so unrelated bits aren't corrupted
  uint8_t com11 = readRegister(OV7670_REG_COM11);
  com11 &= 0b00011111;        // Clear night mode bits
  com11 |= night_bits[night]; // Set night bits for desired mode
  // Write modified result back to COM11 register
  writeRegister(OV7670_REG_COM11, com11);
#endif
}


#if 0

_contrast_settings = [
    [0x20, 0x00], #  0
    [0x24, 0x10], # +1
    [0x28, 0x18], # +2
    [0x2c, 0x1c], # +3
    [0x14, 0x14], # -3
    [0x18, 0x18], # -2
    [0x1c, 0x1c], # -1
]

_sensor_saturation_levels = [
    [0x1D, 0x60, 0x03, 0x0C, 0x78, 0x84, 0x7D, 0x6B, 0x12, 0x01, 0x98],  # 0
    [0x1D, 0x60, 0x03, 0x0D, 0x84, 0x91, 0x8A, 0x76, 0x14, 0x01, 0x98],  # +1
    [0x1D, 0x60, 0x03, 0x0E, 0x90, 0x9E, 0x96, 0x80, 0x16, 0x01, 0x98],  # +2
    [0x1D, 0x60, 0x03, 0x10, 0x9C, 0xAC, 0xA2, 0x8B, 0x17, 0x01, 0x98],  # +3
    [0x1D, 0x60, 0x03, 0x11, 0xA8, 0xB9, 0xAF, 0x96, 0x19, 0x01, 0x98],  # +4
    [0x1D, 0x60, 0x03, 0x07, 0x48, 0x4F, 0x4B, 0x40, 0x0B, 0x01, 0x98],  # -4
    [0x1D, 0x60, 0x03, 0x08, 0x54, 0x5C, 0x58, 0x4B, 0x0D, 0x01, 0x98],  # -3
    [0x1D, 0x60, 0x03, 0x0A, 0x60, 0x6A, 0x64, 0x56, 0x0E, 0x01, 0x98],  # -2
    [0x1D, 0x60, 0x03, 0x0B, 0x6C, 0x77, 0x70, 0x60, 0x10, 0x01, 0x98],  # -1
]

_sensor_ev_levels = [
    [0x38, 0x30, 0x61, 0x38, 0x30, 0x10], #  0
    [0x40, 0x38, 0x71, 0x40, 0x38, 0x10], # +1
    [0x50, 0x48, 0x90, 0x50, 0x48, 0x20], # +2
    [0x60, 0x58, 0xa0, 0x60, 0x58, 0x20], # +3
    [0x10, 0x08, 0x10, 0x08, 0x20, 0x10], # -3
    [0x20, 0x18, 0x41, 0x20, 0x18, 0x10], # -2
    [0x30, 0x28, 0x61, 0x30, 0x28, 0x10], # -1
]

OV5640_WHITE_BALANCE_AUTO = 0
OV5640_WHITE_BALANCE_SUNNY = 1
OV5640_WHITE_BALANCE_FLUORESCENT = 2
OV5640_WHITE_BALANCE_CLOUDY = 3
OV5640_WHITE_BALANCE_INCANDESCENT = 4

_light_registers = [0x3406, 0x3400, 0x3401, 0x3402, 0x3403, 0x3404, 0x3405]
_light_modes = [
    [0x00, 0x04, 0x00, 0x04, 0x00, 0x04, 0x00], # auto
    [0x01, 0x06, 0x1c, 0x04, 0x00, 0x04, 0xf3], # sunny
    [0x01, 0x05, 0x48, 0x04, 0x00, 0x07, 0xcf], # office / fluorescent
    [0x01, 0x06, 0x48, 0x04, 0x00, 0x04, 0xd3], # cloudy
    [0x01, 0x04, 0x10, 0x04, 0x00, 0x08, 0x40], # home / incandescent

]

OV5640_SPECIAL_EFFECT_NONE = 0
OV5640_SPECIAL_EFFECT_NEGATIVE = 1
OV5640_SPECIAL_EFFECT_GRAYSCALE = 2
OV5640_SPECIAL_EFFECT_RED_TINT = 3
OV5640_SPECIAL_EFFECT_GREEN_TINT = 4
OV5640_SPECIAL_EFFECT_BLUE_TINT = 5
OV5640_SPECIAL_EFFECT_SEPIA = 6

_sensor_special_effects = [
    [0x06, 0x40, 0x10, 0x08],  # Normal
    [0x46, 0x40, 0x28, 0x08],  # Negative
    [0x1E, 0x80, 0x80, 0x08],  # Grayscale
    [0x1E, 0x80, 0xC0, 0x08],  # Red Tint
    [0x1E, 0x60, 0x60, 0x08],  # Green Tint
    [0x1E, 0xA0, 0x40, 0x08],  # Blue Tint
    [0x1E, 0x40, 0xA0, 0x08],  # Sepia
]

_sensor_regs_gamma0 = [
    0x5480, 0x01,
    0x5481, 0x08,
    0x5482, 0x14,
    0x5483, 0x28,
    0x5484, 0x51,
    0x5485, 0x65,
    0x5486, 0x71,
    0x5487, 0x7D,
    0x5488, 0x87,
    0x5489, 0x91,
    0x548A, 0x9A,
    0x548B, 0xAA,
    0x548C, 0xB8,
    0x548D, 0xCD,
    0x548E, 0xDD,
    0x548F, 0xEA,
    0x5490, 0x1D,
]

sensor_regs_gamma1 = [
    0x5480, 0x1,
    0x5481, 0x0,
    0x5482, 0x1E,
    0x5483, 0x3B,
    0x5484, 0x58,
    0x5485, 0x66,
    0x5486, 0x71,
    0x5487, 0x7D,
    0x5488, 0x83,
    0x5489, 0x8F,
    0x548A, 0x98,
    0x548B, 0xA6,
    0x548C, 0xB8,
    0x548D, 0xCA,
    0x548E, 0xD7,
    0x548F, 0xE3,
    0x5490, 0x1D,
]

sensor_regs_awb0 = [
    0x5180, 0xFF,
    0x5181, 0xF2,
    0x5182, 0x00,
    0x5183, 0x14,
    0x5184, 0x25,
    0x5185, 0x24,
    0x5186, 0x09,
    0x5187, 0x09,
    0x5188, 0x09,
    0x5189, 0x75,
    0x518A, 0x54,
    0x518B, 0xE0,
    0x518C, 0xB2,
    0x518D, 0x42,
    0x518E, 0x3D,
    0x518F, 0x56,
    0x5190, 0x46,
    0x5191, 0xF8,
    0x5192, 0x04,
    0x5193, 0x70,
    0x5194, 0xF0,
    0x5195, 0xF0,
    0x5196, 0x03,
    0x5197, 0x01,
    0x5198, 0x04,
    0x5199, 0x12,
    0x519A, 0x04,
    0x519B, 0x00,
    0x519C, 0x06,
    0x519D, 0x82,
    0x519E, 0x38,
]


@contrast.setter
    def contrast(self, value):
        if not -3 <= value <= 3:
            raise ValueError(
                "Invalid contrast value {value}, use a value from -3..3 inclusive"
            )
        setting = _contrast_settings[value]
        self._write_group_3_settings([0x5586, setting[0], 0x5585, setting[1]])

@saturation.setter
    def saturation(self, value):
        if not -4 <= value <= 4:
            raise ValueError(
                "Invalid saturation {value}, use a value from -4..4 inclusive"
            )
        for offset, reg_value in enumerate(_sensor_saturation_levels[value]):
            self._write_register(0x5381 + offset, reg_value)
        self._saturation = value

@exposure_value.setter
    def exposure_value(self, value):
        if not -3 <= value <= 3:
            raise ValueError(
                "Invalid exposure value (EV) {value}, use a value from -4..4 inclusive"
            )
        for offset, reg_value in enumerate(_sensor_ev_levels[value]):
            self._write_register(0x5381 + offset, reg_value)

@white_balance.setter
    def white_balance(self, value):
        if not OV5640_WHITE_BALANCE_AUTO <= value <= OV5640_WHITE_BALANCE_INCANDESCENT:
            raise ValueError(
                "Invalid exposure value (EV) {value}, "
                "use one of the OV5640_WHITE_BALANCE_* constants"
            )
        self._write_register(0x3212, 0x3)  # start group 3
        for reg_addr, reg_value in zip(_light_registers, _light_modes[value]):
            self._write_register(reg_addr, reg_value)
        self._write_register(0x3212, 0x13)  # end group 3
        self._write_register(0x3212, 0xA3)  # launch group 3

@effect.setter
    def effect(self, value):
        for reg_addr, reg_value in zip(
            (0x5580, 0x5583, 0x5584, 0x5003), _sensor_special_effects[value]
        ):
            self._write_register(reg_addr, reg_value)
        self._effect = value
#endif // 0

#endif // end ICAP_FULL_SUPPORT
