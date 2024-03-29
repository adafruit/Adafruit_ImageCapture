#include <Adafruit_iCap_OV2640.h>
#include <Arduino.h>

#if defined(ICAP_FULL_SUPPORT)

Adafruit_iCap_OV2640::Adafruit_iCap_OV2640(OV2640_pins &pins, iCap_arch *arch,
                                           TwoWire &twi, uint16_t *pbuf,
                                           uint32_t pbufsize, uint8_t addr,
                                           uint32_t speed, uint32_t delay_us)
    : Adafruit_iCap_parallel((iCap_parallel_pins *)&pins, arch, pbuf, pbufsize,
                             (TwoWire *)&twi, addr, speed, delay_us) {}

Adafruit_iCap_OV2640::~Adafruit_iCap_OV2640() {}

// CAMERA STARTUP ----------------------------------------------------------

static const iCap_parallel_config OV2640_init[] = {
#if 0
// Ideas from esp32-camera
// not working yet (scrambled image)
        {OV2640_REG_RA_DLMT, OV2640_RA_DLMT_DSP},    // DSP bank select 0
        {0x2C, 0xFF},                                // Reserved
        {0x2E, 0xDF},                                // Reserved
        {OV2640_REG_RA_DLMT, OV2640_RA_DLMT_SENSOR}, // Sensor bank sel 1
        {0x3C, 0x32},                                // Reserved
        {OV2640_REG1_CLKRC, 0x01},                   // Clock reserved bit
        {OV2640_REG1_COM2, OV2640_COM2_DRIVE_3X},    // 3X drive select
        {OV2640_REG1_REG04,                          // Mirror + ?
         OV2640_REG04_HFLIP | 0x20 | OV2640_REG04_HREF0},
        {OV2640_REG1_COM8, 0xC0 | OV2640_COM8_BANDING | OV2640_COM8_AGC_AUTO |
                               OV2640_COM8_EXP_AUTO},
        {OV2640_REG1_COM9, OV2640_COM9_AGC_GAIN_8X | 0x08},
        {0x2C, 0x0C},
        {0x33, 0x78},
        {0x3A, 0x33},
        {0x3B, 0xFB},
        {0x3E, 0x00},
        {0x43, 0x11},
        {0x16, 0x10},
        {0x39, 0x92},
        {0x35, 0xDA},
        {0x22, 0x1A},
        {0x37, 0xC3},
        {0x23, 0x00},
        {OV2640_REG1_ARCOM2, 0xC0},  // ?
        {0x06, 0x88},
        {0x07, 0xC0},
        {OV2640_REG1_COM4, 0x87},
        {0x0E, 0x41},
        {0x4C, 0x00},
        {0x4A, 0x81},
        {0x21, 0x99},
        {OV2640_REG1_AEW, 0x40}, // High range for AEC/AGC
        {OV2640_REG1_AEB, 0x38}, // Low range for AEC/AGC
        {OV2640_REG1_VV, 0x82},  // Fast mode thresholds
        {0x5C, 0x00},
        {0x63, 0x00},
        {OV2640_REG1_HISTO_LOW, 0x70},
        {OV2640_REG1_HISTO_HIGH, 0x80},
        {0x7C, 0x05}, // Reserved
        {0x20, 0x80}, // Reserved
        {0x28, 0x30}, // Reserved
        {0x6C, 0x00}, // Reserved
        {0x6D, 0x80}, // Reserved
        {0x6E, 0x00}, // Reserved
        {0x70, 0x02}, // Reserved
        {0x71, 0x94}, // Reserved
        {0x73, 0xC1}, // Reserved
        {0x3D, 0x34}, // Reserved
        {0x5A, 0x57}, // Reserved
        {OV2640_REG1_COM7, OV2640_COM7_RES_SVGA},
        {OV2640_REG1_HREFST, 0x11},  // Horiz window start MSB
        {OV2640_REG1_HREFEND, 0x43}, // Horiz window end MSB
        {OV2640_REG1_VSTRT, 0x00},   // Vert window start MSB
        {OV2640_REG1_VEND, 0x25},    // Vert window end MSB
        {OV2640_REG1_REG32, 0x89},   // Horiz window LSBs
        {0x37, 0xC0},                // Reserved
        {OV2640_REG1_BD50, 0xCA},    // 50 Hz banding AEC MSBs
        {OV2640_REG1_BD60, 0xA8},    // 60 Hz banding AEC MSBs
        {0x6D, 0x00},
        {0x3D, 0x38},
        {OV2640_REG_RA_DLMT, OV2640_RA_DLMT_DSP}, // DSP bank select 0
        {0xE5, 0x7F},                             // Reserved
        {OV2640_REG0_MC_BIST, OV2640_MC_BIST_RESET | OV2640_MC_BIST_BOOTROM},
        {0x41, 0x24}, // Reserved
        {OV2640_REG0_RESET, OV2640_RESET_JPEG | OV2640_RESET_DVP},
        {0x76, 0xFF}, // Reserved
        {0x33, 0xA0}, // Reserved
        {0x42, 0x20}, // Reserved
        {0x43, 0x18}, // Reserved
        {0x4C, 0x00}, // Reserved
        {OV2640_REG0_CTRL3, OV2640_CTRL3_WPC | 0x10},
        {0x88, 0x3F}, // Reserved
        {0xD7, 0x03}, // Reserved
        {0xD9, 0x10}, // Reserved
        {OV2640_REG0_R_DVP_SP, OV2640_R_DVP_SP_AUTO | 0x02},
        {0xC8, 0x08}, // Reserved
        {0xC9, 0x80}, // Reserved
        {OV2640_REG0_BPADDR, 0x00},
        {OV2640_REG0_BPDATA, 0x00},
        {OV2640_REG0_BPADDR, 0x03},
        {OV2640_REG0_BPDATA, 0x48},
        {OV2640_REG0_BPDATA, 0x48},
        {OV2640_REG0_BPADDR, 0x08},
        {OV2640_REG0_BPDATA, 0x20},
        {OV2640_REG0_BPDATA, 0x10},
        {OV2640_REG0_BPDATA, 0x0E},
        {0x90, 0x00}, // Reserved (addr/data?)
        {0x91, 0x0E}, // Reserved
        {0x91, 0x1A}, // Reserved
        {0x91, 0x31}, // Reserved
        {0x91, 0x5A}, // Reserved
        {0x91, 0x69}, // Reserved
        {0x91, 0x75}, // Reserved
        {0x91, 0x7E}, // Reserved
        {0x91, 0x88}, // Reserved
        {0x91, 0x8F}, // Reserved
        {0x91, 0x96}, // Reserved
        {0x91, 0xA3}, // Reserved
        {0x91, 0xAF}, // Reserved
        {0x91, 0xC4}, // Reserved
        {0x91, 0xD7}, // Reserved
        {0x91, 0xE8}, // Reserved
        {0x91, 0x20}, // Reserved
        {0x92, 0x00}, // Reserved (addr/data?)
        {0x93, 0x06}, // Reserved
        {0x93, 0xE3}, // Reserved
        {0x93, 0x05}, // Reserved  (0x02?)
        {0x93, 0x05}, // Reserved  (0x02?)
        {0x93, 0x00}, // Reserved
        {0x93, 0x04}, // Reserved
        {0x93, 0x00}, // Reserved
        {0x93, 0x00}, // Reserved  (0x03?)
        {0x93, 0x00}, // Reserved
        {0x93, 0x00}, // Reserved
        {0x93, 0x00}, // Reserved
        {0x93, 0x00}, // Reserved
        {0x93, 0x00}, // Reserved
        {0x93, 0x00}, // Reserved
        {0x96, 0x00}, // Reserved (addr/data?)
        {0x97, 0x08}, // Reserved
        {0x97, 0x19}, // Reserved
        {0x97, 0x02}, // Reserved
        {0x97, 0x0C}, // Reserved
        {0x97, 0x24}, // Reserved
        {0x97, 0x30}, // Reserved
        {0x97, 0x28}, // Reserved
        {0x97, 0x26}, // Reserved
        {0x97, 0x02}, // Reserved
        {0x97, 0x98}, // Reserved
        {0x97, 0x80}, // Reserved
        {0x97, 0x00}, // Reserved
        {0x97, 0x00}, // Reserved
        {0xA4, 0x00},
        {0xA8, 0x00},
        {0xC5, 0x11},
        {0xC6, 0x51},
        {0xBF, 0x80},
        {0xC7, 0x10},
        {0xB6, 0x66},
        {0xB8, 0xA5},
        {0xB7, 0x64},
        {0xB9, 0x7C},
        {0xB3, 0xAF},
        {0xB4, 0x97},
        {0xB5, 0xFF},
        {0xB0, 0xC5},
        {0xB1, 0x94},
        {0xB2, 0x0F},
        {0xC4, 0x5C},
        {OV2640_REG0_CTRL1, (uint8_t)~OV2640_CTRL1_LENC}, // All but LENC on
        {0x7F, 0x00},
        {0xE5, 0x1F},
        {0xE1, 0x67},
        {0xDD, 0x7F},
        {OV2640_REG0_RESET, 0}, // Go
        {OV2640_REG0_R_BYPASS, OV2640_R_BYPASS_DSP_ENABLE}},
#else
// Ideas from rp2040_ov2640-main repo
        // OV2640 camera initialization after reset
        // WIP STUFF, don't take this seriously yet
        {OV2640_REG_RA_DLMT, OV2640_RA_DLMT_DSP},    // DSP bank select 0
        {0x2C, 0xFF},                                // Reserved
        {0x2E, 0xDF},                                // Reserved
        {OV2640_REG_RA_DLMT, OV2640_RA_DLMT_SENSOR}, // Sensor bank sel 1
        {0x3C, 0x32},                                // Reserved
        {OV2640_REG1_CLKRC, 0x00},                   // Clock doubler OFF
        {OV2640_REG1_COM2, OV2640_COM2_DRIVE_2X},    // 2X drive select
        {OV2640_REG1_REG04,                          // Mirror + ?
         OV2640_REG04_HFLIP | 0x20 | OV2640_REG04_HREF0},
        {OV2640_REG1_COM8, 0xC0 | OV2640_COM8_BANDING | OV2640_COM8_AGC_AUTO |
                               OV2640_COM8_EXP_AUTO},
        {OV2640_REG1_COM9, OV2640_COM9_AGC_GAIN_8X | 0x08},
        {0x2C, 0x0c},            // Reserved
        {0x33, 0x78},            // Reserved
        {0x3A, 0x33},            // Reserved
        {0x3B, 0xfB},            // Reserved
        {0x3E, 0x00},            // Reserved
        {0x43, 0x11},            // Reserved
        {0x16, 0x10},            // Reserved
        {0x4A, 0x81},            // Reserved
        {0x21, 0x99},            // Reserved
//        {OV2640_REG1_AEW, 0x40}, // High range for AEC/AGC
//        {OV2640_REG1_AEB, 0x38}, // Low range for AEC/AGC
        {OV2640_REG1_AEW, 0xFF}, // High range for AEC/AGC
        {OV2640_REG1_AEB, 0x00}, // Low range for AEC/AGC
        {OV2640_REG1_VV, 0x82},  // Fast mode thresholds
        {0x5C, 0x00},            // Reserved
        {0x63, 0x00},            // Reserved
        {OV2640_REG1_FLL, 0x3F}, // Frame length adjustment LSBs
        {OV2640_REG1_COM3, 0x38 | OV2640_COM3_BANDING_50HZ},
        {OV2640_REG1_HISTO_LOW, 0x70},
        {OV2640_REG1_HISTO_HIGH, 0x80},
        {0x7C, 0x05}, // Reserved
        {0x20, 0x80}, // Reserved
        {0x28, 0x30}, // Reserved
        {0x6C, 0x00}, // Reserved
        {0x6D, 0x80}, // Reserved
        {0x6E, 0x00}, // Reserved
        {0x70, 0x02}, // Reserved
        {0x71, 0x94}, // Reserved
        {0x73, 0xC1}, // Reserved
        {0x3D, 0x34}, // Reserved
        {0x5A, 0x57}, // Reserved
        {OV2640_REG1_COM7, OV2640_COM7_RES_UXGA},
        {OV2640_REG1_CLKRC, 0x00},   // Clock doubler off
        {OV2640_REG1_HREFST, 0x11},  // Horiz window start MSB
        {OV2640_REG1_HREFEND, 0x75}, // Horiz window end MSB
        {OV2640_REG1_VSTRT, 0x01},   // Vert window start MSB
        {OV2640_REG1_VEND, 0x97},    // Vert window end MSB
        {OV2640_REG1_REG32, 0x36},   // Horiz window LSBs
        {OV2640_REG1_COM1, 0x0F},    // Vert window LSBs
        {0x37, 0x40},                // Reserved
        {OV2640_REG1_BD50, 0xBB},    // 50 Hz banding AEC MSBs
        {OV2640_REG1_BD60, 0x9C},    // 60 Hz banding AEC MSBs
        {0x5A, 0x57},                // Reserved
        {0x6D, 0x80},                // Reserved
        {0x6D, 0x38},                // Reserved (2nd ref in a row?)
        {0x39, 0x02},                // Reserved
        {0x35, 0x88},                // Reserved
        {0x22, 0x0A},                // Reserved
        {0x37, 0x40},                // Reserved
        {0x23, 0x00},                // Reserved
        {OV2640_REG1_ARCOM2, 0xA0},  // ?
        {0x36, 0x1A},                // Reserved
        {0x06, 0x02},                // Reserved
        {0x07, 0xC0},                // Reserved
        {OV2640_REG1_COM4, 0xB7},
        {0x0E, 0x01},                             // Reserved
        {0x4C, 0x00},                             // Reserved
        {OV2640_REG_RA_DLMT, OV2640_RA_DLMT_DSP}, // DSP bank select 0
        {0xE5, 0x7F},                             // Reserved
        {OV2640_REG0_MC_BIST, OV2640_MC_BIST_RESET | OV2640_MC_BIST_BOOTROM},
        {0x41, 0x24}, // Reserved
        {OV2640_REG0_RESET, OV2640_RESET_JPEG | OV2640_RESET_DVP},
        {0x76, 0xFF}, // Reserved
        {0x33, 0xA0}, // Reserved
        {0x42, 0x20}, // Reserved
        {0x43, 0x18}, // Reserved
        {0x4C, 0x00}, // Reserved
        {OV2640_REG0_CTRL3, OV2640_CTRL3_BPC | OV2640_CTRL3_WPC | 0x10},
        {0x88, 0x3F}, // Reserved
        {0xD7, 0x03}, // Reserved
        {0xD9, 0x10}, // Reserved
        {OV2640_REG0_R_DVP_SP, OV2640_R_DVP_SP_AUTO | 0x02},
        {0xC8, 0x08}, // Reserved
        {0xC9, 0x80}, // Reserved
        {OV2640_REG0_BPDATA, 0x00},
        {OV2640_REG0_BPADDR, 0x03},
        {OV2640_REG0_BPDATA, 0x48},
        {OV2640_REG0_BPADDR, 0x08},
        {OV2640_REG0_BPDATA, 0x20},
        {OV2640_REG0_BPDATA, 0x10},
        {OV2640_REG0_BPDATA, 0x0E},
        {0x90, 0x00}, // Reserved (addr/data?)
        {0x91, 0x0E}, // Reserved
        {0x91, 0x1A}, // Reserved
        {0x91, 0x31}, // Reserved
        {0x91, 0x5A}, // Reserved
        {0x91, 0x69}, // Reserved
        {0x91, 0x75}, // Reserved
        {0x91, 0x7E}, // Reserved
        {0x91, 0x88}, // Reserved
        {0x91, 0x8F}, // Reserved
        {0x91, 0x96}, // Reserved
        {0x91, 0xA3}, // Reserved
        {0x91, 0xaf}, // Reserved
        {0x91, 0xc4}, // Reserved
        {0x91, 0xd7}, // Reserved
        {0x91, 0xe8}, // Reserved
        {0x91, 0x20}, // Reserved
        {0x92, 0x00}, // Reserved (addr/data?)
        {0x93, 0x06}, // Reserved
        {0x93, 0xe3}, // Reserved
        {0x93, 0x02}, // Reserved
        {0x93, 0x02}, // Reserved
        {0x93, 0x00}, // Reserved
        {0x93, 0x04}, // Reserved
        {0x93, 0x00}, // Reserved
        {0x93, 0x03}, // Reserved
        {0x93, 0x00}, // Reserved
        {0x93, 0x00}, // Reserved
        {0x93, 0x00}, // Reserved
        {0x93, 0x00}, // Reserved
        {0x93, 0x00}, // Reserved
        {0x93, 0x00}, // Reserved
        {0x93, 0x00}, // Reserved (end data?)
        {0x96, 0x00}, // Reserved (addr/data?)
        {0x97, 0x08}, // Reserved
        {0x97, 0x19}, // Reserved
        {0x97, 0x02}, // Reserved
        {0x97, 0x0c}, // Reserved
        {0x97, 0x24}, // Reserved
        {0x97, 0x30}, // Reserved
        {0x97, 0x28}, // Reserved
        {0x97, 0x26}, // Reserved
        {0x97, 0x02}, // Reserved
        {0x97, 0x98}, // Reserved
        {0x97, 0x80}, // Reserved
        {0x97, 0x00}, // Reserved
        {0x97, 0x00}, // Reserved
        {OV2640_REG0_CTRL1, (uint8_t)~OV2640_CTRL1_DG},
        {OV2640_REG_RA_DLMT, OV2640_RA_DLMT_DSP}, // DSP bank select 0
        {0xBA, 0xDC},                             // Reserved
        {0xBB, 0x08},                             // Reserved
        {0xB6, 0x24},                             // Reserved
        {0xB8, 0x33},                             // Reserved
        {0xB7, 0x20},                             // Reserved
        {0xB9, 0x30},                             // Reserved
        {0xB3, 0xB4},                             // Reserved
        {0xB4, 0xCA},                             // Reserved
        {0xB5, 0x43},                             // Reserved
        {0xB0, 0x5C},                             // Reserved
        {0xB1, 0x4F},                             // Reserved
        {0xB2, 0x06},                             // Reserved
        {0xC7, 0x00},                             // Reserved
        {0xC6, 0x51},                             // Reserved
        {0xC5, 0x11},                             // Reserved
        {0xC4, 0x9C},                             // Reserved
        {0xBF, 0x00},                             // Reserved
        {0xBC, 0x64},                             // Reserved
        {0xA6, 0x00},                             // Reserved (addr/data?)
        {0xA7, 0x1E},                             // Reserved
        {0xA7, 0x6b},                             // Reserved
        {0xA7, 0x47},                             // Reserved
        {0xA7, 0x33},                             // Reserved
        {0xA7, 0x00},                             // Reserved
        {0xA7, 0x23},                             // Reserved
        {0xA7, 0x2E},                             // Reserved
        {0xA7, 0x85},                             // Reserved
        {0xA7, 0x42},                             // Reserved
        {0xA7, 0x33},                             // Reserved
        {0xA7, 0x00},                             // Reserved
        {0xA7, 0x23},                             // Reserved
        {0xA7, 0x1B},                             // Reserved
        {0xA7, 0x74},                             // Reserved
        {0xA7, 0x42},                             // Reserved
        {0xA7, 0x33},                             // Reserved
        {0xA7, 0x00},                             // Reserved
        {0xA7, 0x23},                             // Reserved
        {OV2640_REG0_HSIZE8, 0xC8},               // Horiz size MSBs
        {OV2640_REG0_VSIZE8, 0x96},               // Vert size MSBs
        {OV2640_REG0_SIZEL, 0x00},                // Size bits
        {OV2640_REG0_CTRL2, OV2640_CTRL2_DCW | OV2640_CTRL2_SDE |
                                OV2640_CTRL2_UV_ADJ | OV2640_CTRL2_UV_AVG |
                                OV2640_CTRL2_CMX},
        {OV2640_REG0_CTRLI, OV2640_CTRLI_LP_DP | 0x82}, // H/V dividers
        {OV2640_REG0_HSIZE, 0x90},                      // H_SIZE low bits
        {OV2640_REG0_VSIZE, 0x2C},                      // V_SIZE low bits
        {OV2640_REG0_XOFFL, 0x00},                      // OFFSET_X LSBs
        {OV2640_REG0_YOFFL, 0x00},                      // OFFSET_Y LSBs
        {OV2640_REG0_VHYX, 0x88},                       // V/H/Y/X MSBs
        {OV2640_REG0_ZMOW, 0x50},                       // OUTW low bits
        {OV2640_REG0_ZMOH, 0x3C},                       // OUTH low bits
        {OV2640_REG0_ZMHH, 0x00},                       // OUTW/H high bits
        {OV2640_REG0_R_DVP_SP, 0x04},                   // Manual DVP PCLK
        {0x7F, 0x00},                                   // Reserved
//        {OV2640_REG0_IMAGE_MODE, 0x00},                 // YUV MSB first
        {0xE5, 0x1F},                                   // Reserved
        {0xE1, 0x67},                                   // Reserved
        {OV2640_REG0_RESET, 0x00},                      // Reset nothing?
        {0xDD, 0x7F},                                   // Reserved
        {OV2640_REG0_R_BYPASS, OV2640_R_BYPASS_DSP_ENABLE},
        {OV2640_REG_RA_DLMT, OV2640_RA_DLMT_DSP}, // DSP bank select 0
        {OV2640_REG0_RESET, OV2640_RESET_DVP},
        {OV2640_REG0_HSIZE8, 0xC8}, // Image horiz size MSBs
        {OV2640_REG0_VSIZE8, 0x96}, // Image vert size MSBs
        {OV2640_REG0_CTRL2, OV2640_CTRL2_DCW | OV2640_CTRL2_SDE |
                                OV2640_CTRL2_UV_ADJ | OV2640_CTRL2_UV_AVG |
                                OV2640_CTRL2_CMX},
        {OV2640_REG0_CTRLI, OV2640_CTRLI_LP_DP | 0x12},
        {OV2640_REG0_HSIZE, 0x90}, // H_SIZE low bits
        {OV2640_REG0_VSIZE, 0x2C}, // V_SIZE low bits
        {OV2640_REG0_XOFFL, 0x00}, // OFFSET_X low bits
        {OV2640_REG0_YOFFL, 0x00}, // OFFSET_y low bits
        {OV2640_REG0_VHYX, 0x88},  // V/H/Y/X high bits
        {OV2640_REG0_TEST, 0x00},
        {OV2640_REG0_ZMOW, 0x50},                 // OUTW low bits
        {OV2640_REG0_ZMOH, 0x3C},                 // OUTH low bits
        {OV2640_REG0_ZMHH, 0x00},                 // OUTW/H high bits
        {OV2640_REG0_R_DVP_SP, 0x04},             // Manual DVP PCLK
        {0xE0, 0x00},                             // Reset nothing?
        {OV2640_REG_RA_DLMT, OV2640_RA_DLMT_DSP}, // DSP bank select 0
        {OV2640_REG0_R_BYPASS, OV2640_R_BYPASS_DSP_ENABLE},
        {OV2640_REG0_IMAGE_MODE, OV2640_IMAGE_MODE_DVP_RGB565},
        {OV2640_REG0_IMAGE_MODE,
         OV2640_IMAGE_MODE_DVP_RGB565 | OV2640_IMAGE_MODE_BYTE_SWAP},
        {0x98, 0x00},                             // Reserved
        {0x99, 0x00},                             // Reserved
        {0x00, 0x00},                             // Reserved
        {OV2640_REG_RA_DLMT, OV2640_RA_DLMT_DSP}, // DSP bank select 0
        {OV2640_REG0_RESET, OV2640_RESET_DVP},
        {OV2640_REG0_HSIZE8, 0xC8}, // H_SIZE high bits
        {OV2640_REG0_VSIZE8, 0x96}, // V_SIZE high bits
        {OV2640_REG0_CTRL2, OV2640_CTRL2_DCW | OV2640_CTRL2_SDE |
                                OV2640_CTRL2_UV_ADJ | OV2640_CTRL2_UV_AVG |
                                OV2640_CTRL2_CMX},
        {OV2640_REG0_CTRLI, OV2640_CTRLI_LP_DP | 0x09},
        {OV2640_REG0_HSIZE, 0x90}, // H_SIZE low bits
        {OV2640_REG0_VSIZE, 0x2C}, // V_SIZE low bits
        {OV2640_REG0_XOFFL, 0x00}, // OFFSET_X low bits
        {OV2640_REG0_YOFFL, 0x00}, // OFFSET_Y low bits
        {OV2640_REG0_VHYX, 0x88},  // V/H/Y/X high bits
        {OV2640_REG0_TEST, 0x00},
        {OV2640_REG0_ZMOW, 0xA0},     // OUTW low bits
        {OV2640_REG0_ZMOH, 0x78},     // OUTH low bits
        {OV2640_REG0_ZMHH, 0x00},     // OUTW/H high bits
        {OV2640_REG0_R_DVP_SP, 0x02}, // Manual DVP PCLK setting
        {OV2640_REG0_RESET, 0x00}},   // Go
#endif
    OV2640_qqvga[] =
        {// Configure OV2640 for QQVGA output
         {OV2640_REG_RA_DLMT, OV2640_RA_DLMT_DSP}, // DSP bank select 0
         {OV2640_REG0_RESET, OV2640_RESET_DVP},
         {OV2640_REG0_HSIZE8, 0x64}, // HSIZE high bits
         {OV2640_REG0_VSIZE8, 0x4B}, // VSIZE high bits
         {OV2640_REG0_CTRL2, OV2640_CTRL2_DCW | OV2640_CTRL2_SDE |
                                 OV2640_CTRL2_UV_AVG | OV2640_CTRL2_CMX},
         {OV2640_REG0_CTRLI, OV2640_CTRLI_LP_DP | 0x12},
         {OV2640_REG0_HSIZE, 0xC8},    // H_SIZE low bits
         {OV2640_REG0_VSIZE, 0x96},    // V_SIZE low bits
         {OV2640_REG0_XOFFL, 0x00},    // OFFSET_X low bits
         {OV2640_REG0_YOFFL, 0x00},    // OFFSET_Y low bits
         {OV2640_REG0_VHYX, 0x00},     // V/H/Y/X high bits
         {OV2640_REG0_TEST, 0x00},     // ?
         {OV2640_REG0_ZMOW, 0x28},     // OUTW low bits
         {OV2640_REG0_ZMOH, 0x1E},     // OUTH low bits
         {OV2640_REG0_ZMHH, 0x00},     // OUTW/H high bits
         {OV2640_REG0_R_DVP_SP, 0x08}, // Manual DVP PCLK setting
         {OV2640_REG0_RESET, 0x00}},   // Go
    OV2640_rgb[] = {{OV2640_REG_RA_DLMT,
                     OV2640_RA_DLMT_DSP}, // DSP bank select 0
                    {OV2640_REG0_RESET, OV2640_RESET_DVP},
                    {OV2640_REG0_IMAGE_MODE, OV2640_IMAGE_MODE_DVP_RGB565 |
                                                 OV2640_IMAGE_MODE_BYTE_SWAP},
                    {0xD7, 0x03},               // Mystery init values
                    {0xE1, 0x77},               // seen in other examples
                    {OV2640_REG0_RESET, 0x00}}, // Go
    OV2640_yuv[] = {
        {OV2640_REG_RA_DLMT, OV2640_RA_DLMT_DSP}, // DSP bank select 0
        {OV2640_REG0_RESET, OV2640_RESET_DVP},
        {OV2640_REG0_IMAGE_MODE,
         OV2640_IMAGE_MODE_DVP_YUV | OV2640_IMAGE_MODE_BYTE_SWAP},
        {0xD7, 0x01},               // Mystery init values
        {0xE1, 0x67},               // seen in other examples
        {OV2640_REG0_RESET, 0x00}}; // Go

iCap_status Adafruit_iCap_OV2640::begin(void) {
  iCap_status status;

  // Initialize peripherals for parallel+I2C camera:
  status = Adafruit_iCap_parallel::begin();
  if (status != ICAP_STATUS_OK) {
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
    delayMicroseconds(1000);
    digitalWrite(pins.reset, HIGH);
  } else {                                                    // Soft reset
    writeRegister(OV2640_REG_RA_DLMT, OV2640_RA_DLMT_SENSOR); // Bank select 1
    writeRegister(OV2640_REG1_COM7, OV2640_COM7_SRST);        // System reset
  }
  delay(1); // Datasheet: tS:RESET = 1 ms

  // Init main camera settings
  writeList(OV2640_init, sizeof OV2640_init / sizeof OV2640_init[0]);

  // Further initialization for specific colorspaces, frame sizes, timing,
  // etc. are done in other functions.

  return ICAP_STATUS_OK;
}

iCap_status Adafruit_iCap_OV2640::begin(OV2640_size size, iCap_colorspace space,
                                        float fps, uint8_t nbuf) {
  iCap_status status = begin();
  if (status == ICAP_STATUS_OK) {
    status = config(size, space, fps, nbuf);
    if (status == ICAP_STATUS_OK) {
      resume();
    }
  }

  return status;
}

void Adafruit_iCap_OV2640::setColorspace(iCap_colorspace space) {
  if (colorspace == ICAP_COLOR_RGB565) {
    writeList(OV2640_rgb, sizeof OV2640_rgb / sizeof OV2640_rgb[0]);
  } else {
    writeList(OV2640_yuv, sizeof OV2640_yuv / sizeof OV2640_yuv[0]);
  }
}

iCap_status Adafruit_iCap_OV2640::config(OV2640_size size,
                                         iCap_colorspace space, float fps,
                                         uint8_t nbuf, iCap_realloc allo) {
  // RIGGED FOR QQVGA FOR NOW, 30 fps
  uint16_t width = 160;
  uint16_t height = 120;
  iCap_status status = bufferConfig(width, height, space, nbuf, allo);
  if (status == ICAP_STATUS_OK) {
    writeList(OV2640_qqvga, sizeof OV2640_qqvga / sizeof OV2640_qqvga[0]);
    if (fps > 0.0) {
      delayMicroseconds((int)(10000000.0 / fps)); // 10 frame settling time
    }
    dma_change(pixbuf[0], _width * _height);
    resume(); // Start DMA cycle
  }

  return status;
}

#endif // end ICAP_FULL_SUPPORT
