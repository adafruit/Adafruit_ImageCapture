#pragma once

#include <Adafruit_iCap_parallel.h>

// OV5640 datasheet claims 6-27 MHz clock input, with internal PLL to step
// up as needed. 24 MHz should be possible if camera connection is super
// clean, but we'll aim for around 12 MHz to minimize trouble.
#if defined(__SAMD51__)
// SAMD timer peripheral as used by this code is clocked from a 48 MHz
// source, so it's always going to be some integer divisor of that.
#define OV5640_XCLK_HZ 12000000 ///< XCLK to camera, 6-27 MHz
#elif defined(ARDUINO_ARCH_RP2040)
// Might want to derive this from F_CPU instead
#define OV5640_XCLK_HZ 12500000 ///< XCLK to camera, 6-27 MHz
#endif

#if defined(ICAP_FULL_SUPPORT)

typedef iCap_parallel_pins OV5640_pins;

#define OV5640_ADDR 0x3C //< Default I2C address if unspecified

/** Supported sizes THIS IS FAKE JUST TO GET THINGS TO COMPILE */
typedef enum {
  OV5640_SIZE_DIV1 = 0,
} OV5640_size;

/*!
    @brief  Class encapsulating OmniVision OV5640 functionality.
*/
class Adafruit_iCap_OV5640 : public Adafruit_iCap_parallel {
public:
  /*!
    @brief  Constructor for OV5640 camera class.
    @param  pins      OV5640_pins structure, describing physical connection
                      to the camera. Required.
    @param  arch      Pointer to iCap_arch structure containing
                      architecture-specific settings. For example, on
                      SAMD51, this structure includes a pointer to a timer
                      peripheral's base address, used to generate the xclk
                      signal. The structure is always of type iCap_arch, but
                      the specific elements within will vary with each
                      supported architecture.
    @param  twi       TwoWire instance (e.g. Wire or Wire1), used for I2C
                      communication with camera.
    @param  pbuf      Preallocated buffer for captured pixel data, or NULL
                      for library to allocate as needed when a camera
                      resolution is selected.
    @param  pbufsize  Size of passed-in buffer (or 0 if NULL).
    @param  addr      I2C address of camera.
    @param  speed     I2C communication speed to camera.
    @param  delay_us  Delay in microseconds between register writes.
  */
  Adafruit_iCap_OV5640(iCap_parallel_pins &pins, iCap_arch *arch = NULL,
                       TwoWire &twi = Wire, uint16_t *pbuf = NULL,
                       uint32_t pbufsize = 0, uint8_t addr = OV5640_ADDR,
                       uint32_t speed = 100000, uint32_t delay_us = 1000);
  ~Adafruit_iCap_OV5640();

  /*!
    @brief   Initialize peripherals behind an Adafruit_iCap_OV5640 instance,
             but do not actually start capture; must follow with a config()
             call for that.
    @return  Status code. ICAP_STATUS_OK on successful init.
  */
  iCap_status begin(void);

  /*!
    @brief   Initialize peripherals and allocate resources behind an
             Adafruit_iCap_OV5640 instance, start capturing data in
             background. Really just a one-step wrapper around begin(void)
             and config(...).
    @param   size   Frame size as a power-of-two reduction of VGA
                    resolution. Available sizes are OV7670_SIZE_DIV1
                    (640x480), OV7670_SIZE_DIV2 (320x240), OV7670_SIZE_DIV4
                    (160x120), OV7670_SIZE_DIV8 and OV7670_SIZE_DIV16.
                    This argument is required.
    @param   space  ICAP_COLOR_RGB565 (default) or ICAP_COLOR_YUV.
    @param   fps    Desired capture framerate, in frames per second, as a
                    float up to 30.0 (default). Actual device frame rate may
                    differ from this, depending on a host's available PWM
                    timing. Generally, the actual device fps will be equal
                    or nearest-available below the requested rate, only in
                    rare cases of extremely low requested frame rates will
                    a higher value be used. Since begin() only returns a
                    status code, if you need to know the actual framerate
                    you can call OV7670_set_fps(NULL, fps) at any time
                    before or after begin() and that will return the actual
                    resulting frame rate as a float.
    @param   nbuf   Number of full-image buffers, 1-3. For now, always use
                    1, multi-buffering isn't handled yet.
    @return  Status code. ICAP_STATUS_OK on successful init.
  */
  iCap_status begin(OV5640_size size, iCap_colorspace space = ICAP_COLOR_RGB565,
                    float fps = 30.0, uint8_t nbuf = 1);

  /*!
    @brief   Change frame configuration on an already-running camera.
    @param   size   Frame size as a power-of-two reduction of VGA
                    resolution. Available sizes are OV7670_SIZE_DIV1
                    (640x480), OV7670_SIZE_DIV2 (320x240), OV7670_SIZE_DIV4
                    (160x120), OV7670_SIZE_DIV8 and OV7670_SIZE_DIV16.
                    This argument is required.
    @param   space  ICAP_COLOR_RGB565 (default) or ICAP_COLOR_YUV.
    @param   fps    Desired capture framerate, in frames per second, as a
                    float up to 30.0 (default). Actual device frame rate may
                    differ from this, depending on a host's available PWM
                    timing. Generally, the actual device fps will be equal
                    or nearest-available below the requested rate, only in
                    rare cases of extremely low requested frame rates will
                    a higher value be used. Since begin() only returns a
                    status code, if you need to know the actual framerate
                    you can call OV7670_set_fps(NULL, fps) at any time
                    before or after begin() and that will return the actual
                    resulting frame rate as a float.
    @param   nbuf   Number of full-image buffers, 1-3. For now, always use
                    1, multi-buffering isn't handled yet.
    @param   allo   (Re-)allocation behavior. This value is IGNORED if a
                    static pixel buffer was passed to the constructor; it
                    only applies to dynamic allocation. ICAP_REALLOC_NONE
                    keeps the existing buffer (if new dimensions still fit),
                    ICAP_REALLOC_CHANGE will reallocate if the new dimensions
                    are smaller or larger than before. ICAP_REALLOC_LARGER
                    reallocates only if the new image specs won't fit in the
                    existing buffer (but ignoring reductions, some RAM will
                    go unused but avoids fragmentation).
    @return  Status code. ICAP_STATUS_OK on successful update, may return
             ICAP_STATUS_ERR_MALLOC if using dynamic allocation and the
             buffer resize fails.
  */
  iCap_status config(OV5640_size size,
                     iCap_colorspace space = ICAP_COLOR_RGB565,
                     float fps = 30.0, uint8_t nbuf = 1, 
                     iCap_realloc allo = ICAP_REALLOC_CHANGE);

  /*!
    @brief  Configure camera colorspace.
    @param  space  ICAP_COLOR_RGB565 or ICAP_COLOR_YUV.
  */
  void setColorspace(iCap_colorspace space = ICAP_COLOR_RGB565);

  // Configure camera frame rate. Actual resulting frame rate (returned) may
  // be different depending on available clock frequencies. Result will only
  // exceed input if necessary for minimum supported rate, but this is very
  // rare, typically below 1 fps. In all other cases, result will be equal
  // or less than the requested rate, up to a maximum of 30 fps (the "or less"
  // is because requested fps may be based on other host hardware timing
  // constraints (e.g. screen) and rounding up to a closer-but-higher frame
  // rate would be problematic). There is no hardcoded set of fixed frame
  // rates because it varies with architecture, depending on OV7670_XCLK_HZ.
  /*!
    @brief   Configure camera frame rate.
    @param   fps  Desired frames-per-second, floating-point value.
    @return  Nearest fps value supported by hardware.
  */
  float setFPS(float fps = 30.0);

  /*!
    @brief  Lower-level resolution register fiddling function, exposed so
            dev code can test variations for setSize() windowing defaults.
    @param  size         One of the OV7670_size enumeration values.
    @param  vstart       Vertical start.
    @param  hstart       Horizontal start.
    @param  edge_offset  Edge offset.
    @param  pclk_delay   PCLK delay.
  */
  void frameControl(OV5640_size size, uint8_t vstart, uint16_t hstart,
                    uint8_t edge_offset, uint8_t pclk_delay);

#if 0 // This is all very 7640-specific right now
  /*!
    @brief  Select one of the camera's night modes. Images are less
            grainy in low light, tradeoff being a reduced frame rate.
    @param  night  One of the OV7670_night_mode types:
                   OV7670_NIGHT_MODE_OFF  Disable night mode, full frame rate
                   OV7670_NIGHT_MODE_2    1/2 frame rate
                   OV7670_NIGHT_MODE_4    1/4 frame rate
                   OV7670_NIGHT_MODE_8    1/8 frame rate
  */
  void night(OV7670_night_mode night);

  /*!
    @brief  Flip camera output on horizontal and/or vertical axes.
            Flipping both axes is equivalent to 180 degree rotation.
    @param  flip_x  If true, flip camera output on horizontal axis.
    @param  flip_y  If true, flip camera output on vertical axis.
    @note   Datasheet refers to horizontal flip as "mirroring," but
            avoiding that terminology here that it might be mistaken for a
            split-down-middle-and-reflect funhouse effect, which it isn't.
  */
  void flip(bool flip_x, bool flip_y);

  /*!
    @brief  Enable/disable camera test pattern output.
    @param  pattern  One of the OV7670_pattern values:
                     OV7670_TEST_PATTERN_NONE
                       Disable test pattern, display normal camera video.
                     OV7670_TEST_PATTERN_SHIFTING_1
                       "Shifting 1" test pattern (seems to be single-column
                       vertical RGB lines).
                     OV7670_TEST_PATTERN_COLOR_BAR
                       Eight color bars.
                     OV7670_TEST_PATTERN_COLOR_BAR_FADE
                       Eight color bars with fade to white.
    @note   This basically works but has some artifacts...color bars are
            wrapped around such that a few pixels of the leftmost (white)
            bar appear at to the right of the rightmost (black) bar. It
            seems that the frame control settings need to be slightly
            different for image sensor vs test patterns (frame control
            that displays the bars correctly has green artifacts along
            right edge when using image sensor). Eventually will want to
            make this handle the different cases and sizes correctly.
            In the meantime, there's minor uglies in test mode.
            Similar issue occurs with image flips.
  */
  void test_pattern(OV7670_pattern pattern);
#endif // 0

private:
};

#endif // end ICAP_FULL_SUPPORT

// OV5640 registers
// System and IO pad control 0x3000-0x3052
#define OV5640_REG_SYSTEM_RESET00 0x3000 //< System reset 0
#define OV5640_REG_SYSTEM_RESET01 0x3001 //< System reset 1
#define OV5640_REG_SYSTEM_RESET02 0x3002 //< System reset 2
#define OV5640_REG_SYSTEM_RESET03 0x3003 //< System reset 3
#define OV5640_REG_CLOCK_ENABLE00 0x3004 //< Clock enable control 0
#define OV5640_REG_CLOCK_ENABLE01 0x3005 //< Clock enable control 1
#define OV5640_REG_CLOCK_ENABLE02 0x3006 //< Clock enable control 2
#define OV5640_REG_CLOCK_ENABLE03 0x3007 //< Clock enable control 3
#define OV5640_REG_SYSTEM_CTROL0  0x3008 //< System control
#define OV5640_REG_CHIP_ID_HIGH   0x300A //< Chip ID high byte
#define OV5640_REG_CHIP_ID_LOW    0x300B //< Chip ID low byte
#define OV5640_REG_MIPI_CONTROL00 0x300E //< MIPI control 0
#define OV5640_REG_PAD_OUTPUT_ENABLE00 0x3016 //< Input/output control 0
#define OV5640_REG_PAD_OUTPUT_ENABLE01 0x3017 //< Input/output control 1
#define OV5640_REG_PAD_OUTPUT_ENABLE02 0x3018 //< Input/output control 2
#define OV5640_REG_PAD_OUTPUT_VALUE00 0x3019 //< PAD output value 0
#define OV5640_REG_PAD_OUTPUT_VALUE01 0x301A //< GPIO output value 1
#define OV5640_REG_PAD_OUTPUT_VALUE02 0x301B //< GPIO output value 2
#define OV5640_REG_PAD_SELECT00 0x301C //< Output selection for GPIO 0
#define OV5640_REG_PAD_SELECT01 0x301D //< Output selection for GPIO 1
#define OV5640_REG_PAD_SELECT02 0x301E //< Output selection for GPIO 2
#define OV5640_REG_CHIP_REVISION 0x302A //< Process & chip revision
#define OV5640_REG_PAD_CONTROL00 0x302C //< Pad control
#define OV5640_REG_SC_PWC 0x3031 //< PWC control
#define OV5640_REG_SC_PLL_CONTROL0 0x3034 //< PLL control 0
#define OV5640_REG_SC_PLL_CONTROL1 0x3035 //< PLL control 1
#define OV5640_REG_SC_PLL_CONTROL2 0x3036 //< PLL control 2
#define OV5640_REG_SC_PLL_CONTROL3 0x3037 //< PLL control 3
#define OV5640_REG_SC_PLL_CONTROL5 0x3039 //< PLL control 5
#define OV5640_REG_SC_PLLS_CTRL0 0x303A //< PLLS control 0
#define OV5640_REG_SC_PLLS_CTRL1 0x303B //< PLLS control 1
#define OV5640_REG_SC_PLLS_CTRL2 0x303C //< PLLS control 2
#define OV5640_REG_SC_PLLS_CTRL3 0x303D //< PLLS control 3
#define OV5640_REG_IO_PAD_VALUE0 0x3050 //< Read pad value
#define OV5640_REG_IO_PAD_VALUE1 0x3051 //< Read pad value
#define OV5640_REG_IO_PAD_VALUE2 0x3052 //< Pad input status
// SCCB control 0x3100-0x3108
#define OV5640_REG_SCCB_ID 0x3100 //< SCCB periph ID
#define OV5640_REG_SCCB_SYSTEM_CTRL0 0x3102 //< SCCB control registers 0
#define OV5640_REG_SCCB_SYSTEM_CTRL1 0x3103 //< SCCB control registers 1
#define OV5640_REG_SYSTEM_ROOT_DIVIDER 0x3108 //< Pad clock div for SCCB
// SRB control 0x3200-0x3211
#define OV5640_REG_GROUP_ADDR0 0x3200 //< SRAM group address 0
#define OV5640_REG_GROUP_ADDR1 0x3201 //< SRAM group address 1
#define OV5640_REG_GROUP_ADDR2 0x3202 //< SRAM group address 2
#define OV5640_REG_GROUP_ADDR3 0x3203 //< SRAM group address 3
#define OV5640_REG_SRM_GROUP_ACCESS 0x3212 //< SRM group access
#define OV5640_REG_SRM_GROUP_STATUS 0x3213 //< SRM group status
// AWB gain control 0x3400-0x3406
#define OV5640_REG_AWB_R_GAIN_HI 0x3400 //< AWB R gain 11:8
#define OV5640_REG_AWB_R_GAIN_LO 0x3401 //< AWB R gain 7:0
#define OV5640_REG_AWB_G_GAIN_HI 0x3402 //< AWB G gain 11:8
#define OV5640_REG_AWB_G_GAIN_LO 0x3403 //< AWB G gain 7:0
#define OV5640_REG_AWB_B_GAIN_HI 0x3404 //< AWB B gain 11:8
#define OV5640_REG_AWB_B_GAIN_LO 0x3405 //< AWB B gain 7:0
#define OV5640_REG_AWB_MANUAL_CONTROL 0x3406 //< AWB manual control
// AEC/AGC control 0x3500-0x350D
#define OV5640_REG_AEC_PK_EXPOSURE0 0x3500 //< AEC exposure output 0
#define OV5640_REG_AEC_PK_EXPOSURE1 0x3501 //< AEC exposure output 1
#define OV5640_REG_AEC_PK_EXPOSURE2 0x3502 //< AEC exposure output 2
#define OV5640_REG_AEC_PK_MANUAL 0x3503 //< AEC manual mode control
#define OV5640_REG_AEC_PK_REAL_GAIN_HI 0x350A //< AEC Real gain 9:8
#define OV5640_REG_AEC_PK_REAL_GAIN_LO 0x350B //< AEC Real gain 7:0
#define OV5640_REG_AEC_PK_VTS_HI 0x350C //< AEC VTS output 15:8
#define OV5640_REG_AEC_PK_VTS_LO 0x350D //< AEC VTS output 7:0
// VCM control 0x3600-0x3606
#define OV5640_REG_VCM_CONTROL0 0x3602 //< VCM control 0
#define OV5640_REG_VCM_CONTROL1 0x3603 //< VCM control 1
#define OV5640_REG_VCM_CONTROL2 0x3604 //< VCM control 2
#define OV5640_REG_VCM_CONTROL3 0x3605 //< VCM control 3
#define OV5640_REG_VCM_CONTROL4 0x3606 //< VCM control 4
// Timing control 0x3800-0x3821
#define OV5640_REG_TIMING_HS_HI 0x3800 //< X address start 11:8
#define OV5640_REG_TIMING_HS_LO 0x3801 //< X address start 7:0
#define OV5640_REG_TIMING_VS_HI 0x3802 //< Y address start 11:8
#define OV5640_REG_TIMING_VS_LO 0x3803 //< Y address start 7:0
#define OV5640_REG_TIMING_HW_HI 0x3804 //< X address end 11:8
#define OV5640_REG_TIMING_HW_LO 0x3805 //< X address end 7:0
#define OV5640_REG_TIMING_VH_HI 0x3806 //< Y address end 10:8
#define OV5640_REG_TIMING_VH_LO 0x3807 //< Y address end 7:0
#define OV5640_REG_TIMING_DVPHO_HI 0x3808 //< DVP output width 11:8
#define OV5640_REG_TIMING_DVPHO_LO 0x3809 //< DVP output width 7:0
#define OV5640_REG_TIMING_DVPVO_HI 0x380A //< DVP output height 10:8
#define OV5640_REG_TIMING_DVPVO_LO 0x380B //< DVP output height 7:0
#define OV5640_REG_TIMING_HTS_HI 0x380C //< Total horiz size 12:8
#define OV5640_REG_TIMING_HTS_LO 0x380D //< Total horiz size 7:0
#define OV5640_REG_TIMING_VTS_HI 0x380E //< Total vert size 15:8
#define OV5640_REG_TIMING_VTS_LO 0x380F //< Total vert size 7:0
#define OV5640_REG_TIMING_HOFFSET_HI 0x3810 //< ISP horiz offset 11:8
#define OV5640_REG_TIMING_HOFFSET_LO 0x3811 //< ISP horiz offset 7:0
#define OV5640_REG_TIMING_VOFFSET_HI 0x3812 //< ISP vert offset 10:8
#define OV5640_REG_TIMING_VOFFSET_LO 0x3813 //< ISP vert offset 7:0
#define OV5640_REG_TIMING_X_INC 0x3814 //< Horiz subsample increment
#define OV5640_REG_TIMING_Y_INC 0x3815 //< Vert subsample increment
#define OV5640_REG_HSYNC_START_HI 0x3816 //< HSYNC start point 11:8
#define OV5640_REG_HSYNC_START_LO 0x3817 //< HSYNC start point 7:0
#define OV5640_REG_HSYNC_WIDTH_HI 0x3818 //< HSYNC width 11:8
#define OV5640_REG_HSYNC_WIDTH_LO 0x3819 //< HSYNC width 7:0
#define OV5640_REG_TIMING_TC_REG20 0x3820 //< Timing control 20
#define OV5640_REG_TIMING_TC_REG21 0x3821 //< Timing control 21
// AEC/AGC power down domain control 0x3A00-0x3A25
#define OV5640_REG_AEC_CTRL00 0x3A00 //< AEC system control
#define OV5640_REG_AEC_MIN_EXPOSURE 0x3A01 //< Min exposure output limit
#define OV5640_REG_AEC_MAX_EXPO_60_HI 0x3A02 //< Max exposure 15:8
#define OV5640_REG_AEC_MAX_EXPO_60_LO 0x3A03 //< Max exposure 7:0
#define OV5640_REG_AEC_CTRL05 0x3A05 //< AEC system control 2
#define OV5640_REG_AEC_CTRL06 0x3A06 //< AEC system control 3
#define OV5640_REG_AEC_CTRL07 0x3A07 //< AEC manual step register
#define OV5640_REG_B50_STEP_HI 0x3A08 //< 50Hz bandwidth step 9:8
#define OV5640_REG_B50_STEP_LO 0x3A09 //< 50Hz bandwidth step 7:0
#define OV5640_REG_B60_STEP_HI 0x3A0A //< 60Hz bandwidth step 13:8
#define OV5640_REG_B60_STEP_LO 0x3A0B //< 60Hz bandwidth step 7:0
#define OV5640_REG_AEC_CTRL0C 0x3A0C //< E1 min/max
#define OV5640_REG_AEC_CTRL0D 0x3A0D //< 60 Hz bands in one frame
#define OV5640_REG_AEC_CTRL0E 0x3A0E //< 50 Hz bands in one frame
#define OV5640_REG_AEC_CTRL0F 0x3A0F //< Stable range high limit
#define OV5640_REG_AEC_CTRL10 0x3A10 //< Stable range low limit
#define OV5640_REG_AEC_CTRL11 0x3A11 //< Fast zone high limit
#define OV5640_REG_AEC_CTRL13 0x3A13 //< AEC control 13
#define OV5640_REG_AEC_MAX_EXPO_50_HI 0x3A14 //< 50 Hz max exposure 15:8
#define OV5640_REG_AEC_MAX_EXPO_50_LO 0x3A15 //< 50 Hz max exposure 7:0
#define OV5640_REG_AEC_CTRL17 0x3A17 //< Gain base in night mode
#define OV5640_REG_AEC_GAIN_CEIL_HI 0x3A18 //< Gain ceiling 9:8
#define OV5640_REG_AEC_GAIN_CEIL_LO 0x3A19 //< Gain ceiling 7:0
#define OV5640_REG_AEC_DIFF_MIN 0x3A1A //< Difference minimal
#define OV5640_REG_AEC_CTRL1B 0x3A1B //< Stable range high limit
#define OV5640_REG_LED_ADD_ROW_HI 0x3A1C //< Exp add when strobe on 15:8
#define OV5640_REG_LED_ADD_ROW_LO 0x3A1D //< Exp add when strobe on 7:0
#define OV5640_REG_AEC_CTRL1E 0x3A1E //< Stable range low limit
#define OV5640_REG_AEC_CTRL1F 0x3A1F //< Fast zone low limit
#define OV5640_REG_AEC_CTRL20 0x3A20 //< AEC control 20
#define OV5640_REG_AEC_CTRL21 0x3A21 //< AEC control 21
#define OV5640_REG_AEC_CTRL25 0x3A25 //< AEC control 25
// Strobe control 0x3B00-0x3B0C
#define OV5640_REG_STROBE_CTRL 0x3B00 //< Strobe control
#define OV5640_REG_FREX_EXPOSURE02 0x3B01 //< FREX exposure time 23:16
#define OV5640_REG_FREX_SHUTTER_DELAY_HI 0x3B02 //< Shutter delay time 12:8
#define OV5640_REG_FREX_SHUTTER_DELAY_LO 0x3B03 //< Shutter delay time 7:0
#define OV5640_REG_FREX_EXPOSURE01 0x3B04 //< FREX exposure time 15:8
#define OV5640_REG_FREX_EXPOSURE00 0x3B05 //< FREX exposure time 7:0
#define OV5640_REG_FREX_CTRL07 0x3B06 //< FREX control 7
#define OV5640_REG_FREX_MODE 0x3B07 //< FREX mode select
#define OV5640_REG_FREX_REQUEST 0x3B08 //< FREX request
#define OV5640_REG_FREX_HREF_DELAY 0x3B09 //< FREX HREF delay
#define OV5640_REG_FREX_RST_LENGTH 0x3B0A //< FREX precharge length
#define OV5640_REG_STROBE_WIDTH_HI 0x3B0B //< Strobe width 19:12
#define OV5640_REG_STROBE_WIDTH_LO 0x3B0C //< Strobe width 11:4
// 50/60 Hz detector control 0x3C00-0x3C1E
#define OV5640_REG_5060HZ_CTRL00 0x3C00 //< 50/60 Hz control 0
#define OV5640_REG_5060HZ_CTRL01 0x3C01 //< 50/60 Hz control 1
#define OV5640_REG_5060HZ_CTRL02 0x3C02 //< 50/60 Hz control 2
#define OV5640_REG_5060HZ_CTRL03 0x3C03 //< 50/60 Hz control 3
#define OV5640_REG_5060HZ_CTRL04 0x3C04 //< 50/60 Hz control 4
#define OV5640_REG_5060HZ_CTRL05 0x3C05 //< 50/60 Hz control 5
#define OV5640_REG_LIGHT_METER1_HI 0x3C06 //< Light meter 1 threshold 15:8
#define OV5640_REG_LIGHT_METER1_LO 0x3C07 //< Light meter 1 threshold 7:0
#define OV5640_REG_LIGHT_METER2_HI 0x3C08 //< Light meter 2 threshold 15:8
#define OV5640_REG_LIGHT_METER2_LO 0x3C09 //< Light meter 2 threshold 7:0
#define OV5640_REG_SAMPLE_NUMBER_HI 0x3C0A //< Sample number 15:8
#define OV5640_REG_SAMPLE_NUMBER_LO 0x3C0B //< Sample number 7:0
#define OV5640_REG_SIGMADELTA_CTRL0C 0x3C0C //< Sigma delta bits
#define OV5640_REG_SUM50_0 0x3C0D //< Sum50 28:24
#define OV5640_REG_SUM50_1 0x3C0E //< Sum50 23:16
#define OV5640_REG_SUM50_2 0x3C0F //< Sum50 15:8
#define OV5640_REG_SUM50_3 0x3C10 //< Sum50 7:0
#define OV5640_REG_SUM60_0 0x3C11 //< Sum60 28:24
#define OV5640_REG_SUM60_1 0x3C12 //< Sum60 23:16
#define OV5640_REG_SUM60_2 0x3C13 //< Sum60 15:8
#define OV5640_REG_SUM60_3 0x3C14 //< Sum60 7:0
#define OV5640_REG_SUM5060_HI 0x3C15 //< SUM50/60 15:8
#define OV5640_REG_SUM5060_LO 0x3C16 //< SUM50/60 7:0
#define OV5640_REG_BLOCK_COUNTER_HI 0x3C17 //< Block counter 15:8
#define OV5640_REG_BLOCK_COUNTER_LO 0x3C18 //< Block counter 7:0
#define OV5640_REG_B6_HI 0x3C19 //< B6 15:8
#define OV5640_REG_B6_LO 0x3C1A //< B6 7:0
#define OV5640_REG_LIGHTMETER_HI 0x3C1B //< Light meter 19:16
#define OV5640_REG_LIGHTMETER_MID 0x3C1C //< Light meter 15:8
#define OV5640_REG_LIGHTMETER_LO 0x3C1D //< Light meter 7:0
#define OV5640_REG_SUM_THRESHOLD 0x3C1E //< Sum threshold
// OTP control 0x3D00-0x3D21
#define OV5640_REG_OTP_DATA00 0x3D00 //< OTP dump/load data 00
#define OV5640_REG_OTP_DATA01 0x3D01 //< OTP dump/load data 01
#define OV5640_REG_OTP_DATA02 0x3D02 //< OTP dump/load data 02
#define OV5640_REG_OTP_DATA03 0x3D03 //< OTP dump/load data 03
#define OV5640_REG_OTP_DATA04 0x3D04 //< OTP dump/load data 04
#define OV5640_REG_OTP_DATA05 0x3D05 //< OTP dump/load data 05
#define OV5640_REG_OTP_DATA06 0x3D06 //< OTP dump/load data 06
#define OV5640_REG_OTP_DATA07 0x3D07 //< OTP dump/load data 07
#define OV5640_REG_OTP_DATA08 0x3D08 //< OTP dump/load data 08
#define OV5640_REG_OTP_DATA09 0x3D09 //< OTP dump/load data 09
#define OV5640_REG_OTP_DATA0A 0x3D0A //< OTP dump/load data 0A
#define OV5640_REG_OTP_DATA0B 0x3D0B //< OTP dump/load data 0B
#define OV5640_REG_OTP_DATA0C 0x3D0C //< OTP dump/load data 0C
#define OV5640_REG_OTP_DATA0D 0x3D0D //< OTP dump/load data 0D
#define OV5640_REG_OTP_DATA0E 0x3D0E //< OTP dump/load data 0E
#define OV5640_REG_OTP_DATA0F 0x3D0F //< OTP dump/load data 0F
#define OV5640_REG_OTP_DATA10 0x3D10 //< OTP dump/load data 10
#define OV5640_REG_OTP_DATA11 0x3D11 //< OTP dump/load data 11
#define OV5640_REG_OTP_DATA12 0x3D12 //< OTP dump/load data 12
#define OV5640_REG_OTP_DATA13 0x3D13 //< OTP dump/load data 13
#define OV5640_REG_OTP_DATA14 0x3D14 //< OTP dump/load data 14
#define OV5640_REG_OTP_DATA15 0x3D15 //< OTP dump/load data 15
#define OV5640_REG_OTP_DATA16 0x3D16 //< OTP dump/load data 16
#define OV5640_REG_OTP_DATA17 0x3D17 //< OTP dump/load data 17
#define OV5640_REG_OTP_DATA18 0x3D18 //< OTP dump/load data 18
#define OV5640_REG_OTP_DATA19 0x3D19 //< OTP dump/load data 19
#define OV5640_REG_OTP_DATA1A 0x3D1A //< OTP dump/load data 1A
#define OV5640_REG_OTP_DATA1B 0x3D1B //< OTP dump/load data 1B
#define OV5640_REG_OTP_DATA1C 0x3D1C //< OTP dump/load data 1C
#define OV5640_REG_OTP_DATA1D 0x3D1D //< OTP dump/load data 1D
#define OV5640_REG_OTP_DATA1E 0x3D1E //< OTP dump/load data 1E
#define OV5640_REG_OTP_DATA1F 0x3D1F //< OTP dump/load data 1F
#define OV5640_REG_OTP_PROGRAM_CTRL 0x3D20 //< OTP program control
#define OV5640_REG_OTP_READ_CTRL 0x3D21 //< OTP read control
// MC control 0x3F00-0x3F0D
#define OV5640_REG_MC_CTRL00 0x3F00 //< MC control 00
#define OV5640_REG_MC_INTERRUPT_MASK0 0x3F01 //< Mask0 for interrupt
#define OV5640_REG_MC_INTERRUPT_MASK1 0x3F02 //< Mask1 for interrupt
#define OV5640_REG_MC_READ_INTERRUPT_ADDR_HI 0x3F03 //< SCCB addr high byte
#define OV5640_REG_MC_READ_INTERRUPT_ADDR_LO 0x3F04 //< SCCB addr low byte
#define OV5640_REG_MC_WRITE_INTERRUPT_ADDR_HI 0x3F05 //< SCCB addr high byte
#define OV5640_REG_MC_WRITE_INTERRUPT_ADDR_LO 0x3F06 //< SCCB addr low byte
#define OV5640_REG_MC_INTERRUPT_SOURCE1 0x3F08 //< Int1 source select
#define OV5640_REG_MC_INTERRUPT_SOURCE2 0x3F09 //< Int1 source select
#define OV5640_REG_MC_INTERRUPT_SOURCE3 0x3F0A //< Int0 source select
#define OV5640_REG_MC_INTERRUPT_SOURCE4 0x3F0B //< Int0 source select
#define OV5640_REG_MC_INTERRUPT0_STATUS 0x3F0C //< Interrupt0 status
#define OV5640_REG_MC_INTERRUPT1_STATUS 0x3F0D //< Interrupt1 status
// BLC control 0x4000-0x4033
#define OV5640_REG_BLC_CTRL00 0x4000 //< BLC Control 00
#define OV5640_REG_BLC_CTRL01 0x4001 //< BLC Control 01
#define OV5640_REG_BLC_CTRL02 0x4002 //< BLC Control 02
#define OV5640_REG_BLC_CTRL03 0x4003 //< BLC Control 03
#define OV5640_REG_BLC_CTRL04 0x4004 //< BLC Control 04
#define OV5640_REG_BLC_CTRL05 0x4005 //< BLC Control 05
#define OV5640_REG_BLC_CTRL07 0x4007 //< BLC Control 07
#define OV5640_REG_BLACK_LEVEL 0x4009 //< Black level target @ 10-bit range
#define OV5640_REG_BLACK_LEVEL00_HI 0x402C //< Black level 00 [15:8]
#define OV5640_REG_BLACK_LEVEL00_LO 0x402D //< Black level 00 [7:0]
#define OV5640_REG_BLACK_LEVEL01_HI 0x402E //< Black level 01 [15:8]
#define OV5640_REG_BLACK_LEVEL01_LO 0x402F //< Black level 01 [7:0]
#define OV5640_REG_BLACK_LEVEL10_HI 0x4030 //< Black level 10 [15:8]
#define OV5640_REG_BLACK_LEVEL10_LO 0x4031 //< Black level 10 [7:0]
#define OV5640_REG_BLACK_LEVEL11_HI 0x4032 //< Black level 11 [15:8]
#define OV5640_REG_BLACK_LEVEL11_LO 0x4033 //< Black level 11 [7:0]
// Frame control 0x4201-0x4202
#define OV5640_REG_FRAME_CTRL01 0x4201 //< Control passed frame number
#define OV5640_REG_FRAME_CTRL02 0x4202 //< Control masked frame number
// Format control 0x4300-0x430D
#define OV5640_REG_FORMAT_CONTROL00 0x4300 //< Format control 00
#define OV5640_REG_FORMAT_CONTROL01 0x4301 //< Format control 01
#define OV5640_REG_YMAX_VALUE_HI 0x4302 //< Y max clip[9:8]
#define OV5640_REG_YMAX_VALUE_LO 0x4303 //< Y max clip[7:0]
#define OV5640_REG_YMIN_VALUE_HI 0x4304 //< Y min clip[9:8]
#define OV5640_REG_YMIN_VALUE_LO 0x4305 //< Y min clip[7:0]
#define OV5640_REG_UMAX_VALUE_HI 0x4306 //< U max clip[9:8]
#define OV5640_REG_UMAX_VALUE_LO 0x4307 //< U max clip[7:0]
#define OV5640_REG_UMIN_VALUE_HI 0x4308 //< U min clip[9:8]
#define OV5640_REG_UMIN_VALUE_LO 0x4309 //< U min clip[7:0]
#define OV5640_REG_VMAX_VALUE_HI 0x430A //< V max clip[9:8]
#define OV5640_REG_VMAX_VALUE_LO 0x430B //< V max clip[7:0]
#define OV5640_REG_VMIN_VALUE_HI 0x430C //< V min clip[9:8]
#define OV5640_REG_VMIN_VALUE_LO 0x430D //< V min clip[7:0]
// JPEG control 0x4400-0x4431
#define OV5640_REG_JPEG_CTRL00 0x4400 //< JPEG control 00
#define OV5640_REG_JPEG_CTRL01 0x4401 //< JPEG control 01
#define OV5640_REG_JPEG_CTRL02 0x4402 //< JPEG control 02
#define OV5640_REG_JPEG_CTRL03 0x4403 //< JPEG control 03
#define OV5640_REG_JPEG_CTRL04 0x4404 //< JPEG control 04
#define OV5640_REG_JPEG_CTRL05 0x4405 //< JPEG control 05
#define OV5640_REG_JPEG_CTRL06 0x4406 //< JPEG control 06
#define OV5640_REG_JPEG_CTRL07 0x4407 //< JPEG control 07
#define OV5640_REG_JPEG_ISI_CTRL0 0x4408 //< JPEG ISI control 0
#define OV5640_REG_JPEG_CTRL09 0x4409 //< JPEG control 09
#define OV5640_REG_JPEG_CTRL0A 0x440A //< JPEG control 0A
#define OV5640_REG_JPEG_CTRL0B 0x440B //< JPEG control 0B
#define OV5640_REG_JPEG_CTRL0C 0x440C //< JPEG control 0C
#define OV5640_REG_JPEG_QT_DATA 0x4410 //< QT data
#define OV5640_REG_JPEG_QT_ADDR 0x4411 //< QT address
#define OV5640_REG_JPEG_ISI_DATA 0x4412 //< ISI data
#define OV5640_REG_JPEG_ISI_CTRL1 0x4413 //< JPEG ISI control 1
#define OV5640_REG_JPEG_LENGTH_HI 0x4414 //< JPEG length[23:16]
#define OV5640_REG_JPEG_LENGTH_MID 0x4415 //< JPEG length[15:8]
#define OV5640_REG_JPEG_LENGTH_LO 0x4416 //< JPEG length[7:0]
#define OV5640_REG_JFIFO_OVERFLOW 0x4417 //< JFIFO overflow indicator
#define OV5640_REG_JPEG_COMMENT_START 0x4420 //< JPEG comment data start
#define OV5640_REG_JPEG_COMMENT_END 0x442F //< JPEG comment data end
#define OV5640_REG_JPEG_COMMENT_LENGTH 0x4430 //< JPEG comment length
#define OV5640_REG_JPEG_COMMENT_MARKER 0x4431 //< JPEG comment data marker
// VFIFO control 0x4600-0x460D
#define OV5640_REG_VFIFO_CTRL00 0x4600 //< VFIFO control 00
#define OV5640_REG_VFIFO_HSIZE_HI 0x4602 //< Compression output width MSB
#define OV5640_REG_VFIFO_HSIZE_LO 0x4603 //< Compression output width LSB
#define OV5640_REG_VFIFO_VSIZE_HI 0x4604 //< Compression output height MSB
#define OV5640_REG_VFIFO_VSIZE_LO 0x4605 //< Compression output height LSB
#define OV5640_REG_VFIFO_CTRL0C 0x460C //< VFIFO control 0C
#define OV5640_REG_VFIFO_CTRL0D 0x460D //< Dummy data
// DVP control 0x4709-0x4745
#define OV5640_REG_DVP_VSYNC_WIDTH0 0x4709 //< VSYNC width line unit
#define OV5640_REG_DVP_VSYNC_WIDTH1 0x470A //< VSYNC width PCLK[15:8]
#define OV5640_REG_DVP_VSYNC_WIDTH2 0x470B //< VSYNC width PCLK[7:0]
#define OV5640_REG_PAD_LEFT_CTRL 0x4711 //< HSYNC left pad pixels
#define OV5640_REG_PAD_RIGHT_CTRL 0x4712 //< HSYNC right pad pixels
#define OV5640_REG_JPG_MODE_SELECT 0x4713 //< JPEG mode select
#define OV5640_REG_656_DUMMY_LINE 0x4715 //< CCIR656 dummy line num
#define OV5640_REG_CCIR656_CTRL 0x4719 //< CCIR656 EAV/SAV option
#define OV5640_REG_HSYNC_CTRL00 0x471B //< HSYNC mode enable
#define OV5640_REG_VSYNC_CTRL 0x471D //< VSYNC mode
#define OV5640_REG_HREF_CTRL 0x471F //< HREF min blanking in JPEG
#define OV5640_REG_VERTICAL_START_OFFSET 0x4721 //< Vert start delay
#define OV5640_REG_VERTICAL_END_OFFSET 0x4722 //< Vert end delay
#define OV5640_REG_DVP_CTRL23 0x4723 //< DVP JPEG mode 456 skip line num
#define OV5640_REG_CCIR656_CTRL00 0x4730 //< CCIR656 control 00
#define OV5640_REG_CCIR656_CTRL01 0x4731 //< CCIR656 control 01
#define OV5640_REG_CCIR656_FS 0x4732 //< CCIR656 SYNC code frame start
#define OV5640_REG_CCIR656_FE 0x4733 //< CCIR656 SYNC code frame end
#define OV5640_REG_CCIR656_LS 0x4734 //< CCIR656 SYNC code line start
#define OV5640_REG_CCIR656_LE 0x4735 //< CCIR656 SYNC code line end
#define OV5640_REG_CCIR656_CTRL06 0x4736 //< CCIR656 control 06
#define OV5640_REG_CCIR656_CTRL07 0x4737 //< CCIR656 control 07
#define OV5640_REG_CCIR656_CTRL08 0x4738 //< CCIR656 control 08
#define OV5640_REG_POLARITY_CTRL00 0x4740 //< PCLK polarity
#define OV5640_REG_TEST_PATTERN 0x4741 //< Test pattern select/enable
#define OV5640_REG_DATA_ORDER 0x4745 //< DVP & output data order
// MIPI control 0x4800-0x4837
#define OV5640_REG_MIPI_CTRL00 0x4800 //< MIPI control 00
#define OV5640_REG_MIPI_CTRL01 0x4801 //< MIPI control 01
#define OV5640_REG_MIPI_CTRL05 0x4805 //< MIPI control 05
#define OV5640_REG_MIPI_DATA_ORDER 0x480A //< MIPI data order
#define OV5640_REG_MIN_HS_ZERO_HI 0x4818 //< hs_zero min[15:8]
#define OV5640_REG_MIN_HS_ZERO_LO 0x4819 //< hs_zero min[7:0]
#define OV5640_REG_MIN_HS_TRAIL_HI 0x481A //< hs_trail min[15:8]
#define OV5640_REG_MIN_HS_TRAIL_LO 0x481B //< hs_trail min[7:0]
#define OV5640_REG_MIN_CLK_ZERO_HI 0x481C //< clk_zero min[15:8]
#define OV5640_REG_MIN_CLK_ZERO_LO 0x481D //< clk_zero min[7:0]
#define OV5640_REG_MIN_CLK_PREPARE_HI 0x481E //< clk_prepare min[15:8]
#define OV5640_REG_MIN_CLK_PREPARE_LO 0x481F //< clk_prepare min[7:0]
#define OV5640_REG_MIN_CLK_POST_HI 0x4820 //< clk_post min[15:8]
#define OV5640_REG_MIN_CLK_POST_LO 0x4821 //< clk_post min[7:0]
#define OV5640_REG_MIN_CLK_TRAIL_HI 0x4822 //< clk_trail min[15:8]
#define OV5640_REG_MIN_CLK_TRAIL_LO 0x4823 //< clk_trail min[7:0]
#define OV5640_REG_MIN_LPX_PCLK_HI 0x4824 //< lpx_p min[15:8]
#define OV5640_REG_MIN_LPX_PCLK_LO 0x4825 //< lpx_p min[7:0]
#define OV5640_REG_MIN_HS_PREPARE_HI 0x4826 //< hs_prepare min[15:8]
#define OV5640_REG_MIN_HS_PREPARE_LO 0x4827 //< hs_prepare min[7:0]
#define OV5640_REG_MIN_HS_EXIT_HI 0x4828 //< hs_exit min[15:8]
#define OV5640_REG_MIN_HS_EXIT_LO 0x4829 //< hs_exit min[7:0]
#define OV5640_REG_MIN_HS_ZERO_UI 0x482A //< hs_zero UI min
#define OV5640_REG_MIN_HS_TRAIL_UI 0x482B //< hs_trail UI min
#define OV5640_REG_MIN_CLK_ZERO_UI 0x482C //< clk_zero UI min
#define OV5640_REG_MIN_CLK_PREPARE_UI 0x482D //< clk_prepare UI min
#define OV5640_REG_MIN_CLK_POST_UI 0x482E //< clk_post UI min
#define OV5640_REG_MIN_CLK_TRAIL_UI 0x482F //< clk_trail UI min
#define OV5640_REG_MIN_LPX_PCLK_UI 0x4830 //< lpx_p UI min
#define OV5640_REG_MIN_HS_PREPARE_UI 0x4831 //< hs_prepare UI min
#define OV5640_REG_MIN_HS_EXIT_UI 0x4832 //< hs_exit UI min
#define OV5640_REG_PCLK_PERIOD 0x4837 //< Period of pixel clock
// ISP frame control 0x4901-0x4902
#define OV5640_REG_ISP_FRAME_CTRL01 0x4901 //< Control passed frame number
#define OV5640_REG_ISP_FRAME_CTRL02 0x4902 //< Control masked frame number







#if 0


/** Supported sizes (VGA division factor) for OV7670_set_size() */
typedef enum {
  OV7670_SIZE_DIV1 = 0, ///< 640 x 480
  OV7670_SIZE_DIV2,     ///< 320 x 240
  OV7670_SIZE_DIV4,     ///< 160 x 120
  OV7670_SIZE_DIV8,     ///< 80 x 60
  OV7670_SIZE_DIV16,    ///< 40 x 30
} OV7670_size;

typedef enum {
  OV7670_TEST_PATTERN_NONE = 0,       ///< Disable test pattern
  OV7670_TEST_PATTERN_SHIFTING_1,     ///< "Shifting 1" pattern
  OV7670_TEST_PATTERN_COLOR_BAR,      ///< 8 color bars
  OV7670_TEST_PATTERN_COLOR_BAR_FADE, ///< Color bars w/fade to white
} OV7670_pattern;

typedef enum {
  OV7670_NIGHT_MODE_OFF = 0, ///< Disable night mode
  OV7670_NIGHT_MODE_2,       ///< Night mode 1/2 frame rate
  OV7670_NIGHT_MODE_4,       ///< Night mode 1/4 frame rate
  OV7670_NIGHT_MODE_8,       ///< Night mode 1/8 frame rate
} OV7670_night_mode;



// registers will be outside the FULL_SUPPORT ifdef


// OV7670 registers
#define OV7670_REG_GAIN 0x00               //< AGC gain bits 7:0 (9:8 in VREF)
#define OV7670_REG_BLUE 0x01               //< AWB blue channel gain
#define OV7670_REG_RED 0x02                //< AWB red channel gain
#define OV7670_REG_VREF 0x03               //< Vert frame control bits
#define OV7670_REG_COM1 0x04               //< Common control 1
#define OV7670_COM1_R656 0x40              //< COM1 enable R656 format
#define OV7670_REG_BAVE 0x05               //< U/B average level
#define OV7670_REG_GbAVE 0x06              //< Y/Gb average level
#define OV7670_REG_AECHH 0x07              //< Exposure value - AEC 15:10 bits
#define OV7670_REG_RAVE 0x08               //< V/R average level
#define OV7670_REG_COM2 0x09               //< Common control 2
#define OV7670_COM2_SSLEEP 0x10            //< COM2 soft sleep mode
#define OV7670_REG_PID 0x0A                //< Product ID MSB (read-only)
#define OV7670_REG_VER 0x0B                //< Product ID LSB (read-only)
#define OV7670_REG_COM3 0x0C               //< Common control 3
#define OV7670_COM3_SWAP 0x40              //< COM3 output data MSB/LSB swap
#define OV7670_COM3_SCALEEN 0x08           //< COM3 scale enable
#define OV7670_COM3_DCWEN 0x04             //< COM3 DCW enable
#define OV7670_REG_COM4 0x0D               //< Common control 4
#define OV7670_REG_COM5 0x0E               //< Common control 5
#define OV7670_REG_COM6 0x0F               //< Common control 6
#define OV7670_REG_AECH 0x10               //< Exposure value 9:2
#define OV7670_REG_CLKRC 0x11              //< Internal clock
#define OV7670_CLK_EXT 0x40                //< CLKRC Use ext clock directly
#define OV7670_CLK_SCALE 0x3F              //< CLKRC Int clock prescale mask
#define OV7670_REG_COM7 0x12               //< Common control 7
#define OV7670_COM7_RESET 0x80             //< COM7 SCCB register reset
#define OV7670_COM7_SIZE_MASK 0x38         //< COM7 output size mask
#define OV7670_COM7_PIXEL_MASK 0x05        //< COM7 output pixel format mask
#define OV7670_COM7_SIZE_VGA 0x00          //< COM7 output size VGA
#define OV7670_COM7_SIZE_CIF 0x20          //< COM7 output size CIF
#define OV7670_COM7_SIZE_QVGA 0x10         //< COM7 output size QVGA
#define OV7670_COM7_SIZE_QCIF 0x08         //< COM7 output size QCIF
#define OV7670_COM7_RGB 0x04               //< COM7 pixel format RGB
#define OV7670_COM7_YUV 0x00               //< COM7 pixel format YUV
#define OV7670_COM7_BAYER 0x01             //< COM7 pixel format Bayer RAW
#define OV7670_COM7_PBAYER 0x05            //< COM7 pixel fmt proc Bayer RAW
#define OV7670_COM7_COLORBAR 0x02          //< COM7 color bar enable
#define OV7670_REG_COM8 0x13               //< Common control 8
#define OV7670_COM8_FASTAEC 0x80           //< COM8 Enable fast AGC/AEC algo,
#define OV7670_COM8_AECSTEP 0x40           //< COM8 AEC step size unlimited
#define OV7670_COM8_BANDING 0x20           //< COM8 Banding filter enable
#define OV7670_COM8_AGC 0x04               //< COM8 AGC (auto gain) enable
#define OV7670_COM8_AWB 0x02               //< COM8 AWB (auto white balance)
#define OV7670_COM8_AEC 0x01               //< COM8 AEC (auto exposure) enable
#define OV7670_REG_COM9 0x14               //< Common control 9 - max AGC value
#define OV7670_REG_COM10 0x15              //< Common control 10
#define OV7670_COM10_HSYNC 0x40            //< COM10 HREF changes to HSYNC
#define OV7670_COM10_PCLK_HB 0x20          //< COM10 Suppress PCLK on hblank
#define OV7670_COM10_PCLK_REV 0x10         //< COM10 PCLK reverse
#define OV7670_COM10_HREF_REV 0x08         //< COM10 HREF reverse
#define OV7670_COM10_VS_EDGE 0x04          //< COM10 VSYNC chg on PCLK rising
#define OV7670_COM10_VS_NEG 0x02           //< COM10 VSYNC negative
#define OV7670_COM10_HS_NEG 0x01           //< COM10 HSYNC negative
#define OV7670_REG_HSTART 0x17             //< Horiz frame start high bits
#define OV7670_REG_HSTOP 0x18              //< Horiz frame end high bits
#define OV7670_REG_VSTART 0x19             //< Vert frame start high bits
#define OV7670_REG_VSTOP 0x1A              //< Vert frame end high bits
#define OV7670_REG_PSHFT 0x1B              //< Pixel delay select
#define OV7670_REG_MIDH 0x1C               //< Manufacturer ID high byte
#define OV7670_REG_MIDL 0x1D               //< Manufacturer ID low byte
#define OV7670_REG_MVFP 0x1E               //< Mirror / vert-flip enable
#define OV7670_MVFP_MIRROR 0x20            //< MVFP Mirror image
#define OV7670_MVFP_VFLIP 0x10             //< MVFP Vertical flip
#define OV7670_REG_LAEC 0x1F               //< Reserved
#define OV7670_REG_ADCCTR0 0x20            //< ADC control
#define OV7670_REG_ADCCTR1 0x21            //< Reserved
#define OV7670_REG_ADCCTR2 0x22            //< Reserved
#define OV7670_REG_ADCCTR3 0x23            //< Reserved
#define OV7670_REG_AEW 0x24                //< AGC/AEC upper limit
#define OV7670_REG_AEB 0x25                //< AGC/AEC lower limit
#define OV7670_REG_VPT 0x26                //< AGC/AEC fast mode op region
#define OV7670_REG_BBIAS 0x27              //< B channel signal output bias
#define OV7670_REG_GbBIAS 0x28             //< Gb channel signal output bias
#define OV7670_REG_EXHCH 0x2A              //< Dummy pixel insert MSB
#define OV7670_REG_EXHCL 0x2B              //< Dummy pixel insert LSB
#define OV7670_REG_RBIAS 0x2C              //< R channel signal output bias
#define OV7670_REG_ADVFL 0x2D              //< Insert dummy lines MSB
#define OV7670_REG_ADVFH 0x2E              //< Insert dummy lines LSB
#define OV7670_REG_YAVE 0x2F               //< Y/G channel average value
#define OV7670_REG_HSYST 0x30              //< HSYNC rising edge delay
#define OV7670_REG_HSYEN 0x31              //< HSYNC falling edge delay
#define OV7670_REG_HREF 0x32               //< HREF control
#define OV7670_REG_CHLF 0x33               //< Array current control
#define OV7670_REG_ARBLM 0x34              //< Array ref control - reserved
#define OV7670_REG_ADC 0x37                //< ADC control - reserved
#define OV7670_REG_ACOM 0x38               //< ADC & analog common - reserved
#define OV7670_REG_OFON 0x39               //< ADC offset control - reserved
#define OV7670_REG_TSLB 0x3A               //< Line buffer test option
#define OV7670_TSLB_NEG 0x20               //< TSLB Negative image enable
#define OV7670_TSLB_YLAST 0x04             //< TSLB UYVY or VYUY, see COM13
#define OV7670_TSLB_AOW 0x01               //< TSLB Auto output window
#define OV7670_REG_COM11 0x3B              //< Common control 11
#define OV7670_COM11_NIGHT 0x80            //< COM11 Night mode
#define OV7670_COM11_NMFR 0x60             //< COM11 Night mode frame rate mask
#define OV7670_COM11_HZAUTO 0x10           //< COM11 Auto detect 50/60 Hz
#define OV7670_COM11_BAND 0x08             //< COM11 Banding filter val select
#define OV7670_COM11_EXP 0x02              //< COM11 Exposure timing control
#define OV7670_REG_COM12 0x3C              //< Common control 12
#define OV7670_COM12_HREF 0x80             //< COM12 Always has HREF
#define OV7670_REG_COM13 0x3D              //< Common control 13
#define OV7670_COM13_GAMMA 0x80            //< COM13 Gamma enable
#define OV7670_COM13_UVSAT 0x40            //< COM13 UV saturation auto adj
#define OV7670_COM13_UVSWAP 0x01           //< COM13 UV swap, use w TSLB[3]
#define OV7670_REG_COM14 0x3E              //< Common control 14
#define OV7670_COM14_DCWEN 0x10            //< COM14 DCW & scaling PCLK enable
#define OV7670_REG_EDGE 0x3F               //< Edge enhancement adjustment
#define OV7670_REG_COM15 0x40              //< Common control 15
#define OV7670_COM15_RMASK 0xC0            //< COM15 Output range mask
#define OV7670_COM15_R10F0 0x00            //< COM15 Output range 10 to F0
#define OV7670_COM15_R01FE 0x80            //< COM15              01 to FE
#define OV7670_COM15_R00FF 0xC0            //< COM15              00 to FF
#define OV7670_COM15_RGBMASK 0x30          //< COM15 RGB 555/565 option mask
#define OV7670_COM15_RGB 0x00              //< COM15 Normal RGB out
#define OV7670_COM15_RGB565 0x10           //< COM15 RGB 565 output
#define OV7670_COM15_RGB555 0x30           //< COM15 RGB 555 output
#define OV7670_REG_COM16 0x41              //< Common control 16
#define OV7670_COM16_AWBGAIN 0x08          //< COM16 AWB gain enable
#define OV7670_REG_COM17 0x42              //< Common control 17
#define OV7670_COM17_AECWIN 0xC0           //< COM17 AEC window must match COM4
#define OV7670_COM17_CBAR 0x08             //< COM17 DSP Color bar enable
#define OV7670_REG_AWBC1 0x43              //< Reserved
#define OV7670_REG_AWBC2 0x44              //< Reserved
#define OV7670_REG_AWBC3 0x45              //< Reserved
#define OV7670_REG_AWBC4 0x46              //< Reserved
#define OV7670_REG_AWBC5 0x47              //< Reserved
#define OV7670_REG_AWBC6 0x48              //< Reserved
#define OV7670_REG_REG4B 0x4B              //< UV average enable
#define OV7670_REG_DNSTH 0x4C              //< De-noise strength
#define OV7670_REG_MTX1 0x4F               //< Matrix coefficient 1
#define OV7670_REG_MTX2 0x50               //< Matrix coefficient 2
#define OV7670_REG_MTX3 0x51               //< Matrix coefficient 3
#define OV7670_REG_MTX4 0x52               //< Matrix coefficient 4
#define OV7670_REG_MTX5 0x53               //< Matrix coefficient 5
#define OV7670_REG_MTX6 0x54               //< Matrix coefficient 6
#define OV7670_REG_BRIGHT 0x55             //< Brightness control
#define OV7670_REG_CONTRAS 0x56            //< Contrast control
#define OV7670_REG_CONTRAS_CENTER 0x57     //< Contrast center
#define OV7670_REG_MTXS 0x58               //< Matrix coefficient sign
#define OV7670_REG_LCC1 0x62               //< Lens correction option 1
#define OV7670_REG_LCC2 0x63               //< Lens correction option 2
#define OV7670_REG_LCC3 0x64               //< Lens correction option 3
#define OV7670_REG_LCC4 0x65               //< Lens correction option 4
#define OV7670_REG_LCC5 0x66               //< Lens correction option 5
#define OV7670_REG_MANU 0x67               //< Manual U value
#define OV7670_REG_MANV 0x68               //< Manual V value
#define OV7670_REG_GFIX 0x69               //< Fix gain control
#define OV7670_REG_GGAIN 0x6A              //< G channel AWB gain
#define OV7670_REG_DBLV 0x6B               //< PLL & regulator control
#define OV7670_REG_AWBCTR3 0x6C            //< AWB control 3
#define OV7670_REG_AWBCTR2 0x6D            //< AWB control 2
#define OV7670_REG_AWBCTR1 0x6E            //< AWB control 1
#define OV7670_REG_AWBCTR0 0x6F            //< AWB control 0
#define OV7670_REG_SCALING_XSC 0x70        //< Test pattern X scaling
#define OV7670_REG_SCALING_YSC 0x71        //< Test pattern Y scaling
#define OV7670_REG_SCALING_DCWCTR 0x72     //< DCW control
#define OV7670_REG_SCALING_PCLK_DIV 0x73   //< DSP scale control clock divide
#define OV7670_REG_REG74 0x74              //< Digital gain control
#define OV7670_REG_REG76 0x76              //< Pixel correction
#define OV7670_REG_SLOP 0x7A               //< Gamma curve highest seg slope
#define OV7670_REG_GAM_BASE 0x7B           //< Gamma register base (1 of 15)
#define OV7670_GAM_LEN 15                  //< Number of gamma registers
#define OV7670_R76_BLKPCOR 0x80            //< REG76 black pixel corr enable
#define OV7670_R76_WHTPCOR 0x40            //< REG76 white pixel corr enable
#define OV7670_REG_RGB444 0x8C             //< RGB 444 control
#define OV7670_R444_ENABLE 0x02            //< RGB444 enable
#define OV7670_R444_RGBX 0x01              //< RGB444 word format
#define OV7670_REG_DM_LNL 0x92             //< Dummy line LSB
#define OV7670_REG_LCC6 0x94               //< Lens correction option 6
#define OV7670_REG_LCC7 0x95               //< Lens correction option 7
#define OV7670_REG_HAECC1 0x9F             //< Histogram-based AEC/AGC ctrl 1
#define OV7670_REG_HAECC2 0xA0             //< Histogram-based AEC/AGC ctrl 2
#define OV7670_REG_SCALING_PCLK_DELAY 0xA2 //< Scaling pixel clock delay
#define OV7670_REG_BD50MAX 0xA5            //< 50 Hz banding step limit
#define OV7670_REG_HAECC3 0xA6             //< Histogram-based AEC/AGC ctrl 3
#define OV7670_REG_HAECC4 0xA7             //< Histogram-based AEC/AGC ctrl 4
#define OV7670_REG_HAECC5 0xA8             //< Histogram-based AEC/AGC ctrl 5
#define OV7670_REG_HAECC6 0xA9             //< Histogram-based AEC/AGC ctrl 6
#define OV7670_REG_HAECC7 0xAA             //< Histogram-based AEC/AGC ctrl 7
#define OV7670_REG_BD60MAX 0xAB            //< 60 Hz banding step limit
#define OV7670_REG_ABLC1 0xB1              //< ABLC enable
#define OV7670_REG_THL_ST 0xB3             //< ABLC target
#define OV7670_REG_SATCTR 0xC9             //< Saturation control



#if 0

// from CircuitPython code:

/*
OV5640_COLOR_RGB = 0
OV5640_COLOR_YUV = 1
OV5640_COLOR_GRAYSCALE = 2
OV5640_COLOR_JPEG = 3

# fmt: off
_SYSTEM_CTROL0 = const(0x3008)
# Bit[7]: Software reset
# Bit[6]: Software power down
# Bit[5]: Reserved
# Bit[4]: SRB clock SYNC enable
# Bit[3]: Isolation suspend select
# Bit[2:0]: Not used

_CHIP_ID_HIGH = const(0x300A)

_DRIVE_CAPABILITY = const(0x302C)
# Bit[7:6]:
#          00: 1x
#          01: 2x
#          10: 3x
#          11: 4x

_SC_PLLS_CTRL0 = const(0x303A)
# Bit[7]: PLLS bypass
_SC_PLLS_CTRL1 = const(0x303B)
# Bit[4:0]: PLLS multiplier
_SC_PLLS_CTRL2 = const(0x303C)
# Bit[6:4]: PLLS charge pump control
# Bit[3:0]: PLLS system divider
_SC_PLLS_CTRL3 = const(0x303D)
# Bit[5:4]: PLLS pre-divider
#          00: 1
#          01: 1.5
#          10: 2
#          11: 3
# Bit[2]: PLLS root-divider - 1
# Bit[1:0]: PLLS seld5
#          00: 1
#          01: 1
#          10: 2
#          11: 2.5

# AEC/AGC control functions
_AEC_PK_MANUAL = const(0x3503)
# AEC Manual Mode Control
# Bit[7:6]: Reserved
# Bit[5]: Gain delay option
#         Valid when 0x3503[4]=1
#         0: Delay one frame latch
#         1: One frame latch
# Bit[4:2]: Reserved
# Bit[1]: AGC manual
#         0: Auto enable
#         1: Manual enable
# Bit[0]: AEC manual
#         0: Auto enable
#         1: Manual enable

# gain = {0x350A[1:0], 0x350B[7:0]} / 16


_X_ADDR_ST_H = const(0x3800)
# Bit[3:0]: X address start[11:8]
_X_ADDR_ST_L = const(0x3801)
# Bit[7:0]: X address start[7:0]
_Y_ADDR_ST_H = const(0x3802)
# Bit[2:0]: Y address start[10:8]
_Y_ADDR_ST_L = const(0x3803)
# Bit[7:0]: Y address start[7:0]
_X_ADDR_END_H = const(0x3804)
# Bit[3:0]: X address end[11:8]
_X_ADDR_END_L = const(0x3805)
# Bit[7:0]:
_Y_ADDR_END_H = const(0x3806)
# Bit[2:0]: Y address end[10:8]
_Y_ADDR_END_L = const(0x3807)
# Bit[7:0]:
# Size after scaling
_X_OUTPUT_SIZE_H = const(0x3808)
# Bit[3:0]: DVP output horizontal width[11:8]
_X_OUTPUT_SIZE_L = const(0x3809)
# Bit[7:0]:
_Y_OUTPUT_SIZE_H = const(0x380A)
# Bit[2:0]: DVP output vertical height[10:8]
_Y_OUTPUT_SIZE_L = const(0x380B)
# Bit[7:0]:
_X_TOTAL_SIZE_H = const(0x380C)
# Bit[3:0]: Total horizontal size[11:8]
_X_TOTAL_SIZE_L = const(0x380D)
# Bit[7:0]:
_Y_TOTAL_SIZE_H = const(0x380E)
# Bit[7:0]: Total vertical size[15:8]
_Y_TOTAL_SIZE_L = const(0x380F)
# Bit[7:0]:
_X_OFFSET_H = const(0x3810)
# Bit[3:0]: ISP horizontal offset[11:8]
_X_OFFSET_L = const(0x3811)
# Bit[7:0]:
_Y_OFFSET_H = const(0x3812)
# Bit[2:0]: ISP vertical offset[10:8]
_Y_OFFSET_L = const(0x3813)
# Bit[7:0]:
_X_INCREMENT = const(0x3814)
# Bit[7:4]: Horizontal odd subsample increment
# Bit[3:0]: Horizontal even subsample increment
_Y_INCREMENT = const(0x3815)
# Bit[7:4]: Vertical odd subsample increment
# Bit[3:0]: Vertical even subsample increment
# Size before scaling
# X_INPUT_SIZE = const(   (X_ADDR_END - X_ADDR_ST + 1 - (2 * X_OFFSET)))
# Y_INPUT_SIZE = const(   (Y_ADDR_END - Y_ADDR_ST + 1 - (2 * Y_OFFSET)))

# mirror and flip registers
_TIMING_TC_REG20 = const(0x3820)
# Timing Control Register
# Bit[2:1]: Vertical flip enable
#         00: Normal
#         11: Vertical flip
# Bit[0]: Vertical binning enable
_TIMING_TC_REG21 = const(0x3821)
# Timing Control Register
# Bit[5]: Compression Enable
# Bit[2:1]: Horizontal mirror enable
#         00: Normal
#         11: Horizontal mirror
# Bit[0]: Horizontal binning enable

_PCLK_RATIO = const(0x3824)
# Bit[4:0]: PCLK ratio manual

# frame control registers
_FRAME_CTRL01 = const(
    0x4201
)
# Control Passed Frame Number When both ON and OFF number set to 0x00,frame
# control is in bypass mode
# Bit[7:4]: Not used
# Bit[3:0]: Frame ON number
_FRAME_CTRL02 = const(
    0x4202
)
# Control Masked Frame Number When both ON and OFF number set to 0x00,frame
# control is in bypass mode
# Bit[7:4]: Not used
# BIT[3:0]: Frame OFF number

# format control registers
_FORMAT_CTRL00 = const(0x4300)

_CLOCK_POL_CONTROL = const(0x4740)
# Bit[5]: PCLK polarity 0: active low
#          1: active high
# Bit[3]: Gate PCLK under VSYNC
# Bit[2]: Gate PCLK under HREF
# Bit[1]: HREF polarity
#          0: active low
#          1: active high
# Bit[0] VSYNC polarity
#          0: active low
#          1: active high

_ISP_CONTROL_01 = const(0x5001)
# Bit[5]: Scale enable
#          0: Disable
#          1: Enable

# output format control registers
_FORMAT_CTRL = const(0x501F)
# Format select
# Bit[2:0]:
#  000: YUV422
#  001: RGB
#  010: Dither
#  011: RAW after DPC
#  101: RAW after CIP

# ISP top control registers
_PRE_ISP_TEST_SETTING_1 = const(0x503D)
# Bit[7]: Test enable
#         0: Test disable
#         1: Color bar enable
# Bit[6]: Rolling
# Bit[5]: Transparent
# Bit[4]: Square black and white
# Bit[3:2]: Color bar style
#         00: Standard 8 color bar
#         01: Gradual change at vertical mode 1
#         10: Gradual change at horizontal
#         11: Gradual change at vertical mode 2
# Bit[1:0]: Test select
#         00: Color bar
#         01: Random data
#         10: Square data
#         11: Black image

# exposure = {0x3500[3:0], 0x3501[7:0], 0x3502[7:0]} / 16 x tROW

_SCALE_CTRL_1 = const(0x5601)
# Bit[6:4]: HDIV RW
#          DCW scale times
#          000: DCW 1 time
#          001: DCW 2 times
#          010: DCW 4 times
#          100: DCW 8 times
#          101: DCW 16 times
#          Others: DCW 16 times
# Bit[2:0]: VDIV RW
#          DCW scale times
#          000: DCW 1 time
#          001: DCW 2 times
#          010: DCW 4 times
#          100: DCW 8 times
#          101: DCW 16 times
#          Others: DCW 16 times

_SCALE_CTRL_2 = const(0x5602)
# X_SCALE High Bits
_SCALE_CTRL_3 = const(0x5603)
# X_SCALE Low Bits
_SCALE_CTRL_4 = const(0x5604)
# Y_SCALE High Bits
_SCALE_CTRL_5 = const(0x5605)
# Y_SCALE Low Bits
_SCALE_CTRL_6 = const(0x5606)
# Bit[3:0]: V Offset

_VFIFO_CTRL0C = const(0x460C)
# Bit[1]: PCLK manual enable
#          0: Auto
#          1: Manual by PCLK_RATIO

_VFIFO_X_SIZE_H = const(0x4602)
_VFIFO_X_SIZE_L = const(0x4603)
_VFIFO_Y_SIZE_H = const(0x4604)
_VFIFO_Y_SIZE_L = const(0x4605)

_COMPRESSION_CTRL00 = const(0x4400)
_COMPRESSION_CTRL01 = const(0x4401)
_COMPRESSION_CTRL02 = const(0x4402)
_COMPRESSION_CTRL03 = const(0x4403)
_COMPRESSION_CTRL04 = const(0x4404)
_COMPRESSION_CTRL05 = const(0x4405)
_COMPRESSION_CTRL06 = const(0x4406)
_COMPRESSION_CTRL07 = const(0x4407)
# Bit[5:0]: QS
_COMPRESSION_ISI_CTRL = const(0x4408)
_COMPRESSION_CTRL09 = const(0x4409)
_COMPRESSION_CTRL0A = const(0x440A)
_COMPRESSION_CTRL0B = const(0x440B)
_COMPRESSION_CTRL0C = const(0x440C)
_COMPRESSION_CTRL0D = const(0x440D)
_COMPRESSION_CTRL0E = const(0x440E)

_TEST_COLOR_BAR = const(0xC0)
# Enable Color Bar roling Test

_AEC_PK_MANUAL_AGC_MANUALEN = const(0x02)
# Enable AGC Manual enable
_AEC_PK_MANUAL_AEC_MANUALEN = const(0x01)
# Enable AEC Manual enable

_TIMING_TC_REG20_VFLIP = const(0x06)
# Vertical flip enable
_TIMING_TC_REG21_HMIRROR = const(0x06)
# Horizontal mirror enable

OV5640_SIZE_96X96 = 0  # 96x96
OV5640_SIZE_QQVGA = 1  # 160x120
OV5640_SIZE_QCIF = 2  # 176x144
OV5640_SIZE_HQVGA = 3  # 240x176
OV5640_SIZE_240X240 = 4  # 240x240
OV5640_SIZE_QVGA = 5  # 320x240
OV5640_SIZE_CIF = 6  # 400x296
OV5640_SIZE_HVGA = 7  # 480x320
OV5640_SIZE_VGA = 8  # 640x480
OV5640_SIZE_SVGA = 9  # 800x600
OV5640_SIZE_XGA = 10  # 1024x768
OV5640_SIZE_HD = 11  # 1280x720
OV5640_SIZE_SXGA = 12  # 1280x1024
OV5640_SIZE_UXGA = 13  # 1600x1200
OV5640_SIZE_QHDA = 14  # 2560x1440
OV5640_SIZE_WQXGA = 15  # 2560x1600
OV5640_SIZE_PFHD = 16  # 1088x1920
OV5640_SIZE_QSXGA = 17  # 2560x1920

_ASPECT_RATIO_4X3 = const(0)
_ASPECT_RATIO_3X2 = const(1)
_ASPECT_RATIO_16X10 = const(2)
_ASPECT_RATIO_5X3 = const(3)
_ASPECT_RATIO_16X9 = const(4)
_ASPECT_RATIO_21X9 = const(5)
_ASPECT_RATIO_5X4 = const(6)
_ASPECT_RATIO_1X1 = const(7)
_ASPECT_RATIO_9X16 = const(8)

_resolution_info = [
    [96, 96, _ASPECT_RATIO_1X1],  # 96x96
    [160, 120, _ASPECT_RATIO_4X3],  # QQVGA
    [176, 144, _ASPECT_RATIO_5X4],  # QCIF
    [240, 176, _ASPECT_RATIO_4X3],  # HQVGA
    [240, 240, _ASPECT_RATIO_1X1],  # 240x240
    [320, 240, _ASPECT_RATIO_4X3],  # QVGA
    [400, 296, _ASPECT_RATIO_4X3],  # CIF
    [480, 320, _ASPECT_RATIO_3X2],  # HVGA
    [640, 480, _ASPECT_RATIO_4X3],  # VGA
    [800, 600, _ASPECT_RATIO_4X3],  # SVGA
    [1024, 768, _ASPECT_RATIO_4X3],  # XGA
    [1280, 720, _ASPECT_RATIO_16X9],  # HD
    [1280, 1024, _ASPECT_RATIO_5X4],  # SXGA
    [1600, 1200, _ASPECT_RATIO_4X3],  # UXGA
    [2560, 1440, _ASPECT_RATIO_16X9], # QHD
    [2560, 1600, _ASPECT_RATIO_16X10], # WQXGA
    [1088, 1920, _ASPECT_RATIO_9X16], # Portrait FHD
    [2560, 1920, _ASPECT_RATIO_4X3], # QSXGA

]


_ratio_table = [
    #  mw,   mh,  sx,  sy,   ex,   ey, ox, oy,   tx,   ty
    [2560, 1920, 0, 0, 2623, 1951, 32, 16, 2844, 1968],  # 4x3
    [2560, 1704, 0, 110, 2623, 1843, 32, 16, 2844, 1752],  # 3x2
    [2560, 1600, 0, 160, 2623, 1791, 32, 16, 2844, 1648],  # 16x10
    [2560, 1536, 0, 192, 2623, 1759, 32, 16, 2844, 1584],  # 5x3
    [2560, 1440, 0, 240, 2623, 1711, 32, 16, 2844, 1488],  # 16x9
    [2560, 1080, 0, 420, 2623, 1531, 32, 16, 2844, 1128],  # 21x9
    [2400, 1920, 80, 0, 2543, 1951, 32, 16, 2684, 1968],  # 5x4
    [1920, 1920, 320, 0, 2543, 1951, 32, 16, 2684, 1968],  # 1x1
    [1088, 1920, 736, 0, 1887, 1951, 32, 16, 1884, 1968],  # 9x16
]

_pll_pre_div2x_factors = [1, 1, 2, 3, 4, 1.5, 6, 2.5, 8]
_pll_pclk_root_div_factors = [1,2,4,8]

_REG_DLY = const(0xFFFF)
_REGLIST_TAIL = const(0x0000)

_sensor_default_regs = [
    _SYSTEM_CTROL0, 0x82,  # software reset
    _REG_DLY, 10,  # delay 10ms
    _SYSTEM_CTROL0, 0x42,  # power down
    # enable pll
    0x3103, 0x13,
    # io direction
    0x3017, 0xFF,
    0x3018, 0xFF,
    _DRIVE_CAPABILITY, 0xC3,
    _CLOCK_POL_CONTROL, 0x21,
    0x4713, 0x02,  # jpg mode select
    _ISP_CONTROL_01, 0x83,  # turn color matrix, awb and SDE
    # sys reset
    0x3000, 0x00,
    0x3002, 0x1C,
    # clock enable
    0x3004, 0xFF,
    0x3006, 0xC3,
    # isp control
    0x5000, 0xA7,
    _ISP_CONTROL_01, 0xA3,  # +scaling?
    0x5003, 0x08,  # special_effect
    # unknown
    0x370C, 0x02,  #!!IMPORTANT
    0x3634, 0x40,  #!!IMPORTANT
    # AEC/AGC
    0x3A02, 0x03,
    0x3A03, 0xD8,
    0x3A08, 0x01,
    0x3A09, 0x27,
    0x3A0A, 0x00,
    0x3A0B, 0xF6,
    0x3A0D, 0x04,
    0x3A0E, 0x03,
    0x3A0F, 0x30,  # ae_level
    0x3A10, 0x28,  # ae_level
    0x3A11, 0x60,  # ae_level
    0x3A13, 0x43,
    0x3A14, 0x03,
    0x3A15, 0xD8,
    0x3A18, 0x00,  # gainceiling
    0x3A19, 0xF8,  # gainceiling
    0x3A1B, 0x30,  # ae_level
    0x3A1E, 0x26,  # ae_level
    0x3A1F, 0x14,  # ae_level
    # vcm debug
    0x3600, 0x08,
    0x3601, 0x33,
    # 50/60Hz
    0x3C01, 0xA4,
    0x3C04, 0x28,
    0x3C05, 0x98,
    0x3C06, 0x00,
    0x3C07, 0x08,
    0x3C08, 0x00,
    0x3C09, 0x1C,
    0x3C0A, 0x9C,
    0x3C0B, 0x40,
    0x460C, 0x22,  # disable jpeg footer
    # BLC
    0x4001, 0x02,
    0x4004, 0x02,
    # AWB
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
    # color matrix (Saturation)
    0x5381, 0x1E,
    0x5382, 0x5B,
    0x5383, 0x08,
    0x5384, 0x0A,
    0x5385, 0x7E,
    0x5386, 0x88,
    0x5387, 0x7C,
    0x5388, 0x6C,
    0x5389, 0x10,
    0x538A, 0x01,
    0x538B, 0x98,
    # CIP control (Sharpness)
    0x5300, 0x10,  # sharpness
    0x5301, 0x10,  # sharpness
    0x5302, 0x18,  # sharpness
    0x5303, 0x19,  # sharpness
    0x5304, 0x10,
    0x5305, 0x10,
    0x5306, 0x08,  # denoise
    0x5307, 0x16,
    0x5308, 0x40,
    0x5309, 0x10,  # sharpness
    0x530A, 0x10,  # sharpness
    0x530B, 0x04,  # sharpness
    0x530C, 0x06,  # sharpness
    # GAMMA
    0x5480, 0x01,
    0x5481, 0x00,
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
    # Special Digital Effects (SDE) (UV adjust)
    0x5580, 0x06,  # enable brightness and contrast
    0x5583, 0x40,  # special_effect
    0x5584, 0x10,  # special_effect
    0x5586, 0x20,  # contrast
    0x5587, 0x00,  # brightness
    0x5588, 0x00,  # brightness
    0x5589, 0x10,
    0x558A, 0x00,
    0x558B, 0xF8,
    0x501D, 0x40,  # enable manual offset of contrast
    # power on
    0x3008, 0x02,
    # 50Hz
    0x3C00, 0x04,
    _REG_DLY, 300,
]

_sensor_format_jpeg = [
    _FORMAT_CTRL, 0x00,  # YUV422
    _FORMAT_CTRL00, 0x30,  # YUYV
    0x3002, 0x00,  # 0x1c to 0x00 !!!
    0x3006, 0xFF,  # 0xc3 to 0xff !!!
    0x471C, 0x50,  # 0xd0 to 0x50 !!!
]

_sensor_format_raw = [
    _FORMAT_CTRL, 0x03,  # RAW (DPC)
    _FORMAT_CTRL00, 0x00,  # RAW
]

_sensor_format_grayscale = [
    _FORMAT_CTRL, 0x00,  # YUV422
    _FORMAT_CTRL00, 0x10,  # Y8
]

_sensor_format_yuv422 = [
    _FORMAT_CTRL, 0x00,  # YUV422
    _FORMAT_CTRL00, 0x30,  # YUYV
]

_sensor_format_rgb565 = [
    _FORMAT_CTRL, 0x01,  # RGB
    _FORMAT_CTRL00, 0x61,  # RGB565 (BGR)
]

_ov5640_color_settings = {
    OV5640_COLOR_RGB: _sensor_format_rgb565,
    OV5640_COLOR_YUV: _sensor_format_yuv422,
    OV5640_COLOR_GRAYSCALE: _sensor_format_grayscale,
    OV5640_COLOR_JPEG: _sensor_format_jpeg,
}

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
# fmt: on


class _RegBits:
    def __init__(self, reg, shift, mask):
        self.reg = reg
        self.shift = shift
        self.mask = mask

    def __get__(self, obj, objtype=None):
        reg_value = obj._read_register(self.reg)
        return (reg_value >> self.shift) & self.mask

    def __set__(self, obj, value):
        if value & ~self.mask:
            raise ValueError(
                f"Value 0x{value:02x} does not fit in mask 0x{self.mask:02x}"
            )
        reg_value = obj._read_register(self.reg)
        reg_value &= ~(self.mask << self.shift)
        reg_value |= value << self.shift
        obj._write_register(self.reg, reg_value)


class _RegBits16:
    def __init__(self, reg, shift, mask):
        self.reg = reg
        self.shift = shift
        self.mask = mask

    def __get__(self, obj, objtype=None):
        reg_value = obj._read_register16(self.reg)
        return (reg_value >> self.shift) & self.mask

    def __set__(self, obj, value):
        if value & ~self.mask:
            raise ValueError(
                f"Value 0x{value:02x} does not fit in mask 0x{self.mask:02x}"
            )
        reg_value = obj._read_register16(self.reg)
        reg_value &= ~(self.mask << self.shift)
        reg_value |= value << self.shift
        obj._write_register16(self.reg, reg_value)


class _SCCB16CameraBase:  # pylint: disable=too-few-public-methods
    def __init__(self, i2c_bus, i2c_address):
        self._i2c_device = I2CDevice(i2c_bus, i2c_address)
        self._bank = None

    def _write_register(self, reg, value):
        b = bytearray(3)
        b[0] = reg >> 8
        b[1] = reg & 0xFF
        b[2] = value
        with self._i2c_device as i2c:
            i2c.write(b)

    def _write_addr_reg(self, reg, x_value, y_value):
        self._write_register16(reg, x_value)
        self._write_register16(reg + 2, y_value)

    def _write_register16(self, reg, value):
        self._write_register(reg, value >> 8)
        self._write_register(reg + 1, value & 0xFF)

    def _read_register(self, reg):
        b = bytearray(2)
        b[0] = reg >> 8
        b[1] = reg & 0xFF
        with self._i2c_device as i2c:
            i2c.write(b)
            i2c.readinto(b, end=1)
        return b[0]

    def _read_register16(self, reg):
        high = self._read_register(reg)
        low = self._read_register(reg + 1)
        return (high << 8) | low

    def _write_list(self, reg_list):
        for i in range(0, len(reg_list), 2):
            register = reg_list[i]
            value = reg_list[i + 1]
            if register == _REG_DLY:
                time.sleep(value / 1000)
            else:
                self._write_register(register, value)

    def _write_reg_bits(self, reg, mask, enable):
        val = val = self._read_register(reg)
        if enable:
            val |= mask
        else:
            val &= ~mask
        self._write_register(reg, val)


class OV5640(_SCCB16CameraBase):  # pylint: disable=too-many-instance-attributes
    """Control & Capture Images from an OV5640 Camera"""

    def __init__(
        self,
        i2c_bus,
        data_pins,
        clock,
        vsync,
        href,
        shutdown=None,
        reset=None,
        mclk=None,
        mclk_frequency=20_000_000,
        i2c_address=0x3C,
        size=OV5640_SIZE_QQVGA,
    ):  # pylint: disable=too-many-arguments
        """
        Args:
            i2c_bus (busio.I2C): The I2C bus used to configure the OV5640
            data_pins (List[microcontroller.Pin]): A list of 8 data pins, in order.
            clock (microcontroller.Pin): The pixel clock from the OV5640.
            vsync (microcontroller.Pin): The vsync signal from the OV5640.
            href (microcontroller.Pin): The href signal from the OV5640, \
                sometimes inaccurately called hsync.
            shutdown (Optional[microcontroller.Pin]): If not None, the shutdown
                signal to the camera, also called the powerdown or enable pin.
            reset (Optional[microcontroller.Pin]): If not None, the reset signal
                to the camera.
            mclk (Optional[microcontroller.Pin]): The pin on which to create a
                master clock signal, or None if the master clock signal is
                already being generated.
            mclk_frequency (int): The frequency of the master clock to generate, \
                ignored if mclk is None, requred if it is specified.
                Note that the OV5640 requires a very low jitter clock,
                so only specific (microcontroller-dependent) values may
                work reliably.  On the ESP32-S2, a 20MHz clock can be generated
                with sufficiently low jitter.
            i2c_address (int): The I2C address of the camera.
        """

        # Initialize the master clock
        if mclk:
            self._mclk_pwm = pwmio.PWMOut(mclk, frequency=mclk_frequency)
            self._mclk_pwm.duty_cycle = 32768
        else:
            self._mclk_pwm = None

        if shutdown:
            self._shutdown = digitalio.DigitalInOut(shutdown)
            self._shutdown.switch_to_output(True)
            time.sleep(0.1)
            self._shutdown.switch_to_output(False)
            time.sleep(0.3)
        else:
            self._shutdown = None

        if reset:
            self._reset = digitalio.DigitalInOut(reset)
            self._reset.switch_to_output(False)
            time.sleep(0.1)
            self._reset.switch_to_output(True)
            time.sleep(0.1)
        else:
            self._reset = None

        # Now that the master clock is running, we can initialize i2c comms
        super().__init__(i2c_bus, i2c_address)

        self._write_list(_sensor_default_regs)

        self._imagecapture = imagecapture.ParallelImageCapture(
            data_pins=data_pins, clock=clock, vsync=vsync, href=href
        )

        self._colorspace = OV5640_COLOR_RGB
        self._flip_x = False
        self._flip_y = False
        self._w = None
        self._h = None
        self._size = None
        self._test_pattern = False
        self._binning = False
        self._scale = False
        self._ev = 0
        self._white_balance = 0
        self.size = size

    chip_id = _RegBits16(_CHIP_ID_HIGH, 0, 0xFFFF)

    def capture(self, buf):
        """Capture an image into the buffer.

        Args:
            buf (Union[bytearray, memoryview]): A WritableBuffer to contain the \
                captured image.  Note that this can be a ulab array or a displayio Bitmap.
        """
        self._imagecapture.capture(buf)
        if self.colorspace == OV5640_COLOR_JPEG:
            eoi = buf.find(b"\xff\xd9")
            if eoi != -1:
                # terminate the JPEG data just after the EOI marker
                return memoryview(buf)[: eoi + 2]
        return None

    @property
    def capture_buffer_size(self):
        """Return the size of capture buffer to use with current resolution & colorspace settings"""
        if self.colorspace == OV5640_COLOR_JPEG:
            return self.width * self.height // self.quality
        if self.colorspace == OV5640_COLOR_GRAYSCALE:
            return self.width * self.height
        return self.width * self.height * 2

    @property
    def mclk_frequency(self):
        """Get the actual frequency the generated mclk, or None"""
        return self._mclk_pwm.frequency if self._mclk_pwm else None

    @property
    def width(self):
        """Get the image width in pixels."""
        return self._w

    @property
    def height(self):
        """Get the image height in pixels."""
        return self._h

    @property
    def colorspace(self):
        """Get or set the colorspace, one of the ``OV5640_COLOR_`` constants."""
        return self._colorspace

    @colorspace.setter
    def colorspace(self, colorspace):
        self._colorspace = colorspace
        self._set_size_and_colorspace()

    def _set_image_options(self):  # pylint: disable=too-many-branches
        reg20 = reg21 = reg4514 = reg4514_test = 0
        if self.colorspace == OV5640_COLOR_JPEG:
            reg21 |= 0x20

        if self._binning:
            reg20 |= 1
            reg21 |= 1
            reg4514_test |= 4
        else:
            reg20 |= 0x40

        if self._flip_y:
            reg20 |= 0x06
            reg4514_test |= 1

        if self._flip_x:
            reg21 |= 0x06
            reg4514_test |= 2

        if reg4514_test == 0:
            reg4514 = 0x88
        elif reg4514_test == 1:
            reg4514 = 0x00
        elif reg4514_test == 2:
            reg4514 = 0xBB
        elif reg4514_test == 3:
            reg4514 = 0x00
        elif reg4514_test == 4:
            reg4514 = 0xAA
        elif reg4514_test == 5:
            reg4514 = 0xBB
        elif reg4514_test == 6:
            reg4514 = 0xBB
        elif reg4514_test == 7:
            reg4514 = 0xAA

        self._write_register(_TIMING_TC_REG20, reg20)
        self._write_register(_TIMING_TC_REG21, reg21)
        self._write_register(0x4514, reg4514)

        if self._binning:
            self._write_register(0x4520, 0x0B)
            self._write_register(_X_INCREMENT, 0x31)
            self._write_register(_Y_INCREMENT, 0x31)
        else:
            self._write_register(0x4520, 0x10)
            self._write_register(_X_INCREMENT, 0x11)
            self._write_register(_Y_INCREMENT, 0x11)

    def _set_colorspace(self):
        colorspace = self._colorspace
        settings = _ov5640_color_settings[colorspace]

        self._write_list(settings)

    def deinit(self):
        """Deinitialize the camera"""
        self._imagecapture.deinit()
        if self._mclk_pwm:
            self._mclk_pwm.deinit()
        if self._shutdown:
            self._shutdown.deinit()
        if self._reset:
            self._reset.deinit()

    @property
    def size(self):
        """Get or set the captured image size, one of the ``OV5640_SIZE_`` constants."""
        return self._size

    def _set_size_and_colorspace(self):  # pylint: disable=too-many-locals
        size = self._size
        width, height, ratio = _resolution_info[size]
        self._w = width
        self._h = height
        (
            max_width,
            max_height,
            start_x,
            start_y,
            end_x,
            end_y,
            offset_x,
            offset_y,
            total_x,
            total_y,
        ) = _ratio_table[ratio]

        self._binning = (width <= max_width // 2) and (height <= max_height // 2)
        self._scale = not (
            (width == max_width and height == max_height)
            or (width == max_width // 2 and height == max_height // 2)
        )

        self._write_addr_reg(_X_ADDR_ST_H, start_x, start_y)
        self._write_addr_reg(_X_ADDR_END_H, end_x, end_y)
        self._write_addr_reg(_X_OUTPUT_SIZE_H, width, height)

        if not self._binning:
            self._write_addr_reg(_X_TOTAL_SIZE_H, total_x, total_y)
            self._write_addr_reg(_X_OFFSET_H, offset_x, offset_y)
        else:
            if width > 920:
                self._write_addr_reg(_X_TOTAL_SIZE_H, total_x - 200, total_y // 2)
            else:
                self._write_addr_reg(_X_TOTAL_SIZE_H, 2060, total_y // 2)
            self._write_addr_reg(_X_OFFSET_H, offset_x // 2, offset_y // 2)

        self._write_reg_bits(_ISP_CONTROL_01, 0x20, self._scale)

        self._set_image_options()

        if self.colorspace == OV5640_COLOR_JPEG:
            sys_mul = 200
            if size < OV5640_SIZE_QVGA:
                sys_mul = 160
            if size < OV5640_SIZE_XGA:
                sys_mul = 180
            self._set_pll(False, sys_mul, 4, 2, False, 2, True, 4)
        else:
            self._set_pll(False, 32, 1, 1, False, 1, True, 4)

        self._set_colorspace()

    def _set_pll(  # pylint: disable=too-many-arguments
        self,
        bypass,
        multiplier,
        sys_div,
        pre_div,
        root_2x,
        pclk_root_div,
        pclk_manual,
        pclk_div,
    ):
        if (  # pylint: disable=too-many-boolean-expressions
            multiplier > 252
            or multiplier < 4
            or sys_div > 15
            or pre_div > 8
            or pclk_div > 31
            or pclk_root_div > 3
        ):
            raise ValueError("Invalid argument to internal function")

        self._write_register(0x3039, 0x80 if bypass else 0)
        self._write_register(0x3034, 0x1A)
        self._write_register(0x3035, 1 | ((sys_div & 0xF) << 4))
        self._write_register(0x3036, multiplier & 0xFF)
        self._write_register(0x3037, (pre_div & 0xF) | (0x10 if root_2x else 0))
        self._write_register(0x3108, (pclk_root_div & 3) << 4 | 0x06)
        self._write_register(0x3824, pclk_div & 0x1F)
        self._write_register(0x460C, 0x22 if pclk_manual else 0x22)
        self._write_register(0x3103, 0x13)

    @size.setter
    def size(self, size):
        self._size = size
        self._set_size_and_colorspace()

    @property
    def flip_x(self):
        """Get or set the X-flip flag"""
        return self._flip_x

    @flip_x.setter
    def flip_x(self, value):
        self._flip_x = bool(value)
        self._set_image_options()

    @property
    def flip_y(self):
        """Get or set the Y-flip flag"""
        return self._flip_y

    @flip_y.setter
    def flip_y(self, value):
        self._flip_y = bool(value)
        self._set_image_options()

    @property
    def test_pattern(self):
        """Set to True to enable a test pattern, False to enable normal image capture"""
        return self._test_pattern

    @test_pattern.setter
    def test_pattern(self, value: bool) -> None:
        self._test_pattern = value
        self._write_register(_PRE_ISP_TEST_SETTING_1, value << 7)

    @property
    def saturation(self):
        """Get or set the saturation value, from -4 to +4."""
        return self._saturation

    @saturation.setter
    def saturation(self, value):
        if not -4 <= value <= 4:
            raise ValueError(
                "Invalid saturation {value}, use a value from -4..4 inclusive"
            )
        for offset, reg_value in enumerate(_sensor_saturation_levels[value]):
            self._write_register(0x5381 + offset, reg_value)
        self._saturation = value

    @property
    def effect(self):
        """Get or set the special effect, one of the ``OV5640_SPECIAL_EFFECT_`` constants"""
        return self._effect

    @effect.setter
    def effect(self, value):
        for reg_addr, reg_value in zip(
            (0x5580, 0x5583, 0x5584, 0x5003), _sensor_special_effects[value]
        ):
            self._write_register(reg_addr, reg_value)
        self._effect = value

    @property
    def quality(self):
        """Controls the JPEG quality.  Valid range is from 2..55 inclusive"""
        return self._read_register(_COMPRESSION_CTRL07) & 0x3F

    @quality.setter
    def quality(self, value: int):
        if not 2 <= value < 55:
            raise ValueError(
                f"Invalid quality value {value}, use a value from 2..55 inclusive"
            )
        self._write_register(_COMPRESSION_CTRL07, value & 0x3F)

    def _write_group_3_settings(self, settings):
        self._write_register(0x3212, 0x3)  # start group 3
        self._write_list(settings)
        self._write_register(0x3212, 0x13)  # end group 3
        self._write_register(0x3212, 0xA3)  # launch group 3

    @property
    def brightness(self):
        """Sensor brightness adjustment, from -4 to 4 inclusive"""
        brightness_abs = self._read_register(0x5587) >> 4
        brightness_neg = self._read_register(0x5588) & 8
        if brightness_neg:
            return -brightness_abs
        return brightness_abs

    @brightness.setter
    def brightness(self, value):
        if not -4 <= value <= 4:
            raise ValueError(
                "Invalid brightness value {value}, use a value from -4..4 inclusive"
            )
        self._write_group_3_settings(
            [0x5587, abs(value) << 4, 0x5588, 0x9 if value < 0 else 0x1]
        )

    @property
    def contrast(self):
        """Sensor contrast adjustment, from -4 to 4 inclusive"""
        contrast_abs = self._read_register(0x5587) >> 4
        contrast_neg = self._read_register(0x5588) & 8
        if contrast_neg:
            return -contrast_abs
        return contrast_abs

    @contrast.setter
    def contrast(self, value):
        if not -3 <= value <= 3:
            raise ValueError(
                "Invalid contrast value {value}, use a value from -3..3 inclusive"
            )
        setting = _contrast_settings[value]
        self._write_group_3_settings([0x5586, setting[0], 0x5585, setting[1]])

    @property
    def exposure_value(self):
        """Sensor exposure (EV) adjustment, from -4 to 4 inclusive"""
        return self._ev

    @exposure_value.setter
    def exposure_value(self, value):
        if not -3 <= value <= 3:
            raise ValueError(
                "Invalid exposure value (EV) {value}, use a value from -4..4 inclusive"
            )
        for offset, reg_value in enumerate(_sensor_ev_levels[value]):
            self._write_register(0x5381 + offset, reg_value)

    @property
    def white_balance(self):
        """The white balance setting, one of the ``OV5640_WHITE_BALANCE_*`` constants"""
        return self._white_balance

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

    @property
    def night_mode(self):
        """Enable or disable the night mode setting of the sensor"""
        return bool(self._read_register(0x3A00) & 0x04)

    @night_mode.setter
    def night_mode(self, value):
        self._write_reg_bits(0x3A00, 0x04, value)
*/
#endif // 0
#endif
