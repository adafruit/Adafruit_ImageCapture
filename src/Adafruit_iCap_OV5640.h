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
    @param   space  ICAP_COLORSPACE_RGB565 (default), ICAP_COLORSPACE_YUV
                    or ICAP_COLORSPACE_GRAYSCALE.
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
  iCap_status begin(OV5640_size size,
                    iCap_colorspace space = ICAP_COLORSPACE_RGB565,
                    float fps = 30.0, uint8_t nbuf = 1);

  /*!
    @brief   Change frame configuration on an already-running camera.
    @param   size   Frame size as a power-of-two reduction of VGA
                    resolution. Available sizes are OV7670_SIZE_DIV1
                    (640x480), OV7670_SIZE_DIV2 (320x240), OV7670_SIZE_DIV4
                    (160x120), OV7670_SIZE_DIV8 and OV7670_SIZE_DIV16.
                    This argument is required.
    @param   space  ICAP_COLORSPACE_RGB565 (default), ICAP_COLORSPACE_YUV
                    or ICAP_COLORSPACE_GRAYSCALE.
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
                     iCap_colorspace space = ICAP_COLORSPACE_RGB565,
                     float fps = 30.0, uint8_t nbuf = 1, 
                     iCap_realloc allo = ICAP_REALLOC_CHANGE);

  /*!
    @brief  Configure camera colorspace.
    @param  space  ICAP_COLORSPACE_RGB565, ICAP_COLORSPACE_YUV or
                   ICAP_COLORSPACE_GRAYSCALE.
  */
  void setColorspace(iCap_colorspace space = ICAP_COLORSPACE_RGB565);

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

// registers belong outside the FULL_SUPPORT ifdef

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
// ISP top control 0x5000-0x5063
#define OV5640_REG_ISP_CONTROL00 0x5000 //< ISP top control 00
#define OV5640_REG_ISP_CONTROL01 0x5001 //< ISP top control 01
#define OV5640_REG_ISP_CONTROL03 0x5003 //< ISP top control 03
#define OV5640_REG_ISP_CONTROL05 0x5005 //< ISP top control 05
#define OV5640_REG_ISP_MISC0 0x501D //< ISP miscellaneous 0
#define OV5640_REG_ISP_MISC1 0x501E //< ISP miscellaneous 1
#define OV5640_REG_FORMAT_MUX_CONTROL 0x501F //< Format MUX control
#define OV5640_REG_DITHER_CTRL0 0x5020 //< Dither MUX
#define OV5640_REG_DRAW_WINDOW_CONTROL00 0x5027 //< Draw window control
#define OV5640_REG_DRAW_WINDOW_LEFT_POSITION_HI 0x5028 //< Draw win left[11:8]
#define OV5640_REG_DRAW_WINDOW_LEFT_POSITION_LO 0x5029 //< Draw win left[7:0]
#define OV5640_REG_DRAW_WINDOW_RIGHT_POSITION_HI 0x502A //< Draw win right[11:8]
#define OV5640_REG_DRAW_WINDOW_RIGHT_POSITION_LO 0x502B //< Draw win right[7:0]
#define OV5640_REG_DRAW_WINDOW_TOP_POSITION_HI 0x502C //< Draw win top[10:8]
#define OV5640_REG_DRAW_WINDOW_TOP_POSITION_LO 0x502D //< Draw win top[7:0]
#define OV5640_REG_DRAW_WINDOW_BOTTOM_POSITION_HI 0x502E //< Draw win bottom[10:8]
#define OV5640_REG_DRAW_WINDOW_BOTTOM_POSITION_LO 0x502F //< Draw win bottom[7:0]
#define OV5640_REG_DRAW_WINDOW_HORIZ_BOUND_WIDTH_HI 0x5030 //< Draw win horiz bound width[11:8]
#define OV5640_REG_DRAW_WINDOW_HORIZ_BOUND_WIDTH_LO 0x5031 //< Draw win horiz bound width[7:0]
#define OV5640_REG_DRAW_WINDOW_VERTZ_BOUND_WIDTH_HI 0x5032 //< Draw win vert bound width[10:8]
#define OV5640_REG_DRAW_WINDOW_VERTZ_BOUND_WIDTH_LO 0x5033 //< Draw win vert bound width[7:0]
#define OV5640_REG_DRAW_WINDOW_Y_CONTROL 0x5034 //< Fixed Y for draw window
#define OV5640_REG_DRAW_WINDOW_U_CONTROL 0x5035 //< Fixed U for draw window
#define OV5640_REG_DRAW_WINDOW_V_CONTROL 0x5036 //< Fixed V for draw window
#define OV5640_REG_PRE_ISP_TEST_SETTING 0x503D //< Pre ISP test settings
#define OV5640_REG_ISP_SENSOR_BIAS 0x5061 //< ISP sensor bias I
#define OV5640_REG_ISP_SENSOR_GAIN_H 0x5062 //< ISP sensor gain MSB
#define OV5640_REG_ISP_SENSOR_GAIN_L 0x5063 //< ISP sensor gain LSB
// AWB control 0x5180-0x51D0
#define OV5640_REG_AWB_CONTROL00 0x5180 //< AWB control 00
#define OV5640_REG_AWB_CONTROL01 0x5181 //< AWB control 01
#define OV5640_REG_AWB_CONTROL02 0x5182 //< AWB control 02
#define OV5640_REG_AWB_CONTROL03 0x5183 //< AWB control 03
#define OV5640_REG_AWB_CONTROL04 0x5184 //< AWB control 04
#define OV5640_REG_AWB_CONTROL05 0x5185 //< AWB control 05
#define OV5640_REG_AWB_CONTROL_ADVANCED 0x5186 //< Advanced AWB registers
#define OV5640_REG_AWB_CONTROL17 0x5191 //< AWB control 17
#define OV5640_REG_AWB_CONTROL18 0x5192 //< AWB control 18
#define OV5640_REG_AWB_CONTROL19 0x5193 //< AWB control 19
#define OV5640_REG_AWB_CONTROL20 0x5194 //< AWB control 20
#define OV5640_REG_AWB_CONTROL21 0x5195 //< AWB control 21
#define OV5640_REG_AWB_CONTROL22 0x5196 //< AWB control 22
#define OV5640_REG_AWB_CONTROL23 0x5197 //< AWB control 23
#define OV5640_REG_AWB_CONTROL30 0x519E //< AWB control 30
#define OV5640_REG_AWB_CURRENT_R_GAIN_HI 0x519F //< Current R setting[11:8]
#define OV5640_REG_AWB_CURRENT_R_GAIN_LO 0x51A0 //< Current R setting[7:0]
#define OV5640_REG_AWB_CURRENT_G_GAIN_HI 0x51A1 //< Current G setting[11:8]
#define OV5640_REG_AWB_CURRENT_G_GAIN_LO 0x51A2 //< Current G setting[7:0]
#define OV5640_REG_AWB_CURRENT_B_GAIN_HI 0x51A3 //< Current B setting[11:8]
#define OV5640_REG_AWB_CURRENT_B_GAIN_LO 0x51A4 //< Current B setting[7:0]
#define OV5640_REG_AWB_AVERAGE_R 0x51A5 //< Average R[9:2]
#define OV5640_REG_AWB_AVERAGE_G 0x51A6 //< Average G[9:2]
#define OV5640_REG_AWB_AVERAGE_B 0x51A7 //< Average B[9:2]
#define OV5640_REG_AWB_CONTROL74 0x51D0 //< AWB control 74
// CIP control 0x5300-0x530F





// Some known OV5640 resolutions
// 2592x1944 15 fps (dummy 16 px horiz, 8 vert -- 2608x1952 w/dummy)
// 1280x960 45 fps
// 1920x1080 30 fps
// 1280x720 60 fps
// 640x480 90 fps
// 320x240 120 fps
// Some of these require very fast pixel clocks, which we know isn't clean
// on a breadboard. So while the resolutions may be valid, actual frame rates
// will be something less.
// Sensor array is 2624x1964. 2592x1944 are active pixels. Others are
// used for black level calibration and interpolation.
// Reg 0x3821 enables horiz binning (2:1 averaging)
// Vertical binning is automatic in vertical-subsampled formats
// Mirror (horiz) and flip (vert) are avail in 0x3820 and 0x3821
// ISP input size is total data read from the pixel array. Larger this is,
// lower the frame rate. Data output size is windowed within this.
// When the scaling function is enabled, it appears that the pre-scaling
// size must be centered. However, it might be possible to change the
// x_addr_st and y_addr_st to do flexible windowing (fig 4-3 in datasheet).
// One test pattern (color bar) is available.
// Enable defect pixel cancellation to remove black/white flecks.
// Datasheet 5.9: scaling. Up to 32x is supported.
// In theory, more flexible sizing should be possible than other cams,
// using the scaling and windowing functions. e.g. different aspect ratios,
// not just divisions-of-VGA as before.

// Group write is used to update a group of registers in the same frame.
// Group addr registers are 3200-3203. See section 2-6 in datasheet.
