#pragma once

#include <Adafruit_iCap_parallel.h>

// OV7670 datasheet claims 10-48 MHz clock input, with 24 MHz typical.
// 24(ish) MHz is OK if camera connection is super clean. If any trouble,
// try dialing this down to 16 or 12 MHz. Even 8 MHz is OK if that's what's
// available. OV7670 has internal PLL and can step up from lower frequencies
// (to a point) if needed.
#if defined(__SAMD51__)
// SAMD timer peripheral as used by this code is clocked from a 48 MHz
// source, so it's always going to be some integer divisor of that.
#define OV7670_XCLK_HZ 24000000 ///< XCLK to camera, 8-24 MHz
#elif defined(ARDUINO_ARCH_RP2040)
// Might want to derive this from F_CPU instead
#define OV7670_XCLK_HZ 12500000 ///< XCLK to camera, 8-24 MHz
#endif

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

#if defined(ICAP_FULL_SUPPORT)

typedef iCap_parallel_pins OV7670_pins;

#define OV7670_ADDR 0x21 //< Default I2C address if unspecified

/*!
    @brief  Class encapsulating OmniVision OV7670 functionality.
*/
class Adafruit_iCap_OV7670 : public Adafruit_iCap_parallel {
 public:
  /*!
    @brief  Constructor for OV7670 camera class.
    @param  pins      OV7670_pins structure, describing physical connection
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
  Adafruit_iCap_OV7670(iCap_parallel_pins& pins, iCap_arch* arch = NULL,
                       TwoWire& twi = Wire, uint16_t* pbuf = NULL,
                       uint32_t pbufsize = 0, uint8_t addr = OV7670_ADDR,
                       uint32_t speed = 100000, uint32_t delay_us = 1000);
  ~Adafruit_iCap_OV7670();

  /*!
    @brief   Initialize peripherals behind an Adafruit_iCap_OV7670 instance,
             but do not actually start capture; must follow with a config()
             call for that.
    @return  Status code. ICAP_STATUS_OK on successful init.
  */
  iCap_status begin(void);

  /*!
    @brief   Initialize peripherals and allocate resources behind an
             Adafruit_iCap_OV7670 instance, start capturing data in
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
  iCap_status begin(OV7670_size size, iCap_colorspace space = ICAP_COLOR_RGB565,
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
  iCap_status config(OV7670_size size,
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
  void frameControl(OV7670_size size, uint8_t vstart, uint16_t hstart,
                    uint8_t edge_offset, uint8_t pclk_delay);

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

 private:
};

#endif // end ICAP_FULL_SUPPORT

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
