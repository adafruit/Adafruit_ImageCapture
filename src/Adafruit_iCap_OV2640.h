#pragma once
#include <Adafruit_iCap_parallel.h>

#define OV2640_ADDR 0x30 //< Default I2C address if unspecified
typedef iCap_parallel_pins OV2640_pins;

/*!
    @brief  Class encapsulating OmniVision OV2640 functionality.
*/
class Adafruit_iCap_OV2640 : public Adafruit_iCap_parallel {
public:
  Adafruit_iCap_OV2640(iCap_parallel_pins &pins, TwoWire &twi = Wire,
                       uint8_t addr = OV2640_ADDR);
  ~Adafruit_iCap_OV2640();
  ICAP_status begin();

private:
};

#define OV2640_REG_RA_DLMT 0xFF    //< Register bank select
#define OV2640_RA_DLMT_DSP 0x00    //< Bank 0 - DSP address
#define OV2640_RA_DLMT_SENSOR 0x01 //< Bank 1 - Sensor address

// OV2640 register bank 0 -- DSP address
// These register names are preceded by 'REG0' as a bank-select reminder
#define OV2640_REG0_R_BYPASS 0x05         //< Bypass DSP
#define OV2640_R_RESERVED_MASK 0xFE       //< Reserved bits
#define OV2640_R_BYPASS_MASK 0x01         //< DSP mask:
#define OV2640_R_BYPASS_DSP_ENABLE 0x00   //< Enable DSP
#define OV2640_R_BYPASS_DSP_BYPASS 0x01   //< Bypass DSP
#define OV2640_REG0_QS 0x44               //< Quantization scale factor
#define OV2640_REG0_CTRLI 0x50            //< ?
#define OV2640_CTRLI_LP_DP 0x80           //< LP_DP bit
#define OV2640_CTRLI_ROUND 0x40           //< Round bit
#define OV2640_CTRLI_V_DIV_MASK 0x38      //< V_DIVIDER mask (3 bits)
#define OV2640_CTRLI_H_DIV_MASK 0x07      //< H_DIVIDER mask (3 bits)
#define OV2640_REG0_HSIZE 0x51            //< H_SIZE[7:0] (real/4)
#define OV2640_REG0_VSIZE 0x52            //< V_SIZE[7:0] (real/4)
#define OV2640_REG0_XOFFL 0x53            //< OFFSET_X[7:0]
#define OV2640_REG0_YOFFL 0x54            //< OFFSET_Y[7:0]
#define OV2640_REG0_VHYX 0x55             //< V/H/X/Y size/offset high bits
#define OV2640_VHYX_V_SIZE_8 0x80         //< V_SIZE[8] bit
#define OV2640_VHYX_OFFSET_Y_MASK 0x70    //< OFFSET_Y[10:8] mask
#define OV2640_VHYX_H_SIZE_8 0x08         //< H_SIZE[8] bit
#define OV2640_VHYX_OFFSET_X_MASK 0x7     //< OFFSET_X[10:8] mask
#define OV2640_REG0_DPRP 0x56             //< ?
#define OV2640_DPRP_DP_SELY_MASK 0xF0     //< DP_SELY mask
#define OV2640_DPRP_DP_SELX_MASK 0x0F     //< DP_SELX mask
#define OV2640_REG0_TEST 0x57             //< ?
#define OV2640_TEST_H_SIZE_9 0x80         //< H_SIZE[9] bit
#define OV2640_TEST_RESERVED_MASK 0x7F    //< Reserved bits
#define OV2640_REG0_ZMOW 0x5A             //< OUTW[7:0] (real/4)
#define OV2640_REG0_ZMOH 0x5B             //< OUTH[7:0] (real/4)
#define OV2640_REG0_ZMHH 0x5C             //< Zoom speed and more
#define OV2640_ZMHH_ZMSPD_MASK 0xF0       //< ZMSPD (zoom speed)
#define OV2640_ZMHH_OUTH_8 0x40           //< OUTH[8] bit
#define OV2640_ZMHH_OUTW_MASK 0x03        //< OUTW[9:8]
#define OV2640_REG0_BPADDR 0x7C           //< SDE indirect reg access: addr
#define OV2640_REG0_BPDATA 0x7D           //< SDE indirect reg access: data
#define OV2640_REG0_CTRL2 0x86            //< Module enable:
#define OV2640_CTRL2_DCW 0x20             //< DCW enable
#define OV2640_CTRL2_SDE 0x10             //< SDE enable
#define OV2640_CTRL2_UV_ADJ 0x08          //< UV_ADJ enable
#define OV2640_CTRL2_UV_AVG 0x04          //< UV_AVG enable
#define OV2640_CTRL2_CMX 0x01             //< CMX enable
#define OV2640_REG0_CTRL3 0x87            //< Module enable, continued
#define OV2640_CTRL3_BPC 0x80             //< BPC enable
#define OV2640_CTRL3_WPC 0x40             //< WPC enable
#define OV2640_CTRL3_RESERVED_MASK 0x3F   //< Reserved bits
#define OV2640_REG0_SIZEL 0x8C            //< HSIZE, VSIZE more bits
#define OV2640_SIZEL_HSIZE11 0x10         //< HSIZE[11]
#define OV2640_SIZEL_HSIZE_MASK 0x0C      //< HSIZE[2:0]
#define OV2640_SIZEL_VSIZE_MASK 0x03      //< VSIZE[2:0]
#define OV2640_REG0_HSIZE8 0xC0           //< Image horiz size HSIZE[10:3]
#define OV2640_REG0_VSIZE8 0xC1           //< Image vert size VSIZE[10:3]
#define OV2640_REG0_CTRL0 0xC2            //< Module enable, continued
#define OV2640_CTRL0_AEC_EN 0x80          //< AEC_EN enable
#define OV2640_CTRL0_AEC_SEL 0x40         //< AEC_SEL enable
#define OV2640_CTRL0_STAT_SEL 0x20        //< STAT_SEL enable
#define OV2640_CTRL0_VFIRST 0x10          //< VFIRST enable
#define OV2640_CTRL0_YUV422 0x08          //< YUV422 enable
#define OV2640_CTRL0_YUV_EN 0x04          //< YUV_EN enable
#define OV2640_CTRL0_RGB_EN 0x02          //< RGB_EN enable
#define OV2640_CTRL0_RAW_EN 0x01          //< RAW_EN enable
#define OV2640_REG0_CTRL1 0xC3            //< Module enable, continued
#define OV2640_CTRL1_CIP 0x80             //< CIP enable
#define OV2640_CTRL1_DMY 0x40             //< DMY enable
#define OV2640_CTRL1_RAW_GMA 0x20         //< RAW_GMA enable
#define OV2640_CTRL1_DG 0x10              //< DG enable
#define OV2640_CTRL1_AWB 0x08             //< AWB enable
#define OV2640_CTRL1_AWB_GAIN 0x04        //< AWB_GAIN enable
#define OV2640_CTRL1_LENC 0x02            //< LENC enable
#define OV2640_CTRL1_PRE 0x01             //< PRE enable
#define OV2640_REG0_R_DVP_SP 0xD3         //< DVP selections
#define OV2640_R_DVP_SP_AUTO 0x80         //< Auto DVP mode
#define OV2640_R_DVP_SP_PCLK_MASK 0x7F    //< Manual DVP PCLK mask
#define OV2640_REG0_IMAGE_MODE 0xDA       //< Image output format select
#define OV2640_IMAGE_MODE_Y8 0x40         //< Y8 enable for DVP
#define OV2640_IMAGE_MODE_JPEG 0x10       //< JPEG output enable mask
#define OV2640_IMAGE_MODE_DVP_MASK 0x0C   //< DVP output format mask
#define OV2640_IMAGE_MODE_DVP_YUV 0x00    //< YUV422
#define OV2640_IMAGE_MODE_DVP_RAW10 0x04  //< RAW10 (DVP)
#define OV2640_IMAGE_MODE_DVP_RGB565 0x08 //< RGB565
#define OV2640_IMAGE_MODE_JPEG_HREF 0x02  //< HREF timing select in JPEG mode
#define OV2640_IMAGE_MODE_BYTE_SWAP 0x01  //< Byte swap enable for DVP
#define OV2640_REG0_RESET 0xE0            //< Reset
#define OV2640_RESET_MCU 0x40             //< Microcontroller reset
#define OV2640_RESET_SCCB 0x20            //< SCCB reset
#define OV2640_RESET_JPEG 0x10            //< JPEG reset
#define OV2640_RESET_DVP 0x04             //< DVP reset
#define OV2640_RESET_IPU 0x02             //< IPU reset
#define OV2640_RESET_CIF 0x01             //< CIF reset
#define OV2640_REG0_MS_SP 0xF0            //< SCCB host speed
#define OV2640_REG0_SS_ID 0xF7            //< SCCB periph ID
#define OV2640_REG0_SS_CTRL 0xF8          //< SCCB periph control 1:
#define OV2640_SS_CTRL_ADDR 0x20          //< Address auto-increment
#define OV2640_SS_CTRL_SCCB 0x08          //< SCCB enable
#define OV2640_SS_CTRL_DELAY 0x04         //< Delay SCCB main clock
#define OV2640_SS_CTRL_ACCESS 0x02        //< Enable SCCB host access
#define OV2640_SS_CTRL_SENSOR 0x01        //< Enable sensor pass-through
#define OV2640_REG0_MC_BIST 0xF9          //< ?
#define OV2640_MC_BIST_RESET 0x80         //< MCU reset
#define OV2640_MC_BIST_BOOTROM 0x40       //< Boot ROM select
#define OV2640_MC_BIST_12K_1 0x20         //< R/W 1 error for 12KB mem
#define OV2640_MC_BIST_12K_0 0x10         //< R/W 0 error for 12KB mem
#define OV2640_MC_BIST_512_1 0x08         //< R/W 1 error for 512B mem
#define OV2640_MC_BIST_512_0 0x04         //< R/W 0 error for 512B mem
#define OV2640_MC_BIST_BUSY 0x02          //< R=BISY busy, W=MCU reset
#define OV2640_MC_BIST_LAUNCH 0x01        //< Launch BIST
#define OV2640_REG0_MC_AL 0xFA            //< Program mem ptr addr low byte
#define OV2640_REG0_MC_AH 0xFB            //< Program mem ptr addr high byte
#define OV2640_REG0_MC_D 0xFC             //< Program mem ptr access address
#define OV2640_REG0_P_CMD 0xFD            //< SCCB protocol command register
#define OV2640_REG0_P_STATUS 0xFE         //< SCCB protocol status register

// OV2640 register bank 1 -- Sensor address
// These register names are preceded by 'REG1' as a bank-select reminder
#define OV2640_REG1_GAIN 0x00              //< AGC gain control LSBs
#define OV2640_REG1_COM1 0x03              //< Common control 1
#define OV2640_COM1_DFRAME_MASK 0xC0       //< Dummy frame control mask
#define OV2640_COM1_DFRAME_1 0x40          //< Allow 1 dummy frame
#define OV2640_COM1_DFRAME_4 0x80          //< Allow 4 dummy frames
#define OV2640_COM1_DFRAME_7 0xC0          //< Allow 7 dummy frames
#define OV2640_COM1_VEND_MASK 0x0C         //< Vert window end line LSBs
#define OV2640_COM1_VSTRT_MASK 0x03        //< Vert window start line LSBs
#define OV2640_REG1_REG04 0x04             //< Register 04
#define OV2640_REG04_HFLIP 0x80            //< Horizontal mirror
#define OV2640_REG04_VFLIP 0x40            //< Vertical mirror
#define OV2640_REG04_VREF0 0x10            //< VREF[0]
#define OV2640_REG04_HREF0 0x08            //< HREF[0]
#define OV2640_REG04_AEC_MASK 0x03         //< AEC[1:0]
#define OV2640_REG1_REG08 0x08             //< Register 08 (frame exposure)
#define OV2640_REG1_COM2 0x09              //< Common control 2
#define OV2640_COM2_RESERVED_MASK 0xE8     //< Reserved bits
#define OV2640_COM2_STANDBY 0x10           //< Standby mode
#define OV2640_COM2_PINUSE 0x04            //< PWDN/RESETB as SLVS/SLHS mask
#define OV2640_COM2_DRIVE_MASK 0x03        //< Output drive select mask
#define OV2640_COM2_DRIVE_1X 0x00          //< 1x
#define OV2640_COM2_DRIVE_3X 0x01          //< 3x (sic)
#define OV2640_COM2_DRIVE_2X 0x02          //< 2x (sic)
#define OV2640_COM2_DRIVE_4X 0x03          //< 4x
#define OV2640_REG1_PIDH 0x0A              //< Product ID MSB (read only)
#define OV2640_REG1_PIDL 0x0B              //< Product ID LSB (read only)
#define OV2640_REG1_COM3 0x0C              //< Common control 3
#define OV2640_COM3_RESERVED_MASK 9xF8     //< Reserved bits
#define OV2640_COM3_BANDING_MASK 0x04      //< Manual banding bit mask
#define OV2640_COM3_BANDING_60HZ 0x00      //< 60 Hz
#define OV2640_COM3_BANDING_50HZ 0x04      //< 50 Hz
#define OV2640_COM3_AUTO_BANDING 0x02      //< Auto-set banding
#define OV2640_COM3_SNAPSHOT 0x01          //< Snapshot option
#define OV2640_REG1_COM4 0x0D              //< Common control 4
#define OV2640_COM4_RESERVED_MASK 0xFB     //< Reserved bits
#define OV2640_COM4_CLOCK_PIN_STATUS 0x04  //< Clock output power pin status
#define OV2640_REG1_AEC 0x10               //< AEC[9:2] auto exposure ctrl
#define OV2640_REG1_CLKRC 0x11             //< Clock rate control
#define OV2640_CLKRC_DOUBLE 0x80           //< Internal freq doubler on/off
#define OV2640_CLKRC_DIV_MASK 0x3F         //< Clock divider mask
#define OV2640_REG1_COM7 0x12              //< Common control 7:
#define OV2640_COM7_SRST 0x80              //< System reset
#define OV2640_COM7_RES_MASK 0x70          //< Resolution mask
#define OV2640_COM7_RES_UXGA 0x00          //< UXGA (full size) mode
#define OV2640_COM7_RES_CIF 0x10           //< CIF mode
#define OV2640_COM7_RES_SVGA 0x40          //< SVGA mode
#define OV2640_COM7_ZOOM 0x04              //< Zoom mode
#define OV2640_COM7_COLORBAR 0x02          //< Color bar test pattern enable
#define OV2640_REG1_COM8 0x13              //< Common control 8:
#define OV2640_COM8_RESERVED_MASK 0xDA     //< Reserved bits
#define OV2640_COM8_BANDING 0x20           //< Banding filter on
#define OV2640_COM8_AGC_AUTO 0x04          //< Auto gain
#define OV2640_COM8_EXP_AUTO 0x01          //< Auto exposure
#define OV2640_REG1_COM9 0x14              //< Common control 9:
#define OV2640_COM9_AGC_GAIN_MASK 0xE0     //< AGC gain ceiling mask, GH[2:0]
#define OV2640_COM9_AGC_GAIN_2X 0x00       //< 2x
#define OV2640_COM9_AGC_GAIN_4X 0x20       //< 4x
#define OV2640_COM9_AGC_GAIN_8X 0x40       //< 8x
#define OV2640_COM9_AGC_GAIN_16X 0x60      //< 16x
#define OV2640_COM9_AGC_GAIN_32X 0x80      //< 32x
#define OV2640_COM9_AGC_GAIN_64X 0xA0      //< 64x
#define OV2640_COM9_AGC_GAIN_128X 0xC0     //< 128x
#define OV2640_COM9_RESERVED_MASK 0x1F     //< Reserved bits
#define OV2640_REG1_COM10 0x15             //< Common control 10:
#define OV2640_COM10_CHSYNC_SWAP_MASK 0x80 //< CHSYNC pin output swap mask
#define OV2640_COM10_CHSYNC_CHSYNC 0x00    //< CHSYNC
#define OV2640_COM10_CHSYNC_HREF 0x80      //< HREF
#define OV2640_COM10_HREF_SWAP_MASK 0x40   //< HREF pin output swap mask
#define OV2640_COM10_HREF_HREF 0x00        //< HREF
#define OV2640_COM10_HREF_CHSYNC 0x40      //< CHSYNC
#define OV2640_COM10_PCLK_MASK 0x20        //< PCLK output selection mask
#define OV2640_COM10_PCLK_ALWAYS 0x00      //< PCLK always output
#define OV2640_COM10_PCLK_HREF 0x20        //< PCLK qualified by HREF
#define OV2640_COM10_PCLK_EDGE_MASK 0x10   //< PCLK edge selection mask
#define OV2640_COM10_PCLK_FALLING 0x00     //< Data updated on falling PCLK
#define OV2640_COM10_PCLK_RISING 0x10      //< Data updated on rising PCLK
#define OV2640_COM10_HREF_MASK 0x08        //< HREF polarity mask
#define OV2640_COM10_HREF_POSITIVE 0x00    //< Positive HREF
#define OV2640_COM10_HREF_NEGATIVE 0x08    //< Negative HREF for data valid
#define OV2640_COM10_VSYNC_MASK 0x02       //< VSYNC polarity mask
#define OV2640_COM10_VSYNC_POSITIVE 0x00   //< Positive VSYNC
#define OV2640_COM10_VSYNC_NEGATIVE 0x02   //< Negative VSYNC
#define OV2640_COM10_HSYNC_MASK 0x01       //< HSYNC polarity mask
#define OV2640_COM10_HSYNC_POSITIVE 0x00   //< Positive HSYNC
#define OV2640_COM10_HSYNC_NEGATIVE 0x01   //< Negative HSYNC
#define OV2640_REG1_HREFST 0x17            //< Horizontal window start MSB
#define OV2640_REG1_HREFEND 0x18           //< Horizontal window end MSB
#define OV2640_REG1_VSTRT 0x19             //< Vertical window line start MSB
#define OV2640_REG1_VEND 0x1A              //< Vertical window line end MSB
#define OV2640_REG1_MIDH 0x1C              //< Manufacturer ID MSB (RO=0x7F)
#define OV2640_REG1_MIDL 0x1D              //< Manufacturer ID LSB (RO=0xA2)
#define OV2640_REG1_AEW 0x24               //< Luminance signal high range
#define OV2640_REG1_AEB 0x25               //< Luminance signal low range
#define OV2640_REG1_VV 0x26                //< Fast mode large step threshold
#define OV2640_VV_HIGH_MASK 0xF0           //< High threshold mask
#define OV2640_VV_LOW_MASK 0x0F            //< Low threshold mask
#define OV2640_REG1_REG2A 0x2A             //< Register 2A
#define OV2640_REG2A_LINE_MASK 0xF0        //< Line interval adjust MSBs
#define OV2640_REG2A_HSYNC_END_MASK 0x0C   //< HSYNC timing end point MSBs
#define OV2640_REG2A_HSYNC_START_MASK 0x03 //< HSYNC timing start point MSBs
#define OV2640_REG1_FRARL 0x2B             //< Line interval adjust LSB
#define OV2640_REG1_ADDVSL 0x2D            //< VSYNC pulse width LSB
#define OV2640_REG1_ADDVSH 0x2E            //< VSYNC pulse width MSB
#define OV2640_REG1_YAVG 0x2F              //< Luminance average
#define OV2640_REG1_HSDY 0x30              //< HSYNC pos+width start LSB
#define OV2640_REG1_HEDY 0x31              //< HSYNC pos+width end LSB
#define OV2640_REG1_REG32 0x32             //< Common control 32
#define OV2640_REG32_PCLK_MASK 0xC0        //< Pixel clock divide option mask
#define OV2640_REG32_PCLK_DIV1 0x00        //< No effect on PCLK
#define OV2640_REG32_PCLK_DIV2 0x80        //< PCLK frequency / 2
#define OV2640_REG32_PCLK_DIV4 0xC0        //< PCLK frequency / 4
#define OV2640_REG32_HREFEND_MASK 0x38     //< HREFEND LSBs
#define OV2640_REG32_HREFST_MASK 0x07      //< HREFST LSBs
#define OV2640_REG1_ARCOM2 0x34            //< ?
#define OV2640_ARCOM2_ZOOM 0x04            //< Zoom window horiz start point
#define OV2640_REG1_REG45 0x45             //< Register 45:
#define OV2640_REG45_AGC_MASK 0xC0         //< AGC[9:8] highest gain control
#define OV2640_REG45_AEC_MASK 0x3F         //< AEC[15:10] AEC MSBs
#define OV2640_REG1_FLL 0x46               //< Frame length adjustment LSBs
#define OV2640_REG1_FLH 0x47               //< Frame length adjustment MSBs
#define OV2640_REG1_COM19 0x48             //< Frame length adjustment MSBs
#define OV2640_COM19_ZOOM_MASK 0x03        //< Zoom mode vert window LSBs
#define OV2640_REG1_ZOOMS 0x49             //< Zoom mode vert window MSB
#define OV2640_REG1_COM22 0x4B             //< Common control 22 (flash strobe)
#define OV2640_REG1_COM25 0x4E             //< Common control 25
#define OV2640_COM25_50HZ_MASK 0xC0        //< 50 Hz banding AEC MSBs
#define OV2640_COM25_60HZ_MASK 0x30        //< 60 Hz banding AEC MSBs
#define OV2640_REG1_BD50 0x4F              //< 50 Hz banding AEC LSBs
#define OV2640_REG1_BD60 0x50              //< 60 Hz banding AEC LSBs
#define OV2640_REG1_REG5D 0x5D             //< AVGsel[7:0] 16-zone avg weight
#define OV2640_REG1_REG5E 0x5E             //< AVGsel[15:8]
#define OV2640_REG1_REG5F 0x5F             //< AVGsel[23:16]
#define OV2640_REG1_REG60 0x60             //< AVGsel[31:24]
#define OV2640_REG1_HISTO_LOW 0x61         //< Histogram low level
#define OV2640_REG1_HISTO_HIGH 0x62        //< Histogram high level
