#pragma once

#include <Adafruit_ImageCapture.h>

#if defined(ICAP_FULL_SUPPORT)

#include <Wire.h>

/** Pin identifiers for parallel+I2C cameras. */
typedef struct {
  iCap_pin enable;  ///< Also called PWDN, or set to -1 and tie to GND
  iCap_pin reset;   ///< Cam reset, or set to -1 and tie to 3.3V
  iCap_pin xclk;    ///< MCU clock out / cam clock in, -1 if self-clocking
  iCap_pin pclk;    ///< Cam clock out / MCU clock in
  iCap_pin vsync;   ///< Also called DEN1
  iCap_pin hsync;   ///< Also called DEN2
  iCap_pin data[8]; ///< Camera parallel data out
  iCap_pin sda;     ///< I2C data
  iCap_pin scl;     ///< I2C clock
} iCap_parallel_pins;

/** Register/value combo for camera configuration over I2C. */
typedef struct {
  uint8_t reg;   ///< Register address
  uint8_t value; ///< Value to store
} iCap_parallel_config;

/*!
    @brief  Class encapsulating functionality common to image sensors using
            a parallel data interface + I2C for configuration. (This is the
            only type currently implemented, but leaving things open in case
            any later sensors instead use SPI, etc. for the pixel interface.)
*/
class Adafruit_iCap_parallel : public Adafruit_ImageCapture {
public:
  /*!
    @brief  Constructor for Adafruit_iCap_parallel class. This constructor
            is never called directly by user code. Instead, a subclass is
            used, which implicitly calls this constructor...hence no
            defaults here.
    @param  pins_ptr  Pointer iCap_parallel_pins structure, describing
                      physical connection to the camera.
    @param  twi_ptr   Pointer to TwoWire instance (e.g. &Wire or &Wire1),
                      used for I2C communication with camera.
    @param  arch      Pointer to structure containing architecture-specific
                      settings. For example, on SAMD51, this structure
                      includes a pointer to a timer peripheral's base address,
                      used to generate the xclk signal. The structure is
                      always of type iCap_arch, but the specific elements
                      within will vary with each supported architecture.
    @param  addr      I2C address of camera.
    @param  speed     I2C speed in Hz (100000 typ.)
    @param  delay_us  Delay, in microseconds, between I2C commands (some
                      MCUs and/or cameras may cause lockup without some
                      delay).
  */
  Adafruit_iCap_parallel(iCap_parallel_pins *pins_ptr, TwoWire *twi_ptr,
                         iCap_arch *arch, uint8_t addr, uint32_t speed,
                         uint32_t delay_us);
  ~Adafruit_iCap_parallel(); // Destructor

  /*!
    @brief   Allocate and initialize resources behind an Adafruit_OV7670
             instance.
    @return  Status code. ICAP_STATUS_OK on successful init.
    @param   space  One of the iCap_colorspace enumeration values,
                    passed through to Adafruit_ImageCapture::begin().
  */
  iCap_status begin(iCap_colorspace space);

  /*!
    @brief   Reads value of one register from the camera over I2C.
    @param   reg  Register to read, from values defined in camera-specific
                  header.
    @return  Integer value: 0-255 (register contents) on successful read,
             -1 on error.
  */
  int readRegister(uint8_t reg);

  /*!
    @brief  Writes value of one register to the camera over I2C.
    @param  reg    Register to read, from values defined in camera-specific
                   header.
    @param  value  Value to write, 0-255.
  */
  void writeRegister(uint8_t reg, uint8_t value);

  /*!
    @brief  Writes a list of settings to the camera over I2C.
    @param  cfg  Array (pointer-to) of settings to write.
    @param  len  Length of array.
  */
  void writeList(const iCap_parallel_config *cfg, uint16_t len);

  /*!
    @brief  Pause DMA background capture (if supported by architecture)
            before capturing, to avoid tearing. Returns as soon as the
            current frame has finished loading. If DMA background capture
            is not supported, this function has no effect. This is NOT a
            camera sleep function!
  */
  void suspend(void);

  /*!
    @brief  Resume DMA background capture after suspend. If DMA is not
            supported, this function has no effect.
  */
  void resume(void);

protected:
  /*!
    @brief   Starter function for the XCLK output signal.
    @param   freq  Desired XCLK frequency in Hz. Typically the ICAP_XCLK_HZ
                   value defined in the arch-specific header would be used,
                   but leaving this open to future options.
    @return  ICAP_STATUS_OK on success.
  */
  iCap_status xclk_start(uint32_t freq);

  /*!
    @brief   Starter function for parallel capture DMA, etc.
    @param   dest        Destination for data received from camera.
    @param   num_pixels  Number of pixels in image.
    @return  ICAP_STATUS_OK on success.
  */
  iCap_status pcc_start(uint16_t *dest, uint32_t num_pixels);

  TwoWire *wire;            ///< Associated I2C instance
  iCap_parallel_pins pins;  ///< Pin structure (copied in constructor)
  uint32_t i2c_speed;       ///< I2C bus speed
  uint32_t i2c_delay_us;    ///< Delay in microseconds between I2C writes
  uint8_t i2c_address;      ///< Camera I2C address
  bool i2c_started = false; ///< Set true after I2C started
};

#endif // end ICAP_FULL_SUPPORT
