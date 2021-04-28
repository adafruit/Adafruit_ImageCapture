#pragma once
#include <Wire.h>
#include <Adafruit_ImageCapture.h>

typedef struct {
  iCap_pin enable;  ///< Also called PWDN, or set to -1 and tie to GND
  iCap_pin reset;   ///< Cam reset, or set to -1 and tie to 3.3V
  iCap_pin xclk;    ///< MCU clock out / cam clock in
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
  // This constructor is never called directly by user code.
  // Instead, a subclass is used (this is called implicitly, using any
  // defaults established by the subclass, hence no defaults here).
  Adafruit_iCap_parallel(iCap_parallel_pins *pins_ptr, TwoWire *twi_ptr,
                         iCap_arch *arch, uint8_t addr, uint32_t speed,
                         uint32_t delay_us);
  ~Adafruit_iCap_parallel();
  ICAP_status begin();

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
    @param  reg  Array (pointer-to) of settings to write.
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
  TwoWire *wire;           ///< Associated I2C instance
  iCap_parallel_pins pins; ///< Pin structure (copied in constructor)
  uint32_t i2c_speed;      ///< I2C bus speed
  uint32_t i2c_delay_us;   ///< Delay in microseconds between I2C writes
  uint8_t i2c_address;     ///< Camera I2C address
};
