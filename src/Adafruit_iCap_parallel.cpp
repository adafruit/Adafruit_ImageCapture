#include <Adafruit_iCap_parallel.h>

#if defined(ICAP_FULL_SUPPORT)

#include <Arduino.h>

Adafruit_iCap_parallel::Adafruit_iCap_parallel(iCap_parallel_pins *pins_ptr,
                                               iCap_arch *arch,
                                               uint16_t *pbuf,
                                               uint32_t pbufsize,
                                               TwoWire *twi_ptr,
                                               uint8_t addr, uint32_t speed,
                                               uint32_t delay_us)
    : i2c(addr & 0x7F, twi_ptr), i2c_speed(speed), i2c_delay_us(delay_us),
      Adafruit_ImageCapture(arch, pbuf, pbufsize) {
  memcpy(&pins, pins_ptr, sizeof pins); // Save pins struct in object
#if defined(ARDUINO_ARCH_RP2040)
  wire = twi_ptr; // Needed for setSDA/setSCL in begin()
#endif
}

Adafruit_iCap_parallel::~Adafruit_iCap_parallel() {}

iCap_status Adafruit_iCap_parallel::begin() {
  // Set up XCLK out unless it's a self-clocking camera. This must be done
  // BEFORE any I2C commands, as cam may require clock for I2C timing.
  if (pins.xclk >= 0) {
    xclk_start(ICAP_XCLK_HZ);
  }

#if defined(ARDUINO_ARCH_RP2040)
  wire->setSDA(pins.sda);
  wire->setSCL(pins.scl);
#endif
  i2c.begin();
  i2c.setSpeed(i2c_speed);

  // Set up parallel capture peripheral & DMA. Camera is initially suspended,
  // calling code resumes cam DMA only after I2C init sequence is sent.
  pcc_start();

  return ICAP_STATUS_OK;
}

// Is this one not needed? Does this happen in subclass?
// See same question in header.
#if 0
iCap_status Adafruit_iCap_parallel::begin(uint16_t width, uint16_t height,
                                          iCap_colorspace space, uint8_t nbuf) {
  iCap_status status = begin(); // Sets up peripherals
  if (status == ICAP_STATUS_OK) {
    status = bufferConfig(width, height, space, nbuf); // Allocs RAM
    if (status == ICAP_STATUS_OK) {
      // Start DMA here
// Need arch function to set up next DMA xfer, trigger, etc.
    }
  }

  return status;
}
#endif

int Adafruit_iCap_parallel::readRegister(uint8_t reg) {
  i2c.write_then_read(&reg, 1, &reg, 1);
  return reg; // Result is stored back in reg var
}

void Adafruit_iCap_parallel::writeRegister(uint8_t reg, uint8_t value) {
  uint8_t buf[] = { reg, value };
  i2c.write(buf, sizeof buf);
}

void Adafruit_iCap_parallel::writeList(const iCap_parallel_config *cfg,
                                       uint16_t len) {
  for (int i = 0; i < len; i++) {
    writeRegister(cfg[i].reg, cfg[i].value);
    delayMicroseconds(i2c_delay_us); // Some cams require, else lockup on init
  }
}

void Adafruit_iCap_parallel::writeList(const iCap_parallel_config16x8 *cfg,
                                       uint16_t len) {
  for (int i = 0; i < len; i++) {
    writeRegister16x8(cfg[i].reg, cfg[i].value);
    delayMicroseconds(i2c_delay_us); // Some cams require, else lockup on init
  }
}

void Adafruit_iCap_parallel::writeList(const iCap_parallel_config16x16 *cfg,
                                       uint16_t len) {
  for (int i = 0; i < len; i++) {
    writeRegister16x16(cfg[i].reg, cfg[i].value);
    delayMicroseconds(i2c_delay_us); // Some cams require, else lockup on init
  }
}

// 16-bit registers/values are issued/recived in big-endian order, hence
// all the shift/mask operations here (most MCUs being LE these days).

int Adafruit_iCap_parallel::readRegister16x8(uint16_t reg) {
  uint8_t buf[] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
  i2c.write_then_read(buf, sizeof buf, buf, 1);
  return buf[0];
}

void Adafruit_iCap_parallel::writeRegister16x8(uint16_t reg, uint8_t value) {
  uint8_t buf[] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), value };
  i2c.write(buf, sizeof buf);
}

int Adafruit_iCap_parallel::readRegister16x16(uint16_t reg) {
  uint8_t buf[] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
  i2c.write_then_read(buf, sizeof buf, buf, 2);
  return (buf[0] << 8) | buf[1];
}

void Adafruit_iCap_parallel::writeRegister16x16(uint16_t reg, uint16_t value) {
  uint8_t buf[] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
                    (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
  i2c.write(buf, sizeof buf);
}

#endif // end ICAP_FULL_SUPPORT
