#include <Adafruit_iCap_parallel.h>

#if defined(ICAP_FULL_SUPPORT)

#include <Arduino.h>

Adafruit_iCap_parallel::Adafruit_iCap_parallel(iCap_arch *arch,
                                               iCap_parallel_pins *pins_ptr,
                                               uint16_t *pbuf,
                                               uint32_t pbufsize,
                                               TwoWire *twi_ptr,
                                               uint8_t addr, uint32_t speed,
                                               uint32_t delay_us)
    : i2c_address(addr & 0x7F), i2c_speed(speed), i2c_delay_us(delay_us),
      wire(twi_ptr), Adafruit_ImageCapture(arch, pbuf, pbufsize) {
  memcpy(&pins, pins_ptr, sizeof pins); // Save pins struct in object
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
  wire->begin();
  wire->setClock(i2c_speed);

  // Set up parallel capture peripheral & DMA. Camera is initially suspended,
  // calling code resumes cam DMA only after I2C init sequence is sent.
  pcc_start();

  return ICAP_STATUS_OK;
}

iCap_status Adafruit_iCap_parallel::begin(iCap_colorspace space,
                                          uint16_t width, uint16_t height,
                                          uint8_t nbuf) {
  begin();
  setSize(space, width, height, nbuf, ICAP_REALLOC_CHANGE);
}

void Adafruit_iCap_parallel::startI2C(void) {
}

int Adafruit_iCap_parallel::readRegister(uint8_t reg) {
  wire->beginTransmission(i2c_address);
  wire->write(reg);
  wire->endTransmission();
  wire->requestFrom(i2c_address, (uint8_t)1);
  return wire->read();
}

void Adafruit_iCap_parallel::writeRegister(uint8_t reg, uint8_t value) {
  wire->beginTransmission(i2c_address);
  wire->write(reg);
  wire->write(value);
  wire->endTransmission();
}

void Adafruit_iCap_parallel::writeList(const iCap_parallel_config *cfg,
                                       uint16_t len) {
  for (int i = 0; i < len; i++) {
    writeRegister(cfg[i].reg, cfg[i].value);
    delayMicroseconds(i2c_delay_us); // Some cams require, else lockup on init
  }
}

#endif // end ICAP_FULL_SUPPORT
