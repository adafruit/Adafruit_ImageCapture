#include <Arduino.h>
#include <Adafruit_iCap_parallel.h>

Adafruit_iCap_parallel::Adafruit_iCap_parallel(iCap_parallel_pins *pins_ptr,
                                               TwoWire *twi_ptr,
                                               iCap_arch *arch, uint8_t addr,
                                               uint32_t speed,
                                               uint32_t delay_us)
    : i2c_address(addr & 0x7F), i2c_speed(speed), i2c_delay_us(delay_us),
      wire(twi_ptr), Adafruit_ImageCapture(arch) {
  memcpy(&pins, pins_ptr, sizeof pins); // Save pins struct in object
}

Adafruit_iCap_parallel::~Adafruit_iCap_parallel() {}

ICAP_status Adafruit_iCap_parallel::begin() {
  wire->begin();
  wire->setClock(i2c_speed);

  // Alloc and initialize peripherals
  // XCLK is PWM out
  // And then parallel capture
  if (pins.xclk >= 0) {
    iCap_xclk_start(pins.xclk, arch);
  }

  iCap_pcc_start();

  // Use writeList() to issue setup to cam

  // Periph setup goes here
  return ICAP_STATUS_OK;
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
