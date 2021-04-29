#include <Adafruit_iCap_OV2640.h>

Adafruit_iCap_OV2640::Adafruit_iCap_OV2640(OV2640_pins &pins, TwoWire &twi,
                                           iCap_arch *arch, uint8_t addr)
    : Adafruit_iCap_parallel((iCap_parallel_pins *)&pins, (TwoWire *)&twi, arch,
                             addr, 100000, 1000) {}

Adafruit_iCap_OV2640::~Adafruit_iCap_OV2640() {}

iCap_status Adafruit_iCap_OV2640::begin() {
  Adafruit_iCap_parallel::begin();
  return ICAP_STATUS_OK;
}
