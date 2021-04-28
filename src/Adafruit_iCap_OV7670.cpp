#include <Adafruit_iCap_OV7670.h>

Adafruit_iCap_OV7670::Adafruit_iCap_OV7670(OV7670_pins &pins,
                                           TwoWire &twi, uint8_t addr)
    : Adafruit_iCap_parallel((iCap_parallel_pins *)&pins, (TwoWire *)&twi,
                             addr, 100000, 1000) {}

Adafruit_iCap_OV7670::~Adafruit_iCap_OV7670() {}

ICAP_status Adafruit_iCap_OV7670::begin(ICAP_colorspace colorspace,
                    OV7670_size size, float fps, uint32_t bufsiz) {
  // Do basic allocs here:
  Adafruit_iCap_parallel::begin();

  // Do camera init here (command table, etc.)

  resume(); // Start DMA cycle

  return ICAP_STATUS_OK;
}
