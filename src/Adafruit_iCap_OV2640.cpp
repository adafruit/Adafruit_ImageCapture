#include <Adafruit_iCap_OV2640.h>

Adafruit_iCap_OV2640::Adafruit_iCap_OV2640() {}

Adafruit_iCap_OV2640::~Adafruit_iCap_OV2640() {}

ICAP_status Adafruit_iCap_OV2640::begin() {
  Adafruit_iCap_parallel::begin();
  return ICAP_STATUS_OK;
}
