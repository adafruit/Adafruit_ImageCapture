#include <Adafruit_iCap_OV7670.h>

Adafruit_iCap_OV7670::Adafruit_iCap_OV7670() {}

Adafruit_iCap_OV7670::~Adafruit_iCap_OV7670() {}

ICAP_status Adafruit_iCap_OV7670::begin() {
  Adafruit_iCap_parallel::begin();
  return ICAP_STATUS_OK;
}
