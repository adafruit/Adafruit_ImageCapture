#include <Adafruit_iCap_parallel.h>

Adafruit_iCap_parallel::Adafruit_iCap_parallel() {}

Adafruit_iCap_parallel::~Adafruit_iCap_parallel() {}

ICAP_status Adafruit_iCap_parallel::begin() {
  // Periph setup goes here
  return ICAP_STATUS_OK;
}
