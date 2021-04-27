#pragma once
#include <Adafruit_ImageCapture.h>

/*!
    @brief  Class encapsulating functionality common to image sensors using
            a parallel data interface + I2C for configuration. (This is the
            only type currently implemented, but leaving things open in case
            any later sensors instead use SPI, etc. for the pixel interface.)
*/
class Adafruit_iCap_parallel : public Adafruit_ImageCapture {
public:
  Adafruit_iCap_parallel();
  ~Adafruit_iCap_parallel();
  ICAP_status begin();

private:
};
