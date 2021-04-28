#pragma once

#if defined(__SAMD51__)
#include <Adafruit_ImageCapture.h>

typedef int8_t iCap_pin;

// Device-specific structure attached to Adafruit_ImageCapture.arch,
// if low-level peripherals can't be inferred from pin numbers.
typedef struct {
  void *timer;    ///< TC or TCC peripheral base address for XCLK out
  bool xclk_pdec; ///< If true, XCLK needs special PDEC pin mux
} iCap_arch;

extern ICAP_status iCap_xclk_start(iCap_pin pin, iCap_arch *arch = NULL,
                                   uint32_t freq = 12000000);

extern ICAP_status iCap_pcc_start();

#endif // end __SAMD51__
