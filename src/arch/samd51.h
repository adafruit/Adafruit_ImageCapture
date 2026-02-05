#pragma once

#if defined(__SAMD51__)

#define ICAP_XCLK_HZ 12000000

typedef int8_t iCap_pin;

// Device-specific structure attached to Adafruit_ImageCapture.arch,
// if low-level peripherals can't be inferred from pin numbers.
typedef struct {
  void* timer;    ///< TC or TCC peripheral base address for XCLK out
  bool xclk_pdec; ///< If true, XCLK needs special PDEC pin mux
} iCap_arch;

#endif // end __SAMD51__
