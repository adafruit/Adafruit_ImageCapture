#pragma once

#if defined(ARDUINO_ARCH_RP2040)
#include <stdint.h>

typedef int8_t iCap_pin;

// Device-specific structure attached to Adafruit_ImageCapture.arch,
// if low-level peripherals can't be inferred from pin numbers.
typedef struct {
} iCap_arch;

extern ICAP_status iCap_xclk_start(iCap_pin pin, iCap_arch *arch = NULL,
                                   uint32_t freq = 12500000);

extern ICAP_status iCap_pcc_start();

#endif // end ARDUINO_ARCH_RP2040
