#pragma once

#if defined(__SAMD51__)
#include <stdint.h>

typedef int8_t iCap_pin;

#ifdef __cplusplus
extern "C" {
#endif

extern void iCap_xclk_start(iCap_pin pin, uint32_t freq = 12000000);

extern void iCap_pcc_start();

#ifdef __cplusplus
};
#endif


#endif // end __SAMD51__
