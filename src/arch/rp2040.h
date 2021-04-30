#pragma once

#if defined(ARDUINO_ARCH_RP2040)
#include "../../hardware_dma/include/hardware/dma.h"
#include "hardware/pio.h"

#define ICAP_XCLK_HZ 12500000

typedef int8_t iCap_pin;

// Device-specific structure attached to Adafruit_ImageCapture.arch,
// if low-level peripherals can't be inferred from pin numbers.
typedef struct {
  PIO pio;                       ///< PIO peripheral
  uint8_t sm;                    ///< State machine #
  int dma_channel;               ///< DMA channel #
  dma_channel_config dma_config; ///< DMA configuration
  bool bswap;                    ///< DMA byte-swap behavior
} iCap_arch;

#endif // end ARDUINO_ARCH_RP2040
