#if defined(ARDUINO_ARCH_RP2040)
#include "../../hardware_dma/include/hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include <Adafruit_iCap_parallel.h>

// PIO code in this table is modified at runtime so that PCLK is
// configurable (rather than fixed GP## or PIN offset). Data pins
// must be contiguous but are otherwise configurable.
static uint16_t iCap_pio_opcodes[] = {
    // Only monitor PCLK when HSYNC is high. This is more noise-immune
    // than letting it fly.
    0b0010000010000000, // WAIT 1 GPIO 0 (mask in HSYNC pin before use)
    0b0010000010000000, // WAIT 1 GPIO 0 (mask in PCLK pin before use)
    0b0100000000001000, // IN PINS 8 -- 8 bits into RX FIFO
    0b0010000000000000, // WAIT 0 GPIO 0 (mask in PCLK pin before use)
};

static struct pio_program iCap_pio_program = {
    .instructions = iCap_pio_opcodes,
    .length = sizeof iCap_pio_opcodes / sizeof iCap_pio_opcodes[0],
    .origin = -1,
};

// Because interrupts exist outside the class context, but our interrupt
// needs to access to object- and arch-specific data like the camera buffer
// and DMA settings, pointers are kept (initialized in the begin() function).
// This does mean that only a single OV7670 can be active.
static Adafruit_ImageCapture *capptr = NULL; // Camera buffer, size, etc.
static iCap_arch *archptr = NULL;            // DMA settings
static volatile bool frameReady = false;     // true at end-of-frame
static volatile bool suspended = true;       // Initially stopped

// This is NOT a sleep function, it just pauses background DMA.

void Adafruit_iCap_parallel::suspend(void) {
  while (!frameReady)
    ;               // Wait for current frame to finish loading
  suspended = true; // Don't load next frame (camera runs, DMA stops)
}

// NOT a wake function, just resumes background DMA.

void Adafruit_iCap_parallel::resume(void) {
  frameReady = false;
  suspended = false; // Resume DMA transfers
}

// INTERRUPT HANDLING AND RELATED CODE -------------------------------------

// To do: use the arch state variable to detect when a VSYNC IRQ occurs
// before the last DMA transfer is complete (suggesting one or more pixels
// dropped, likely bad PCLK signal). If that happens, abort the DMA
// transfer and return, skip frames and don't begin next DMA until abort
// has completed (this is what state var is for, to detect if in this
// holding pattern).

// Pin interrupt on VSYNC calls this to start DMA transfer (unless suspended).
static void iCap_vsync_irq(uint gpio, uint32_t events) {
  if (!suspended) {
    frameReady = false;
    // Clear PIO FIFOs and start DMA transfer
    pio_sm_clear_fifos(archptr->pio, archptr->sm);
    dma_channel_start(archptr->dma_channel);
  }
}

static void iCap_dma_finish_irq() {
  // DMA transfer completed. Set up (but do not trigger) next one.
  frameReady = true;
  // Channel MUST be reconfigured each time (to reset the dest address).
  dma_channel_set_write_addr(archptr->dma_channel,
                             (uint8_t *)(capptr->getBuffer()), false);
  dma_hw->ints0 = 1u << archptr->dma_channel; // Clear IRQ
}

// XCLK clock out setup. For self-clocking cameras, don't call this function,
// e.g. Adafruit_iCap_parallel.begin() checks the value of the xlck pin and
// skips this if -1.
iCap_status Adafruit_iCap_parallel::xclk_start(uint32_t freq) {

  // LOOK UP PWM SLICE & CHANNEL BASED ON XCLK PIN -------------------------

  uint8_t slice = pwm_gpio_to_slice_num(pins.xclk);
  uint8_t channel = pwm_gpio_to_channel(pins.xclk);

  // CONFIGURE PWM FOR XCLK OUT --------------------------------------------
  // XCLK to camera is required for it to communicate over I2C!

  pwm_config config = pwm_get_default_config();
#if 1
  pwm_config_set_clkdiv(&config, (float)F_CPU / (float)(freq * 2));
  pwm_config_set_wrap(&config, 1); // 2 clocks/cycle
  pwm_init(slice, &config, true);
  pwm_set_chan_level(slice, channel, 1); // 50% duty cycle
#else
  pwm_config_set_clkdiv(&config, 1); // PWM clock = F_CPU
  pwm_config_set_wrap(&config, F_CPU / freq);
  pwm_init(slice, &config, true);
  pwm_set_chan_level(slice, channel, F_CPU / freq / 2); // 50% duty
#endif
  gpio_set_function(pins.xclk, GPIO_FUNC_PWM);

  return ICAP_STATUS_OK;
}

// Start parallel capture peripheral and DMA. Transfers are not started
// yet, until frame size and buffer are established.
iCap_status Adafruit_iCap_parallel::pcc_start(void) {

  // TO DO: verify the DATA pins are contiguous.

  archptr = arch; // Save arch pointer for interrupts
  capptr = this;  // Save object pointer, same

  // Set up GPIO pins (other than I2C, done in the Wire lib)
  gpio_init(pins.pclk);
  gpio_set_dir(pins.pclk, GPIO_IN);
  gpio_init(pins.vsync);
  gpio_set_dir(pins.vsync, GPIO_IN);
  gpio_init(pins.hsync);
  gpio_set_dir(pins.hsync, GPIO_IN);
  for (uint8_t i = 0; i < 8; i++) {
    gpio_init(pins.data[i]);
    gpio_set_dir(pins.data[i], GPIO_IN);
  }

  // PIO periph to use is currently specified by the user in the arch struct,
  // but I suppose this could be written to use whatever PIO has resources.

  // Mask the GPIO pin used PCLK into the PIO opcodes -- see notes at top
  iCap_pio_opcodes[0] |= (pins.hsync & 31);
  iCap_pio_opcodes[1] |= (pins.pclk & 31);
  iCap_pio_opcodes[3] |= (pins.pclk & 31);

  // Here's where resource check & switch between pio0/1 might go
  uint offset = pio_add_program(arch->pio, &iCap_pio_program);
  arch->sm = pio_claim_unused_sm(arch->pio, true); // 0-3

  // host->pins->data[0] is data bit 0. PIO code requires all 8 data be
  // contiguous.
  pio_sm_set_consecutive_pindirs(arch->pio, arch->sm, pins.data[0], 8, false);

  pio_sm_config c = pio_get_default_sm_config();
  c.pinctrl = 0; // SDK fails to set this
  sm_config_set_wrap(&c, offset, offset + iCap_pio_program.length - 1);

  sm_config_set_in_pins(&c, pins.data[0]);
  sm_config_set_in_shift(&c, false, true, 16); // 1 pixel (16b) ISR to FIFO
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

  pio_sm_init(arch->pio, arch->sm, offset, &c);
  pio_sm_set_enabled(arch->pio, arch->sm, true);

  // SET UP DMA ------------------------------------------------------------

  arch->dma_channel = dma_claim_unused_channel(false); // don't panic

  arch->dma_config = dma_channel_get_default_config(arch->dma_channel);
  channel_config_set_transfer_data_size(&arch->dma_config, DMA_SIZE_16);
  channel_config_set_read_increment(&arch->dma_config, false);
  channel_config_set_write_increment(&arch->dma_config, true);
  channel_config_set_bswap(&arch->dma_config, arch->bswap);
  // Set PIO RX as DMA trigger. Input shift register saturates at 16 bits
  // (1 pixel), configured in data size above and in PIO setup elsewhere.
  channel_config_set_dreq(&arch->dma_config,
                          pio_get_dreq(arch->pio, arch->sm, false));
  // Set up baseline DMA configuration...it's initially lacking destination
  // and count, set later (dma_change()) after resolution is known. DMA
  // isn't started until later, and is triggered in the vsync interrupt.
  dma_channel_configure(arch->dma_channel, &arch->dma_config, NULL,
                        &arch->pio->rxf[arch->sm], 0, false);

  // Set up end-of-DMA interrupt
  dma_channel_set_irq0_enabled(arch->dma_channel, true);
  irq_set_exclusive_handler(DMA_IRQ_0, iCap_dma_finish_irq);
  irq_set_enabled(DMA_IRQ_0, true);

  // SET UP VSYNC INTERRUPT ------------------------------------------------

  gpio_set_irq_enabled_with_callback(pins.vsync, GPIO_IRQ_EDGE_RISE, true,
                                     &iCap_vsync_irq);

  return ICAP_STATUS_OK;
}

// Need to do this on startup and when changing resolution.
// Changing resolution also requires stopping DMA temporarily...
// wait for frame to finish, do realloc/cam config, then restart.
// That'll go in Adafruit_iCap_parallel.cpp
void Adafruit_iCap_parallel::dma_change(uint16_t *dest, uint32_t num_pixels) {
  dma_channel_set_write_addr(archptr->dma_channel, dest, false);
  dma_channel_set_trans_count(archptr->dma_channel, num_pixels, false);
}

#endif // end ARDUINO_ARCH_RP2040
