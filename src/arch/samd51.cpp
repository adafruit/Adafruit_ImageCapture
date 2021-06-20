// This is the SAMD51-specific parts of camera interfacing. It configures
// and accesses hardware-specific peripherals (timer PWM and the parallel
// capture controller, others might follow in the future if needed).

#if defined(__SAMD51__)
#include <Adafruit_ZeroDMA.h>
#include <Adafruit_iCap_parallel.h>
#include <Arduino.h>
#include <wiring_private.h> // pinPeripheral() function

// Because interrupts exist outside the class context, but our interrupt
// needs to access to an active ZeroDMA object, a separate ZeroDMA pointer
// is kept (initialized in the begin() function). This does mean that only
// a single OV7670 can be active (probably no big deal, as there's only a
// single parallel capture peripheral).

static Adafruit_ZeroDMA dma;
static DmacDescriptor *descriptor;       ///< DMA descriptor
static volatile bool frameReady = false; ///< true at end-of-frame
static volatile bool suspended = true;   ///< Start in suspended state

// Since ZeroDMA suspend/resume functions don't yet work, these functions
// use static vars to indicate whether to trigger DMA transfers or hold off
// (camera keeps running, data is simply ignored without a DMA transfer).

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

// Pin interrupt on VSYNC calls this to start DMA transfer (unless suspended).
static void startFrame(void) {
  if (!suspended) {
    frameReady = false;
    (void)dma.startJob();
  }
}

// End-of-DMA-transfer callback
static void dmaCallback(Adafruit_ZeroDMA *dma) { frameReady = true; }

// XCLK clock out setup. For self-clocking cameras, don't call this function,
// e.g. Adafruit_iCap_parallel.begin() checks the value of the xlck pin and
// skips this if -1.
iCap_status Adafruit_iCap_parallel::xclk_start(uint32_t freq) {

  // LOOK UP TIMER OR TCC BASED ON ADDRESS IN ARCH STRUCT ------------------

  static const struct {
    void *base;       ///< TC or TCC peripheral base address
    uint8_t GCLK_ID;  ///< Timer ID for GCLK->PCHCTRL
    const char *name; ///< Printable timer ID for debug use
  } timer[] = {
#if defined(TC0)
    {TC0, TC0_GCLK_ID, "TC0"},
#endif
#if defined(TC1)
    {TC1, TC1_GCLK_ID, "TC1"},
#endif
#if defined(TC2)
    {TC2, TC2_GCLK_ID, "TC2"},
#endif
#if defined(TC3)
    {TC3, TC3_GCLK_ID, "TC3"},
#endif
#if defined(TC4)
    {TC4, TC4_GCLK_ID, "TC4"},
#endif
#if defined(TC5)
    {TC5, TC5_GCLK_ID, "TC5"},
#endif
#if defined(TC6)
    {TC6, TC6_GCLK_ID, "TC6"},
#endif
#if defined(TC7)
    {TC7, TC7_GCLK_ID, "TC7"},
#endif
#if defined(TC8)
    {TC8, TC8_GCLK_ID, "TC8"},
#endif
#if defined(TC9)
    {TC9, TC9_GCLK_ID, "TC9"},
#endif
#if defined(TC10)
    {TC10, TC10_GCLK_ID, "TC10"},
#endif
#if defined(TC11)
    {TC11, TC11_GCLK_ID, "TC11"},
#endif
#if defined(TC12)
    {TC12, TC12_GCLK_ID, "TC12"},
#endif
    {NULL, 0, NULL}, // NULL separator between TC and TCC lists
#if defined(TCC0)
    {TCC0, TCC0_GCLK_ID, "TCC0"},
#endif
#if defined(TCC1)
    {TCC1, TCC1_GCLK_ID, "TCC1"},
#endif
#if defined(TCC2)
    {TCC2, TCC2_GCLK_ID, "TCC2"},
#endif
#if defined(TCC3)
    {TCC3, TCC3_GCLK_ID, "TCC3"},
#endif
#if defined(TCC4)
    {TCC4, TCC4_GCLK_ID, "TCC4"},
#endif
  };

  uint8_t timer_list_index;
  bool is_tcc = false; // Initial part of list is timer/counters, not TCCs
  // Scan timer[] list until a matching timer/TCC is found...
  for (timer_list_index = 0;
       (timer_list_index < sizeof timer / sizeof timer[0]) &&
       (timer[timer_list_index].base != arch->timer);
       timer_list_index++) {
    if (!timer[timer_list_index].base) { // NULL separator?
      is_tcc = true; // In the TCC (not TC) part of the list now
    }
  }
  if (timer_list_index >= sizeof timer / sizeof timer[0]) {
    return ICAP_STATUS_ERR_PERIPHERAL; // No matching TC/TCC found
  }

  // CONFIGURE TIMER FOR XCLK OUT ------------------------------------------

  uint8_t id = timer[timer_list_index].GCLK_ID;

  // Route timer's peripheral channel control to GCLK1 (48 MHz)

  GCLK->PCHCTRL[id].bit.CHEN = 0;    // Peripheral channel disable
  while (GCLK->PCHCTRL[id].bit.CHEN) // Wait for disable
    ;
  // Select generator 1, enable channel, use .reg so it's an atomic op
  GCLK->PCHCTRL[id].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;
  while (!GCLK->PCHCTRL[id].bit.CHEN) // Wait for enable
    ;

  if (is_tcc) { // Is a TCC peripheral

    Tcc *tcc = (Tcc *)timer[timer_list_index].base;

    tcc->CTRLA.bit.ENABLE = 0; // Disable TCC before configuring
    while (tcc->SYNCBUSY.bit.ENABLE)
      ;
    tcc->CTRLA.bit.PRESCALER = TCC_CTRLA_PRESCALER_DIV1_Val; // 1:1 Prescale
    tcc->WAVE.bit.WAVEGEN = TCC_WAVE_WAVEGEN_NPWM_Val;       // Normal PWM
    while (tcc->SYNCBUSY.bit.WAVE)
      ;

    uint16_t period = 48000000 / freq - 1;
    tcc->PER.bit.PER = period;
    while (tcc->SYNCBUSY.bit.PER)
      ;
    tcc->CC[1].bit.CC = (period + 1) / 2; // Aim for ~50% duty cycle
    while (tcc->SYNCBUSY.bit.CC1)
      ;
    tcc->CTRLA.bit.ENABLE = 1;
    while (tcc->SYNCBUSY.bit.ENABLE)
      ;

    pinPeripheral(pins.xclk, arch->xclk_pdec ? PIO_TCC_PDEC : PIO_TIMER_ALT);

  } else { // Is a TC peripheral

    Tc *tc = (Tc *)timer[timer_list_index].base;

    // TO DO: ADD TC PERIPHERAL (NOT TCC) CODE HERE

    pinPeripheral(pins.xclk, PIO_TIMER);

  } // end TC/TCC

  return ICAP_STATUS_OK;
}

// Start parallel capture peripheral
iCap_status Adafruit_iCap_parallel::pcc_start(void) {

  PCC->MR.bit.PCEN = 0; // Make sure PCC is disabled before setting MR reg

  PCC->IDR.reg = 0b1111;       // Disable all PCC interrupts
  MCLK->APBDMASK.bit.PCC_ = 1; // Enable PCC clock

  // Set up pin MUXes for the camera clock, sync and data pins.
  // I2C pins are already configured, XCLK is handled elsewhere,
  // and the reset and enable pins are handled in the calling code.
  // Must do this before setting MR reg.
  // PCC pins are set in stone on SAMD51
  pinPeripheral(PIN_PCC_CLK, PIO_PCC);
  pinPeripheral(PIN_PCC_DEN1, PIO_PCC); // VSYNC
  pinPeripheral(PIN_PCC_DEN2, PIO_PCC); // HSYNC
  pinPeripheral(PIN_PCC_D0, PIO_PCC);
  pinPeripheral(PIN_PCC_D1, PIO_PCC);
  pinPeripheral(PIN_PCC_D2, PIO_PCC);
  pinPeripheral(PIN_PCC_D3, PIO_PCC);
  pinPeripheral(PIN_PCC_D4, PIO_PCC);
  pinPeripheral(PIN_PCC_D5, PIO_PCC);
  pinPeripheral(PIN_PCC_D6, PIO_PCC);
  pinPeripheral(PIN_PCC_D7, PIO_PCC);

  // Accumulate 4 bytes into RHR register (two 16-bit pixels)
  PCC->MR.reg = PCC_MR_CID(0x1) |   // Clear on falling DEN1 (VSYNC)
                PCC_MR_ISIZE(0x0) | // Input data bus is 8 bits
                PCC_MR_DSIZE(0x2);  // "4 data" at a time (accumulate in RHR)

  PCC->MR.bit.PCEN = 1; // Enable PCC

  // Sets up DMA for the parallel capture controller.

  ZeroDMAstatus dma_status = dma.allocate();
  // To do: handle dma_status here, map to OV7670_status on error
  dma.setAction(DMA_TRIGGER_ACTON_BEAT);
  dma.setTrigger(PCC_DMAC_ID_RX);
  dma.setCallback(dmaCallback);
  dma.setPriority(DMA_PRIORITY_3);

  // Use 32-bit PCC transfers (4 bytes accumulate in RHR.reg)
  descriptor = dma.addDescriptor((void *)(&PCC->RHR.reg), // Source
                                 NULL,                    // Dest set later
                                 0,                       // Count set later
                                 DMA_BEAT_SIZE_WORD,      // 32-bit words
                                 false,                   // Don't src++
                                 true);                   // Do dest++

  // A pin FALLING interrupt is used to detect the start of a new frame.
  // Seems like the PCC RXBUFF and/or ENDRX interrupts could take care
  // of this, but in practice that didn't seem to work.
  // DEN1 is the PCC VSYNC pin.
  attachInterrupt(PIN_PCC_DEN1, startFrame, FALLING);

  return ICAP_STATUS_OK;
}

void Adafruit_iCap_parallel::dma_change(uint16_t *dest, uint32_t num_pixels) {
  dma.changeDescriptor(descriptor, (void *)(&PCC->RHR.reg), (void *)dest,
                       num_pixels / 2);
}

#endif // end __SAMD51__

#if 0

// Non-DMA capture function using previously-initialized PCC peripheral.
void OV7670_capture(uint32_t *dest, uint16_t width, uint16_t height,
                    volatile uint32_t *vsync_reg, uint32_t vsync_bit,
                    volatile uint32_t *hsync_reg, uint32_t hsync_bit) {

  while (*vsync_reg & vsync_bit)
    ; // Wait for VSYNC low (frame end)
  OV7670_disable_interrupts();
  while (!*vsync_reg & vsync_bit)
    ; // Wait for VSYNC high (frame start)

  width /= 2;                             // PCC receives 2 pixels at a time
  for (uint16_t y = 0; y < height; y++) { // For each row...
    while (*hsync_reg & hsync_bit)
      ; //  Wait for HSYNC low (row end)
    while (!*hsync_reg & hsync_bit)
      ;                               //  Wait for HSYNC high (row start)
    for (int x = 0; x < width; x++) { //   For each column pair...
      while (!PCC->ISR.bit.DRDY)
        ;                     //    Wait for PCC data ready
      *dest++ = PCC->RHR.reg; //    Store 2 pixels
    }
  }

  OV7670_enable_interrupts();
}

#endif // 0
