/*
  fan_dma_test.cpp — STANDALONE DSHOT300 via TIM1 + DMA burst.      env: fandma

  PURPOSE
  -------
  Prove the hardware-generated DSHOT path in isolation, before any of it goes
  near the RTOS firmware. Same ESC, same motors, same commands as fan_test.cpp
  (the confirmed-good bit-bang reference) so the two binaries can be A/B'd on
  the same hardware by just changing -e. fan_test.cpp is NOT modified.

  WHY THE BIT-BANG CANNOT COME ACROSS
  -----------------------------------
  fan_test.cpp masks interrupts with noInterrupts() (= PRIMASK) for the ~53 us
  of each frame, four times per 2 ms cycle. PRIMASK masks EVERYTHING -- TIM9,
  SysTick, PendSV -- so it doesn't just delay commutation, it stops the kernel
  from scheduling at all. Measured control release is 5000 +/- 6 us; a 53 us
  mask makes that +/- 53 us, a 9x degradation of the number the RTOS migration
  was built to prove. It is also a busy-wait: ~213 us of DWT spinning per 2 ms
  is ~10.6% of the CPU, against 27% idle.

  THE REPLACEMENT: TIM1 + DMA BURST
  ---------------------------------
  DSHOT is just PWM at a fixed period where the DUTY encodes the bit, so no
  software timing is needed at all.

      DSHOT300   bit period 3.333 us    '0' = 37.5% duty    '1' = 75% duty

  TIM1 is on APB2. At HCLK 180 MHz the F446 runs APB2 /2 -> PCLK2 90 MHz, and
  APB timers run at 2x the bus clock when the prescaler != 1, so TIM1 counts at
  180 MHz:

      PSC = 0, ARR = 599   -> 600 ticks = 3.3333 us = 300.000 kHz
      CCR '0' = 225        -> 1.250 us high
      CCR '1' = 450        -> 2.500 us high

  Burst mode is what lets ONE DMA stream drive FOUR channels. TIM1->DMAR is a
  window register: a write to it is redirected by hardware to CR1 + DBA + idx,
  with idx auto-incrementing across the burst. CCR1 sits at offset 0x34, so
  DBA = 0x34/4 = 13, and DBL = 3 means "4 transfers per request". One update
  event (one counter rollover, i.e. one bit period) therefore moves four words
  into CCR1..CCR4. The source buffer is simply interleaved:

      [b15 ch1][b15 ch2][b15 ch3][b15 ch4][b14 ch1]...

  TIM1_UP maps to DMA2 Stream 5, Channel 6. That stream is free: SPI2 (encoder)
  is blocking with no DMA, I2C1 is on DMA1, and USART6 (the future Pi link)
  maps to DMA2 streams 1/2/6/7 on channel 5.

  NO INTERRUPT IS USED. The frame is re-armed by polling DMA_SxCR.EN, which
  sidesteps the STM32duino strong-IRQHandler collision trap entirely -- there is
  no DMA2_Stream5_IRQHandler here to collide with anything.

  WHY 18 BUFFER ENTRIES PER CHANNEL, NOT 16
  -----------------------------------------
  The two trailing CCR = 0 entries are NOT cosmetic. Without them CCRx retains
  the last data bit's value (225 or 450) when the transfer ends and the pin
  keeps pulsing at 37.5%/75% duty forever. The trailing zeros are what put the
  line low. CCRx preload (OCxPE) is enabled, so a value DMA'd on update event N
  takes effect for period N+1 -- one period of pipeline, which is why 16 data
  bits need 18 entries to fully flush.

  HAZARDS SPECIFIC TO TIM1 ON THIS BOARD
  --------------------------------------
  1. MOE (BDTR bit 15). Advanced-timer outputs are dead until it is set, and the
     symptom is "every register reads perfect, pin never moves."
  2. The COMPLEMENTARY outputs land on pins we cannot afford to touch:
     TIM1_CH1N/2N/3N map to PA7, PB0/PB13/PB14, PB1/PB15 -- motor phase C, the
     encoder SCK/MISO/MOSI, and the encoder CS. They are gated by each pin's
     GPIO alternate-function mux so they cannot appear unless AF1 is also set
     on those pins, which we never do. Margin for error is zero, so CCxNE stays
     clear in CCER and BKE stays clear in BDTR (break input BKIN is PA6 = motor
     phase B).
  3. URS (CR1 bit 2) is set. Otherwise the software UG write used to latch
     PSC/ARR at init ALSO generates an update event, firing a spurious DMA
     request before the buffer is armed and desyncing the frame by one word.

  ------------------------------------------------------------------ SAFETY ---
  PROPS OFF. Throttle capped at FAN_THROTTLE_MAX. Unrecognised input stops all.

  ----------------------------------------------------------------- WIRING ----
      ESC pad 1 -> D7        -> PA8   (TIM1_CH1)   unchanged
      ESC pad 2 -> D8        -> PA9   (TIM1_CH2)   unchanged
      ESC pad 3 -> D2        -> PA10  (TIM1_CH3)   unchanged
      ESC pad 4 -> CN10-14   -> PA11  (TIM1_CH4)   *** MOVED from A0/PA0 ***
      ESC '-'   -> GND
      ESC '+' and 'N' -> NOT CONNECTED

  CN10 is the ST morpho header = MALE pins. PA11 is not on any Arduino header,
  so a male-ended jumper has nothing to plug into; use a female-female Dupont
  as a coupler. PA11 is forced, not preferred: TIM1_CH4 maps only to PA11 and
  PE14, and the F446RE is LQFP64 with no GPIOE.
*/

#include <Arduino.h>

/* ------------------------------------------------------------ config ------ */
#define HC05_EN_PIN       PC12
#define HC05_BAUD         115200

#define FRAME_PERIOD_MS   2        /* resend rate; ESCs disarm if frames stop  */
#define FAN_THROTTLE_MAX  30.0f    /* bring-up ceiling. Raise DELIBERATELY.    */

#define DSHOT_BITS        16
#define BUF_BITS          18       /* 16 data + 2 trailing zeros (line low)    */
#define BUF_WORDS         (BUF_BITS * 4)

#define TIM1_ARR_VAL      599      /* 600 ticks @180 MHz = 3.3333 us           */
#define CCR_ZERO          225      /* 37.5% -> logical 0                       */
#define CCR_ONE           450      /* 75.0% -> logical 1                       */

#define DMA_CHSEL_TIM1_UP 6        /* DMA2 Stream5 Channel6 = TIM1_UP          */

/* Stream 5 flag bits live in DMA2 HISR/HIFCR (which cover streams 4..7):
   FEIF5=6, DMEIF5=8, TEIF5=9, HTIF5=10, TCIF5=11.                             */
#define DMA_S5_FLAGS      ((1UL<<6)|(1UL<<8)|(1UL<<9)|(1UL<<10)|(1UL<<11))

HardwareSerial hc05Serial(PC11, PC10);

/* DMA source buffer. F446 has no CCM RAM, so any static array is reachable by
   DMA2 -- on parts that DO have CCM this would have to be placed explicitly. */
static uint32_t dmaBuf[BUF_WORDS];
static uint16_t dshotValue[5] = { 0, 0, 0, 0, 0 };   /* 0 = disarmed          */
static int      selected      = 2;
static uint32_t framesSent    = 0;
static uint32_t frameOverruns = 0;   /* previous DMA had not finished          */

static void say(const String& s) { Serial.println(s); hc05Serial.println(s); }

/* ---- build a DSHOT frame (value + telemetry bit + CRC) ---------------------
   Ported VERBATIM from fan_test.cpp -- confirmed good, do not "improve".      */
static uint16_t dshotFrame(uint16_t value, bool telem) {
  uint16_t packet = (uint16_t)((value << 1) | (telem ? 1 : 0));
  uint16_t csum = 0, tmp = packet;
  for (int i = 0; i < 3; i++) { csum ^= tmp; tmp >>= 4; }
  return (uint16_t)((packet << 4) | (csum & 0x0F));
}

/* ---- interleave four frames into the burst buffer -------------------------
   Layout must match the burst order CCR1,CCR2,CCR3,CCR4 exactly.             */
static void buildBuffer(void) {
  uint16_t f[5];
  for (int ch = 1; ch <= 4; ch++) f[ch] = dshotFrame(dshotValue[ch], false);

  for (int b = 0; b < DSHOT_BITS; b++) {
    uint16_t mask = (uint16_t)(1U << (15 - b));        /* MSB first           */
    for (int ch = 1; ch <= 4; ch++)
      dmaBuf[b * 4 + (ch - 1)] = (f[ch] & mask) ? CCR_ONE : CCR_ZERO;
  }
  for (int i = DSHOT_BITS * 4; i < BUF_WORDS; i++) dmaBuf[i] = 0;  /* line low */
}

/* ---- one-time hardware setup ---------------------------------------------- */
static void dshotHwInit(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA2EN;
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  (void)RCC->APB2ENR;                       /* ensure the clock is up          */

  /* --- GPIO PA8..PA11 -> AF1 (TIM1), push-pull, very high speed ------------ */
  for (int p = 8; p <= 11; p++) {
    GPIOA->MODER   = (GPIOA->MODER   & ~(3UL << (p * 2)))  | (2UL << (p * 2));
    GPIOA->OTYPER &= ~(1UL << p);                          /* push-pull        */
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3UL << (p * 2)))  | (3UL << (p * 2));
    GPIOA->PUPDR  &= ~(3UL << (p * 2));                    /* no pull          */
    GPIOA->AFR[1]  = (GPIOA->AFR[1]  & ~(0xFUL << ((p - 8) * 4)))
                                     |  (1UL   << ((p - 8) * 4));  /* AF1      */
  }

  /* --- TIM1 timebase ------------------------------------------------------- */
  TIM1->CR1  = 0;
  TIM1->PSC  = 0;
  TIM1->ARR  = TIM1_ARR_VAL;
  TIM1->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;  /* URS: only overflow -> request   */

  /* --- four channels, PWM mode 1, preload on ------------------------------- */
  TIM1->CCMR1 = (6UL << 4)  | TIM_CCMR1_OC1PE      /* OC1M = 110 = PWM mode 1  */
              | (6UL << 12) | TIM_CCMR1_OC2PE;
  TIM1->CCMR2 = (6UL << 4)  | TIM_CCMR2_OC3PE
              | (6UL << 12) | TIM_CCMR2_OC4PE;

  /* CCxE only. CCxNE deliberately left clear -- the complementary outputs map
     onto the motor phases and the encoder bus (see header hazard 2).          */
  TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

  TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0; TIM1->CCR4 = 0;

  /* Advanced timer: outputs are dead until MOE. BKE stays clear (BKIN = PA6). */
  TIM1->BDTR = TIM_BDTR_MOE;

  /* Latch PSC/ARR. URS is already set, so this UG cannot fire a DMA request.  */
  TIM1->EGR = TIM_EGR_UG;
  TIM1->SR  = 0;

  /* --- DMA burst window: 4 transfers starting at CCR1 ---------------------- */
  TIM1->DCR = (3UL << 8) | 13UL;            /* DBL = 4 transfers, DBA = CCR1   */

  /* --- DMA2 Stream 5, Channel 6, memory -> peripheral ---------------------- */
  DMA2_Stream5->CR &= ~DMA_SxCR_EN;
  while (DMA2_Stream5->CR & DMA_SxCR_EN) { }
  DMA2->HIFCR = DMA_S5_FLAGS;

  DMA2_Stream5->PAR  = (uint32_t)&TIM1->DMAR;
  DMA2_Stream5->M0AR = (uint32_t)dmaBuf;
  DMA2_Stream5->NDTR = BUF_WORDS;
  DMA2_Stream5->FCR  = 0;                   /* direct mode, FIFO disabled      */
  DMA2_Stream5->CR   = (DMA_CHSEL_TIM1_UP << 25)   /* CHSEL                    */
                     | (2UL << 16)                 /* PL   = high              */
                     | (2UL << 13)                 /* MSIZE = 32-bit           */
                     | (2UL << 11)                 /* PSIZE = 32-bit           */
                     | DMA_SxCR_MINC                /* walk the buffer         */
                     | (1UL << 6);                 /* DIR = memory-to-periph   */
}

/* ---- clock out one frame on all four channels -----------------------------
   Deterministic sequence: stop the timer, re-arm DMA, restart from CNT = 0.
   Stopping first removes any possibility of a stale update request arriving
   between arming and starting, which would desync the frame by one word.
   With CEN = 0 and CCRx = 0 the outputs sit low, which is what we want in the
   ~1.94 ms gap between frames anyway.                                        */
static void dshotSendFrame(void) {
  TIM1->CR1 &= ~TIM_CR1_CEN;

  if (DMA2_Stream5->CR & DMA_SxCR_EN) {
    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
    uint32_t guard = 0;
    while ((DMA2_Stream5->CR & DMA_SxCR_EN) && ++guard < 10000) { }
    frameOverruns++;                       /* previous frame had not drained   */
  }

  buildBuffer();

  DMA2->HIFCR        = DMA_S5_FLAGS;
  DMA2_Stream5->M0AR = (uint32_t)dmaBuf;
  DMA2_Stream5->NDTR = BUF_WORDS;

  TIM1->SR   = ~TIM_SR_UIF;                /* rc_w0: clear any pending update  */
  TIM1->DIER = TIM_DIER_UDE;               /* update event -> DMA request      */

  DMA2_Stream5->CR |= DMA_SxCR_EN;

  TIM1->CNT  = 0;
  TIM1->CR1 |= TIM_CR1_CEN;

  framesSent++;
}

/* ---- register dump — the instrument, since a meter cannot verify this ------
   Trap T6: a DC voltmeter confirms DUTY, not PULSE WIDTH. 5% at 50 Hz and 5%
   at 5 kHz read identically and only one is a valid DSHOT bit.               */
static uint32_t tim1ClockHz(void) {
  uint32_t ppre2 = (RCC->CFGR >> 13) & 0x7UL;
  uint32_t pdiv  = (ppre2 < 4) ? 1UL : (1UL << (ppre2 - 3));
  uint32_t pclk2 = SystemCoreClock / pdiv;
  return (pdiv == 1) ? pclk2 : pclk2 * 2UL;      /* APB timers run at 2x       */
}

static void dumpRegs(void) {
  uint32_t tclk = tim1ClockHz();
  uint32_t arr  = TIM1->ARR;
  float bit_us  = (float)(arr + 1) * 1e6f / (float)tclk;

  say("");
  say("--- CLOCKS ---------------------------------------------------");
  say("  SystemCoreClock = " + String(SystemCoreClock) + " Hz");
  say("  TIM1 clock      = " + String(tclk) + " Hz  (expect 180000000)");
  say("--- TIM1 -----------------------------------------------------");
  say("  PSC=" + String(TIM1->PSC) + "  ARR=" + String(arr)
      + "  -> bit " + String(bit_us, 4) + " us = "
      + String(1e3f / bit_us, 1) + " kHz   (expect 3.3333 us / 300.0 kHz)");
  say("  CR1  =0x" + String(TIM1->CR1,  HEX)
      + "  CEN=" + String((TIM1->CR1 & TIM_CR1_CEN)  ? 1 : 0)
      + " URS=" + String((TIM1->CR1 & TIM_CR1_URS)  ? 1 : 0)
      + " ARPE=" + String((TIM1->CR1 & TIM_CR1_ARPE) ? 1 : 0));
  say("  CCMR1=0x" + String(TIM1->CCMR1, HEX) + "  CCMR2=0x" + String(TIM1->CCMR2, HEX));
  say("  CCER =0x" + String(TIM1->CCER,  HEX) + "   (expect 0x1111 = CC1E..CC4E, no N)");
  say("  BDTR =0x" + String(TIM1->BDTR,  HEX)
      + "   MOE=" + String((TIM1->BDTR & TIM_BDTR_MOE) ? 1 : 0) + "  <-- must be 1");
  say("  DCR  =0x" + String(TIM1->DCR,   HEX)
      + "   DBA=" + String(TIM1->DCR & 0x1F)
      + " (expect 13=CCR1)  DBL=" + String(((TIM1->DCR >> 8) & 0x1F) + 1)
      + " transfers (expect 4)");
  say("  DIER =0x" + String(TIM1->DIER,  HEX)
      + "   UDE=" + String((TIM1->DIER & TIM_DIER_UDE) ? 1 : 0));
  say("  CCR  = " + String(TIM1->CCR1) + " / " + String(TIM1->CCR2)
      + " / " + String(TIM1->CCR3) + " / " + String(TIM1->CCR4));
  say("  expected CCR: '0'=" + String(CCR_ZERO) + " ("
      + String(CCR_ZERO * bit_us / (arr + 1), 3) + " us)  '1'="
      + String(CCR_ONE) + " (" + String(CCR_ONE * bit_us / (arr + 1), 3) + " us)");
  say("--- GPIOA (PA8..PA11 must be MODER=10 AF, AFRH nibble=1) -----");
  say("  MODER =0x" + String(GPIOA->MODER,  HEX)
      + "   bits[23:16]=0x" + String((GPIOA->MODER >> 16) & 0xFF, HEX)
      + " (expect 0xAA)");
  say("  AFRH  =0x" + String(GPIOA->AFR[1], HEX)
      + "   nibbles[3:0]=0x" + String(GPIOA->AFR[1] & 0xFFFF, HEX)
      + " (expect 0x1111)");
  say("--- DMA2 Stream5 (ch6 = TIM1_UP) -----------------------------");
  say("  CR   =0x" + String(DMA2_Stream5->CR, HEX)
      + "   CHSEL=" + String((DMA2_Stream5->CR >> 25) & 7) + " (expect 6)"
      + "  DIR=" + String((DMA2_Stream5->CR >> 6) & 3) + " (expect 1=mem2periph)");
  say("       MINC=" + String((DMA2_Stream5->CR >> 10) & 1)
      + "  PSIZE=" + String((DMA2_Stream5->CR >> 11) & 3)
      + "  MSIZE=" + String((DMA2_Stream5->CR >> 13) & 3) + " (2 = 32-bit)"
      + "  EN=" + String(DMA2_Stream5->CR & 1));
  say("  NDTR =" + String(DMA2_Stream5->NDTR) + " (reloaded to " + String(BUF_WORDS) + ")");
  say("  PAR  =0x" + String(DMA2_Stream5->PAR,  HEX)
      + "   (&TIM1->DMAR = 0x" + String((uint32_t)&TIM1->DMAR, HEX) + ")");
  say("  M0AR =0x" + String(DMA2_Stream5->M0AR, HEX)
      + "   (dmaBuf = 0x" + String((uint32_t)dmaBuf, HEX) + ")");
  say("--- buffer head (bit15, bit14 for ch1..ch4) ------------------");
  say("  " + String(dmaBuf[0]) + " " + String(dmaBuf[1]) + " " + String(dmaBuf[2])
      + " " + String(dmaBuf[3]) + " | " + String(dmaBuf[4]) + " " + String(dmaBuf[5])
      + " " + String(dmaBuf[6]) + " " + String(dmaBuf[7]));
  say("  tail (must be 0 0 0 0): " + String(dmaBuf[BUF_WORDS - 4]) + " "
      + String(dmaBuf[BUF_WORDS - 3]) + " " + String(dmaBuf[BUF_WORDS - 2]) + " "
      + String(dmaBuf[BUF_WORDS - 1]));
  say("--- counters -------------------------------------------------");
  say("  frames=" + String(framesSent) + "  overruns=" + String(frameOverruns)
      + "   dshot: " + String(dshotValue[1]) + " / " + String(dshotValue[2])
      + " / " + String(dshotValue[3]) + " / " + String(dshotValue[4]));
  say("");
}

/* ---- commands -------------------------------------------------------------- */
static void stopAll(const char* why) {
  for (int ch = 1; ch <= 4; ch++) dshotValue[ch] = 0;
  say("STOP (" + String(why) + ") -- all channels to DSHOT 0 (disarmed)");
}

static void setThrottlePct(float pct) {
  if (pct < 0.0f) pct = 0.0f;
  if (pct > FAN_THROTTLE_MAX) {
    pct = FAN_THROTTLE_MAX;
    say("(capped at " + String(FAN_THROTTLE_MAX, 0) + "%)");
  }
  /* 0% -> 48 (minimum throttle), 100% -> 2047 */
  uint16_t v = (pct <= 0.0f) ? 48 : (uint16_t)(48.0f + pct * (2047.0f - 48.0f) / 100.0f);
  dshotValue[selected] = v;
  say("ch" + String(selected) + "  throttle " + String(pct, 1)
      + "%  ->  DSHOT " + String(v));
}

static void handleLine(String s) {
  s.trim();
  if (!s.length()) return;
  char c = (char)toupper(s.charAt(0));

  if (c == 'C') {
    int n = s.substring(1).toInt();
    if (n >= 1 && n <= 4) { stopAll("channel change"); selected = n;
                            say("selected channel " + String(selected)); }
    else say("channel must be 1-4");
    return;
  }
  /* B : DSHOT command 1 = beep. Chirps WITHOUT spinning the motor -- the
     cleanest proof the ESC is decoding our frames. Safe with props on. */
  if (c == 'B') {
    dshotValue[selected] = 1;
    say("ch" + String(selected) + "  DSHOT cmd 1 (BEEP) -- listen for a chirp");
    for (int i = 0; i < 20; i++) { dshotSendFrame(); delay(2); }
    dshotValue[selected] = 0;
    say("beep command sent, back to 0");
    return;
  }
  if (c == 'D') { dumpRegs(); return; }
  if (c >= '0' && c <= '9') { setThrottlePct(s.toFloat()); return; }
  stopAll("operator");
}

static void pump(Stream& s, String& buf) {
  while (s.available()) {
    char ch = (char)s.read();
    if (ch == '\n' || ch == '\r') { if (buf.length()) { handleLine(buf); buf = ""; } }
    else if (buf.length() < 32) buf += ch;
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  pinMode(HC05_EN_PIN, OUTPUT);
  digitalWrite(HC05_EN_PIN, LOW);
  hc05Serial.begin(HC05_BAUD);

  buildBuffer();
  dshotHwInit();

  say("");
  say("=== fan ESC test — DSHOT300 via TIM1 + DMA burst ===");
  say("PROPS OFF.  ch4 must be on CN10-14 (PA11), NOT A0.");
  say("sending DSHOT 0 (disarmed) to arm the ESC...");
  for (int i = 0; i < 500; i++) { dshotSendFrame(); delay(2); }   /* ~1 s of zeros */
  say("armed (expect a tone from the ESC)");
  say("");
  say("commands (USB or HC-05, newline-terminated):");
  say("  <number>  throttle percent 0-" + String(FAN_THROTTLE_MAX, 0));
  say("  B         BEEP via DSHOT cmd 1 — proves decoding WITHOUT spinning");
  say("  C<n>      select channel 1-4   (1=D7 2=D8 3=D2 4=CN10-14/PA11)");
  say("  D         dump TIM1 + DMA2 registers and verify the arithmetic");
  say("  X / any   STOP");
  say("");
  say("start with D (checks config), then B (checks decoding), then 10.");
}

void loop() {
  static String bufU, bufB;
  pump(Serial,     bufU);
  pump(hc05Serial, bufB);

  static uint32_t lastFrame = 0;
  if (millis() - lastFrame >= FRAME_PERIOD_MS) { lastFrame = millis(); dshotSendFrame(); }

  static uint32_t lastBeat = 0;
  if (millis() - lastBeat >= 2000) {
    lastBeat = millis();
    say("[alive " + String(millis() / 1000) + "s]  sel=ch" + String(selected)
        + "  frames=" + String(framesSent) + "  overruns=" + String(frameOverruns)
        + "  dshot: " + String(dshotValue[1]) + " / " + String(dshotValue[2])
        + " / " + String(dshotValue[3]) + " / " + String(dshotValue[4]));
  }
}
