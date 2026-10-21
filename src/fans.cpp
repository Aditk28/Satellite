#include "fans.h"
#include "telemetry.h"
#include "timebase.h"
#include <STM32FreeRTOS.h>
#include <task.h>

/* DSHOT300 via TIM1 + DMA burst.

   DSHOT encodes each bit in the PWM duty, so no software timing is needed. TIM1 runs
   at 180 MHz with ARR=599, a 3.333 us bit. TIM1->DMAR writes are redirected to
   CR1 + DBA + idx; with DBA=13 (CCR1) and DBL=4 each update event moves four words
   into CCR1..CCR4, so one DMA request per bit serves all four channels from one
   interleaved buffer. TIM1_UP is DMA2 Stream 5 Channel 6.

   No DMA interrupt is used (avoids clashing with STM32duino's IRQ handlers); the next
   frame is armed by polling DMA_SxCR.EN. */

#define DSHOT_BITS        16
#define BUF_BITS          18       /* 16 data + 2 trailing zeros (line low) */
#define BUF_WORDS         (BUF_BITS * 4)

#define TIM1_ARR_VAL      599      /* 600 ticks @180 MHz = 3.333 us */
#define CCR_ZERO          225      /* 37.5% -> logical 0 */
#define CCR_ONE           450      /* 75.0% -> logical 1 */

#define DMA_CHSEL_TIM1_UP 6        /* DMA2 Stream5 Channel6 = TIM1_UP */

/* Stream 5 flags in DMA2 HISR/HIFCR: FEIF5=6, DMEIF5=8, TEIF5=9, HTIF5=10, TCIF5=11. */
#define DMA_S5_FLAGS      ((1UL<<6)|(1UL<<8)|(1UL<<9)|(1UL<<10)|(1UL<<11))

/* F446 has no CCM RAM, so any static buffer is reachable by DMA2. */
static uint32_t dmaBuf[BUF_WORDS];

/* dshotValue/dshotTelem belong to fanTask. Other tasks write s_req/s_pct, which
   fanTask snapshots under a critical section. */
static uint16_t dshotValue[5] = { 0, 0, 0, 0, 0 };   /* 0 = disarmed */
static bool     dshotTelem[5] = { false, false, false, false, false };  /* set for special commands */
static volatile uint16_t s_req[5] = { 0, 0, 0, 0, 0 };
static volatile float    s_pct[5] = { 0, 0, 0, 0, 0 };

static TaskHandle_t      s_task      = nullptr;
static volatile bool     s_armed     = false;   /* ramp and boot config both done */
static          bool     s_rampDone  = false;
static          int      s_cfgCh     = 0;       /* channel being configured, 0 = none */
static volatile bool     s_killed    = false;
/* Set once the TIM1/DMA2/GPIOA clocks are on. fans_stopAll() can be called from an
   early fault, and touching TIM1 with its clock gated can bus-fault. */
static volatile bool     s_hwReady   = false;
static volatile uint32_t s_frames    = 0;
static volatile uint32_t s_overruns  = 0;
static volatile uint32_t s_rejects   = 0;
static volatile uint32_t s_budgetHits = 0;
static uint32_t          s_stuck     = 0;   /* consecutive overruns */
static volatile uint32_t s_skips       = 0;
static          uint32_t s_lastFrameUs = 0; /* us_now() of the last emission */
static volatile uint32_t s_stuckNDTR    = 0;
static volatile uint32_t s_stuckNDTRmin = 0xFFFFFFFF;
static volatile uint32_t s_stuckNDTRmax = 0;
static volatile uint32_t s_stuckCEN     = 0;
static volatile bool     s_announce  = false; /* arming message pending */
static volatile uint32_t s_lastCmdMs = 0;     /* tick of the last throttle request */
/* Pending special-command burst, sent by fanTask. */
static volatile int      s_escCmdCh   = 0;
static volatile uint16_t s_escCmdVal  = 0;
static volatile uint16_t s_escCmdLeft = 0;
static volatile bool     s_escCmdDone = false;
static volatile bool     s_timedOut  = false; /* timeout message pending */

/* 11-bit value, telemetry bit, 4-bit XOR checksum of the three nibbles. */
static uint16_t dshotFrame(uint16_t value, bool telem) {
  uint16_t packet = (uint16_t)((value << 1) | (telem ? 1 : 0));
  uint16_t csum = 0, tmp = packet;
  for (int i = 0; i < 3; i++) { csum ^= tmp; tmp >>= 4; }
  return (uint16_t)((packet << 4) | (csum & 0x0F));
}

/* Interleave four frames in burst order CCR1..CCR4. CCR preload delays each value by
   one period, so two trailing zero entries are needed to leave the line low;
   without them the pin keeps pulsing at the last bit's duty. */
static void buildBuffer(void) {
  uint16_t f[5];
  for (int ch = 1; ch <= 4; ch++) f[ch] = dshotFrame(dshotValue[ch], dshotTelem[ch]);

  for (int b = 0; b < DSHOT_BITS; b++) {
    uint16_t mask = (uint16_t)(1U << (15 - b));        /* MSB first */
    for (int ch = 1; ch <= 4; ch++)
      dmaBuf[b * 4 + (ch - 1)] = (f[ch] & mask) ? CCR_ONE : CCR_ZERO;
  }
  for (int i = DSHOT_BITS * 4; i < BUF_WORDS; i++) dmaBuf[i] = 0;
}

/* TIM1 notes for this board:
   - Outputs stay dead until BDTR.MOE is set.
   - The complementary outputs CH1N..CH3N map to motor phase C and the encoder SPI/CS
     pins, so CCxNE stays clear. BKE stays clear because BKIN is PA6 (motor phase B).
   - CR1.URS is set so the UG write that latches PSC/ARR does not raise a DMA request
     and shift the frame by one word. */
static void dshotHwInit(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA2EN;
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  (void)RCC->APB2ENR;                       /* read back so the clock is up */

  /* PA8..PA11 -> AF1 (TIM1), push-pull, very high speed, no pull */
  for (int p = 8; p <= 11; p++) {
    GPIOA->MODER   = (GPIOA->MODER   & ~(3UL << (p * 2)))  | (2UL << (p * 2));
    GPIOA->OTYPER &= ~(1UL << p);
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3UL << (p * 2)))  | (3UL << (p * 2));
    GPIOA->PUPDR  &= ~(3UL << (p * 2));
    GPIOA->AFR[1]  = (GPIOA->AFR[1]  & ~(0xFUL << ((p - 8) * 4)))
                                     |  (1UL   << ((p - 8) * 4));
  }

  TIM1->CR1  = 0;
  TIM1->PSC  = 0;
  TIM1->ARR  = TIM1_ARR_VAL;
  TIM1->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

  /* PWM mode 1 (OCxM = 110), preload on, all four channels */
  TIM1->CCMR1 = (6UL << 4)  | TIM_CCMR1_OC1PE
              | (6UL << 12) | TIM_CCMR1_OC2PE;
  TIM1->CCMR2 = (6UL << 4)  | TIM_CCMR2_OC3PE
              | (6UL << 12) | TIM_CCMR2_OC4PE;

  TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

  TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0; TIM1->CCR4 = 0;

  TIM1->BDTR = TIM_BDTR_MOE;

  TIM1->EGR = TIM_EGR_UG;                   /* latch PSC/ARR */
  TIM1->SR  = 0;

  TIM1->DCR = (3UL << 8) | 13UL;            /* DBL = 4 transfers, DBA = CCR1 */

  /* DMA2 Stream 5, Channel 6, memory -> peripheral */
  DMA2_Stream5->CR &= ~DMA_SxCR_EN;
  while (DMA2_Stream5->CR & DMA_SxCR_EN) { }
  DMA2->HIFCR = DMA_S5_FLAGS;

  DMA2_Stream5->PAR  = (uint32_t)&TIM1->DMAR;
  DMA2_Stream5->M0AR = (uint32_t)dmaBuf;
  DMA2_Stream5->NDTR = BUF_WORDS;
  DMA2_Stream5->FCR  = 0;                   /* direct mode */
  DMA2_Stream5->CR   = (DMA_CHSEL_TIM1_UP << 25)   /* CHSEL */
                     | (2UL << 16)                 /* PL = high */
                     | (2UL << 13)                 /* MSIZE = 32-bit */
                     | (2UL << 11)                 /* PSIZE = 32-bit */
                     | DMA_SxCR_MINC
                     | (1UL << 6);                 /* DIR = memory-to-peripheral */

  s_hwReady = true;
}

/* Clock out one frame on all four channels: stop the timer, re-arm DMA, restart from
   CNT = 0. Stopping first guarantees no stale update request lands between arming and
   starting. With CEN = 0 and CCRx = 0 the outputs idle low between frames. */
static void dshotSendFrame(void) {
  /* Too soon after the last frame (scheduling jitter, see FAN_FRAME_MS). Benign. */
  uint32_t now = us_now();
  if (s_lastFrameUs && (uint32_t)(now - s_lastFrameUs) < FRAME_MIN_GAP_US) {
    s_skips++;
    return;
  }

  /* The transfer has had over 100 us for a 60 us frame and is still running: a real
     fault (timer stopped or misconfigured). Skip rather than abort it, which would
     put a truncated frame on the wire, but force a restart after 4 in a row. */
  if (DMA2_Stream5->CR & DMA_SxCR_EN) {
    s_stuckNDTR = DMA2_Stream5->NDTR;
    s_stuckCEN  = (TIM1->CR1 & TIM_CR1_CEN) ? 1 : 0;
    if (s_stuckNDTR > s_stuckNDTRmax) s_stuckNDTRmax = s_stuckNDTR;
    if (s_stuckNDTR < s_stuckNDTRmin) s_stuckNDTRmin = s_stuckNDTR;
    s_overruns++;
    if (++s_stuck < 4) return;
    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
    uint32_t guard = 0;
    while ((DMA2_Stream5->CR & DMA_SxCR_EN) && ++guard < 10000) { }
  }
  s_stuck = 0;

  TIM1->CR1 &= ~TIM_CR1_CEN;

  buildBuffer();

  DMA2->HIFCR        = DMA_S5_FLAGS;
  DMA2_Stream5->M0AR = (uint32_t)dmaBuf;
  DMA2_Stream5->NDTR = BUF_WORDS;

  TIM1->SR   = ~TIM_SR_UIF;                /* rc_w0: clear any pending update */
  TIM1->DIER = TIM_DIER_UDE;               /* update event -> DMA request */

  DMA2_Stream5->CR |= DMA_SxCR_EN;

  TIM1->CNT  = 0;
  TIM1->CR1 |= TIM_CR1_CEN;

  s_lastFrameUs = now;
  s_frames++;
}

static void fanTask(void*) {
  TickType_t last     = xTaskGetTickCount();
  uint32_t   armCount = 0;

  for (;;) {
    /* Never catch up after starvation: vTaskDelayUntil would otherwise return
       immediately for each missed period and fire frames back to back. */
    TickType_t nowT = xTaskGetTickCount();
    if ((int32_t)(nowT - last) >= (int32_t)pdMS_TO_TICKS(FAN_FRAME_MS))
      last = nowT;

    vTaskDelayUntil(&last, pdMS_TO_TICKS(FAN_FRAME_MS));

    /* Hard-killed: send nothing. The ESCs disarm when frames stop. */
    if (s_killed) continue;

    /* Arming ramp: FAN_ARM_FRAMES of DSHOT 0. */
    if (!s_rampDone) {
      for (int ch = 1; ch <= 4; ch++) dshotValue[ch] = 0;
      dshotSendFrame();
      if (++armCount >= FAN_ARM_FRAMES) s_rampDone = true;
      continue;
    }

    /* Boot config: queue the direction-reverse command for FAN_REVERSE_MASK channels
       while the motors are still stopped, then accept throttle. */
    if (!s_armed) {
      int next = 0;
      for (int ch = s_cfgCh + 1; ch <= 4; ch++)
        if (FAN_REVERSE_MASK & (1 << (ch - 1))) { next = ch; break; }
      if (next) {
        s_cfgCh      = next;
        s_escCmdCh   = next;
        s_escCmdVal  = DSHOT_CMD_DIR_REVERSED;
        s_escCmdLeft = FAN_CFG_REPS;
      } else {
        s_armed    = true;
        s_announce = true;
      }
      continue;
    }

    /* Print only once telemTask owns the serial ports. Before that, telem_print()
       writes the port directly from the calling task, which would race other writers. */
    if (s_announce && telem_isActive()) {
      s_announce = false;
      String rev = "";
      for (int ch = 1; ch <= 4; ch++)
        if (FAN_REVERSE_MASK & (1 << (ch - 1))) rev += String(ch) + " ";
      telem_print("fans: armed (" + String(FAN_ARM_FRAMES) + " zero frames, cap "
                  + String(FAN_THROTTLE_MAX, 0) + "%)"
                  + (rev.length() ? ("  direction REVERSED applied to fan " + rev
                                     + "(re-done every boot: ESC ignores SAVE)") : ""));
    }

    /* Special-command burst takes priority over throttle, with all other channels
       held at 0 since ESCs only honour commands while stopped. */
    if (s_escCmdLeft > 0) {
      for (int ch = 1; ch <= 4; ch++) { dshotValue[ch] = 0; dshotTelem[ch] = false; }
      dshotValue[s_escCmdCh] = s_escCmdVal;
      dshotTelem[s_escCmdCh] = true;
      dshotSendFrame();
      if (--s_escCmdLeft == 0) {
        dshotValue[s_escCmdCh] = 0;
        dshotTelem[s_escCmdCh] = false;
        s_escCmdDone = true;
      }
      continue;
    }
    if (s_escCmdDone && telem_isActive()) {
      s_escCmdDone = false;
      telem_print("fans: ESC command sent (motors held at 0 during the burst)");
    }

    /* Dead-man timeout, only while something is spinning. */
    bool spinning = false;
    for (int ch = 1; ch <= 4; ch++) if (s_req[ch] > 48) spinning = true;
    if (spinning &&
        (uint32_t)(xTaskGetTickCount() - s_lastCmdMs) > pdMS_TO_TICKS(FAN_CMD_TIMEOUT_MS)) {
      taskENTER_CRITICAL();
      for (int ch = 1; ch <= 4; ch++) { s_req[ch] = 0; s_pct[ch] = 0.0f; }
      taskEXIT_CRITICAL();
      s_timedOut = true;
    }
    if (s_timedOut && telem_isActive()) {
      s_timedOut = false;
      telem_print("fans: TIMEOUT, no throttle command for "
                  + String(FAN_CMD_TIMEOUT_MS / 1000) + " s, all channels zeroed");
    }

    /* Snapshot the request set. The critical section also masks the TIM9 FOC tick
       (it sits at the syscall ceiling), but only for ~170 ns. */
    taskENTER_CRITICAL();
    for (int ch = 1; ch <= 4; ch++) dshotValue[ch] = s_req[ch];
    taskEXIT_CRITICAL();

    dshotSendFrame();
  }
}

void fans_init(void) {
  dshotHwInit();
  buildBuffer();
  configASSERT(xTaskCreate(fanTask, "fan", 384, nullptr, 2, &s_task) == pdPASS);
}

/* 0% -> 48 (minimum throttle), 100% -> 2047. */
static uint16_t pctToDshot(float pct) {
  return (pct <= 0.0f) ? 48 : (uint16_t)(48.0f + pct * (2047.0f - 48.0f) / 100.0f);
}

/* Per-motor current estimate, cube law anchored at FAN_AMPS_AT_50PCT. */
static float estMotorAmps(float pct) {
  float r = pct / 50.0f;
  return FAN_AMPS_AT_50PCT * r * r * r;
}

/* Scale the whole set down if its estimated current exceeds the budget. Uniform
   scaling keeps the direction of the requested thrust vector, which per-channel
   clamping would not. Returns the factor applied (1.0 = untouched). */
static float applyCurrentBudget(float pc[5]) {
  float total = 0.0f;
  for (int ch = 1; ch <= 4; ch++) total += estMotorAmps(pc[ch]);
  if (total <= FAN_CURRENT_BUDGET_A || total <= 0.0f) return 1.0f;
  /* current ~ pct^3, so scale throttle by the cube root of the current ratio */
  float k = powf(FAN_CURRENT_BUDGET_A / total, 1.0f / 3.0f);
  for (int ch = 1; ch <= 4; ch++) pc[ch] *= k;
  s_budgetHits++;
  return k;
}

void fans_setThrottle(int ch, float pct) {
  if (ch < 1 || ch > 4) return;
  if (!s_armed || s_killed) { s_rejects++; return; }

  if (pct < 0.0f) pct = 0.0f;
  if (pct > FAN_THROTTLE_MAX) pct = FAN_THROTTLE_MAX;

  /* Budget the resulting set, not just this channel. */
  float pc[5];
  for (int c = 1; c <= 4; c++) pc[c] = s_pct[c];
  pc[ch] = pct;
  applyCurrentBudget(pc);

  taskENTER_CRITICAL();
  for (int c = 1; c <= 4; c++) { s_pct[c] = pc[c]; s_req[c] = pctToDshot(pc[c]); }
  taskEXIT_CRITICAL();
  s_lastCmdMs = xTaskGetTickCount();
}

void fans_setAll(float f1, float f2, float f3, float f4) {
  if (!s_armed || s_killed) { s_rejects++; return; }
  const float in[5] = { 0.0f, f1, f2, f3, f4 };
  uint16_t    out[5];
  float       pc[5];
  for (int ch = 1; ch <= 4; ch++) {
    float p = in[ch];
    if (p < 0.0f) p = 0.0f;
    if (p > FAN_THROTTLE_MAX) p = FAN_THROTTLE_MAX;
    pc[ch]  = p;
  }
  applyCurrentBudget(pc);
  for (int ch = 1; ch <= 4; ch++) out[ch] = pctToDshot(pc[ch]);
  taskENTER_CRITICAL();
  for (int ch = 1; ch <= 4; ch++) { s_pct[ch] = pc[ch]; s_req[ch] = out[ch]; }
  taskEXIT_CRITICAL();
  s_lastCmdMs = xTaskGetTickCount();
}

bool fans_sendCommand(int ch, uint16_t cmd, uint16_t reps) {
  if (ch < 1 || ch > 4)        return false;
  if (cmd < 1 || cmd > 47)     return false;   /* 0 = stop, 48+ = throttle */
  if (!s_armed || s_killed)    return false;
  if (s_escCmdLeft > 0)        return false;   /* one burst at a time */
  for (int c = 1; c <= 4; c++) if (s_req[c] > 48) return false;

  s_escCmdCh   = ch;
  s_escCmdVal  = cmd;
  s_escCmdLeft = reps ? reps : 10;
  return true;
}

void fans_stopAll(void) {
  /* Must work from an ISR or with interrupts disabled: no FreeRTOS calls. */
  s_killed = true;
  for (int ch = 1; ch <= 4; ch++) { s_req[ch] = 0; dshotValue[ch] = 0; s_pct[ch] = 0.0f; }

  if (!s_hwReady) return;

  DMA2_Stream5->CR &= ~DMA_SxCR_EN;
  TIM1->CR1  &= ~TIM_CR1_CEN;
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0; TIM1->CCR4 = 0;

  /* Drive the pins low as GPIO. Write BSRR before MODER so ODR is already low when
     the pad switches to output; the other order can briefly drive a stale ODR. */
  GPIOA->BSRR = (1UL << (8 + 16)) | (1UL << (9 + 16))
              | (1UL << (10 + 16)) | (1UL << (11 + 16));
  for (int p = 8; p <= 11; p++)
    GPIOA->MODER = (GPIOA->MODER & ~(3UL << (p * 2))) | (1UL << (p * 2));  /* output */
}

void fans_rearm(void) {
  if (!s_killed) return;
  dshotHwInit();          /* restores AF mode on PA8..PA11 and MOE */
  buildBuffer();
  s_armed    = false;     /* fanTask repeats the arming ramp and boot config */
  s_rampDone = false;
  s_cfgCh    = 0;
  s_killed   = false;
}

bool     fans_armedAndLive(void) { return s_armed && !s_killed; }
bool     fans_armed(void)     { return s_armed; }
bool     fans_killed(void)    { return s_killed; }
uint32_t fans_frames(void)    { return s_frames; }
uint32_t fans_overruns(void)  { return s_overruns; }
uint32_t fans_skips(void)     { return s_skips; }
uint32_t fans_budgetHits(void){ return s_budgetHits; }
uint32_t fans_rejects(void)   { return s_rejects; }
float    fans_pct(int ch)     { return (ch >= 1 && ch <= 4) ? s_pct[ch] : 0.0f; }

void fans_overrunDetail(uint32_t* lastNDTR, uint32_t* minNDTR, uint32_t* maxNDTR,
                        uint32_t* cenWasSet) {
  if (lastNDTR)  *lastNDTR  = s_stuckNDTR;
  if (minNDTR)   *minNDTR   = (s_stuckNDTRmin == 0xFFFFFFFF) ? 0 : s_stuckNDTRmin;
  if (maxNDTR)   *maxNDTR   = s_stuckNDTRmax;
  if (cenWasSet) *cenWasSet = s_stuckCEN;
}

uint32_t fans_stackFreeWords(void) {
  return s_task ? (uint32_t)uxTaskGetStackHighWaterMark(s_task) : 0;
}
