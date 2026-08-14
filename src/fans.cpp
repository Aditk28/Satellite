#include "fans.h"
#include "telemetry.h"
#include "timebase.h"      /* us_now() -- real elapsed time, not tick granularity */
#include <STM32FreeRTOS.h>
#include <task.h>

/* ============================================================================
   DSHOT300 via TIM1 + DMA burst.

   Ported from src/fan_dma_test.cpp (env fandma), which proved this on hardware in
   Step 1.1: 22,278 frames, 0 overruns, all four channels spinning, every register
   dumped and checked against the arithmetic. The four driver functions below are
   CHARACTER-FOR-CHARACTER the proven versions so the two files stay diffable.

   The mechanism, briefly (full derivation is in fan_dma_test.cpp's header):
     DSHOT is PWM where the DUTY encodes the bit, so no software timing is needed.
     TIM1 counts at 180 MHz; ARR=599 gives a 3.3333 us bit. TIM1->DMAR is a WINDOW
     register whose writes hardware-redirects to CR1 + DBA + idx, idx incrementing
     across the burst. DBA=13 points at CCR1 and DBL=4 makes each update event move
     four words into CCR1..CCR4 -- so ONE DMA request per bit period serves all four
     channels, from one interleaved buffer, perfectly synchronised by construction.
     TIM1_UP is DMA2 Stream 5 Channel 6.

   NO DMA INTERRUPT IS USED. The frame is re-armed by polling DMA_SxCR.EN, which
   sidesteps the STM32duino strong-IRQHandler collision entirely rather than working
   around it (there is no DMA2_Stream5_IRQHandler here to collide with anything).
   ============================================================================ */

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

/* DMA source buffer. F446 has no CCM RAM, so any static array is reachable by
   DMA2 -- on parts that DO have CCM this would have to be placed explicitly. */
static uint32_t dmaBuf[BUF_WORDS];

/* dshotValue is owned by fanTask ALONE. s_req is what everyone else writes.
   Split because four halfword stores are individually atomic but not atomic as a
   SET; fanTask snapshots s_req -> dshotValue under a critical section so a frame
   can never carry a torn allocation. */
static uint16_t dshotValue[5] = { 0, 0, 0, 0, 0 };   /* 0 = disarmed          */
static volatile uint16_t s_req[5] = { 0, 0, 0, 0, 0 };
static volatile float    s_pct[5] = { 0, 0, 0, 0, 0 };

static TaskHandle_t      s_task      = nullptr;
static volatile bool     s_armed     = false;
static volatile bool     s_killed    = false;
/* Set once dshotHwInit() has enabled the TIM1/DMA2/GPIOA clocks. fans_stopAll() is
   wired into faults_safeStop(), which can fire arbitrarily early -- e.g. a
   configASSERT during another module's init. Touching TIM1 with its APB clock still
   gated is at best ignored and at worst a bus fault, so the kill is a no-op until
   the hardware actually exists. */
static volatile bool     s_hwReady   = false;
static volatile uint32_t s_frames    = 0;
static volatile uint32_t s_overruns  = 0;
static volatile uint32_t s_rejects   = 0;
static uint32_t          s_stuck     = 0;   /* consecutive in-flight skips */
static volatile uint32_t s_skips       = 0; /* benign: emitted inside the last frame */
static          uint32_t s_lastFrameUs = 0; /* TIM5 us stamp of the last emission */
/* Step 1.4 overrun diagnostic -- see the comment in dshotSendFrame(). */
static volatile uint32_t s_stuckNDTR    = 0;
static volatile uint32_t s_stuckNDTRmin = 0xFFFFFFFF;
static volatile uint32_t s_stuckNDTRmax = 0;
static volatile uint32_t s_stuckCEN     = 0;
static volatile bool     s_announce  = false; /* arming message owed to telemTask */
static volatile uint32_t s_lastCmdMs = 0;     /* last throttle request, ms (dead-man) */
static volatile bool     s_timedOut  = false; /* timeout message owed to telemTask */

/* ---- build a DSHOT frame (value + telemetry bit + CRC) ---------------------
   VERBATIM from fan_dma_test.cpp, itself verbatim from fan_test.cpp.          */
static uint16_t dshotFrame(uint16_t value, bool telem) {
  uint16_t packet = (uint16_t)((value << 1) | (telem ? 1 : 0));
  uint16_t csum = 0, tmp = packet;
  for (int i = 0; i < 3; i++) { csum ^= tmp; tmp >>= 4; }
  return (uint16_t)((packet << 4) | (csum & 0x0F));
}

/* ---- interleave four frames into the burst buffer -------------------------
   Layout must match the burst order CCR1,CCR2,CCR3,CCR4 exactly.

   The two trailing CCR = 0 entries are NOT cosmetic: without them CCRx retains the
   last data bit's value (225 or 450) when the transfer ends and the pin keeps
   pulsing at 37.5%/75% forever. They are what puts the line low. CCRx preload is
   on, so a value DMA'd on update N takes effect for period N+1 -- one period of
   pipeline, which is why 16 data bits need 18 entries to fully flush.           */
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

/* ---- one-time hardware setup ----------------------------------------------
   HAZARDS SPECIFIC TO TIM1 ON THIS BOARD:
   1. MOE (BDTR bit 15). Advanced-timer outputs are dead until it is set, and the
      symptom is "every register reads perfect, pin never moves" -- the failure
      that killed the servo-PWM attempt.
   2. The COMPLEMENTARY outputs land on pins we cannot afford to touch:
      TIM1_CH1N/2N/3N map to PA7, PB0/PB13/PB14, PB1/PB15 -- motor phase C, the
      encoder SCK/MISO/MOSI, and the encoder CS. They are gated by each pin's GPIO
      alternate-function mux so they cannot appear unless AF1 is also set on those
      pins, which we never do. Margin for error is zero, so CCxNE stays clear in
      CCER and BKE stays clear in BDTR (break input BKIN is PA6 = motor phase B).
   3. URS (CR1 bit 2) is set. Otherwise the software UG write used to latch PSC/ARR
      ALSO generates an update event, firing a spurious DMA request before the
      buffer is armed and desyncing the frame by one word.                      */
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

  /* CCxE only. CCxNE deliberately left clear -- see hazard 2 above.           */
  TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

  TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0; TIM1->CCR4 = 0;

  TIM1->BDTR = TIM_BDTR_MOE;                /* BKE stays clear (BKIN = PA6)    */

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

  s_hwReady = true;      /* fans_stopAll() may now touch TIM1/DMA2/GPIOA */
}

/* ---- clock out one frame on all four channels -----------------------------
   Deterministic sequence: stop the timer, re-arm DMA, restart from CNT = 0.
   Stopping first removes any possibility of a stale update request arriving
   between arming and starting, which would desync the frame by one word.
   With CEN = 0 and CCRx = 0 the outputs sit low, which is what we want in the
   ~1.94 ms gap between frames anyway.

   Kept over the cheaper "leave TIM1 free-running, re-arm DMA only" variant
   deliberately: this is the version with 22k proven frames behind it, and the
   cost is a handful of register writes every 2 ms.                            */
static void dshotSendFrame(void) {
  /* ---- expected case: emitted too soon after the last one -------------------
     vTaskDelayUntil schedules from the WAKE time, but the frame goes out when the
     task gets the CPU, and those differ by up to a tick (see FAN_FRAME_MS). So two
     emissions can land tens of microseconds apart even though the schedule is a
     correct 3 ms. That is normal and benign -- a skipped frame simply means the ESC
     gets the next one a few ms later -- so it is counted as a SKIP, not a fault.

     Gated on the TIM5 microsecond timebase deliberately. The original version used
     tick arithmetic, which cannot see a 50 us reality at 1 ms granularity, and that
     blind spot is exactly what made this look like an unexplained "overrun". */
  uint32_t now = us_now();
  if (s_lastFrameUs && (uint32_t)(now - s_lastFrameUs) < FRAME_MIN_GAP_US) {
    s_skips++;
    return;
  }

  /* Previous frame still in flight? SKIP this one and let it finish.
     A frame needs ~60 us of timer time to drain, so this can only happen if we were
     called back-to-back -- and the original code's response (abort the transfer and
     start a new one) is strictly worse than waiting: it puts a truncated, CRC-failing
     frame on the wire for no benefit. The next 2 ms tick sends a clean one.
     Measured at boot before this fix: 99 aborted frames in the first ~698, all inside
     the hwSetup() starvation window. See the Step 1.3 result note.

     STUCK RECOVERY: if the transfer is still in flight after several attempts the
     timer is not advancing the DMA at all (CEN cleared, clock gated, ARR clobbered),
     and skipping forever would wedge the fans silently. Force a full restart instead. */
  if (DMA2_Stream5->CR & DMA_SxCR_EN) {
    /* Reaching here means >= FRAME_MIN_GAP_US has genuinely elapsed and the transfer
       STILL has not drained -- a 60 us frame that has had 100+ us. That is a real
       anomaly (timer stopped, clock gated, ARR clobbered), unlike the benign skip
       above, and this counter should sit at 0. NDTR + CEN are kept because they are
       what discriminated the benign case from a real one in the first place:
         NDTR small, CEN=1 -> frame merely still clocking (now handled by the skip)
         NDTR large, CEN=0 -> timer genuinely not advancing the DMA */
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

  TIM1->SR   = ~TIM_SR_UIF;                /* rc_w0: clear any pending update  */
  TIM1->DIER = TIM_DIER_UDE;               /* update event -> DMA request      */

  DMA2_Stream5->CR |= DMA_SxCR_EN;

  TIM1->CNT  = 0;
  TIM1->CR1 |= TIM_CR1_CEN;

  s_lastFrameUs = now;
  s_frames++;
}

/* ============================================================================
   The task
   ============================================================================ */

static void fanTask(void*) {
  TickType_t last     = xTaskGetTickCount();
  uint32_t   armCount = 0;

  for (;;) {
    /* NEVER CATCH UP. If this task was starved, vTaskDelayUntil would otherwise
       return immediately once per missed period and fire a burst of back-to-back
       frames -- and a frame needs ~60 us of timer time to drain, so the burst
       aborts its own transfers mid-flight. That is exactly what produced 99 corrupt
       frames inside the hwSetup() starvation window (Step 1.3 result note); the
       first version of this guard only resynced past 8 ms of backlog, which let
       2-7 ms windows through.

       Fans want a STEADY STREAM, not replayed history: a missed frame is simply
       gone, and the ESC only cares that the next one arrives in time. So resync
       whenever the next deadline has already passed. In normal operation this never
       fires -- a frame costs microseconds against a 2 ms period. */
    TickType_t nowT = xTaskGetTickCount();
    if ((int32_t)(nowT - last) >= (int32_t)pdMS_TO_TICKS(FAN_FRAME_MS))
      last = nowT;

    /* Absolute-time wake: execution time does NOT accumulate as period drift,
       which vTaskDelay would cause (Trap 7). */
    vTaskDelayUntil(&last, pdMS_TO_TICKS(FAN_FRAME_MS));

    /* Hard-killed: send NOTHING. Frames stopping is itself the fail-safe -- the
       ESC disarms on its own a few tens of ms later, on top of the pins already
       being held low as GPIO by fans_stopAll(). */
    if (s_killed) continue;

    /* Arming ramp, ON THE TASK. The standalone sketch used delay(), which is
       Trap 8/19: it busy-spins on yield() and silently starves every lower-priority
       task -- here that would be telemTask, so the boot output would never get
       written out. vTaskDelayUntil above paces this instead, at no cost. */
    if (!s_armed) {
      for (int ch = 1; ch <= 4; ch++) dshotValue[ch] = 0;
      dshotSendFrame();
      if (++armCount >= FAN_ARM_FRAMES) { s_armed = true; s_announce = true; }
      continue;
    }

    /* Announce arming ONLY once telemTask owns the ports.
       Before telem_activate(), telem_print() writes the port DIRECTLY from the
       calling task -- and arming completes ~1 s after the scheduler starts, which
       is still inside hwSetup() while the control task is direct-writing its boot
       output. Printing here unguarded made fanTask a SECOND concurrent writer into
       a non-reentrant HardwareSerial: invariant B15 / Trap 17, the half-applied
       single-writer bug that froze the board and produced sensor reads that looked
       like a hardware fault. It interleaved harmlessly the one time it shipped,
       which is precisely why that trap says a partial version is worse than none. */
    if (s_announce && telem_isActive()) {
      s_announce = false;
      telem_print("fans: armed (" + String(FAN_ARM_FRAMES) + " zero frames sent, cap "
                  + String(FAN_THROTTLE_MAX, 0) + "%)");
    }

    /* Dead-man timeout (see FAN_CMD_TIMEOUT_MS in fans.h). Only meaningful while
       something is actually spinning: if all four are already zero there is nothing
       to time out, and we must not spam. */
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
      telem_print("fans: TIMEOUT -- no throttle command for "
                  + String(FAN_CMD_TIMEOUT_MS / 1000) + " s, all channels zeroed");
    }

    /* Snapshot the request set atomically. taskENTER_CRITICAL writes BASEPRI =
       configMAX_SYSCALL_INTERRUPT_PRIORITY (5<<4 = 0x50), which masks every
       exception at priority value >= 0x50 -- including TIM9 at exactly 5, so the
       FOC tick IS masked here. Unavoidable and not a mistake: TIM9's ISR calls
       vTaskNotifyGiveFromISR, so it must sit at or below the syscall ceiling or it
       would corrupt the kernel (Trap 3). Cost is four halfword copies plus loop
       overhead, ~30 cycles = ~170 ns against a 250 us tick: 0.07%. */
    taskENTER_CRITICAL();
    for (int ch = 1; ch <= 4; ch++) dshotValue[ch] = s_req[ch];
    taskEXIT_CRITICAL();

    dshotSendFrame();
  }
}

/* ============================================================================
   Public API
   ============================================================================ */

void fans_init(void) {
  dshotHwInit();
  buildBuffer();                     /* all zeros; nothing goes out until the task */
  configASSERT(xTaskCreate(fanTask, "fan", 384, nullptr, 2, &s_task) == pdPASS);
}

/* 0% -> 48 (minimum throttle), 100% -> 2047. Same mapping as fan_test.cpp. */
static uint16_t pctToDshot(float pct) {
  return (pct <= 0.0f) ? 48 : (uint16_t)(48.0f + pct * (2047.0f - 48.0f) / 100.0f);
}

void fans_setThrottle(int ch, float pct) {
  if (ch < 1 || ch > 4) return;
  if (!s_armed || s_killed) { s_rejects++; return; }

  /* Clamp at the SETTER, not at the caller. With the props unguarded (B7) this is
     the ceiling a runaway control law also has to obey -- a caller-side clamp only
     constrains callers that remember to clamp. */
  if (pct < 0.0f) pct = 0.0f;
  if (pct > FAN_THROTTLE_MAX) pct = FAN_THROTTLE_MAX;

  s_pct[ch] = pct;
  s_req[ch] = pctToDshot(pct);
  s_lastCmdMs = xTaskGetTickCount();      /* pet the dead-man */
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
    out[ch] = pctToDshot(p);
  }
  /* Publish as a SET so the four values reach the wire in the same frame. */
  taskENTER_CRITICAL();
  for (int ch = 1; ch <= 4; ch++) { s_pct[ch] = pc[ch]; s_req[ch] = out[ch]; }
  taskEXIT_CRITICAL();
  s_lastCmdMs = xTaskGetTickCount();      /* pet the dead-man */
}

void fans_stopAll(void) {
  /* NO FreeRTOS API, no lock, no unbounded wait: this must work when called from
     an ISR or from faults_safeStop(), which has already disabled interrupts. */
  s_killed = true;
  for (int ch = 1; ch <= 4; ch++) { s_req[ch] = 0; dshotValue[ch] = 0; s_pct[ch] = 0.0f; }

  /* A fault can fire before fans_init() has enabled the peripheral clocks. */
  if (!s_hwReady) return;

  DMA2_Stream5->CR &= ~DMA_SxCR_EN;         /* stop the transfer               */
  TIM1->CR1  &= ~TIM_CR1_CEN;               /* stop the counter                */
  TIM1->BDTR &= ~TIM_BDTR_MOE;              /* outputs off at the timer        */
  TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0; TIM1->CCR4 = 0;

  /* Drive the pins low as plain GPIO -- deliberately NOT via the DMA path, which
     is already dead by the time a fault path calls this (Step 1.3 trap).
     ORDER MATTERS: BSRR first, then MODER. Writing BSRR while the pin is still in
     AF mode updates ODR (harmlessly disconnected from the pad); switching MODER to
     output then connects an already-low ODR. Reverse the order and the pad briefly
     drives whatever ODR happened to be holding. */
  GPIOA->BSRR = (1UL << (8 + 16)) | (1UL << (9 + 16))
              | (1UL << (10 + 16)) | (1UL << (11 + 16));
  for (int p = 8; p <= 11; p++)
    GPIOA->MODER = (GPIOA->MODER & ~(3UL << (p * 2))) | (1UL << (p * 2));  /* output */
}

void fans_rearm(void) {
  if (!s_killed) return;
  dshotHwInit();          /* restores AF mode on PA8..PA11 and re-sets MOE */
  buildBuffer();
  s_armed  = false;       /* fanTask repeats the ~1 s zero ramp before accepting */
  s_killed = false;
}

bool     fans_armedAndLive(void) { return s_armed && !s_killed; }
bool     fans_armed(void)     { return s_armed; }
bool     fans_killed(void)    { return s_killed; }
uint32_t fans_frames(void)    { return s_frames; }
uint32_t fans_overruns(void)  { return s_overruns; }
uint32_t fans_skips(void)     { return s_skips; }
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
