/* ============================================================================
   faults.cpp  —  implementation of the one stop path + black box (Step 1.2)
   ============================================================================ */
#include <Arduino.h>
#include <STM32FreeRTOS.h>     /* TaskHandle_t, taskDISABLE_INTERRUPTS */
#include "faults.h"
#include "timebase.h"          /* TIM5 for rtRunTimeCounter */

/* ---- the black box -------------------------------------------------------
   .noinit is not zeroed by the C startup, so this survives a warm reset (but
   NOT a power cycle — RAM is volatile). The magic guards against reading random
   RAM as a "fault" on a cold boot. NOTE: the F446RE variant ldscript has no
   dedicated .noinit section, so this lands as an orphan; the magic makes a
   stray zeroing benign (reads as clean). The Step 1.3 assert test confirms it
   actually survives; if not, add the INSERT-AFTER .bss linker fragment. */
#define FAULT_MAGIC 0x5A1F0B0Bu   /* "SAlF-BOB" — arbitrary, just unlikely */
typedef struct {
  uint32_t magic;
  fault_t  reason;
  int      line;
  char     file[24];
} fault_record_t;
__attribute__((section(".noinit"))) static fault_record_t g_fault;

/* ---- safe-stop configuration (set by faults_init) ------------------------ */
static int  g_enablePin = -1;
static int  g_ledPin    = -1;
static void (*g_stopHook)(void) = 0;

void faults_init(int hwEnablePin, int ledPin, void (*stopHook)(void)) {
  g_enablePin = hwEnablePin;
  g_ledPin    = ledPin;
  g_stopHook  = stopHook;
}

/* ---- helpers ------------------------------------------------------------- */
static const char* reasonName(fault_t r) {
  switch (r) {
    case FAULT_ASSERT:             return "ASSERT";
    case FAULT_STACK_OVERFLOW:     return "STACK_OVERFLOW";
    case FAULT_MALLOC:             return "MALLOC";
    case FAULT_SCHEDULER_RETURNED: return "SCHEDULER_RETURNED";
    case FAULT_WHEEL_SAT:          return "WHEEL_SAT";
    case FAULT_UNDERVOLT:          return "UNDERVOLT";
    case FAULT_OVERCURRENT:        return "OVERCURRENT";
    case FAULT_HEARTBEAT:          return "HEARTBEAT";
    case FAULT_I2C_TIMEOUT:        return "I2C_TIMEOUT";
    default:                       return "NONE";
  }
}

static void latch(fault_t reason, const char* file, int line) {
  g_fault.magic  = FAULT_MAGIC;
  g_fault.reason = reason;
  g_fault.line   = line;
  /* store the basename only, bounded — never the full build path */
  const char* base = file ? file : "";
  for (const char* p = base; *p; ++p)
    if (*p == '/' || *p == '\\') base = p + 1;
  int i = 0;
  for (; base[i] && i < (int)sizeof(g_fault.file) - 1; ++i)
    g_fault.file[i] = base[i];
  g_fault.file[i] = '\0';
}

/* Cycle-counting delay. delay()/millis() are dead here (interrupts masked, so
   SysTick doesn't tick), hence a raw spin. Not calibrated — only needs to make
   the blink visible. */
static void spin(volatile uint32_t n) { while (n--) { __asm__ volatile(""); } }

/* Kill the motor and blink the reason code forever. Never returns. */
static void faults_halt(fault_t reason) __attribute__((noreturn));
static void faults_halt(fault_t reason) {
  taskDISABLE_INTERRUPTS();

  /* 1) hardware kill FIRST — independent of any SimpleFOC object state */
  if (g_enablePin >= 0) { pinMode(g_enablePin, OUTPUT); digitalWrite(g_enablePin, LOW); }
  /* 2) graceful stop if the sketch registered one */
  if (g_stopHook) g_stopHook();

  /* 3) blink `reason` pulses, pause, repeat. Encodes the code with no serial. */
  const int pin = g_ledPin;
  if (pin >= 0) pinMode(pin, OUTPUT);
  int pulses = (int)reason; if (pulses < 1) pulses = 1;
  for (;;) {
    for (int i = 0; i < pulses; i++) {
      if (pin >= 0) digitalWrite(pin, HIGH);
      spin(7000000);
      if (pin >= 0) digitalWrite(pin, LOW);
      spin(7000000);
    }
    spin(35000000);   /* long gap between bursts */
  }
}

/* ---- public stop paths --------------------------------------------------- */
void faults_safeStop(fault_t reason) {
  latch(reason, "", 0);
  faults_halt(reason);
}

void rtAssertFail(const char* file, int line) {
  latch(FAULT_ASSERT, file, line);
  faults_halt(FAULT_ASSERT);
}

void faults_reportLastBoot(void) {
  if (g_fault.magic == FAULT_MAGIC) {
    Serial.print("[BLACKBOX] previous boot died: ");
    Serial.print(reasonName(g_fault.reason));
    Serial.print(" (reason ");
    Serial.print((int)g_fault.reason);
    Serial.print(")");
    if (g_fault.file[0]) {
      Serial.print(" at ");
      Serial.print(g_fault.file);
      Serial.print(":");
      Serial.print(g_fault.line);
    }
    Serial.println();
    g_fault.magic = 0;   /* consume it so the next clean boot reports clean */
  } else {
    Serial.println("[BLACKBOX] clean boot (no prior fault recorded)");
  }
}

/* ---- run-time-stats clock ------------------------------------------------ */
uint32_t rtRunTimeCounter(void) { return TIM5->CNT; }

/* ---- FreeRTOS failure hooks ---------------------------------------------
   The library only defines these when its *_BLINK config macros are 1; we set
   those to 0 in STM32FreeRTOSConfig.h, so these are ours with no collision.
   extern "C" because the kernel calls them by unmangled C name. ------------- */
extern "C" void vApplicationMallocFailedHook(void) {
  faults_safeStop(FAULT_MALLOC);
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
  (void)xTask; (void)pcTaskName;
  faults_safeStop(FAULT_STACK_OVERFLOW);
}
