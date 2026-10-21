/* Fault stop path, black box and FreeRTOS failure hooks. */
#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "faults.h"
#include "timebase.h"

/* .noinit is not zeroed at startup, so this survives a warm reset but not a power
   cycle. The F446RE linker script has no .noinit section, so it lands as an orphan
   section. The magic makes random RAM on a cold boot read as clean. */
#define FAULT_MAGIC 0x5A1F0B0Bu
typedef struct {
  uint32_t magic;
  fault_t  reason;
  int      line;
  char     file[24];
} fault_record_t;
__attribute__((section(".noinit"))) static fault_record_t g_fault;

static int  g_enablePin = -1;
static int  g_ledPin    = -1;
static void (*g_stopHook)(void) = 0;
/* Not touched by faults_init: it is registered in setup() before hwSetup() calls
   faults_init. */
static void (*g_hwKillHook)(void) = 0;

void faults_init(int hwEnablePin, int ledPin, void (*stopHook)(void)) {
  g_enablePin = hwEnablePin;
  g_ledPin    = ledPin;
  g_stopHook  = stopHook;
}

void faults_setHwKillHook(void (*fn)(void)) { g_hwKillHook = fn; }

static const char* reasonName(fault_t r) {
  switch (r) {
    case FAULT_ASSERT:             return "ASSERT";
    case FAULT_STACK_OVERFLOW:     return "STACK_OVERFLOW";
    case FAULT_MALLOC:             return "MALLOC";
    case FAULT_SCHEDULER_RETURNED: return "SCHEDULER_RETURNED";
    case FAULT_WHEEL_SAT:          return "WHEEL_SAT";
    case FAULT_HEARTBEAT:          return "HEARTBEAT";
    default:                       return "NONE";
  }
}

static void latch(fault_t reason, const char* file, int line) {
  g_fault.magic  = FAULT_MAGIC;
  g_fault.reason = reason;
  g_fault.line   = line;
  /* basename only, truncated to fit */
  const char* base = file ? file : "";
  for (const char* p = base; *p; ++p)
    if (*p == '/' || *p == '\\') base = p + 1;
  int i = 0;
  for (; base[i] && i < (int)sizeof(g_fault.file) - 1; ++i)
    g_fault.file[i] = base[i];
  g_fault.file[i] = '\0';
}

/* Busy-wait. SysTick is stopped with interrupts masked, so delay() cannot be used.
   Uncalibrated; it only has to make the blink visible. */
static void spin(volatile uint32_t n) { while (n--) { __asm__ volatile(""); } }

static void faults_halt(fault_t reason) __attribute__((noreturn));
static void faults_halt(fault_t reason) {
  taskDISABLE_INTERRUPTS();

  if (g_hwKillHook) g_hwKillHook();   /* fans first */
  if (g_enablePin >= 0) { pinMode(g_enablePin, OUTPUT); digitalWrite(g_enablePin, LOW); }
  if (g_stopHook) g_stopHook();

  /* Blink `reason` pulses, pause, repeat. */
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
    spin(35000000);
  }
}

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
    g_fault.magic = 0;
  } else {
    Serial.println("[BLACKBOX] clean boot (no prior fault recorded)");
  }
}

uint32_t rtRunTimeCounter(void) { return TIM5->CNT; }

/* The library only defines these hooks when its *_BLINK options are 1. They are 0
   in STM32FreeRTOSConfig.h, so these are ours. */
extern "C" void vApplicationMallocFailedHook(void) {
  faults_safeStop(FAULT_MALLOC);
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
  (void)xTask; (void)pcTaskName;
  faults_safeStop(FAULT_STACK_OVERFLOW);
}
