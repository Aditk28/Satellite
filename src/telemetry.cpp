#include "telemetry.h"
#include <STM32FreeRTOS.h>
#include <task.h>
#include <queue.h>

// Tagged record. The union keeps every queue item one size (queues copy by value)
// while carrying different payloads.
enum : uint8_t { K_TEXT = 0, K_CALL = 1 };

struct TelemMsg {
  uint8_t kind;
  union {
    char text[100];      // K_TEXT: one line, NUL-terminated, no newline
    void (*fn)();        // K_CALL: run this on telemTask
  };
};

static QueueHandle_t     s_q       = nullptr;
static TaskHandle_t      s_task    = nullptr;
static Print*            s_usb     = nullptr;
static Print*            s_bt      = nullptr;
static volatile bool     s_active  = false;
static volatile uint32_t s_drops   = 0;
static volatile uint32_t s_pending = 0;   // enqueued but not yet executed

// The ONLY place the serial ports are touched once telemTask is active.
static void writeBoth(const char* s) {
  if (s_usb) s_usb->println(s);
  if (s_bt)  s_bt->println(s);
}

static void telemTask(void*) {
  TelemMsg m;
  for (;;) {
    // Blocks while the queue is empty -> consumes no CPU. Wakes only once the
    // control task has yielded (blocked on its own notification), so the two
    // never run concurrently in the serial driver.
    if (xQueueReceive(s_q, &m, portMAX_DELAY) == pdTRUE) {
      switch (m.kind) {
        case K_TEXT: writeBoth(m.text); break;
        case K_CALL: if (m.fn) m.fn(); break;   // fn owns the ports while it runs
        default: break;
      }
      if (s_pending) s_pending--;
    }
  }
}

void telem_init(Print& usb, Print& bt) {
  s_usb = &usb; s_bt = &bt;
  s_q = xQueueCreate(24, sizeof(TelemMsg));
  configASSERT(s_q != nullptr);
  configASSERT(xTaskCreate(telemTask, "telem", 512, nullptr, 1, &s_task) == pdPASS);
}

void telem_activate() { s_active = true; }

void telem_print(const String& s) {
  // Boot path: telemTask can't drain until the control loop yields, so write
  // straight through. Single-writer still holds -- the task isn't running yet.
  if (!s_active || !s_q) { writeBoth(s.c_str()); return; }

  TelemMsg m;
  m.kind = K_TEXT;
  strncpy(m.text, s.c_str(), sizeof(m.text) - 1);
  m.text[sizeof(m.text) - 1] = '\0';
  s_pending++;
  if (xQueueSend(s_q, &m, 0) != pdTRUE) { s_drops++; if (s_pending) s_pending--; }
}

void telem_run(void (*fn)()) {
  if (!fn) return;
  // Before activation there is no other writer, so just run it inline.
  if (!s_active || !s_q) { fn(); return; }

  TelemMsg m;
  m.kind = K_CALL;
  m.fn   = fn;
  s_pending++;
  if (xQueueSend(s_q, &m, 0) != pdTRUE) { s_drops++; if (s_pending) s_pending--; }
}

bool     telem_busy()  { return s_pending > 0; }
uint32_t telem_drops() { return s_drops; }

uint32_t telem_stackFreeWords() {
  return s_task ? (uint32_t)uxTaskGetStackHighWaterMark(s_task) : 0;
}
