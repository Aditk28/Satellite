#include "commands.h"
#include <STM32FreeRTOS.h>
#include <task.h>
#include <queue.h>

#define CMD_MAX_LEN     40
#define CMD_QUEUE_DEPTH  8
#define POLL_PERIOD_MS   2   /* 500 Hz. At 115200 that is ~23 bytes per poll,
                                comfortably inside the core's 64-byte RX buffer. */

struct CmdMsg { char line[CMD_MAX_LEN]; };

static QueueHandle_t     s_q       = nullptr;
static TaskHandle_t      s_task    = nullptr;
static Stream*           s_usb     = nullptr;
static Stream*           s_bt      = nullptr;
static void            (*s_emergency)(void) = nullptr;
static volatile uint32_t s_rxBytes = 0;
static volatile uint32_t s_drops   = 0;

static void dispatch(String& line) {
  line.trim();
  if (!line.length()) return;

  /* X FAST PATH: kill the motor here, now, before queueing. Waiting for the
     control task to drain the queue would add up to one control period; this
     store reaches focTask within a tick. The line is still queued so that
     handleCommand runs the full stop (flags, operator message) as usual. */
  char c = line.charAt(0);
  if ((c == 'X' || c == 'x') && s_emergency) s_emergency();

  CmdMsg m;
  strncpy(m.line, line.c_str(), CMD_MAX_LEN - 1);
  m.line[CMD_MAX_LEN - 1] = '\0';
  if (xQueueSend(s_q, &m, 0) != pdTRUE) s_drops++;   /* never block on RX */
}

static void pump(Stream* s, String& buf) {
  if (!s) return;
  while (s->available()) {
    char ch = (char)s->read();
    s_rxBytes++;
    if (ch == '\n' || ch == '\r') {
      if (buf.length()) { dispatch(buf); buf = ""; }
    } else if (buf.length() < CMD_MAX_LEN - 1) {
      buf += ch;
    }
    /* else: overlong line, drop the excess rather than grow unboundedly */
  }
}

static void commsTask(void*) {
  String bufU, bufB;
  for (;;) {
    /* vTaskDelay (blocking), NOT delay(): delay() busy-spins on yield() and would
       starve telemTask (1) -- Trap 19, learned in Step 5.1. */
    vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    pump(s_usb, bufU);
    pump(s_bt,  bufB);
  }
}

void commands_init(Stream& usb, Stream& bt, void (*emergencyHook)(void)) {
  s_usb = &usb; s_bt = &bt; s_emergency = emergencyHook;
  s_q = xQueueCreate(CMD_QUEUE_DEPTH, sizeof(CmdMsg));
  configASSERT(s_q != nullptr);
  configASSERT(xTaskCreate(commsTask, "comms", 256, nullptr, 2, &s_task) == pdPASS);
}

bool commands_next(String& out) {
  if (!s_q) return false;
  CmdMsg m;
  if (xQueueReceive(s_q, &m, 0) != pdTRUE) return false;   /* never block control */
  out = String(m.line);
  return true;
}

uint32_t commands_rxBytes(void) { return s_rxBytes; }
uint32_t commands_drops(void)   { return s_drops; }

uint32_t commands_stackFreeWords(void) {
  return s_task ? (uint32_t)uxTaskGetStackHighWaterMark(s_task) : 0;
}
