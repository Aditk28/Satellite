/* ============================================================================
   trace.cpp  —  scheduler tracer ring buffer + switch hooks (Step 1.4)
   ============================================================================ */
#include "trace.h"
#include <STM32FreeRTOS.h>   /* pvTaskGetThreadLocalStoragePointer */
#include "timebase.h"        /* us_now() -> TIM5->CNT */

static trace_ev_t        traceBuf[TRACE_N];   /* 24 KB, .bss */
static volatile uint16_t traceHead    = 0;    /* next slot to write */
static volatile bool     traceOn      = false;
static volatile bool     traceWrapped = false;

/* Append one event. LOCK-FREE by design: the switch hooks are the only writers
   in normal use, and context switches are fully serialized (a switch runs to
   completion before the next; they never nest), so writes can't race each other.
   Power-of-two TRACE_N makes the wrap a single AND. (Caveat: a user-marker call
   from a task could in principle be preempted mid-write by a switch; the worst
   case is one torn/lost event, which is harmless for a Gantt chart.) */
void trace_mark(uint8_t id, uint8_t evt) {
  if (!traceOn) return;
  uint16_t h = traceHead;
  traceBuf[h].t_us = us_now();
  traceBuf[h].id   = id;
  traceBuf[h].evt  = evt;
  h = (uint16_t)((h + 1) & (TRACE_N - 1));
  if (h == 0) traceWrapped = true;   /* buffer has filled at least once */
  traceHead = h;
}

/* Context-switch hooks. Called from vTaskSwitchContext (tasks.c) via the
   traceTASK_SWITCHED_IN/OUT macros in STM32FreeRTOSConfig.h.
   pvTaskGetThreadLocalStoragePointer(NULL, 0) returns the CURRENT task's stashed
   index — the outgoing task in traceOut, the incoming in traceIn (see the FreeRTOS
   call order). NULL slot (idle) reads as 0 = TRACE_ID_IDLE. */
extern "C" void traceIn(void) {
  trace_mark((uint8_t)(uintptr_t)pvTaskGetThreadLocalStoragePointer(NULL, 0), 1);
}
extern "C" void traceOut(void) {
  trace_mark((uint8_t)(uintptr_t)pvTaskGetThreadLocalStoragePointer(NULL, 0), 0);
}

void trace_start(void) { traceHead = 0; traceWrapped = false; traceOn = true; }
void trace_stop(void)  { traceOn = false; }

/* Stop, dump the whole buffer oldest-first as CSV, then restart fresh.
   MUST stop first (Trap 1: reading while writing gives torn data). If wrapped,
   the oldest event is at traceHead; otherwise events are simply [0, head). */
void trace_dump(Print& out) {
  trace_stop();
  uint16_t head    = traceHead;
  bool     wrapped = traceWrapped;
  uint16_t count   = wrapped ? TRACE_N : head;
  uint16_t start   = wrapped ? head    : 0;

  out.print("# trace: n="); out.print(count);
  out.print(" wrap=");       out.println(wrapped ? "yes" : "no");
  out.println("t_us,id,evt");
  for (uint16_t i = 0; i < count; i++) {
    const trace_ev_t& e = traceBuf[(uint16_t)((start + i) & (TRACE_N - 1))];
    out.print(e.t_us);          out.print(",");
    out.print((unsigned)e.id);  out.print(",");
    out.println((unsigned)e.evt);
  }
  out.println("# end trace");
  trace_start();
}
