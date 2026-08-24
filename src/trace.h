#pragma once
/* ============================================================================
   trace.h  —  C++ API for the scheduler tracer (Step 1.4)
   ----------------------------------------------------------------------------
   Application-facing. Included by project .cpp files and by trace.cpp; NEVER by
   the kernel (the kernel only sees trace_c.h). Lives in src/ for that reason.
   ============================================================================ */
#include <Arduino.h>        /* Print (for trace_dump) */
#include "trace_c.h"        /* the TRACE_ID_* enum + traceIn/traceOut */

/* One event = 6 bytes, packed so the buffer is exactly TRACE_N*6 with no
   per-element padding (matters at 4096 entries). */
typedef struct __attribute__((packed)) {
  uint32_t t_us;   /* TIM5 microsecond timestamp (Step 0.2 timebase) */
  uint8_t  id;     /* TRACE_ID_* — which task, or which task emitted a marker */
  uint8_t  evt;    /* 0 = switched OUT, 1 = switched IN, >=10 = user marker */
} trace_ev_t;

#define TRACE_N  4096      /* power of two => wrap is a mask. 4096*6 = 24 KB */

/* evt codes for user markers — things a raw switch trace cannot see. Recorded
   with trace_mark(TRACE_ID_owner, code). */
#define TRC_MARK_I2C_START  10
#define TRC_MARK_I2C_END    11
#define TRC_MARK_STEP       20
#define TRC_MARK_SAFESTOP   30

void trace_start(void);           /* clear + begin recording */
void trace_stop(void);            /* stop recording (buffer preserved) */
void trace_mark(uint8_t id, uint8_t evt);   /* append one event (used by markers + hooks) */
void trace_dump(Print& out);      /* stop, emit CSV (oldest first), restart */
