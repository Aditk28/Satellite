#pragma once
/* ============================================================================
   trace_c.h  —  C-side boundary for the scheduler tracer (Step 1.4)
   ----------------------------------------------------------------------------
   This is the ONLY trace header the kernel sees. FreeRTOSConfig.h #includes it
   and defines traceTASK_SWITCHED_IN/OUT in terms of traceIn()/traceOut(), so it
   is compiled by tasks.c (a plain-C kernel TU). Therefore it must be:
     - plain C, extern "C" guarded (no Arduino/C++),
     - in include/ so the KERNEL's include path finds it (same -Iinclude lesson
       as STM32FreeRTOSConfig.h from Step 1.3 — src/ is invisible to kernel TUs).
   The C++ API (ring buffer, dump, markers) lives in src/trace.h + src/trace.cpp,
   which the kernel never includes.
   ============================================================================ */
#include <stdint.h>

/* Per-task index, stashed in each task's thread-local-storage slot 0 (needs
   configNUM_THREAD_LOCAL_STORAGE_POINTERS >= 1, which the config sets). The
   switch hooks read it back to know which task ran. Idle's slot is NULL => 0
   => TRACE_ID_IDLE, which is exactly what we want with no extra wiring. */
enum {
  TRACE_ID_IDLE   = 0,
  TRACE_ID_CTRL   = 1,   /* control task   (Phase 2) */
  TRACE_ID_FOC    = 2,   /* FOC task        (Phase 4) */
  TRACE_ID_SAFETY = 3,   /* safety task     (Phase 5) */
  TRACE_ID_COMMS  = 4,   /* comms task      (Phase 6) */
  TRACE_ID_TELEM  = 5,   /* telemetry task  (Phase 3) */
  TRACE_ID_TEST   = 6,   /* Phase 1 harness task A (periodic) */
  TRACE_ID_TEST2  = 7    /* Phase 1 harness task B (spinner)  */
};

#ifdef __cplusplus
extern "C" {
#endif

/* Called from vTaskSwitchContext via the traceTASK_SWITCHED_IN/OUT macros.
   Defined in src/trace.cpp. */
void traceIn(void);    /* context switched IN  — records the incoming task */
void traceOut(void);   /* context switched OUT — records the outgoing task */

#ifdef __cplusplus
}
#endif
