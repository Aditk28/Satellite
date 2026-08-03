#pragma once
#include "timebase.h"

typedef struct { uint32_t n, mn, mx; uint64_t sum; } stat_t;

static inline void stat_reset(stat_t* s) {
  s->n = 0; s->mn = 0; s->mx = 0; s->sum = 0;
}

static inline void stat_add(stat_t* s, uint32_t v) {
  if (!s->n || v < s->mn) s->mn = v;
  if (v > s->mx)          s->mx = v;
  s->sum += v; s->n++;
}

// Stream-generic so the same dump can go to USB and HC-05 at once (and so the
// Phase 3 telemetry task, which will own Serial, can point it at either stream).
static inline void stat_print(Print& out, const char* name, const stat_t* s) {
  out.print(name);
  if (!s->n) { out.println("  (no samples)"); return; }
  out.print("  n=");    out.print(s->n);
  out.print("  min=");  out.print(s->mn);
  out.print("  mean="); out.print((uint32_t)(s->sum / s->n));
  out.print("  MAX=");  out.println(s->mx);
}

// Convenience overload: default to USB Serial.
static inline void stat_print(const char* name, const stat_t* s) {
  stat_print(Serial, name, s);
}

#define TIME_BLOCK(st, code) do { uint32_t _a = us_now(); \
                                  code; \
                                  stat_add(&(st), us_since(_a)); } while (0)
