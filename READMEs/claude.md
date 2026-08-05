 CLAUDE.md — Reaction Wheel Attitude Control Platform

Context file for the reaction-wheel / autonomous docking platform firmware.

Read `docs/RTOS_MIGRATION_GUIDE.md` for the full plan; this file is the summary

plus working conventions.

---

## 1. What this is

A tabletop platform on three ball transfer units that points itself using a

reaction wheel — a benchtop analogue of spacecraft attitude control. Long-term

goal is autonomous vision-guided docking (translation via four fans, overhead

AprilTag, Pi doing guidance). **Translation is not started.** Everything current

is rotation-only.

The project exists to demonstrate a full engineering stack: system

identification, state-space/LQR control design, FOC motor control, RTOS

firmware, and later multi-rate sensor fusion and guidance. Career-relevant skill

depth matters as much as the demo working.

## 2. Current state

**Rotation control is written, tuned, and working.** Plant fully identified over

8 calibration runs (~150 trials). Closed-loop envelope tested ±30° to ±180°.

**Active work: migrating the super-loop to FreeRTOS.** Currently in Phase 0

(measurement). See §6.

### Tuned constants — do not change casually

```

A_1 = 45.5              wheel accel per volt, (rad/s²)/V

A_2 = 5.35              wheel damping pole, 1/s

a   = 0.19              J_w/J_p momentum coupling ratio

A_FRICTION = 22.3       Coulomb feedforward magnitude, rad/s²

GYRO_SIGN  = -1

compFrac   = 0.89       back-EMF compensation fraction

K_θ = 119.3   K_ω = 35.1        (the 1.2 s row, ω_n = 4.76, ζ = 0.7)

ffFrac = 0.90

deadzone = 2.0°   deadzoneFine = 1.0°   FINE_WW = 5 rad/s

ALPHA_STALL_MAX = 55   STALL_WW = 25    WHEEL_SAT_LIMIT = 45

control rate = 200 Hz, logged at full rate

```

### Two properties that constrain every design decision

**Bandwidth ceiling.** `ω_n = 4.76` against a wheel pole at `A_2 = 5.35`. Above

the pole the feedback linearization stops cancelling cleanly. There is no margin

to spend on added latency in the sense→command path.

**Passive desaturation with a ~15% margin.** Inside the deadzone `α = 0`, so

`u = 0.105·ω_w` against a `0.123·ω_w` hold voltage — the wheel bleeds down

because 0.105 < 0.123. Anything that changes how `ω_w` is estimated can flip that

margin and turn passive unwind into passive windup toward the 45 rad/s abort.

**Verify unwind behavior after any change touching FOC rate or velocity

estimation.**

### Known-before anomaly

Some closed-loop tests showed the platform rotating back after the step

completed; re-running fixed it. Unexplained. Predates the RTOS work — do not

attribute it to FreeRTOS.

## 3. Hardware

| Part | Role | Notes |

|---|---|---|

| STM32 Nucleo-F446RE | control, 180 MHz | PlatformIO + STM32duino core |

| 4015 BLDC hollow-shaft external-rotor | reaction wheel | pole-pair count: TBD, confirm in `BLDCMotor(N)` |

| MT6701 magnetic encoder | wheel angle/velocity | **SSI (SPI) mode**, not I2C |

| SimpleFOC Mini (DRV8313) | 3-phase gate driver | **no onboard MCU** — Nucleo runs FOC |

| MPU6050 | platform gyro Z | I2C1, PB8/PB9, 400 kHz |

| INA219 | motor supply V/I | I2C1, same bus |

| HC-05 | wireless command + telemetry | 115200, PC10/PC11/PC12/PC0 |

**Timer allocation (audited, Step 0.3):**

| Timer | Owner |

|---|---|

| TIM2 | SimpleFOC PWM, PA5. PSC=0 ARR=1799 CR1=0x61 → center-aligned, **25 kHz** |

| TIM3 | SimpleFOC PWM, PA6/PA7. Same config |

| TIM5 | free-running 1 MHz µs timebase (`timebase.cpp`) |

| TIM9 | reserved for the FOC tick (Phase 1.5) |

| TIM1, TIM4, TIM8, TIM10–12 | free |

TIM4 is free (PB6/D10 is a plain GPIO driver-enable) but deliberately **not**

used — TIM4_CH1 is PB6, so a stray channel-enable would toggle the driver enable.

**TIM9 has no `TIM9_IRQHandler`.** Use `TIM1_BRK_TIM9_IRQHandler` /

`TIM1_BRK_TIM9_IRQn`. Writing the obvious name compiles, links, and never fires.

## 4. Tooling constraints

- **No logic analyzer, no oscilloscope, no second UART adapter.** All timing

  verification is in-firmware: TIM5 timestamps, online min/max/mean stats, and

  (from Phase 1) a software scheduler tracer dumped as CSV and plotted in Python.

  Never propose a solution that requires a scope.

- Two serial channels exist: USB CDC and HC-05.

- Windows dev machine. `py -m pip`, not bare `pip`. WDAC blocks

  `arm-none-eabi-gcc-ar.exe` — worked around by `use_plain_ar.py` registered

  **without** the `pre:` prefix.

## 5. Codebase conventions

- PlatformIO builds every `.cpp` in `src/`; only one may define

  `setup()`/`loop()`. Alternate sketches live in `unflashed_files/`, or use

  `build_src_filter` per environment.

- `MagneticSensorMT6701SSI.h/.cpp` stay in `src/` always.

- Infrastructure goes in its own files, not inline in a sketch: `timebase.*`,

  `timing_stats.h`, `hw_timers.*`, `faults.*`, `trace.*`.

- `#pragma once` in every header.

- Anything referenced from `FreeRTOSConfig.h` must be C-compatible and

  `extern "C"` guarded — the kernel's C sources include it. Hence `trace_c.h`

  separate from `trace.h`.

- `A0`–`A15` are Arduino analog-pin macros. Constants are `A_1`/`A_2`. Mode enums

  are `CTRL_*`, never `MODE_*`.

- I2C runs at 400 kHz (`Wire.setClock(400000)`); at 100 kHz bus reads dominated

  the loop.

## 6. RTOS migration status

Phases per `docs/RTOS_MIGRATION_GUIDE.md`. Tag each exit `rtos-pN-name`.

- [x] 0.1 baseline frozen (summary table deliberately skipped — see below)

- [x] 0.2 TIM5 microsecond timebase verified (PSC=89, 1000 ms → 999,993 µs)

- [x] 0.3 timer audit complete

- [x] 0.4 worst-case execution times (loopFOC 40µs MAX, MPU read 2.37ms, superloop 2.67ms MAX = FOC starvation)

- [x] 0.5 FOC rate determination (4 kHz FOC / 200 Hz control, CTRL_DIVISOR=20, ~16% CPU — Appendix B4)

- [x] 0.6 golden dataset re-capture (178° slew → −0.32°, wheel unwinds cleanly; superloop MAX 12.1s = blocking capture dump, the Phase-3 "before")

- [ ] 1.x kernel config, FPU port proof, NVIC audit, software tracer ← **current step**

- [ ] 2.x single-task port

- [ ] 3.x telemetry extraction

- [ ] 4.x FOC split

- [ ] 5.x safety task + I2C mutex

- [ ] 6.x comms task

- [ ] 7.x consolidation

**The Phase 0 baseline summary table was skipped by choice.** Raw run-8 CSVs are

archived. The accepted risk is reduced ability to detect small regressions; the

substitute is watching wheel-speed decay after a `T90` at each phase exit as a

compFrac sanity check. Don't relitigate this.

**Target architecture** (Phase 4 onward): TIM9 ISR notifies `focTask` (prio 4)

every tick and `controlTask` (prio 3) every Nth tick — one timer, phase-locked.

`safetyTask` and `commsTask` at prio 2, `telemTask` at prio 1. `motor.target` and

`motor.shaft_velocity` are atomic 32-bit float accesses needing no mutex; the

I2C bus needs one; SPI is exclusive to the FOC task.

## 7. Working style

**Background:** solid embedded C and assembly. Register-level detail is

appropriate and welcome — CMSIS register names, bit fields, exception behavior,

what PendSV actually stacks. Don't simplify hardware explanations.

**Not doing background reading.** Fill concept gaps inline, at the moment they

become relevant, not as front-loaded theory. If a step depends on understanding

priority inheritance or lazy FPU stacking, explain it there in a few paragraphs.

The goal is understanding everything by the end, arrived at incrementally.

**One step at a time, interactive.** Work through a single guide step, wait for

the result of flashing/running it, then move on. Don't run ahead.

**Code is integrated manually** by copy-paste. Give complete, self-contained

blocks — whole files, whole functions — not fragments to splice. State where each

piece goes.

**Explain every new file before its code:** what problem it solves, why it's

structured that way, and why non-obvious idioms are there (`do{}while(0)` in

macros, `_`-prefixed macro locals, `uint64_t` accumulators, header-only vs `.cpp`).

This applies to every new file, not just the first few.

**Prefers building new files from scratch** and porting pieces across rather than

editing in place. When porting, the rule is **verbatim** — no renaming, no

cleanup, no "improving while I'm in here." If a real bug turns up, fix it in the

old file first, re-verify, then carry it across. Otherwise the reference moves and

regression comparison becomes meaningless.

**Maintains the guide, not the user.** Update `docs/RTOS_MIGRATION_GUIDE.md`

directly — Appendix B decision log, trap table, phase checkboxes — rather than

telling the user what to change.

**Decisions get recorded with their reasoning**, not just their outcome. Appendix

B entries should say why TIM9 over TIM4, not just "TIM9."

**Skipped work stays skipped.** When the user judges something not worth the

effort, flag the risk once, suggest a cheap substitute, and move on. Don't

re-raise it.

## 8. Things that have already cost real time

- Gyro vs encoder sign conventions (three separate ones). `GYRO_SIGN = -1`.

  `sensor.getAngle()` and `motor.shaft_velocity` have opposite signs;

  `filter_calibration.py` auto-corrects. Logging `motor.shaft_angle` would fix it

  at the source.

- LQR rate term is `α = −K_θ·e **+** K_ω·ω_p`. Plus, not minus. Writing

  `−(K_θ·e + K_ω·ω)` diverges.

- Coulomb feedforward has two opposite-signed branches. Moving branch negative,

  stuck branch positive. Getting the moving branch backwards is worse than no

  feedforward at all.

- `capture_calibration.py` must detect the CSV header (`t_us,`), never

  pattern-match the metadata line. A regex on metadata silently lost a full sweep.

- `getTimerClkFreq` is a `HardwareTimer` member, not a free function. Compute the

  timer clock directly; remember APB timers run at **2× the bus clock** when the

  APB prescaler ≠ 1 (F446: PCLK1 45 MHz → TIM5 90 MHz).

- Serial prompts vs abort-on-any-byte: a trailing Enter arrives milliseconds

  after the character. Drain until the link is quiet for 400 ms.

- `STM32FreeRTOSConfig.h` (our full FreeRTOS override) MUST live in `include/`

  AND `platformio.ini` build_flags MUST include `-Iinclude`. include/ alone only

  reaches project TUs; the FreeRTOS *kernel* TUs (port.c, tasks.c…) need the

  global `-Iinclude` or they silently use `FreeRTOSConfig_Default.h` (configASSERT

  = `for(;;)` hang). Split-brain symptom: thread-context assert works (project TU)

  but ISR/kernel assert hangs with no LED (kernel TU). Verify a temporary

  `#pragma message` fires for port.c/tasks.c, not just faults.cpp (13 TUs, not 2).

  Do NOT use `${platformio.include_dir}` in build_flags — mangles on Windows; use

  relative `-Iinclude`. Cost real time in Step 1.3.

- STM32duino core defines most `TIMx_IRQHandler` symbols (strong) in

  `HardwareTimer.cpp` — defining your own collides at link. Use the HardwareTimer

  API + `attachInterrupt`, or a timer whose vector the core leaves free (TIM9 via

  `TIM1_BRK_TIM9_IRQHandler`).

## 9. Analysis pipeline

```

tools/capture_calibration.py  →  calibration_run_<ts>/        raw

tools/filter_calibration.py   →  <run>/filtered/              corrected + derived

tools/plot_calibration.py     →  <run>/filtered/plots/*.png

tools/make_replay.py          →  <run>/filtered/replay.html

tools/plot_trace.py           →  scheduler Gantt + inter-arrival (Phase 1.4, done)

```

Heading controller capture format:

`t_us,target_deg,theta_deg,omega_p,omega_w,alpha,u`

## 10. Safety

The flywheel stores real energy and the platform is untethered.

- `X` stops the motor. Any unrecognised serial input stops the motor —

  deliberate, keep it.

- `WHEEL_SAT_LIMIT = 45 rad/s` is a hard abort.

- Every failure path (assert, stack overflow, malloc fail, watchdog) must disable

  the driver **first**, then latch a reason code, then report.

- Never propose a test that requires hands near the platform while the wheel is

  spinning.

