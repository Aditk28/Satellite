 CLAUDE.md — Reaction Wheel Attitude Control Platform

Context file for the reaction-wheel / autonomous docking platform firmware.

**Guides, in reading order:** `CONTROL_README.md` (plant, constants, control law)

→ `RTOS_migration.md` (the five-task firmware, COMPLETE — read its STATUS block and

Appendix A trap table) → **`TRANSLATION_DOCKING.md` (the ACTIVE guide — translation,

vision, docking)**. This file is the summary plus working conventions.

---

## 1. What this is

A tabletop platform on three ball transfer units that points itself using a

reaction wheel — a benchtop analogue of spacecraft attitude control. Long-term

goal is autonomous vision-guided docking (translation via four fans, AprilTag

on the dock, camera on the platform). **That goal is MET as of 2026-08-21: the

platform docks itself.** Slide it anywhere by hand and it returns to the dock in

short hops, correcting against vision at each stop — see `TRANSLATION_DOCKING.md`.

The project exists to demonstrate a full engineering stack: system

identification, state-space/LQR control design, FOC motor control, RTOS

firmware, and later multi-rate sensor fusion and guidance. Career-relevant skill

depth matters as much as the demo working.

## 2. Current state

**Phases 0-6 of `TRANSLATION_DOCKING.md` are COMPLETE.** Tags `trans-p1-fans`,
`trans-p2-plantid`, `trans-p3-link`, `trans-p4-vision`, `trans-p5-estimator`. The RTOS
migration was completed before them (`rtos-p7-complete`). **Phase 6 -- translation
control -- WORKS: the platform docks itself, somewhat consistently.** Phases 7
(combined 3-DOF) and 8 (SEARCH/ACQUIRE state machine) remain, but the demo the
project was built for is running.

**Rotation control: re-identified and retuned 2026-08-19. 0.47 deg mean final error
over +-5 to +-180 deg, every slew in ONE move, wheel returning fully to rest.**
This supersedes every earlier performance figure — those were on the lighter,
pre-translation platform and are no longer comparable.

**Translation plant: identified in ACCELERATION units** (mass and force cancel — see
guide decision B14). `A(throttle) = 2.1e-4*pct^2 m/s^2`, `A_c = 0.26 m/s^2`, breakaway
~35% throttle, thrust-to-friction ratio **2.9x — PASS**. Per-fan constants since
measured separately (`FAN_K_A`) because the four channels are NOT equal.

**Firmware: six FreeRTOS tasks.** `fanTask` (prio 2, 333 Hz) is the sole fan writer,
driving four DSHOT300 channels over TIM1 + DMA burst. Fans are killed ahead of the
wheel in every fault path. Fan 4's spin direction is re-applied at every boot because
this ESC silently ignores DSHOT SAVE_SETTINGS. Control loop measured 4999/5000/5001 us
with fans running — they cost it nothing.

**Yaw coupling measured:** thrust-line offset, 2-3 rad/s^2, wheel peaks 4-12 rad/s.
Small enough that no mechanical correction is needed.

**Phase 3 COMPLETE (2026-08-19, tag `trans-p3-link`): the Pi <-> STM32 link is wired,
framed, and fail-safe.** USART6, PC6 (CN10-4) / PC7 (D9) / GND. The Pi was moved off the
mini-UART onto the PL011 with `dtoverlay=disable-bt` -- on a 3B+ the header pins are
`ttyS0`, whose baud is clocked from the scaling VPU core clock, so the link would have
started failing exactly when AprilTag loaded the CPU (trap T28).

`src/pi_link.*` owns USART6 as sole reader AND sole writer; `pi_poll()` rides
`commsTask`'s 2 ms poll via `commands_setAuxPoll()` and is deliberately kept out of the
operator parser, because a binary `0x58` would otherwise read as `X` and stop the wheel
(B18 -- confirmed by measurement: `comms rx=4` against 2508 Pi bytes).

```
wire     [0xA5][0x5A][len=28][payload][crc16]  CRC-16/CCITT-FALSE, little-endian
payload  seq flags tag_id range_m bearing_rad relyaw_rad quality age_us n_tags
         POLAR not Cartesian (B19) -- keeps the estimator's R diagonal
         AGE not a timestamp (B20) -- the two clocks are never synchronised
ladder   250 ms pose invalid -> 1 s fans zeroed -> 3 s terminal hook (B21)
```

Verified: RTT 2.61/3.52/5.32 ms; 62 frames / 2508 B reconciling exactly against 14 CRC
rejects and 26 seq gaps; ladder reaching DEAD and zeroing the fans. **Synthetic frames
only -- no camera involved yet.**

⚠️ **Known-wrong-for-Phase-6:** tier 2 calls `fans_stopAll()`, the HARD kill, which
latches until `R`. B2 loses the tag at close range, so a 1 s dropout during docking would
strand the approach waiting for an operator. Tier 2 must become a soft zero with the ESC
left armed.

**Phase 4 IN PROGRESS. Step 4.2 DONE (2026-08-20): vision pipeline measured and
chosen.** 1280x720 MJPG, grayscale decode, `decimate=2.0`, threaded capture ->
**44.9 ms capture->pose latency, 14.8 fps, 3 of 3 tags every frame** (B22/B23), down
from 443 ms. Detector is Debian `python3-apriltag` (the reference AprilTag 3.4.2);
`cv2.aruco` is ~4x slower and ignores decimation (T32); `pupil-apriltags` cannot be
built on 512 MB. 720p is forced -- lower modes detect only 2 of 3 tags, losing the
flanking tags that resolve yaw ambiguity (T30).

⚠️ **Camera exposure must be locked manually before calibrating** -- auto-exposure caps
the rate at 15 fps, implies ~15 px blur at 30 deg/s, and changes effective focal length,
so a calibration taken with it on is invalid (T35).

⚠️ **Hardware: the Pi 3B+ died** (PMIC failure -- 3.3 V rail absent, board cold, would
not boot from SD or USB). Replaced with a **Pi 3A+**: identical BCM2837B0 at 1.4 GHz so
identical vision performance, 512 MB, one USB port, no Ethernet, ~1/5 the current draw.
Nothing architectural changed. `tools/pi_setup.sh` rebuilds a fresh Pi in one command.

**Step 4.3 DONE: vision produces pose.** Bundle `solvePnP` (IPPE) over every visible tag
-> range / bearing / rel-yaw / ambiguity ratio. Range matches a tape measure at 0.5 and
1.0 m. Dock: 12 cm centre tag (id 0), 4 cm flanking tags (id 1 left, id 2 right) at
**+-14.15 cm** centre-to-centre, all coplanar at 9 cm height.

```
SIGN CONVENTIONS -- verified on hardware, never compensate downstream (T11)
  bearing  0 = dead ahead   POSITIVE = counter-clockwise
                            sliding the platform LEFT -> bearing POSITIVE
  relyaw   0 = square-on    POSITIVE = viewing from the dock's RIGHT
```

**Step 4.1 chessboard calibration SKIPPED (B24).** Focal length measured directly
instead: **`f = 947 px`**, which revealed the camera is **~76 deg FOV, not the
advertised 120** (B25). Every geometry number derived from the spec sheet was wrong --
detection range is better than planned, close-range framing much tighter -- and the dock
wall moved 25 -> 35 cm so all three tags stay framed when docked. Distortion and
principal point remain uncorrected; revisit if terminal alignment shows a bias no tuning
fixes. `tools/pi_calibrate.py` is written and ready whenever it is wanted.

**Phase 5 DONE (2026-08-20): `src/estimator.*` produces dock-relative platform pose**
at 200 Hz -- `x`, `y`, `psi`, velocity, and the magnet position. Dock frame: origin at
tag 0 projected to the table, +X right as you face the wall, +Y out from it, psi = 0
facing the wall square-on. All of x/y/psi refer to the platform's CENTRE OF ROTATION;
the camera sits 13.46 cm ahead and the magnet 9.46 cm ahead, and those lever arms
ROTATE with heading.

**The sensor split was forced by measurement, not chosen.** Raw vision yaw was
compressing reported `x` to ~35% of true, because `x = range*sin(psi - bearing)`
collapses when psi drifts with bearing (T39). Root cause is T30: coplanar tags are
ill-conditioned in yaw near square-on. So:

```
x, y        <- vision, corrected hard (aPos 0.35)
psi fast    <- gyro, via the controller's existing theta (0.8 deg/min drift)
psi absolute<- vision, corrected SLOWLY (aPsi 0.02, ~1.8 s) -- averages the noise
psi = theta + psi_offset, and ONLY the offset is estimated
```

`PI_FLAG_AMBIGUOUS` drops the heading gain to a quarter. Accelerometer prediction
deliberately not used yet -- its body-frame axis signs are unverified (T11).

**PHASE 6 DONE -- THE DOCKING DEMO WORKS (2026-08-21).** Place the platform on the
dock, `TI`, slide it anywhere by hand, `TG` and it returns in 6-10 cm hops with a
vision fix at each stop. Somewhat consistent; still biases right on arrival.

```
DEMO:  B  ->  R  ->  TB  ->  TI  ->  (slide by hand)  ->  TG        X stops all
       TB re-measures accel bias WITH FANS AT IDLE. Never re-run B after it.
       SLIDE it, never lift -- 1 deg of tilt is 0.17 m/s^2, 14x the bias.
```

**Architecture: position is IMU dead reckoning; vision is a stationary-only fix.**
Vision x/y proved unusable in motion, so above 2 cm/s it contributes nothing, and
at rest 8 frames are averaged and ADOPTED (replacing, not blending). Heading stays
gyro-led. Docking is STEPPED because dead-reckoning error grows as `bias*T^2` --
short hops bound it. See guide B30/B31.

**Two constants cost two days and were both settled by runtime sweep, not
argument:** `EST_ACC_ROT_DEG = 270` (not the reasoned 90) and `FAN_ANGLE_DEG =
{+140,+230,+50,-40}` (remembered table +180). **Read trap T47 before touching
either:** a wrong actuator map and a broken position estimate produce the
identical symptom, so three earlier "corrections" were each derived from the
estimator that was lying. Fix the instrument, then identify the plant.

⚠️ **Accel bias with fans OFF does not apply with fans ON (T49)** -- rectified
prop vibration shifts it 0.158 m/s^2, ten times the boot bias. `TB` corrects at
12% idle; hops run 30-60%, so it is partial. This is the leading suspect for the
remaining rightward bias.

⚠️ **Fans 2 and 3 deliver ~65% of fans 1 and 4**, compensated per-fan in
`FAN_K_A`. Re-measure with `TC` after any prop or ESC-direction change.

**Every constant that has ever been wrong is runtime-settable** -- `TA TR TM TL
TH TP TD TF TV TE TW TB`. Full table in the guide's command reference.

**Raw calibration data** lives in `calibration/runs/`, one folder per experiment named
by what it established, indexed in `calibration/runs/INDEX.md`.
`capture_calibration.py` writes new runs there automatically.

### Tuned constants — RE-IDENTIFIED AND RETUNED 2026-08-19

Measured on the CURRENT platform (translation hardware fitted). Everything before this
date is superseded; see `CONTROL_README` §12 for the derivation.

```

A_1 = 47.9              wheel accel per volt, (rad/s^2)/V

A_2 = 4.97              wheel damping pole, 1/s     (tau' = 0.201 s, free decay)

K'  = 9.64              incremental, rad/s per V    (NOT constant: plateau line is
                        omega = 9.64*V - 1.23, the intercept is wheel Coulomb friction)

a   = 0.098             J_w/J_p, pooled over 11 rising-phase fits, +-10%

compFrac   = 0.90       residual pole -0.92 measured; 0.89 was mildly UNSTABLE (+0.04)

K_theta = 216   K_omega = 52       zeta = 0.54, NOT 0.7 -- friction already damps

ffFrac = 0.95

A_static = 60   A_moving = 34   A_viscous = 0      <-- feedforward is SPLIT

deadzone = 1.5 deg   deadzoneFine = 0.8 deg   FINE_WW = 5 rad/s

ALPHA_STALL_MAX = 70   STALL_WW = 20   STALL_MS = 300   STALL_HOLD_MS = 4500

WHEEL_SAT_LIMIT = 55   MAX_STALL_RETRIES = 3

control rate = 200 Hz, logged at full rate

```

**Performance: 0.47 deg mean final error over +-5 to +-180 deg, every slew in ONE
move, wheel returning fully to rest.** (Previous entry: 1.20 deg mean, 1.57 s settling,
half of large negative slews stalling — on a lighter platform.)

**Four things mattered, three of them latent bugs rather than tuning:**

- **The Coulomb feedforward conflated STATIC with KINETIC friction.** One constant
  cannot both beat breakaway and cancel drag; set it high and the moving branch
  over-cancels into negative damping. Split into `A_static`/`A_moving` (commands `A`
  and `AM`), and 5 deg corrections went from 5/11 to 8/8.
- **`compFrac` 0.89 was mildly UNSTABLE**, not neutral -- residual pole +0.04. That is
  why passive desaturation had been dead since the hardware rework.
- **`ALPHA_STALL_MAX` failed a THIRD time** (28, 40, 55) and was the binding
  constraint, not `A_FRICTION`. Now 70, with the stall detector tightened so it wins
  the race against the abort.
- **zeta = 0.54, not 0.7.** Coulomb friction already damps heavily, so a textbook
  zeta-0.7 design brakes twice and the platform stops short -- then cannot restart,
  because breaking stiction from rest costs far more than finishing a move that still
  has momentum. Read the gain table's zeta column as a starting point, not a target.

### Two properties that constrain every design decision

**Bandwidth ceiling.** `ω_n = 4.76` against a wheel pole at `A_2 = 5.35`. Above

the pole the feedback linearization stops cancelling cleanly. There is no margin

to spend on added latency in the sense→command path.

**Passive desaturation with a ~15% margin.** Inside the deadzone `α = 0`, so

`u = 0.105·ω_w` against a `0.123·ω_w` hold voltage — the wheel bleeds down

because 0.105 < 0.123. Anything that changes how `ω_w` is estimated can flip that

margin and turn passive unwind into passive windup toward the 55 rad/s abort.

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

| 4× FEICHAO 2204 2300KV | translation fans | 420 gf claim, 12 A max, 0.112 Ω |

| HQProp 4043 3-blade | props | **orientation matters — backwards ≈ half thrust** |

| AERO SELFIE 45A 4-in-1 ESC | fan drive | **Bluejay firmware — DSHOT ONLY, no analog PWM** |

| Raspberry Pi 3B+ (1 GB) | AprilTag vision | mounted; link = wired UART (USART6) |

**Fan DSHOT pins — all on GPIOA and all on TIM1:** ch1 **PA8**/D7 (TIM1_CH1), ch2

**PA9**/D8 (CH2), ch3 **PA10**/D2 (CH3), ch4 **PA11 = CN10 pin 14** (CH4). Splitting

across GPIO ports broke every channel, repeatedly; TIM1_CH1–CH4 satisfy one-timer,

one-port by construction. **ch4 moved from PA0 to PA11 in Phase 1** — PA11 is on no

Arduino header, so it needs a female Dupont (or an F–F jumper as a coupler onto the

male morpho pin). Its neighbours at CN10-13/15 are PA6/PA7, two motor PWM phases —

miscount a row and the ESC signal lands on the motor driver.

Driven by `fanTask` via **TIM1 + DMA2 Stream 5 Channel 6 (TIM1_UP) burst mode**: `DBA`

points at CCR1, `DBL`=4, so one update event per DSHOT bit moves four words into

CCR1–CCR4. No DMA interrupt — the frame is re-armed by polling `DMA_SxCR.EN`, which

avoids the STM32duino strong-`IRQHandler` collision entirely. The old bit-bang survives

only in `fan_test.cpp` (env `fantest`); it masks interrupts ~53 µs per frame and must

never enter the RTOS build.

**Planned Pi link:** USART6, PC6 (TX) / PC7 (RX) / GND. 3.3 V both sides.

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

**TIM9's vector is `TIM1_BRK_TIM9_IRQn` (not `TIM9_IRQn`).** But you CANNOT define

`TIM1_BRK_TIM9_IRQHandler` yourself — the STM32duino core strongly defines it in

`HardwareTimer.cpp` (guarded only by `#if TIM9_BASE`, always true here), so a raw

handler collides at link, same as TIM7. **Use the `HardwareTimer(TIM9)` API +**

`attachInterrupt`, then override the NVIC priority (Step 1.5, `focTick_init`).

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

- [x] 1.x kernel config, FPU port, NVIC/assert, software tracer, TIM9 FOC tick (tag `rtos-p1-kernel`)

- [x] 2.x single-task port — monolithic; golden dataset matches, ω_w 17.2@2V, 0 overhead (tag `rtos-p2-single-task`)

- [x] 4.x FOC split — focTask prio 4, TIM9-notified 4 kHz, preempts the blocking MPU read

- [x] 3.x telemetry extraction — telemTask prio 1 = SOLE serial writer; ctrl period MAX
      11.5 s → 5.006 ms (Phases 3 & 4 were SWAPPED — see guide Appendix B13)

- [x] 5.x safety task + I2C mutex — independent watchdog (wheel + time-based heartbeat +
      INA219 power trips); mutex with priority inheritance, control degrades on timeout

- [x] 6.x comms task — commsTask = sole serial reader; lines queued, commands still
      execute on the control task; `X` fast path ≤250 µs

- [x] 7.x consolidation — stacks resized (12.8 KB reclaimed), CPU 27% idle, deadlines
      proven by response-time analysis, CONTROL_README rewritten

**RTOS MIGRATION COMPLETE.** Remaining: optional tracer Gantt plots, and the plant-dependent
artifacts folded into the combined hardware retune (see guide 7.5).

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

- **The ESC runs Bluejay = DSHOT ONLY.** Servo PWM produces nothing even when

  measured perfect (TIM1 50.000 Hz, 999.8 µs, MOE set, verified at the ESC pad).

  Throttle calibration also gives no beeps — analog range calibration doesn't

  exist in that firmware. Cost most of a session.

- **All DSHOT channels must share one GPIO port.** Moving ch4 to PC7 (GPIOC)

  forced a port parameter or a duplicate send function, and *every* variant broke

  the previously-working channels. Root cause never proven; one port + one

  `dshotSend()` is the fix. Do not "clean this up."

- **A backwards prop costs ~half its thrust** — 75% throttle became 50% by

  flipping one prop. Check orientation before concluding you need bigger props.

- **`pinMode()` is overloaded on `PinName` and `uint32_t` pin index** (different

  numbering systems). Passing pin constants through an integer array silently

  retargets them. Type pin tables as `PinName`.

- **A DC voltmeter verifies duty cycle, not pulse width.** 5% at 50 Hz and 5% at

  5 kHz read identically; only one is a valid servo pulse. Dump timer registers.

- **When the user suggests testing a different variable, try it before theorising

  again.** "Should I try a different channel?" was right and was dismissed; it

  cost hours.

- `STM32FreeRTOSConfig.h` (our full FreeRTOS override) MUST live in `include/`

  AND `platformio.ini` build_flags MUST include `-Iinclude`. include/ alone only

  reaches project TUs; the FreeRTOS *kernel* TUs (port.c, tasks.c…) need the

  global `-Iinclude` or they silently use `FreeRTOSConfig_Default.h` (configASSERT

  = `for(;;)` hang). Split-brain symptom: thread-context assert works (project TU)

  but ISR/kernel assert hangs with no LED (kernel TU). Verify a temporary

  `#pragma message` fires for port.c/tasks.c, not just faults.cpp (13 TUs, not 2).

  Do NOT use `${platformio.include_dir}` in build_flags — mangles on Windows; use

  relative `-Iinclude`. Cost real time in Step 1.3.

- STM32duino core defines `TIMx_IRQHandler` for EVERY timer with a `_BASE`

  (strong, in `HardwareTimer.cpp`) — including TIM7 and TIM9's combined

  `TIM1_BRK_TIM9_IRQHandler`. Defining your own collides at link. There is no

  "free vector" to raw-define; always use the `HardwareTimer` API +

  `attachInterrupt` (the core's handler dispatches to your callback), then

  `HAL_NVIC_SetPriority(..IRQn, prio, 0)` to force the priority you need.

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

The flywheel stores real energy and the platform is untethered — and there are now

**four exposed 4-inch 3-blade props capable of ~25,000 RPM** at the platform's

perimeter, i.e. exactly where you reach to pick it up. **Props are UNGUARDED and will

stay that way** — guards were deliberately skipped (2026-08-13, `TRANSLATION_DOCKING.md`

decision B7). Wiring IS secured and routed clear of all four prop discs (Step 0.2 done).

Because there is no mechanical containment, the safety story is entirely firmware and

procedure: **`FAN_THROTTLE_MAX` compiled in (60%, raised from the 30% bring-up ceiling

for Phase 2 plant ID; clamped inside

`fans_setThrottle()`), props physically off for any test that does not need thrust, the

battery disconnect as the real e-stop, and `fans_stopAll()` driving the pins low as GPIO

in the hardware-kill step.** Never propose a test with hands near props or wheel.

Fan current goes as throttle³ (~1.5 A/motor at 50%, ~5 A at 75%) — the fitted 10 A

fuse and 18 AWG wiring (15 A max) were sized for low throttle.

- `X` stops the motor. Any unrecognised serial input stops the motor —

  deliberate, keep it.

- `WHEEL_SAT_LIMIT = 55 rad/s` is a hard abort (`rtos_main.cpp`; the `superloop`

  regression sketch still carries the older 45).

- Every failure path (assert, stack overflow, malloc fail, watchdog) must disable

  the driver **first**, then latch a reason code, then report.

- Never propose a test that requires hands near the platform while the wheel is

  spinning.

