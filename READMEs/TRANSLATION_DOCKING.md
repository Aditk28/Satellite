# Translation, Vision, and Autonomous Docking — Build Guide

Taking a working single-axis reaction-wheel platform to a **3-DOF (x, y, θ)
vision-guided autonomous docking demonstrator**, without breaking the controller
and RTOS that already work.

**Companions:** `CONTROL_README.md` (the rotation plant, identified constants,
control law), `RTOS_migration.md` (how the five-task FreeRTOS firmware was built,
and every trap found doing it), `claude.md` (conventions, hardware, working style).

**No logic analyzer, no oscilloscope.** All timing verification is in-firmware:
TIM5 timestamps, `timing_stats.h`, the software scheduler tracer, and serial dumps.
Never propose a solution that requires a scope.

---

# ═══════════════════════════════════════════════════════════════════════
# STATUS & RESUME — read this first
# ═══════════════════════════════════════════════════════════════════════

> This block is the single source of truth for *where we are*. A fresh session —
> human or a new AI agent — should be able to read only this section plus the named
> step below and continue without re-deriving anything. **Update it at every step
> exit.**

## Current position

**Phase 0 COMPLETE with one deliberate exception (below). Phase 1 IN PROGRESS —
Steps 1.1, 1.2 and 1.3 PASSED 2026-08-13. Step 1.4 (manual fan commands) is NEXT,
and it is the last step before the `trans-p1-fans` tag.**

**Where the fans stand:** DSHOT300 via TIM1 + DMA burst is proven standalone (`fandma`,
22,278 frames / 0 overruns) **and** integrated into the RTOS firmware as `fans.*` +
`fanTask` (prio 2, 500 Hz, sole fan writer). The ESC arms on boot, `overruns` stays 0,
and the FOC tick / `ctrl period` / `loopFOC` figures are unchanged — fans cost the
control loop nothing, as the priority model predicted. **Hardware for Phase 1 is done:**
ch4 moved A0 → CN10-14 (PA11); ch1–3 unchanged because D7/D8/D2 *are* PA8/PA9/PA10.

**Safety paths are done (1.3):** `X`, `stopMotor()`, every `faults_safeStop()` path and
the new fanTask stall watchdog all kill the fans, and the fault path kills them *ahead*
of the wheel (B10). `R` re-arms.

**⚠️ What is still NOT true:** there is no way to command fan throttle from serial, so
the fans arm and sit at DSHOT 0 forever. That is Step 1.4 — and it is the first time
this project will command real thrust from the RTOS firmware, so it is the first step
where `FAN_THROTTLE_MAX` and the props-off rule actually bite.

**⚠️ The exception: prop guards were SKIPPED by user decision (B7). The props are and
will remain EXPOSED.** Everything else in Phase 0 is done — wires are routed clear of
all four prop discs, the fuse and wiring are confirmed, and the battery disconnect is
the real e-stop. The compensating mitigations are firmware-side and are now
**requirements on Phase 1**, not suggestions: a compiled-in `FAN_THROTTLE_MAX` ceiling,
props-off for any test that does not need thrust, and `fans_stopAll()` in the
hardware-kill step. See B7 and Step 0.1's Result note.

Everything upstream is DONE and verified on hardware:

- **Rotation control** — working, tuned, closed-loop envelope tested ±30° to ±180°.
- **RTOS migration** — COMPLETE, all seven phases, tagged `rtos-p7-complete`.
  Five tasks, deadlines proven by response-time analysis, 27% CPU idle.
- **Translation hardware** — 4× 2204 2300KV motors on a 45A 4-in-1 ESC, all four
  channels confirmed spinning under DSHOT300. Props fitted.
- **Vision hardware** — Raspberry Pi 3B+ (1 GB) and camera mounted and wired.

## What is NOT done

| item | state |
|---|---|
| **Fan guards** | ⚠️ **SKIPPED — props are EXPOSED and will stay that way** (B7). Mitigated in firmware, not mechanically. |
| Wire management | ✅ done 2026-08-13 — wires separated from all four prop discs |
| Fans in the RTOS firmware | **in progress** — TIM1+DMA driver proven standalone (`fandma`, Step 1.1 ✅); not yet ported into `rtos_main.cpp` (Step 1.2) |
| Pi ↔ STM32 link | **no wire run yet** — Pi is powered and the camera is connected, but UART is unrun (decision made: wired UART, see B1). Deferred to Phase 3 / when needed. |
| AprilTag detection | not started |
| Translation plant ID | not started |
| Any translation control | not started |
| Combined retune (rotation) | deliberately deferred — see "the retune question" below |

## Decisions already made (do not relitigate — see Appendix B)

| # | decision |
|---|---|
| **B1** | **Pi ↔ STM32 = wired UART**, not Bluetooth. Chosen for latency and dropout immunity. |
| **B2** | **Camera rides the platform, AprilTag is fixed on the target/dock.** Gives relative bearing + range, NOT absolute position. The platform must search to acquire. |
| **B3** | **All control runs on the STM32; the Pi is a pose sensor only.** Open to revisiting if MPC later needs to live on the Pi. |
| **B4** | **LQR first, MPC later.** Three double integrators (x, y, θ). Get a working baseline before adding complexity. |

## How to resume (new session checklist)

1. **Read, in order:** `CONTROL_README.md` (plant + control law), `claude.md`
   (conventions, hardware, working style), `RTOS_migration.md` **STATUS block +
   Appendix A trap table** (the traps are still live — every one of them can bite
   again), then this file's Status block and the current step.
2. **Working style — do not violate:**
   - **One step at a time, interactive.** Do one step, then STOP and wait. Never run ahead.
   - **The user flashes hardware and reports results.** You write code and
     compile-check (`pio run -e <env>`). Hand all flashing and serial interaction
     to the user. Only drive the board yourself if strictly unavoidable, and ask first.
   - **Explain before code.** Every new file and concept gets explained first —
     what problem it solves, why it is structured that way, why non-obvious idioms
     are there. Register-level detail (CMSIS names, bit fields, DMA streams,
     exception behavior) is welcome and should not be simplified.
   - **Verbatim port rule.** Control-law and sensor code moves character-for-character.
     No renaming, no cleanup, no "improving while I'm in here." If you find a real
     bug, fix it in the ORIGINAL first, re-verify, then carry it across.
   - **Maintain this guide, not the user.** Update the STATUS block, the checklist,
     Appendix B, and a dated Result note under the step, at every step exit.
   - **Record decisions with their reasoning**, not just the outcome.
   - **When the user suggests a different variable to test, try it before
     theorising further.** This was learned expensively — see Trap T7.
3. **Safety** (read Phase 0 before touching hardware): four exposed props at
   ~25,000 RPM plus an untethered flywheel. `X` stops everything. Never propose a
   test with hands near props or wheel.
4. Go to the step named under "Current position" and follow Concept → Do → Verify → Trap.

## Build & flash cheat-sheet

- **`pio`** at `C:\Users\k28ad\.platformio\penv\Scripts\platformio.exe` (on PATH in
  new terminals; from a Claude shell use `~/.platformio/penv/Scripts/platformio.exe`).
- **Environments** (`build_src_filter` per env, all sketches live in `src/`):

| env | sketch | purpose |
|---|---|---|
| `rtos` | `rtos_main.cpp` | **THE firmware** — five-task FreeRTOS controller |
| `superloop` | `heading_control.cpp` | original bare-metal controller, regression reference |
| `fantest` | `fan_test.cpp` | standalone DSHOT fan test, no RTOS |
| `enctest` | `enc_test.cpp` | standalone bare-metal MT6701 read, no RTOS/SimpleFOC |
| `p1test` | `p1_test.cpp` | Phase-1 RTOS throwaway harness |

- **Flash:** `pio run -e <env> -t upload` · **Monitor:** `pio device monitor -e <env>`
- **Serial:** COM6 = USB/ST-LINK, COM7/COM8 = HC-05. Both channels work for commands;
  set your terminal's **line ending to LF** or nothing parses.
- **Captures** land in `calibration_run_<timestamp>/`.

## Critical environment facts (hard-won — do NOT relearn these)

These are in addition to `RTOS_migration.md`'s list, which all still applies.

1. **The ESC runs Bluejay — DSHOT ONLY.** Bluejay is a BLHeli_S fork that removed
   analog input. Servo PWM produces *nothing*, no matter how perfect: we verified
   TIM1 at 50.000 Hz with 999.8 µs pulses and MOE set, measured at the ESC's own
   pad, and the ESC never armed. Throttle-range calibration also produces no beeps,
   because analog range calibration does not exist in that firmware.
2. **All four DSHOT channels must be on the SAME GPIO PORT** (currently GPIOA:
   PA8/PA9/PA10/PA0). Putting channel 4 on PC7 forced either a port parameter or a
   duplicated send function, and **every such variant broke the previously-working
   channels**. Root cause never fully proven; the fix that works is to keep one
   port and one `dshotSend()`. Do not "clean this up."
3. **A backwards prop costs about half its thrust.** Needing 75% throttle turned
   into a comfortable 50% purely by flipping one prop. Check prop orientation and
   handedness before concluding you need bigger props.
4. **`pinMode()` in STM32duino is overloaded on `PinName` and on `uint32_t` pin
   index** — different numbering systems. Passing pin constants through an
   integer-typed array silently retargets them. If you need a pin table, type it
   `PinName`.
5. **`E` reads SimpleFOC's cached angle, which is multiplied by `sensor_direction`.**
   With the motor supply off, `initFOC()` cannot align, `sensor_direction` stays
   `UNKNOWN` (= 0), and everything reads **exactly 0** — looking like a dead encoder
   when the hardware is fine. Use `enctest` (raw SSI, no SimpleFOC) to check hardware.
6. **Arduino `delay()` busy-spins on `yield()`** and only yields to equal-or-higher
   priority — it silently starves every lower-priority task. Use `vTaskDelay()`
   anywhere inside an RTOS task.
7. **Fuse and wire are sized for low throttle.** 18 AWG → 15 A fuse maximum, 10 A
   fitted. Prop current goes as throttle³ (~1.5 A per motor at 50%, ~5 A at 75%),
   so four fans at high throttle will blow it. Raise wire gauge before fuse rating.
8. **The DSHOT beep command (cmd 1, the `B` key) does not work on this ESC and never
   has** — no chirp, on the bit-bang path or the DMA path. Not investigated, and it
   does not need to be: the beep was only ever a *proxy* for "is the ESC decoding our
   frames," and motors spinning on command proves that far more strongly. **Do not
   spend time debugging `B`.** If you ever want a spin-free decode check, use a
   throttle value below the commutation floor rather than the beacon commands.
9. **Arduino header vs morpho pins are the SAME NET, not two nets shorted.** `D7`/`D8`/
   `D2` *are* `PA8`/`PA9`/`PA10` — one MCU pin brought out at two connector positions.
   Only `PA11` is morpho-exclusive (CN10-14), which is why ch4 needed a female–female
   Dupont coupler while ch1–3 needed nothing at all. **CN10-14's neighbours in the odd
   column are PA6 and PA7 — two motor PWM phases.** Miscount by one row and the ESC
   signal lands on the motor driver.

## Status checklist

| Phase | State | One-line result |
|---|---|---|
| **0 safety hardening** | ✅ **(2026-08-13)** | Wires cleared, fuse/wiring confirmed, battery disconnect = e-stop. **Prop guards deliberately skipped (B7)** — mitigations moved into Phase 1. |
| 1 fans into the RTOS (DSHOT + DMA) | 🟡 **IN PROGRESS** | **1.1 ✅** TIM1+DMA burst proven standalone (`fandma`), ch4 → PA11. **1.2 ✅** `fans.*` + `fanTask` prio 2, sole fan writer. **1.3 ✅** hw-kill hook ahead of the wheel, `X`/`stopMotor`/`R`/stall-watchdog; found+fixed frame-aborts (99→0) and a latent boot double-writer; ctrl period 4999/5000/5001. **1.4 next** (manual fan commands). |
| 2 translation plant ID | ⬜ | — |
| 3 Pi ↔ STM32 wired link | ⬜ | — |
| 4 vision: AprilTag pose on the Pi | ⬜ | — |
| 5 estimator: fuse tag + IMU | ⬜ | — |
| 6 translation control (LQR, x/y) | ⬜ | — |
| 7 combined 3-DOF + allocation | ⬜ | — |
| 8 acquisition and docking sequence | ⬜ | — |
| 9 consolidation and evidence | ⬜ | — |

---

# The system as it stands today

## Hardware inventory

| part | role | notes |
|---|---|---|
| STM32 Nucleo-F446RE | all control, 180 MHz | PlatformIO + STM32duino + FreeRTOS |
| 4015 BLDC hollow-shaft | reaction wheel | pole pairs = 11 |
| MT6701 magnetic encoder | wheel angle/velocity | **SSI (SPI) mode**, not I2C |
| SimpleFOC Mini (DRV8313) | 3-phase gate driver | no onboard MCU — Nucleo runs FOC |
| MPU6050 | platform gyro Z (+ unused accel) | I2C1 @ 400 kHz |
| INA219 | wheel-supply V/I | I2C1, same bus, `setCalibration_32V_2A()` |
| HC-05 | wireless telemetry + command | 115200 |
| **4× FEICHAO 2204 2300KV** | translation fans | 420 gf claimed, 12 A max, 0.112 Ω |
| **HQProp 4×4.3×3 (4043)** | 3-blade props | 4 CW + 4 CCW available |
| **AERO SELFIE 45A 4-in-1 ESC** | fan drive | **Bluejay firmware — DSHOT only** |
| **Raspberry Pi 3B+ (1 GB)** | AprilTag vision | mounted, camera wired |
| 3S ~12 V 2300 mAh | main battery | 10 A fuse, 18 AWG |

## Complete pin map

**Do not assign a new pin without checking this table.**

| pin | Arduino | function | owner |
|---|---|---|---|
| PA5 | D13 | motor PWM phase A | SimpleFOC / TIM2_CH1 |
| PA6 | D12 | motor PWM | SimpleFOC / TIM3_CH1 |
| PA7 | D11 | motor PWM | SimpleFOC / TIM3_CH2 |
| PB6 | D10 | DRV8313 enable | GPIO (**not** TIM4 — see below) |
| PB13 | — | encoder SCK | SPI2, CN10-30 |
| PB14 | — | encoder DO (MISO) | SPI2, CN10-28 |
| PB15 | — | *(MOSI, unused by SSI)* | CN10-26 |
| PB1 | — | encoder CS | GPIO, CN10-24 |
| PB8 / PB9 | D15 / D14 | I2C1 SCL / SDA | MPU6050 + INA219 |
| PC10 / PC11 / PC12 | — | HC-05 TX / RX / EN | USART3 |
| PA2 / PA3 | D1 / D0 | ST-LINK VCP | USART2 |
| **PA8** | **D7** | **fan ch1 DSHOT** | GPIO bit-bang |
| **PA9** | **D8** | **fan ch2 DSHOT** | GPIO bit-bang |
| **PA10** | **D2** | **fan ch3 DSHOT** | GPIO bit-bang |
| **PA0** | **A0** | **fan ch4 DSHOT** | GPIO bit-bang (**not** D9/PC7) |
| PC1 | A4 | ESC current sense (optional) | unwired |
| **PC6 / PC7** | — / D9 | **PROPOSED Pi link** | USART6 — free |

**Timers:** TIM2 + TIM3 = motor PWM (25 kHz, centre-aligned) · TIM5 = 1 MHz µs
timebase · TIM9 = 4 kHz FOC/control tick · **free: TIM1, TIM4, TIM8, TIM10–12.**
TIM4 is free but **deliberately unused** — TIM4_CH1 is PB6, the driver enable.

## ESC connector — silkscreen `C N 4 3 2 1 + −`

| pad | function | wire to |
|---|---|---|
| C | current sense (analog) | A4 (PC1), optional |
| N | **"No Output"** — genuinely unconnected | nothing |
| 4 / 3 / 2 / 1 | throttle signals | **A0** / D2 / D8 / D7 |
| + | **raw battery 12 V** | ⚠️ **nothing** — would destroy the Nucleo |
| − | battery negative | GND, routed in the signal bundle |

Battery input is the separate large `⊕`/`⊖` pads; the supplied capacitor goes
directly across them with the shortest possible leads.

## Firmware architecture (five FreeRTOS tasks)

| task | prio | period | owns |
|---|---|---|---|
| `focTask` | 4 | 250 µs (4 kHz) | `loopFOC()` + `move()` — commutation only |
| `controlTask` | 3 | 5 ms (200 Hz) | gyro read, control law, capture, command execution |
| `safetyTask` | 2 | 50 ms (20 Hz) | wheel overspeed, heartbeat watchdog, INA219 power |
| `commsTask` | 2 | 2 ms poll | serial RX + line assembly — **sole reader** |
| `telemTask` | 1 | event | **all** serial output — **sole writer** |

One TIM9 interrupt at 4 kHz notifies `focTask` every tick and `controlTask` every
20th (`CTRL_DIVISOR`), so control is phase-locked to commutation.

**Two invariants hold the design together — both learned the hard way:**
- **One writer.** Only `telemTask` writes serial. A *partially* applied version of
  this rule corrupted the driver, froze the board, and produced frozen sensor reads
  that looked exactly like a hardware fault.
- **One reader.** Only `commsTask` reads serial. Two readers race the RX ring
  buffer and one silently eats the other's bytes.

**Measured (2026-08-08):** control period 4994 / 5000 / 5006 µs (200.00 Hz, ±6 µs);
FOC tick 238–261 µs; CPU 42% ctrl / 15% foc / 11% telem / 2% safety / **27% idle**;
stacks resized to ≥61% free margin. Response-time analysis proves all deadlines met
(ctrl R = 3599 µs vs 5000 µs) despite U = 0.868 exceeding the RM bound of 0.757.

## Fan subsystem as it stands

Working, but **only in the standalone `fantest` sketch** — none of it is in the
RTOS firmware yet.

```
protocol   DSHOT300, 16-bit frames, MSB first
frame      [11-bit value][1-bit telemetry][4-bit CRC]
           0 = disarmed, 1..47 = commands, 48..2047 = throttle
timing     bit 3.333 us ; '0' high 1.25 us ; '1' high 2.50 us
transport  GPIO bit-bang, DWT cycle counter, interrupts masked ~53 us per frame
rate       every 2 ms per channel
```

**Measured:** unloaded commutation floor ~2% (1% pulses = desync/retry, 2% runs).
With props fitted, ~50% throttle gives usable translation velocity. Prop
orientation matters enormously — a backwards prop nearly doubled the throttle
required.

**⚠️ The bit-bang CANNOT go into the RTOS firmware.** Masking interrupts for
~53 µs × 4 channels = ~212 µs per cycle would punch holes in the 250 µs FOC tick.
Phase 1 replaces it with timer + DMA.

## The retune question — read before touching gains

`CONTROL_README` §17 planned **one** combined identification campaign across x, y,
and θ after translation was built, on the grounds that adding fans changes mass,
inertia and friction so every rotation constant must be re-identified anyway.

**Current position: the user has chosen to skip a dedicated rotation retune and go
straight to translation.** Rotation "works fine for the most part." The known
consequences, which must not be silently forgotten:

- **Passive desaturation does not complete.** The wheel holds speed in the deadzone
  instead of bleeding to zero (effective K′ ≈ 9.57 vs identified 8.50). Confirmed
  pre-existing — it fails on the OLD bare-metal firmware too, so it is a plant
  shift from the hardware rework, not an RTOS defect.
- **The terminal-approach figures in `CONTROL_README` §2 predate the mass change.**
- **`A_FRICTION` was never swept** — the long-standing tuning gap.

**Phase 2 of this guide re-identifies the plant anyway**, because translation
control needs mass and friction numbers that do not currently exist. Rotation
constants come along for free at that point. So the retune is not being skipped so
much as folded into Phase 2 — do it there rather than as a separate campaign.

---

# The mission

**Autonomous vision-guided docking.** The platform starts at an unknown pose,
searches for an AprilTag mounted on a fixed target, then approaches and docks.

```
SEARCH   →  rotate until the tag enters the camera's field of view
ACQUIRE  →  extract relative bearing + range, initialise the estimator
APPROACH →  translate toward the target while holding heading on the tag
ALIGN    →  null lateral offset and heading error at close range
DOCK     →  final low-speed closure until contact
```

**Success criterion (define concretely in Phase 8, but as a starting target):**
repeatable contact within the docking magnets' tolerance (several degrees of
heading, ~1 cm lateral) from a random starting pose, with no operator input.

## Why this is hard, honestly

1. **Unidirectional actuators.** Fans only push. Allocation is constrained
   (thrust ≥ 0), so opposing pairs need an idle bias for bidirectional authority —
   which costs power continuously and has no analogue in the wheel axis.
2. **No actuator feedback.** ESCs are open-loop from the MCU: no RPM, no thrust,
   no current per channel. Contrast the wheel, where the MT6701 made feedback
   linearisation possible. **None of the CONTROL_README §5/§7 machinery transfers.**
3. **Relative, intermittent measurement.** The tag gives bearing + range only when
   visible, at ~10–30 Hz, with latency. It drops out during search and at close
   range when the tag leaves the frame.
4. **Coupled axes.** Thrust-line offset from the CoM produces yaw the wheel must
   reject; wheel torque disturbs heading during translation.
5. **Friction dominates here too.** Inferred translational breakaway is 30–64 gf
   depending on mass — and mass has grown. Expect a deadband exactly like the
   rotation axis had.

---

# Phase map

| Phase | Name | Est. | Tag |
|---|---|---|---|
| **0** | Safety hardening | 0.5 d | `trans-p0-safe` |
| **1** | Fans into the RTOS (DSHOT via DMA) | 2 d | `trans-p1-fans` |
| **2** | Translation plant identification | 2 d | `trans-p2-plantid` |
| **3** | Pi ↔ STM32 wired link | 1 d | `trans-p3-link` |
| **4** | Vision: AprilTag pose on the Pi | 2 d | `trans-p4-vision` |
| **5** | Estimator: fuse tag + IMU | 2 d | `trans-p5-estimator` |
| **6** | Translation control (LQR, x/y) | 2 d | `trans-p6-translation` |
| **7** | Combined 3-DOF + allocation | 2 d | `trans-p7-3dof` |
| **8** | Acquisition and docking sequence | 2 d | `trans-p8-docking` |
| **9** | Consolidation and evidence | 1 d | `trans-p9-complete` |

**Total: ~16 focused days.** Phases 1 and 5 are the heavy ones.

---

# Phase 0 — Safety hardening

**Goal:** make the platform safe to iterate on. **This blocks everything.**

The rotation work had one dangerous element: an untethered flywheel storing real
energy. There are now **four exposed 4-inch 3-blade props capable of ~25,000 RPM**
in addition. Those cut to the bone, and unlike the flywheel they are at hand height
and at the platform's edge.

## Step 0.1 — Prop guards

**Concept.** The props are the highest-severity hazard on the platform and the one
most likely to be contacted accidentally, because they sit at the perimeter exactly
where you reach to pick the platform up.

**Do.** Fit a guard around each prop disc. Options in order of preference:

1. **Full circular duct/shroud per fan** — also improves static thrust slightly by
   reducing tip losses, so this is not purely a safety cost.
2. **Perimeter bumper ring** around the whole platform at prop height.
3. **Minimum acceptable:** rigid guards on the outboard arc of each prop.

**Verify.** You can push a finger toward each prop from any direction the platform
can be approached and be stopped by structure before reaching the disc.

**Trap.** A guard that flexes into the prop is worse than no guard. Check clearance
with the prop stationary *and* confirm nothing resonates at speed.

**Result (2026-08-13) — SKIPPED, deliberately. See B7.** The user judged the build time
unavailable and chose to proceed with exposed props. The risk was flagged once and is not
being re-raised. **The accepted hazard:** four unguarded 4-inch 3-blade props at the
platform perimeter, at hand height, on a chassis that must be picked up by hand. The
realistic failure mode is not deliberate contact — it is an unexpected arm or spin-up
while hands are on the chassis.

**Compensating mitigations, now binding requirements on Phase 1:**

1. **`FAN_THROTTLE_MAX`, compiled in, 30% during bring-up.** Clamp applied inside
   `fans_setThrottle()` so *every* path is covered, including a runaway control law.
   Raised only deliberately, when Phase 2 plant ID needs the range.
2. **Props off for any test that does not need thrust.** DSHOT frame verification,
   register dumps, DMA bring-up, and all Step 1.3 fault-path provocation run bare.
3. **The battery disconnect is the e-stop; serial `X` is the convenience.** Confirmed
   reachable from a standing position without leaning over the platform.
4. **`fans_stopAll()` drives the pins low as GPIO in the hardware-kill step** — never
   via the DMA path, which `faults_safeStop()` has already killed interrupts for.
   (Already the Step 1.3 trap; it is now load-bearing rather than belt-and-braces.)

## Step 0.2 — Wire management

**Concept.** The user has flagged this: loose wiring near spinning props gets cut,
and a severed motor or signal wire mid-run is both a control failure and a short
hazard. This platform also *rotates*, so anything not secured will eventually be
dragged into something.

**Do.**
- Route and **tape/tie every wire** clear of all four prop discs.
- Keep the **motor phase and battery leads away from the SSI encoder (PB13/14/15)
  and I2C (PB8/PB9)** — this bus has already gone marginal once from the rework
  (MPU read drifted 2373 → 2510 µs) and it produced false 150 rad/s saturation aborts.
- Strain-relieve everything that crosses between the platform and anything fixed.
- Confirm nothing can foul the reaction wheel.

**Verify.** Spin the platform by hand through 360° in both directions. Nothing
tugs, snags, or approaches a prop.

**Result (2026-08-13) — DONE.** All wiring separated from the prop discs and routed
clear; nothing sits near a prop. With guards skipped (B7) this step carries more weight
than it otherwise would — a severed wire is now the *only* thing standing between loose
routing and a prop strike, so re-check routing after any rework that disturbs the
harness. Note the I2C/SSI adjacency warning above remains live: this bus already went
marginal once from the hardware rework (MPU read drifted 2373 → 2510 µs) and produced
false 150 rad/s saturation aborts, which is why `WW_MAX_JUMP` rejection exists.

## Step 0.3 — Emergency stop discipline

**Concept.** `X` currently stops the wheel. Once fans are in the firmware it must
stop **everything**, and the operator must be able to reach it instantly.

**Do.**
- Confirm the **battery disconnect / switch is reachable without leaning over the
  platform.** This is the real e-stop; serial is the convenient one.
- Phase 1 must extend `X` and every fault path to zero all four fan channels
  **before** anything else. Write the requirement down now so it is not forgotten.
- Re-check the **fuse**: 10 A fitted, 18 AWG wire, 15 A absolute maximum. Fan
  current goes as throttle³ (~1.5 A/motor at 50%, ~5 A at 75%).

**Verify.** Battery can be disconnected in under a second from a standing position.

**Result (2026-08-13) — DONE.** Battery disconnect confirmed reachable without leaning
over the platform; it is the real e-stop. Fuse and wiring confirmed unchanged: 10 A
fitted, 18 AWG, 15 A absolute ceiling. **The Phase-1 requirement is recorded here so it
cannot be forgotten:** `X`, `faults_safeStop()`, the heartbeat trip, and every power trip
must zero all four fan channels **first**, before anything else. With guards skipped
(B7), `FAN_THROTTLE_MAX` joins that list as a compiled-in ceiling rather than a runtime
setting.

**Also as of 2026-08-13:** the Raspberry Pi is mounted, powered, and connected to its
camera. The USART6 link to the STM32 is **not run** — deferred to Phase 3 or to whenever
vision is actually needed. Nothing before Phase 3 depends on it.

**Phase 0 exit:** `git tag trans-p0-safe`

---

# Phase 1 — Fans into the RTOS firmware

**Goal:** four DSHOT channels driven from `rtos_main.cpp` **without disturbing the
250 µs FOC tick**, with fans included in every safety path.

## Step 1.1 — Why the bit-bang cannot come across

**Concept.** The `fantest` sketch masks interrupts for ~53 µs per frame. Four
channels every 2 ms is ~212 µs of blackout per cycle. The FOC tick is 250 µs.
Dropping that into the RTOS would starve commutation and destroy the timing
guarantees Phase 7 of the RTOS migration proved.

**The fix is hardware generation: timer + DMA.** The timer produces the PWM
waveform; DMA feeds compare values from a RAM buffer. The CPU writes 16 words and
walks away — no interrupt masking, no code-layout sensitivity, no jitter.

**Do — the mechanism, in detail.**

DSHOT is PWM where the *duty* encodes the bit, at a fixed bit period:

```
DSHOT300 bit period 3.333 us    '0' = 37.5% duty    '1' = 75% duty
```

So: run a timer channel in PWM mode with `ARR` = one bit period, and DMA a 16-entry
array of `CCR` values (one per bit) on each update event. The hardware clocks out
the frame.

On TIM1 at 180 MHz for DSHOT300:
```
ARR + 1 = 180e6 / 300e3 = 600      (PSC = 0)
CCR for '0' = 0.375 x 600 = 225
CCR for '1' = 0.750 x 600 = 450
buffer      = 16 entries + a trailing 0 to leave the line low between frames
```

**TIM1 has four channels (CH1–CH4) and supports DMA burst**, so one timer and one
DMA stream can drive all four fans from a single interleaved buffer. That is the
target design.

**⚠️ Pin consequence.** TIM1_CH1–CH4 are **PA8, PA9, PA10, PA11**. Channels 1–3
already match. **Channel 4 must move from PA0 to PA11 = CN10 pin 14** on the ST morpho
header. The morpho headers are **male pins**, so this needs a *female* Dupont end —
no soldering. (Worth stating only because the Arduino headers CN5/6/8/9 are female
sockets, so the lead currently plugged into A0 has the wrong gender on it.)

**PA11 is forced, not preferred.** TIM1_CH4 maps only to PA11 and PE14, and the F446RE
is LQFP64 — there is no GPIOE. Every other GPIOA pin with a usable timer is taken:
PA0/PA1 → TIM2_CH1/TIM5_CH1 (motor PWM + µs timebase), PA2/PA3 → ST-LINK VCP and
TIM9 (the FOC tick), PA5/PA6/PA7 → motor PWM, PA15 → TIM2_CH1. PA11 is the only pin
that satisfies "one timer *and* one GPIO port" at the same time.

**Trap 1 — do not lose the same-port lesson.** Critical fact #2 exists because
splitting channels across ports broke everything. With TIM1 CH1–4 all four are on
GPIOA *and* on one timer, so the lesson is respected by construction. Keep it that way.

**Trap 2 — TIM1 is an advanced timer.** Its outputs do nothing until **MOE**
(`BDTR` bit 15) is set. `HAL_TIM_PWM_Start` does it; raw register setup must do it
explicitly. Symptom of forgetting: registers all look perfect, pin never moves.

**Trap 3 — verify with the `T`-style register dump, not a meter.** A DC voltmeter
confirms *duty cycle*, not *pulse width*: 5% at 50 Hz and 5% at 5 kHz read
identically and only one is valid. Dump `PSC`/`ARR`/`CCR`/`BDTR`/`CCER` and compute.
This exact blind spot cost hours during fan bring-up.

**Verify.**
- Register dump shows the arithmetic above.
- Each channel spins its motor at 10% via the new path.
- **`M` shows FOC tick dt still 238–261 µs and `ctrl period` MAX ≈ 5006 µs.** This
  is the gate: fans must cost the control loop nothing. **→ deferred to Step 1.2 by
  B8** — it is an RTOS measurement and cannot be taken from the standalone sketch.

**Result (2026-08-13) — PASSED.** Built as `src/fan_dma_test.cpp`, env `fandma`
(standalone, raw CMSIS, no FreeRTOS/SimpleFOC — decision B8). Hardware change was
exactly one wire: fan ch4 signal A0/PA0 → **CN10 pin 14 = PA11**, via a female–female
Dupont as a coupler since PA11 appears on no Arduino header. ch1–3 needed nothing —
D7/D8/D2 *are* PA8/PA9/PA10, the same MCU nets brought out at two connector positions.

**All four channels spin under the DMA path.** Register dump verified end to end:

```
TIM1 clock 180000000 Hz   PSC=0 ARR=599 -> bit 3.3333 us (300 kHz)
CR1  =0x85     CEN + URS + ARPE
CCMR1=CCMR2=0x6868        PWM mode 1 + preload, all four channels
CCER =0x1111   CCxE set, CCxNE CLEAR  <- complementary outputs stay off the
                                          motor phases and the encoder bus
BDTR =0x8000   MOE=1
DCR  =0x30d    DBA=13 (CCR1), DBL=4 transfers
DIER =0x100    UDE
AFRH =0x1111   PA8..PA11 on AF1;  MODER[23:16]=0xAA
DMA2_S5 CR=0x0C025440  CHSEL=6  DIR=1  MINC=1  PSIZE=MSIZE=2 (32-bit)
        PAR=0x4001004C = TIM1_BASE+0x4C = DMAR   M0AR=0x20000470 (SRAM)
        EN=0  NDTR=0   <- previous frame fully drained
frames=22278  overruns=0
```

**The health signature to re-check after any change here:** `EN=0` with `NDTR=0` at
rest, and `overruns` pinned at 0. Together they say every frame drained well inside the
2 ms period. `overruns` incrementing would mean the timer has stopped or the DMA is
being re-armed before it finishes.

**Known non-working, deliberately not chased: the `B` beep command (DSHOT cmd 1)
produces no chirp.** It never worked, on the bit-bang path either, so it is not a
regression and not a DMA problem. Not investigated — and it does not need to be,
because the beep was only ever a *proxy* for "is the ESC decoding our frames," and four
motors spinning on command answers that far more strongly than a chirp would.
**Do not spend time on `B` in a future session.**

**One bug found and fixed in the dump itself** (not the driver): the bit-rate line
printed `1e6f/bit_us` with `bit_us` in microseconds, yielding Hz under a `kHz` label —
it read `300000.0 kHz`. Timing was always correct; the label was not. Fixed to
`1e3f/bit_us`. Worth noting because this dump is the *instrument* for the rest of
Phase 1 (Trap T6 — a meter cannot verify pulse width), and a mislabelled instrument is
how a future session gets misled.

## Step 1.2 — `fans.*` module and `fanTask`

**Concept.** Follow the established subsystem pattern (`telemetry.*`, `safety.*`,
`commands.*`): a small module with a clear ownership rule.

**Ownership rule for fans:** **only `fanTask` writes fan outputs.** Everything else
requests a thrust vector. Same discipline as one-writer/one-reader, for the same
reason.

**Do.**
```c
// fans.h
void fans_init(void);                       // timer + DMA + task, all channels 0
void fans_setThrottle(int ch, float pct);   // 0..100, clamped
void fans_setAll(float f1,f2,f3,f4);
void fans_stopAll(void);                    // MUST be callable from any fault path
uint32_t fans_frames(void);                 // diagnostics for G
```
- `fanTask` priority **2** (with safety and comms; below control, above telemetry).
- Re-arm/refresh frames at a fixed rate (2–4 ms) via `vTaskDelayUntil`.
- ESCs disarm if frames stop — that is a **feature**: a hung `fanTask` fails safe.

**Trap — arming.** ESCs need a stream of DSHOT 0 before accepting throttle. Do this
in `fans_init()` and do not accept throttle commands until it completes.

**Result (2026-08-13) — PASSED, with a defect found later at Step 1.3.**
`src/fans.h` + `src/fans.cpp`, `fanTask` prio 2 at 500 Hz, `fans_init()` called
pre-scheduler from `setup()`, two status lines added to `G`. All six build
environments compile. On hardware the ESC arms, `frames` advances, and the deferred
Step-1.1 gate passes — FOC tick dt 241–259 µs, `ctrl period` mean 4999 / MAX 5002 µs,
`loopFOC` 32/32/45 µs, all unchanged or slightly better than the migration baseline.

> **⚠️ CORRECTION (2026-08-13).** This note originally claimed **`overruns=0`**. That
> was false — the first `G` of the Step 1.3 session showed **`overruns=99`**, and it
> had been 99 since boot. The claim was written from a verbal "everything checked out"
> rather than from transcribed figures. Root cause and fix are in the Step 1.3 result
> note. **Process lesson, worth more than the bug:** do not write a specific measured
> number into this guide unless it was actually read off the terminal. A wrong number
> in the permanent record is worse than no number, because the next session trusts it
> and stops looking.

**Four design points worth keeping:**

1. **Two arrays, not one.** A DSHOT frame is a *set* of four values clocked out
   together. Each `uint16_t` store is atomic on Cortex-M4 but four of them are not
   atomic **as a set** — a preemption mid-write puts a torn allocation on the wire.
   So requests land in `s_req[]` and `fanTask` snapshots them into its own
   `dshotValue[]` under a critical section.
2. **That critical section masks the FOC tick, and that is unavoidable.**
   `taskENTER_CRITICAL()` writes `BASEPRI = configMAX_SYSCALL_INTERRUPT_PRIORITY`
   (`5<<4 = 0x50`), which blocks every exception at priority value ≥ 0x50 — and TIM9
   is at exactly 5. It *must* be, because its ISR calls `vTaskNotifyGiveFromISR` and
   anything above the syscall ceiling would corrupt the kernel (RTOS Trap 3). Cost is
   ~30 cycles ≈ **170 ns against a 250 µs tick, 0.07%**. Paid deliberately rather than
   hand-waved.
3. **Arming runs ON the task, not in `fans_init()`.** The standalone sketch armed with
   `for(500){send; delay(2);}`; `delay()` is Trap T8/RTOS-19 and would have starved
   `telemTask`, so the boot output would never have been written out. The task paces
   its own ramp with `vTaskDelayUntil`. Throttle requests before arming completes are
   **rejected and counted** (`fans_rejects()`), not silently dropped.
4. **`fans_stopAll()` uses no FreeRTOS API at all** — no lock, no blocking, no
   unbounded wait — because it has to work when called from an ISR or from
   `faults_safeStop()`, which has already disabled interrupts. It drives PA8–PA11 low
   as plain GPIO. **Order matters: BSRR before MODER.** Writing BSRR while the pin is
   still in AF mode updates ODR (harmlessly disconnected from the pad); switching
   MODER to output then connects an already-low ODR. Reverse it and the pad briefly
   drives whatever ODR was holding.

**Kept from Step 1.1 rather than optimised:** `dshotSendFrame()` still stops TIM1 and
restarts from `CNT=0` every frame instead of leaving it free-running and re-arming
only the DMA. Free-running is marginally cheaper and is what Betaflight does, but
stop/restart is the version with 22k proven frames behind it and the cost is a handful
of register writes per 2 ms.

**Not tested, deliberately: wheel unwind after `T90`.** `claude.md` asks for an unwind
check after anything touching timing, but rotation control is not currently consistent
enough post-hardware-change for that observation to mean anything — passive
desaturation is already known dead (B6). Deferred to Phase 2 with the rest of the
rotation work rather than recorded as a meaningless pass.

## Step 1.3 — Fans in every safety path

**Concept.** The safety architecture currently only knows about the wheel. Fans are
now the larger hazard and must be wired into the same paths.

**Do.**
- `stopMotor()` → also `fans_stopAll()`.
- `faults_safeStop()` → fans off in the **hardware-kill** step, before anything else.
- `safetyTask` → add fan-related checks. ~~extend the power trip thresholds~~ —
  **this instruction was WRONG, corrected 2026-08-13, see B9.** The INA219 sits on the
  **wheel supply**, so it cannot see fan current at all; raising the current trip
  would only weaken the wheel's own protection in exchange for nothing. Thresholds
  stay at 10.0 V / 2500 mA. The *undervoltage* threshold does need re-measuring once
  fans draw real current — **that moves to Phase 2**, where it can be set from data
  instead of guessed.
- `X` fast path in `commsTask` → zero fans immediately, alongside `motor.target = 0`.
- Heartbeat failure → fans off.

**Verify — provoke each deliberately, props OFF:**
- `X` during fan run → all four stop within one frame period.
- Heartbeat trip → fans off, LED blinks, black box latches.
- Undervoltage trip → fans off.

**Trap.** `faults_safeStop()` disables interrupts first. If fan shutdown depends on
DMA still running, it will not happen. **Drive the fan pins low as GPIO in the
hardware-kill step** rather than relying on the DMA path.

**Result (2026-08-13) — PASSED on the stop paths, and it surfaced two defects from
1.2 that are now fixed.**

Wired up: `faults_setHwKillHook(fans_stopAll)` (B10) runs at the top of `faults_halt()`
before the DRV8313 enable; `stopMotor()` hard-kills fans; `commsEmergencyStop()` (the
`X` fast path on commsTask) hard-kills in microseconds; `R` calls `fans_rearm()`;
`safetyTask` gained a wall-clock, latched fanTask-stall watchdog. Power thresholds
deliberately unchanged — **the guide's instruction there was wrong, see B9.**

Verified on hardware: `X` → `fans: KILLED` with `frames` **frozen** (1819 → 1819 across
several seconds, which is the real proof the task stopped sending rather than merely
zeroing throttle). `R` → re-arm message, then `fans: armed`, `frames` climbing again
(6279 → 9466). An unrecognised command `q` → `!! STOP` and `fans: KILLED`, confirming
the existing "any unrecognised input stops the motor" rule now covers the props.
`M` after all of it: `ctrl period` mean 4999 / MAX **5002** µs, FOC tick 241–259 µs,
`loopFOC` 32/32/45 — no regression from the added stop-path work.

**Heartbeat/assert provocation: NOT TESTED, by choice.** The hw-kill hook is therefore
verified by inspection only. Recorded rather than glossed: the one fault path exercised
in anger is the recoverable stop, not `faults_halt()`.

**Defect 1 — `overruns=99`, present since 1.2 and mis-recorded as 0.** 99 of the first
~698 frames, then **one** in the next ~8,700. A boot-window phenomenon, and the timer
audits prove why: both show `TIM1 CEN=0 DIER=0x0`, so `fanTask` had not executed at all
yet — `controlTask` is inside `hwSetup()`, where SimpleFOC's `_delay()` becomes Arduino
`delay()` and busy-spins at prio 3, starving prio 2 (Trap T8/RTOS-19, biting indirectly
through a library this time).

The mechanism was **the resync guard being too loose**. After starvation
`vTaskDelayUntil` returns immediately once per missed period; the guard only resynced
past 8 ms of backlog, so 2–7 ms windows produced 2–3 **back-to-back** frames. A frame
needs ~60 µs of timer time to drain, so each burst found the previous transfer still in
flight — and the original code's response was to **abort it mid-frame** and start a new
one, putting a truncated CRC-failing frame on the wire for no benefit.

Two fixes: **never catch up** (fans want a steady stream, not replayed history — resync
whenever the deadline has already passed), and **skip rather than abort** when a
transfer is in flight, with a 4-attempt stuck-detector that force-restarts so a
genuinely wedged DMA cannot wedge silently.

*Severity, honestly: not a hazard.* A truncated frame fails the ESC's DSHOT CRC and the
next good frame corrects 2 ms later — it armed fine throughout. But it is corrupt
traffic on the wire, and under real throttle a burst of rejected frames is precisely how
T12 (motor stops mid-manoeuvre) happens.

**Defect 2 — a latent Trap 17 / B15 violation introduced in 1.2.** `telem_print()`
before `telem_activate()` writes the port **directly from the calling task**. Arming
completes ~1 s after the scheduler starts, still inside `hwSetup()` while `controlTask`
is direct-writing boot output — so `fanTask` was a **second concurrent writer into
non-reentrant `HardwareSerial`**. It interleaved harmlessly the one time it shipped,
which is exactly what that trap says makes a partial single-writer worse than none.
Fixed with a new `telem_isActive()`: the announcement is deferred until telemTask owns
the ports. **Any future task that wants to print during boot must gate on it.**

**Post-fix verification (2026-08-13, figures read off the terminal):**

```
fans: armed  frames=599   overruns=0     <- first G after boot (was 99)
fans: armed  frames=3737  overruns=0     <- still 0
"fans: armed" now prints AFTER the boot banner (i.e. after telem_activate)

M! then M, clean 10 s window:
  loopFOC       32 / 32 / 34      (migration baseline MAX was 45)
  ctrl period   4999 / 5000 / 5001   (migration baseline 4994 / 5000 / 5006)
  FOC tick dt   239 - 261 us      (baseline 238 - 261)
  control law   2415 mean  -- ENCLOSES the MPU read; real compute
                = st_law - st_mpu = 2415 - 2399 = 16 us (documented ~14)
fanTask stack: 182 of 384 words used at peak (the String in the arm message)
```

Timing is at or slightly better than the pre-fan baseline on every line, which settles
the Step-1.1 deferred gate properly rather than by construction.

**Three log readings that look alarming and are not** — recorded so they are not chased
again: (1) `control law mean=2415 µs` *encloses* the MPU6050 read, so the real compute
is `st_law − st_mpu` ≈ 16 µs, per RTOS B6. (2) A `ctrl period min` of 3571 µs appears
if `M` is run without `M!` first, because the window then spans `hwSetup()`; on a clean
window it is 4999. (3) The timer audit prints **TIM2/TIM3 at "50000.0 Hz"** because it
divides by `ARR+1` without accounting for centre-aligned mode (`CR1=0xC1` counts up
*and* down) — the real PWM rate is the documented 25 kHz. Pre-existing quirk in
`hw_timers.cpp`, unrelated to fans.

## Step 1.4 — Manual fan commands in the main firmware

**Do.** Port the useful `fantest` commands so translation can be exercised without
the vision stack: per-channel throttle, all-stop, and a thrust-vector command.
Route all output through `telem_print` — **never write serial directly** (Trap 17).

**Phase 1 exit:** `git tag trans-p1-fans`

---

# Phase 2 — Translation plant identification

**Goal:** the numbers translation control cannot be designed without. This is the
`CONTROL_README` §14 method applied to a new axis, and it also re-identifies the
rotation axis for free.

**None of the rotation constants transfer.** `A_1`, `A_2`, `compFrac`, `K_HOLD` are
all properties of the wheel loop and have no fan analogue.

## Step 2.1 — Static thrust curve

**Concept.** Fans are open-loop: no RPM, no thrust feedback. The only way to know
what a throttle command does is to measure it once and store the curve.

**Do.** Mount one motor on a fixed arm over a kitchen scale, props fitted, and
sweep throttle 0 → 60% in 5% steps. Record grams-force at each point. Repeat for
all four (they will differ by 5–10%).

**Verify.** Fit `F = k · throttle²` — thrust should go roughly as the square, since
thrust ∝ RPM² and RPM ≈ linear in throttle. Record `k` per motor and the **fit
residual**; a poor fit means the linear-RPM assumption is failing.

This gives the inverse you need for allocation:
```
throttle = sqrt(F_desired / F_max)
```

**Trap 1.** Measure at your **actual battery voltage**, and note the voltage — the
curve shifts as the pack drains. This is the fan analogue of the wheel's `K'`.
**Trap 2.** The scale must measure *thrust*, not the motor's weight. Mount so the
thrust axis is vertical and tare with the motor mounted but stopped.

## Step 2.2 — Translational breakaway force

**Concept.** `CONTROL_README` §6 established that Coulomb friction dominates
rotation (4.24 rad/s² against a 1.57 rad/s² maneuver). Translation will have the
same character, and the inferred estimate (30–64 gf) is the widest uncertainty in
the whole design — and the mass has grown since it was written.

**Do — the five-minute test from §18.** Tie a string to the platform, run it over
the table edge over a low-friction pulley, and add weight until the platform slides.
Repeat from several starting positions and orientations; stiction is stochastic.

**Verify.** Record breakaway force in gf, the spread across trials, and the
platform's current total mass.

**Compare against the thrust curve from 2.1.** The ratio of available thrust to
breakaway force is the single number that determines whether this is controllable:
- **> 3×** — comfortable
- **2–3×** — workable, expect a deadband
- **< 2×** — stop and fix friction or thrust before writing any control

**Trap.** Test on the **cleaned** table. The user already observed a curved
translation path attributed to dust. Surface condition is a real plant parameter here.

## Step 2.3 — Step-response identification

**Concept.** With thrust and friction known, get the dynamics: apply a known thrust
step and measure the acceleration response.

**Do.** Use the **MPU6050 accelerometer** — currently unused and already on the
bus. Command a step on one fan, log accel X/Y at the control rate, integrate.

**Verify.** Fit `m·a = F − F_friction·sign(v)`. Extract effective mass and confirm
the Coulomb model beats a viscous one (as it did for rotation, 0.918 vs 0.826 R²).

**Trap 1 — the accelerometer measures specific force**, including any gravity
component. Level check first; a small tilt appears as constant acceleration.
**Trap 2 — the platform rotates.** Accel is in the body frame; you must rotate into
the world frame using `θ`, or restrict tests to a held heading.

## Step 2.4 — Yaw coupling

**Concept.** Thrust-line offset from the CoM produces yaw the wheel must reject.
Quantify it now rather than discovering it as a mystery disturbance in Phase 7.

**Do.** Command each fan individually with heading control **active** and log the
wheel speed required to hold heading. That wheel effort is a direct measure of the
yaw disturbance torque.

**Verify.** Record disturbance torque per fan. If the wheel saturates holding
heading against one fan, the thrust line needs mechanical correction — no control
law fixes a persistent torque with a momentum-limited actuator.

**Phase 2 exit:** `git tag trans-p2-plantid`. Update `CONTROL_README` §3 with every
new constant and correct the rotation constants that moved.

---

# Phase 3 — Pi ↔ STM32 wired link

**Goal:** a framed, fail-safe serial link carrying pose from the Pi to the STM32.

**Decision B1: wired UART, not Bluetooth.** Bluetooth adds 10–100 ms of *variable*
latency and can drop out. A control loop's measurement path cannot tolerate either;
the estimator would need to model a jittery delay it cannot observe. The wire costs
three conductors between two boards already bolted to the same platform.

## Step 3.1 — Physical link

**Do.** **USART6 on PC6 (TX) / PC7 (RX)** — both free, and it keeps USART3 (HC-05)
and USART2 (ST-LINK VCP) intact.

```
Pi GPIO14 (TXD, pin  8) ──> STM32 PC7  (USART6_RX)
Pi GPIO15 (RXD, pin 10) <── STM32 PC6  (USART6_TX)
Pi GND    (pin 6)       ─── STM32 GND
```

**⚠️ Both are 3.3 V logic — do NOT connect the Pi's 5 V pin to anything on the
STM32.** Only TX, RX, and a common ground.

On the Pi: disable the serial console on `/dev/ttyAMA0` (it owns the UART by
default) and enable the hardware UART. **Do not skip this** — the console will
otherwise inject boot text into your protocol.

**Trap — ground.** The two boards need a common reference even though both are
powered from the same battery. Run the ground wire *in the same bundle* as TX/RX.
This is the same shared-impedance argument as the ESC signal ground.

## Step 3.2 — Protocol

**Concept.** The Phase-6 RTOS work established deferred interrupt processing:
ISR/driver buffers bytes, a task parses. The Pi link uses the same structure — this
is exactly what `commsTask` was built for.

**Do.** A binary framed packet, not text:
```
[0xA5][0x5A][len][payload...][crc16]
```
Payload: tag visible flag, relative bearing, range, tag id, Pi-side timestamp,
detection latency estimate.

**Requirements, all non-negotiable:**
- **CRC on every frame.** Reject silently and count.
- **Timestamp / latency field.** The estimator must know how old a measurement is;
  AprilTag detection on a Pi 3B+ is not instant.
- **Timeout with fail-safe.** No valid frame for N ms → mark pose invalid → the
  estimator coasts on IMU → after a longer timeout, safe-stop. **Never let a stale
  pose drive control.**
- **Never block the control task.** Queue, timeout 0, drop and count.

**Verify.** Frame rate, CRC error count, and measured round-trip latency all
reported in `G`. Unplug the Pi mid-run and confirm the fail-safe fires.

**Trap.** Do not reuse the HC-05 line-based ASCII parser. Binary framing needs
byte-level state machine handling, and a `0x0A` inside a payload must not be
mistaken for a line terminator.

**Phase 3 exit:** `git tag trans-p3-link`

---

# Phase 4 — Vision: AprilTag pose on the Pi

**Goal:** relative bearing and range to the target tag, streaming over the link.

**Decision B2: camera on the platform, tag on the target.** This gives *relative*
measurements, not absolute position — the estimator and controller work in
target-relative coordinates throughout, and there is no global frame.

## Step 4.1 — Camera calibration

**Concept.** AprilTag pose estimation needs intrinsics (focal length, principal
point, distortion). Uncalibrated, range will be systematically wrong and bearing
will be wrong off-axis.

**Do.** Standard OpenCV chessboard calibration, ≥20 images across the frame.
Store the matrix and distortion coefficients.

**Verify.** Reprojection error < 0.5 px. Then measure a tag at known distances
(0.3, 0.5, 1.0, 1.5 m) and check reported range against a tape measure.

**Trap.** Calibrate at the **resolution you will run at**. Intrinsics scale with
resolution, and a Pi 3B+ will not run AprilTag at full sensor resolution in real time.

## Step 4.2 — Detection rate on a Pi 3B+

**Concept.** This is a 2016 quad-core with 1 GB RAM. AprilTag detection is not free,
and **the achievable rate directly limits control bandwidth**.

**Do.** Benchmark `pupil_apriltags`/`apriltag` at candidate resolutions. Expect to
trade resolution for rate. Use `tag36h11`, decimation ≥ 2.

**Verify.** Record detections/sec and per-frame latency at each setting. **Pick the
lowest resolution that still gives reliable detection at maximum working range.**

**Trap 1.** Detection rate is not the same as *latency*. A 30 Hz pipeline with
100 ms of buffering is worse for control than 15 Hz with 30 ms. Measure end-to-end.
**Trap 2.** Motion blur during rotation will kill detection. Note the maximum
angular rate at which the tag is still detected — this bounds the SEARCH sweep speed.

## Step 4.3 — Pose extraction

**Do.** From the tag detection produce: **bearing** (angle from camera axis to tag),
**range** (distance), and **tag-face angle** (how obliquely you are viewing it —
needed to know whether you are approaching square-on, which matters for docking).

**Verify.** Static accuracy at several known poses; note noise σ on each quantity —
the estimator needs those as measurement covariances.

**Trap.** Range from a single tag is the **least** accurate quantity and degrades
with distance squared. Bearing is much better. Weight them accordingly in Phase 5.

**Phase 4 exit:** `git tag trans-p4-vision`

---

# Phase 5 — Estimator: fusing tag and IMU

**Goal:** a smooth, low-latency estimate of pose relative to the target that
survives tag dropouts.

**Concept.** `CONTROL_README` §11 already frames this: gyro is fast but drifts
(0.8°/min after bias removal); vision is absolute but slow, latent, and drops out.
They fail in complementary ways, which is exactly when fusion helps.

Now there is more to fuse: gyro Z, accelerometer X/Y, wheel speed, and tag
bearing/range.

## Step 5.1 — Complementary filter first

**Do.** Start simple, as the rotation axis did:
```c
theta = 0.98f*(theta + w_p*dt) + 0.02f*theta_vision;
```
Extend to bearing and range with IMU-integrated motion for prediction.

**Verify.** Estimate stays smooth across a deliberate tag occlusion of 1–2 s and
re-converges without a jump when the tag returns.

**Trap.** Naive complementary filtering **cannot handle measurement latency**. If
the tag pose is 100 ms old, blending it against the current prediction injects an
error proportional to velocity. Either compensate using the timestamp from Phase 3,
or keep the vision weight low enough that the error is tolerable — and *know which
you chose*.

## Step 5.2 — Kalman filter

**Concept.** The upgrade the rotation axis never got. State:
```
[ x, y, theta, vx, vy, omega, b_gyro ]     (target-relative)
```
Predict with IMU (accel + gyro), correct with tag bearing/range. Estimating
`b_gyro` **as a state** makes it self-calibrating against thermal drift rather than
relying on the boot-time measurement.

**Do.** Derive process and measurement models from the Phase 2 identification —
**do not guess covariances.** Process noise comes from measured IMU noise; the
measurement covariance from the Phase 4.3 σ values.

**Verify.**
- Innovation sequence is zero-mean and white. **A biased innovation means your model
  is wrong — fix the model, not the covariance.**
- Estimate is stable through dropouts.
- Compare against ground truth (tape measure at known poses).

**Trap 1.** The Kalman filter runs on the STM32 in float. A 7-state filter needs
7×7 matrix operations at the control rate — **budget the CPU before writing it.**
You have 27% idle; measure the actual cost with `TIME_BLOCK` and re-run the Phase-7
schedulability analysis afterwards.
**Trap 2.** Tag measurements arrive at ~10–30 Hz into a 200 Hz loop. Run predict
every cycle and correct only when a fresh measurement arrives. Never re-apply the
same measurement twice — an easy bug that produces overconfident, drifting estimates.

**Phase 5 exit:** `git tag trans-p5-estimator`

---

# Phase 6 — Translation control (LQR)

**Goal:** commanded x/y motion under closed-loop control, heading held by the
existing wheel controller.

**Decision B4: LQR first.** Three decoupled double integrators, exactly as
`CONTROL_README` §18 proposed. You already have the derivation and the tuning
method from the wheel axis.

## Step 6.1 — Plant model and gains

**Concept.** After the square-law inversion (`throttle = sqrt(F/F_max)`), each
translational axis is a double integrator with Coulomb friction:
```
m·ẍ = F_x − F_c·sign(ẋ)
```
Structurally identical to the rotation axis — which means **the same feedforward
architecture applies**: cancel friction when moving, push through it when stuck,
with a deadzone so the integrator does not wind up.

**Do.** Design LQR gains per axis from the Phase 2 constants. Same as rotation:
```
K_p = ω_n²·m        K_d = 2ζω_n·m        ω_n = 5.714 / t_settle
```

**Trap — work DOWN the settle-time table, not up** (CONTROL_README §9). Slow gains
keep feedforward active longer, which is worse on *both* accuracy and saturation.
This was counterintuitive and confirmed on hardware for rotation.

## Step 6.2 — Coulomb feedforward

**Concept.** §6 established this is not optional: friction is comparable to the
maneuver. **The two branches have opposite signs** and getting the moving branch
backwards is *worse than no feedforward at all*.

**Do.** Port the structure verbatim from the rotation axis, with translational
constants:
```c
if (|e| > deadzone) {
    ff = (|v| > V_MOVING) ? -F_FRICTION*sign(v)     // MOVING: cancel
                          : +F_FRICTION*sign(F);    // STUCK:  break free
    F += ffFrac * ff;
}
```

**Verify.** Small commanded moves (2–5 cm) from rest, repeated 10×. **This is the
discriminating test** — large moves succeed even with badly wrong feedforward
because the proportional term covers the shortfall. Exactly the lesson from §13.9.

## Step 6.3 — Fan allocation

**Concept.** The genuinely new problem. Fans are **unidirectional**: an opposing
pair can produce ±F only if both idle above zero.

**Do.**
```
F_desired  →  opposing pair (A, B) on that axis
A = idle + max(0,  F/2)
B = idle + max(0, -F/2)
```
`idle` must exceed the commutation floor (~2% unloaded, higher with props) or the
motor stops and takes time to restart.

**Verify.** Sweep commanded force through zero and confirm the transition is smooth,
with no dead spot and no motor stopping.

**Trap 1.** Idle bias costs power **continuously** and produces heat. Measure the
current draw at idle bias and check the fuse margin.
**Trap 2.** The commutation floor with props fitted is higher than the 2% measured
bare. Re-measure it loaded.

**Phase 6 exit:** `git tag trans-p6-translation`

---

# Phase 7 — Combined 3-DOF control

**Goal:** x, y, and θ controlled simultaneously.

## Step 7.1 — Frame transform

**Concept.** Fans are fixed to the platform and push in **body** axes. Control
commands are in **target-relative** axes. The rotation between them is `θ`, which
the estimator provides.

**Do.**
```
[F_bx]   [ cos θ   sin θ ] [F_wx]
[F_by] = [-sin θ   cos θ ] [F_wy]
```

**Trap.** Sign errors here are the single most likely bug in the phase, and they
produce *plausible-looking* motion in the wrong direction — exactly like the two
sign traps documented in §8. **Verify open-loop first**: command a pure +x world
force at several fixed headings and confirm the platform always moves the same
world direction.

## Step 7.2 — Cross-coupling

**Concept.** The axes are not truly independent: fan thrust offset from the CoM
yaws the platform (measured in Phase 2.4), and wheel acceleration disturbs heading
during translation.

**Do.** Run all three loops together. The wheel controller rejects yaw disturbance
as it already does; the question is whether it saturates.

**Verify.** Translate 30 cm while holding heading. Log wheel speed throughout —
**if `ω_w` walks toward the 45 rad/s abort, the yaw disturbance exceeds what passive
desaturation can absorb** and the thrust lines need mechanical correction.

**Trap.** Passive desaturation is *already* not completing (see "the retune
question"). Adding a persistent yaw disturbance on top will surface that quickly.
Watch `ω_w` on every translation test.

## Step 7.3 — CPU and schedulability

**Do.** Re-run the Phase-7 RTOS analysis with the estimator, allocation, and three
control axes added. Update `U`, the WCET table, and the response-time analysis.

**Verify.** `ctrl period` MAX still ≈ 5006 µs. If control-law compute has grown
enough to threaten the 5 ms budget, **reduce the control rate deliberately** rather
than letting deadlines slip — and re-derive the gains for the new rate.

**Phase 7 exit:** `git tag trans-p7-3dof`

---

# Phase 8 — Acquisition and docking

**Goal:** the full autonomous sequence.

## Step 8.1 — State machine

**Do.** Implement as an explicit state machine, not implicit flags:
```
SEARCH → ACQUIRE → APPROACH → ALIGN → DOCK → DOCKED
                 ↖ (tag lost, any state) ↙
```
Every state needs an explicit **timeout** and a **loss-of-tag transition**. The
rotation controller's `MAX_STALL_RETRIES` lesson applies: **without a retry cap,
a controller will cycle forever.** Cap every retry and report `PARKED`-style status.

## Step 8.2 — Search

**Do.** Rotate at a fixed slow rate until the tag is detected. The rate is bounded
by the motion-blur limit measured in Phase 4.2.

**Verify.** Reliable acquisition from random starting headings, 10 trials.

## Step 8.3 — Approach and terminal alignment

**Concept.** The rotation axis taught that **the terminal approach is the hard
part**, not the large maneuver. Expect the same here — and expect the tag to leave
the field of view at close range.

**Do.** Two-stage tolerance, exactly like the rotation deadzone: coarse while far,
tighten as range closes. Handle tag loss at close range by coasting on the IMU
estimate for the final few centimetres.

**Verify.** 10 docking attempts from random poses. Record success rate, final
lateral error, final heading error, and time to dock.

**Trap.** **Never let a stale pose drive the terminal approach.** If the tag is lost
and the estimate is coasting, the uncertainty grows — enforce a hard time limit on
dead-reckoned closure and abort past it.

**Phase 8 exit:** `git tag trans-p8-docking`

---

# Phase 9 — Consolidation and evidence

**Do.**
1. Right-size stacks for the new tasks (`uxTaskGetStackHighWaterMark`, ≥3× margin).
2. Re-run `U`: per-task CPU and idle headroom.
3. Re-run the schedulability analysis with all tasks and the estimator.
4. Update `CONTROL_README.md`: translation plant constants, the 3-DOF architecture,
   the allocation scheme, the docking state machine.
5. Update `claude.md` §6 status.
6. Capture the artifacts: docking success rate, a tracer Gantt with all tasks, the
   estimator innovation plot, thrust curves, breakaway measurement.

**Be honest about what was not measured.** The RTOS guide's Step 7.5 marked several
artifacts "deferred with reasons" rather than fabricating them, and that made the
write-up more credible, not less. Do the same here.

**Phase 9 exit:** `git tag trans-p9-complete`

---

# Appendix A — Trap quick reference

| # | Trap | Symptom |
|---|---|---|
| T1 | ESC is Bluejay — **DSHOT only** | Perfect servo PWM produces nothing; no calibration beeps |
| T2 | DSHOT channels on different GPIO ports | Adding a channel breaks previously-working ones |
| T3 | Backwards prop | ~half thrust; needs ~75% where 50% should do |
| T4 | Bit-banged DSHOT in the RTOS | ~212 µs interrupt blackout wrecks the 250 µs FOC tick |
| T5 | TIM1 MOE not set | Registers perfect, pin never moves |
| T6 | DC meter to verify PWM | Confirms duty, **not** pulse width — 5% @50 Hz and @5 kHz read alike |
| T7 | Theorising instead of testing the user's suggestion | Cost hours on the channel-4 problem; the user's "try another port" was right |
| T8 | Arduino `delay()` in a task | Silently starves all lower-priority tasks |
| T9 | Stale pose driving control | Estimator coasts, error grows, terminal approach fails unsafely |
| T10 | Kalman measurement applied twice | Overconfident, drifting estimate |
| T11 | Frame-transform sign error | Plausible motion in the wrong direction — verify open-loop |
| T12 | Idle bias below the commutation floor | Motor stops mid-manoeuvre and must restart |
| T13 | No retry cap in the state machine | Infinite search/approach cycling — the `MAX_STALL_RETRIES` lesson |
| T14 | Calibrating the camera at the wrong resolution | Systematic range error |
| T15 | Fuse/wire sized for low throttle | Prop current goes as throttle³; four fans at 75% ≈ 20 A |
| T16 | **Props are UNGUARDED (B7)** | There is no mechanical stop between a hand and a 25,000 RPM disc. Any test that arms the fans with the platform within reach is a hazard, not an inconvenience. Props off unless the test needs thrust; `FAN_THROTTLE_MAX` compiled in; battery disconnect is the e-stop. |
| T17 | **`vTaskDelayUntil` "catching up" after starvation** | Returns immediately once per missed period, firing a burst of back-to-back iterations. For a periodic *emitter* like `fanTask` that burst aborts its own in-flight DMA transfers — 99 corrupt DSHOT frames inside the `hwSetup()` window. Resync whenever the deadline has already passed: a missed frame is gone, not owed. Sibling of RTOS Trap 20 (the watchdog version of the same bug). |
| T18 | **`telem_print()` from a second task during boot** | Before `telem_activate()` it writes the port *directly from the calling task*. Any task printing during `hwSetup()` becomes a second concurrent writer into non-reentrant `HardwareSerial` — invariant B15 / RTOS Trap 17. Gate on `telem_isActive()` and defer. It will interleave harmlessly for a long time before it doesn't. |
| T19 | **Trusting a number you did not read off the terminal** | The 1.2 result note recorded `overruns=0` from a verbal "everything checked out"; it was 99 and had been since boot. A wrong number in the guide is worse than no number — the next session trusts it and stops looking. |

Plus **every trap in `RTOS_migration.md` Appendix A** — the one-writer/one-reader
invariants, `delay()`, watchdogs measuring iterations instead of time, and the rest.

---

# Appendix B — Decision log

| # | Decision | Reasoning |
|---|---|---|
| **B1** | **Pi ↔ STM32 = wired UART (USART6, PC6/PC7)** | Bluetooth adds 10–100 ms *variable* latency and can drop out. A measurement path feeding a control loop cannot tolerate unobservable jitter. Both boards are on the same platform, so the wire is three conductors. USART6 chosen because USART3 is the HC-05 and USART2 is the ST-LINK VCP. |
| **B2** | **Camera on platform, tag on target** | Gives relative bearing + range. Chosen over an overhead camera because it needs no off-platform infrastructure and matches the docking mission (approach a target you must first find). Cost: no absolute position, must search to acquire, tag lost at close range. |
| **B3** | **All control on the STM32; Pi = pose sensor** | Keeps the deterministic-timing story intact and means a Pi hang cannot destabilise control — it degrades to IMU coasting and then a safe stop. Revisit only if MPC genuinely needs the Pi. |
| **B4** | **LQR first, MPC later** | Three double integrators; the machinery and tuning method already exist from the wheel axis. Gives a working baseline before adding complexity. MPC earns its cost specifically at *terminal docking constraints* and *unidirectional allocation* — add it on the Pi when those bite, per CONTROL_README §17. |
| **B5** | **Fan ch4 moves PA0 → PA11 for the DMA rewrite** | TIM1_CH1–CH4 are PA8/9/10/11: one timer, one DMA stream, all on GPIOA — which satisfies the same-port constraint (T2) by construction. PA0's timers (TIM2/TIM5) are the motor PWM and µs timebase. |
| **B6** | **Rotation retune folded into Phase 2** | The user chose to skip a separate rotation retune. Phase 2 must re-identify mass and friction for translation regardless, so rotation constants come along for free. Known open items meanwhile: passive desaturation incomplete, `A_FRICTION` never swept. |
| **B7** | **Prop guards skipped — props stay exposed; safety moved into firmware** | 2026-08-13, user decision. Build time for guards/ducts/a bumper ring was not available, and the alternative was blocking all translation work indefinitely on a fabrication task. Risk flagged once and accepted: four unguarded 4-inch 3-blade props at hand height on a chassis that gets picked up by hand. **The tradeoff is explicit — mechanical containment is replaced by removing the reasons to touch an armed platform:** (1) `FAN_THROTTLE_MAX` compiled in at 30% for bring-up, clamped inside `fans_setThrottle()` so a runaway control law cannot exceed it either; (2) props physically off for any test that does not need thrust — DSHOT/DMA/register verification and all fault-path provocation; (3) battery disconnect is the e-stop, serial `X` is the convenience; (4) `fans_stopAll()` drives pins low as GPIO in the hardware-kill step, never via the already-dead DMA path. Items 1–4 are **binding requirements on Phase 1**, not advice. Cost also noted honestly: a duct would have slightly *increased* static thrust by cutting tip losses, so this trades a small thrust margin away as well. Revisit only if the platform starts being handled by people other than the user. |
| **B8** | **Step 1.1 split: prove TIM1+DMA standalone (`fan_dma_test.cpp`, env `fandma`) before integrating** | 2026-08-13. The guide's Step 1.1 Verify mixes a driver question ("each channel spins via the new path") with an RTOS question ("`M` shows FOC tick still 238–261 µs"). Split them: 1.1 proves the driver alone, 1.2 integrates into `fans.*`/`fanTask` and runs the `M` gate there. Reasoning: `fan_test.cpp` is the confirmed-good bit-bang reference and the verbatim rule says don't edit it, so the DMA path needs its own file regardless. More importantly a new DMA driver has ~6 independently-configurable register blocks (GPIO AF, TIM timebase, CCMRx/CCER, BDTR/MOE, DCR burst window, DMA2_S5) and **every one of them fails identically — the pin doesn't move.** Bisecting that with the wheel, FOC, telemetry and five tasks in the picture is strictly harder than bisecting it alone, and the same board can then A/B bit-bang vs DMA by changing `-e`. Cost: one throwaway file + one env. Accepted downside: the FOC-noninterference gate defers to 1.2 — acceptable because it is inherently an RTOS measurement that cannot be taken standalone. Also decided: **no DMA interrupt at all** — the frame is re-armed by polling `DMA_SxCR.EN`, which sidesteps the STM32duino strong-`IRQHandler` collision trap (Appendix A #2 / RTOS trap 13b) rather than working around it. |
| **B9** | **Power trip thresholds NOT changed for the fans — the guide's Step 1.3 instruction was wrong** | 2026-08-13. Step 1.3 said to "extend the power trip thresholds, since four fans change the current envelope completely." That is based on a false premise: **the INA219 is on the wheel supply, not the fan supply** (`CONTROL_README` §1 "bus voltage and current on the motor supply"; this guide's own inventory "wheel-supply V/I"). Consequences: (a) the **2500 mA current trip cannot see fan current at all**, so raising it would not extend any protection — it would only weaken the wheel's own overcurrent trip in exchange for nothing; (b) the **10.0 V undervoltage trip *does* see fan draw indirectly**, because the fans share the battery and their current sags the pack. Whether four fans at 30% pull the bus under 10.0 V depends on pack internal resistance and state of charge, **neither of which has been measured**. Picking a new number now would be exactly the mistake `safety.cpp`'s own comments warn against ("a wrong guess means nuisance safe-stops on a spinning flywheel"). **Both thresholds left as-is; undervoltage re-measurement moved to Phase 2**, alongside the thrust curve, where fans will finally be drawing real current. One good side effect banked now: an undervoltage trip goes through `stopMotor()`, which after this step stops the wheel **and** the fans — a collapsing pack should stop everything, and until now it would not have. |
| **B10** | **Fan kill runs BEFORE the wheel kill in every fault path, via a registered hook** | 2026-08-13. `faults_halt()` now calls a `faults_setHwKillHook()` function pointer immediately after masking interrupts and *before* pulling the DRV8313 enable low. Ordering: with the props unguarded (B7) the fans are the larger hazard, and if only one actuator kill ever completes it should be that one — both are a handful of register writes, so the ordering costs nanoseconds either way. Implemented as a **function pointer rather than `#include "fans.h"` in `faults.cpp`** deliberately: `faults.h` is C-compatible and dependency-free *because* `FreeRTOSConfig.h` (a C header pulled into the kernel's C sources) names `rtAssertFail` and `rtRunTimeCounter`, and including a C++ Arduino module there would break that property. The hook contract is "safe with interrupts already disabled" — no FreeRTOS API, no lock, no unbounded wait — which is what `fans_stopAll()` was built to. Guarded by `s_hwReady` so a fault firing before `fans_init()` cannot poke TIM1 with its APB clock still gated. **`stopMotor()` uses the HARD kill, not throttle-to-zero**, so recovery needs `R` and a ~1 s re-arm; a soft stop would leave the ESC armed and still depending on the DMA path to keep working, which is the wrong guarantee for a stop path. |
| **B11** | *(next decision goes here)* | |

---

# Appendix C — Target architecture

```
                   ┌──────────────── Raspberry Pi 3B+ ────────────────┐
                   │  camera → AprilTag detect → bearing/range/latency │
                   └──────────────────────┬───────────────────────────┘
                                          │ USART6, framed + CRC, ~10-30 Hz
                                          ▼
  ┌───────────────────────────── STM32 F446RE ─────────────────────────────┐
  │  focTask   (4) 4 kHz   commutation                                      │
  │  controlTask(3) 200 Hz  IMU → estimator → 3-DOF LQR → allocation        │
  │  safetyTask (2) 20 Hz   wheel + fans + power + heartbeat                │
  │  commsTask  (2) 2 ms    HC-05 operator + Pi pose link                   │
  │  fanTask    (2) 2-4 ms  DSHOT via TIM1 + DMA  (sole fan writer)         │
  │  telemTask  (1) event   sole serial writer                              │
  └────────────────────────────────────────────────────────────────────────┘
            │                          │                        │
       reaction wheel            4× fans (ESC)            MPU6050 / INA219
```

---

# Appendix D — Understanding check

An agent or human should be able to answer these before writing control code:

1. Why can servo PWM never work with this ESC, no matter how accurate?
2. Why must all four DSHOT channels share a GPIO port and a timer?
3. Why is the bit-banged DSHOT unusable in the RTOS firmware, quantitatively?
4. Why does a DC voltmeter fail to verify a PWM signal's validity?
5. Why do the two Coulomb feedforward branches have opposite signs?
6. Why are fans unidirectional a *control allocation* problem rather than just a
   sizing problem?
7. Why does the estimator need the measurement's *timestamp*, not just its value?
8. Why is the terminal approach harder than the large maneuver — on both axes?
9. Why is `telemTask` the lowest priority, and why is that safe?
10. What happens if the Pi stops sending, at each of 100 ms, 1 s, and 10 s?
