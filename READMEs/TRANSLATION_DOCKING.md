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

**Phases 0, 1, 2 and 3 are COMPLETE. Phase 4 is IN PROGRESS — Step 4.2
(detection rate) is DONE; Step 4.1 (calibration) is NEXT.**
Tags: `trans-p1-fans`, `trans-p2-plantid`, `trans-p3-link`.

**Vision pipeline measured and chosen (2026-08-20):** 1280×720 MJPG, grayscale
decode, `decimate=2.0`, threaded capture → **44.9 ms capture→pose latency,
14.8 fps, 3 of 3 tags every frame** (B22/B23). Detector is Debian's
`python3-apriltag`. **⚠️ Camera exposure MUST be locked manually before
calibrating** — auto-exposure caps the rate at 15 fps, implies ~15 px of blur at
30 °/s, and changes effective focal length so any calibration taken with it
enabled is invalid (T35).

**⚠️ Hardware changed:** the Pi 3B+ died mid-phase (PMIC failure — 3.3 V rail
absent, cold, would not boot from SD or USB). Replaced with a **Pi 3A+**:
identical BCM2837B0 at 1.4 GHz, so identical detection performance; 512 MB RAM,
one USB port, no Ethernet, ~1/5 the current draw. Nothing architectural changed.
`pupil-apriltags` cannot be built on 512 MB — use the Debian package.
**`tools/pi_setup.sh` rebuilds a fresh Pi in one command.**

**The Pi link is up, framed, and fail-safe.** USART6 PC6/PC7, Pi moved onto the
PL011 rather than the mini-UART (**T28** — the one that would otherwise have bitten
in Phase 4, as framing errors appearing exactly when AprilTag loads the CPU).
`pi_link.*` is sole reader *and* sole writer of the port, polled from `commsTask`
and deliberately outside the operator parser (**B18**).

```
wire     [0xA5][0x5A][len=28][payload][crc16]   CRC-16/CCITT-FALSE, little-endian
payload  seq, flags, tag_id, range_m, bearing_rad, relyaw_rad, quality,
         age_us, n_tags                          POLAR not Cartesian (B19)
                                                 AGE not a timestamp (B20)
ladder   250 ms -> pose invalid, estimator coasts
         1 s    -> fans zeroed
         3 s    -> terminal hook (NOT stopMotor yet -- B21)
```

**Verified 2026-08-19:** RTT 2.61/3.52/5.32 ms (3.1); 62 frames / 2508 B
reconciling exactly with 14 CRC rejects and 26 gaps, `badlen=0 resync=0`, pose
decoding correctly, ladder reaching DEAD and zeroing the fans (3.2). **Tested
with synthetic frames — no camera involved yet.**

**Both plants are identified and the rotation axis is retuned and working:**

```
TRANSLATION (acceleration units -- mass and force cancel, B14)
  x_ddot = A(throttle) - A_c*sign(v)
  A(throttle) = 2.1e-4 * pct^2  m/s^2      A(60%) = 0.76
  A_c         = 0.26 m/s^2                 breakaway ~35% throttle
  GO/NO-GO    = 2.9x single fan, 4.2x diagonal   -> PASS

ROTATION (re-identified 2026-08-15, retuned 2026-08-19)
  A_1 47.9   A_2 4.97   a 0.098   K' 9.64   compFrac 0.90
  K_theta 216   K_omega 52  (zeta 0.54, NOT 0.7 -- friction already damps)
  ffFrac 0.95   A_static 60   A_moving 34   A_viscous 0
  deadzone 1.5/0.8 deg   ALPHA_STALL_MAX 70   WHEEL_SAT_LIMIT 55
  --> 0.47 deg mean over +-5 to +-180 deg, EVERY slew in one move,
      wheel returns fully to rest
```

**Yaw coupling (2.4):** thrust-line offset, 2–3 rad/s², wheel peaks 4–12 rad/s.
Small enough that **no mechanical correction is needed** — the Phase-7 saturation
warning is retired.

**Firmware state:** six FreeRTOS tasks. `fanTask` (prio 2, 333 Hz) is the sole fan
writer; fans are killed ahead of the wheel in every fault path; fan 4's spin direction
is re-applied automatically at every boot because this ESC ignores SAVE_SETTINGS.
Control loop measured at 4999/5000/5001 µs throughout — fans cost it nothing.

**Data:** every raw capture is in `calibration/runs/`, renamed by what it established
and indexed in `calibration/runs/INDEX.md`. `capture_calibration.py` writes new runs
there automatically.

## What is NOT done

| item | state |
|---|---|
| **Fan guards** | ⚠️ **SKIPPED — props are EXPOSED and will stay that way** (B7). Mitigated in firmware, not mechanically. |
| Pi ↔ STM32 link | ✅ **COMPLETE** (Phase 3, 2026-08-19) — framed, CRC'd, 3-tier fail-safe, verified with synthetic frames. ⬜ **No real pose yet** — nothing has looked through the camera; that is Phase 4. |
| Tier-2 fan cutout | ⚠️ **uses the hard kill, which latches until `R`.** Fine now, **wrong for docking** — B2 loses the tag at close range, so a 1 s dropout would strand the approach. Phase 6 must make it a soft zero. |
| AprilTag detection | not started (Phase 4) |
| Any translation CONTROL | not started (Phase 6) — the plant is identified, no controller written |
| Fan 2 thrust | ⚠️ produced no measurable thrust in the 2.4 runs. **Spin it and watch before Phase 7 relies on it.** |
| ESC current sense | wired to A4 and reading, but only 0–3 counts at 10-bit. Needs `analogReadResolution(12)` + oversampling to be useful for channel matching. |

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
   **PA8/PA9/PA10/PA11** — ch4 moved off PA0 in Phase 1, B5). Putting channel 4 on
   PC7 forced either a port parameter or a
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
| 1 fans into the RTOS (DSHOT + DMA) | ✅ **(2026-08-13)** `trans-p1-fans` | Four DSHOT channels on TIM1+DMA burst, `fanTask` prio 2 = sole fan writer, fans in every fault path *ahead* of the wheel, `S`/`L` manual commands, 30% ceiling + 10 s dead-man. Control loop untouched: ctrl period 4999/5000/5001 µs. |
| 2 translation plant ID | ✅ **(2026-08-19)** `trans-p2-plantid` | `A(pct)=2.1e-4·pct²`, `A_c≈0.26`, **go/no-go 2.9× PASS**. Rotation re-identified + retuned: **0.47° mean, ±180° in one move, wheel returns to rest**. Yaw coupling is thrust-line offset, small (2–3 rad/s²) — no mechanical correction needed. |
| 3 Pi ↔ STM32 wired link | ✅ **(2026-08-19)** `trans-p3-link` | USART6 PC6/PC7, Pi forced onto the PL011 (T28). `pi_link.*` is sole reader *and* writer of the port, polled from `commsTask`, deliberately outside the operator parser (B18 — verified: `comms rx=4` against 2508 Pi bytes). Framed `[A5 5A][len][28B][crc16]`, polar pose + `age_us` not a timestamp (B19/B20). 3-tier staleness ladder, tier 3 not wired to `stopMotor()` yet (B21). 62 frames / 2508 B reconcile exactly. ⚠️ tier 2 hard-kills the fans — Phase 6 must make it a soft zero. |
| 4 vision: AprilTag pose on the Pi | 🔶 **IN PROGRESS** | **4.2 ✅ (2026-08-20)** — 720p MJPG grey / `decimate=2.0` / threaded: **44.9 ms capture→pose, 14.8 fps, 3/3 tags** (B22, B23). Pi 3B+ died mid-phase (PMIC), replaced by a **3A+** — same SoC and clock, so same performance. Detector is Debian `python3-apriltag`. ⚠️ **Lock exposure before 4.1** (T35) — auto-exposure caps rate at 15 fps, blurs ~15 px at 30 °/s, and invalidates any calibration taken with it on. **4.1 calibration ⬜ NEXT, 4.3 pose extraction ⬜.** |
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
| **PA8** | **D7** | **fan ch1 DSHOT** | TIM1_CH1, AF1 + DMA2_S5 burst |
| **PA9** | **D8** | **fan ch2 DSHOT** | TIM1_CH2, AF1 + DMA2_S5 burst |
| **PA10** | **D2** | **fan ch3 DSHOT** | TIM1_CH3, AF1 + DMA2_S5 burst |
| **PA11** | **CN10-14** | **fan ch4 DSHOT** | TIM1_CH4, AF1 + DMA2_S5 burst (**moved from PA0/A0 in Phase 1 — B5**) |
| PA0 | A0 | *(free — was fan ch4 before B5)* | — |
| PC1 | A4 | ESC current sense | **WIRED and read** — `ESC_CUR_PIN`, `analogRead` at 20 Hz on `controlTask`, logged as `cap_cur`. ⚠️ 10-bit, reads only 0–3 counts; needs `analogReadResolution(12)` + oversampling to be useful |
| **PC6** | **CN10-4** | **Pi link TX** | USART6_TX → Pi GPIO15 (pin 10). **WIRED 2026-08-19.** ⚠️ `CN10-8` = **U5V**, three positions along the same column — meter the pin before connecting the Pi |
| **PC7** | **D9** | **Pi link RX** | USART6_RX ← Pi GPIO14 (pin 8). **WIRED 2026-08-19.** PC7's old DSHOT-ch4 warning is retired: ch4 is on PA11 |

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

## Firmware architecture (six FreeRTOS tasks)

| task | prio | period | owns |
|---|---|---|---|
| `focTask` | 4 | 250 µs (4 kHz) | `loopFOC()` + `move()` — commutation only |
| `controlTask` | 3 | 5 ms (200 Hz) | gyro read, control law, capture, command execution |
| `safetyTask` | 2 | 50 ms (20 Hz) | wheel overspeed, heartbeat watchdog, INA219 power |
| `commsTask` | 2 | 2 ms poll | serial RX + line assembly — **sole reader** |
| `fanTask` | 2 | 3 ms (333 Hz) | DSHOT300 on TIM1 + DMA burst — **sole fan writer** (added Phase 1; period 2→3 ms by B13) |
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

**Result (2026-08-13) — PASSED.** Two commands, both on previously-free letters:

| send | does |
|---|---|
| `S<n>` | select fan channel **1–4**, or **`S0` = all four** (routes via `fans_setAll`, one frame) |
| `L<pct>` | throttle the selection, 0–`FAN_THROTTLE_MAX`. **`L0` = fans off but still armed** |

`L` echoes the **applied** percentages, not the requested ones, so the clamp is visible
rather than hidden. Verified on hardware: per-channel and all-four throttle both work,
`L50` echoes `30.0 / 30.0 / 30.0 / 30.0` (the B7 ceiling holding), `L0` stops without
disarming, `X` while spinning gives `fans: KILLED` with frames frozen and `L` then
refuses until `R`.

**Scope call — the thrust-VECTOR command is deferred to Phase 6.3.** Converting a
desired force to throttle needs `throttle = sqrt(F/F_max)`, and `F_max` comes from the
Step 2.1 thrust curve, which does not exist yet. Building it now would mean inventing a
calibration constant and putting allocation in a bring-up command instead of next to
the opposing-pair idle-bias logic where it belongs. Raw throttle is precisely what
Phase 2 needs in order to *measure* that curve.

**Added beyond the guide: a dead-man timeout on commanded throttle**
(`FAN_CMD_TIMEOUT_MS`, 10 s). Manually-commanded throttle auto-zeros unless refreshed.
With the props unguarded (B7) a fan left spinning because the operator got distracted
is exactly what a guard would have covered, and this is the cheapest substitute. **It
costs nothing in closed loop** — a controller calls `fans_setAll()` every cycle, which
pets it continuously — so it can only fire on a manual command nobody is tending.
⚠️ Phase 2.1's thrust sweep will need to either refresh within 10 s per step or raise
this.

## The overrun investigation — worth reading before touching `fanTask`

Step 1.4 first reported **1,565 overruns in 70,111 frames (2.2%)**, against 0 in the
1.3 run. **The subsystem was fine; the instrumentation was wrong.** Recorded in full
because the diagnostic method is the reusable part.

Two hypotheses were formulated, and deliberately **not** acted on, because they
predicted different observable values: a stopped timer would leave `NDTR` **large**,
while a stale-DMA-request desync would leave it **small (1–3)**. Adding two register
reads at the failure point cost one flash cycle and settled it — instead of an argument
(the T7 lesson, applied to my own reasoning rather than the user's).

**Measured: `NDTR last=8 min=4 max=16` of 72, `CEN=1`.** Both hypotheses dead. The
timer was running and the transfer was 1–4 bursts from completion — the multiples of 4
confirming bursts are atomic as designed. **The frame was simply still legitimately in
flight**, checked ~47–57 µs into its 60 µs life.

**The real mechanism:** `vTaskDelayUntil` schedules from the **wake time**, but a frame
is emitted when the task actually **gets the CPU**, and those differ by up to a tick —
`controlTask` (prio 3) holds the CPU ~2.4 ms of every 5 ms for the MPU read, and
`commsTask` shares priority 2.

```
last = 102, released at tick 102
  controlTask holds the CPU; fanTask actually runs at true t = 103.95
  frame emitted 103.95, finishes 104.01
  loop: nowT = 103, last = 102 -> diff = 1 tick -> resync does NOT fire (needs >= 2)
  vTaskDelayUntil -> next wake = tick 104 = true 104.00
  next emission 50 us after the last -> previous frame still clocking
```

The Step-1.3 resync guard cannot see this: the backlog is one tick, which looks
healthy. It also explains why the rate tracked **command traffic** rather than throttle
— `G` and each `L` wake `commsTask` and `telemTask`, which is what generates the jitter.

**Two fixes.** (1) Gate on **elapsed microseconds from the TIM5 timebase**, not tick
arithmetic — 1 ms granularity structurally cannot see a 50 µs reality, which is why
this looked mysterious. A frame emitted inside `FRAME_MIN_GAP_US` is now a **skip**:
benign, expected, counted separately. `overruns` now means "still not drained after
100 µs genuinely elapsed" — a real fault, and it should be 0. (2) `FAN_FRAME_MS` 2 → 3,
turning ~−10 µs of margin into ~600 µs. 333 Hz is an order of magnitude above what the
ESC needs (Bluejay disarms after ~250 ms of silence). `FAN_ARM_FRAMES` 500 → 350 to
keep the arming ramp at ~1 s.

**Confirmed after the fix: `overruns=0`.** `skips` is non-zero and expected — watch the
*rate*, never expect zero. *(Figure not transcribed — per T19, no number goes in this
guide that was not read off the terminal.)*

**The honest summary:** steps 1–6 passing was the real signal all along, and the counter
was the noise. A counter that conflates a structurally-expected scheduling artifact with
a fault is worse than no counter, because it burns a debugging session and trains you to
ignore it.

**Phase 1 exit:** `git tag trans-p1-fans`

---

# Phase 2 — Translation plant identification

**Goal:** the numbers translation control cannot be designed without. This is the
`CONTROL_README` §14 method applied to a new axis, and it also re-identifies the
rotation axis for free.

**None of the rotation constants transfer.** `A_1`, `A_2`, `compFrac`, `K_HOLD` are
all properties of the wheel loop and have no fan analogue.

> ### ⚠️ THIS PHASE WAS REFORMULATED — read B14 before following the steps below
>
> Steps 2.1–2.3 as originally written measure **force** (scale rig, string-and-pulley,
> effective mass). That was abandoned: the motors cannot easily be dismounted, and more
> importantly **the controller never needs force or mass.** Working in acceleration
> units, `ẍ = A(throttle) − A_c·sign(v)`, makes `m`, `F` and `F_c` cancel out — exactly
> as `CONTROL_README` §3 avoided ever pinning down `J_w` or `J_p`.
>
> | step | as written | as actually done |
> |---|---|---|
> | 2.1 | thrust in gf, scale rig | **`A(throttle)` in m/s², from the IMU** ✅ |
> | 2.2 | breakaway in gf, string + weights | **`A_c` from motion-onset throttle** ✅ |
> | 2.3 | effective mass from step response | **DROPPED** — was only ever a route to `A` |
> | 2.4 | yaw coupling | unchanged, still to do |
>
> The go/no-go is unaffected: `F_thrust/F_breakaway` becomes `A_max/A_c`, the identical
> ratio. **Do not "restore" the force measurements** — they were removed deliberately.

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

**Result (2026-08-14) — DONE, in acceleration units (B14). 66 runs over 5 sessions.**

Method: `I<pct>` commands an automatic thrust step on the `S`-selected fan — 500 ms
quiet, 1000 ms thrust, 2500 ms coast — logging body-frame accel at 200 Hz. `A` is
extracted as the **vector difference between the thrust-phase and coast-phase mean
acceleration**, which is the key trick: `a_thr = A − A_c`, `a_cst = −A_c`, so
`A = a_thr − a_cst` and **any constant accelerometer bias cancels identically.**

Analysed in the **body frame, never rotated by θ** — a fan is bolted to the platform
and so is the IMU, so thrust is body-fixed and heading is irrelevant (absent tilt, B15).

```
fan 1, pooled, moving runs only:
  pct   n    A (m/s^2)         k = A/pct^2
   50  14   0.495 +/- 0.148     1.98e-4
   55   5   0.678 +/- 0.203     2.24e-4
   60   5   0.762 +/- 0.163     2.12e-4

  -->  A(throttle) = 2.1e-4 * pct^2   m/s^2      A(60%) = 0.76
```

**The square law holds where motion is reliable.** Two independent sessions agreed on
`k` to within their scatter, which is the strongest evidence the model form is right.

⚠️ **Fit `k` on ≥50% data only.** The 35/40/45% rows return `k` = 1.47–1.79e-4, but
that is an artifact: those runs sit on the stiction threshold, so several are
*detected-but-degenerate* — motion so brief that the thrust and coast windows barely
populate and `A` collapses toward 0. They bias the mean down, not the physics.

**Channel matching at 50%: `fan1 0.495 (n=14) · fan2 0.485 (n=3) · fan3 0.519 (n=3)`
— within 7%.** Fans 1–3 are well matched.

**Fan 4 had a reversed spin direction** and did not break stiction at all. Fixed
**in firmware over DSHOT** (`S4` → `J21` → `J12` → power-cycle) with no hardware
contact — see B17. After the fix: `55% → 0.541 / 0.578 / 0.611` (mean 0.577, ±6%, the
tightest cluster in the whole dataset) and `60% → 0.572`. **The backwards-prop
hypothesis is dead** — half thrust would have shown ~0.38 against fan 1's 0.76.

**Trap that bit twice — within-session mobility drift, ~+30%.** Runs get *stronger*
through a session (ball transfers freeing up, and/or a swept track through the dust).
It is the largest single error source and it **invalidates any cross-session channel
comparison**: fan 1 first appeared 33% weaker than fans 2/3 purely because it was
measured first, and its pooled n=14 mean later landed exactly between them. Anything
comparing motors must run **in one session, interleaved**, or use the current sensor.

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

**Result (2026-08-14) — DONE, by motion onset instead of string-and-weights.**

Rather than measuring a force, `A_c` is read off the throttle at which the platform
**just breaks loose** — at onset, `A(throttle_break) = A_c` exactly. This is a
*motion-detection* measurement, so **accelerometer bias is irrelevant to it**, which
makes it the most trustworthy number in the phase. It is the same logic as
`CONTROL_README` §14 run 5 (net displacement, not peak rate) moved to the new axis.

```
fan 1, fraction of runs that moved, all sessions pooled:
   25%  0/1        45%   5/5
   30%  1/1        50%  14/14
   35%  5/7  <--   55%   5/5
   40%  7/7        60%   5/5
```

Clean threshold at **35%**, reliable from 40% up → **`A_c ≈ A(35%) ≈ 0.26 m/s²`**.
The stochastic band at 35% (5 of 7) is the translational twin of §6's "whether it
moves is effectively random" — you are sitting on stiction, exactly as theory predicts.

### The go/no-go — **PASS**

```
A_max (60%)  = 0.76 m/s^2
A_c          = 0.26 m/s^2
ratio        = 2.9x   single fan, cardinal push
             = 4.2x   diagonal (two fans, each contributing cos45)
```

Right at the **"comfortable"** 3× boundary and well clear of the 2× stop-line.
**Translation control is viable**, with a friction deadband to design around —
structurally the same problem the rotation axis solved with Coulomb feedforward, so
the architecture transfers even though none of the constants do.

**Open, carried into Phase 6:** the coast-phase deceleration *rises* with throttle
(0.15 at 30% → ~0.35 at 50%), which pure Coulomb should not do. Suggests a viscous
term on top — the rotation axis showed the same signature (R² 0.918 Coulomb alone vs
0.951 with both). Not modelled yet; revisit if the feedforward misbehaves at speed.

## Step 2.3′ — Rotation re-identification and retune — ✅ **DONE 2026-08-19**

**This was the deferred campaign from `CONTROL_README` §17 / B6, and it finally
happened here** — because Step 2.4 uses the heading controller as an *instrument*, and
a controller that stalls cannot measure a disturbance. Ordering matters: translation
(2.1/2.2) needed nothing from rotation, which is why it succeeded with the wheel off;
2.4 is the first genuine coupling, so it goes after.

**Result — 6 consecutive runs, mean final error 0.47°, every slew in ONE move, wheel
returning fully to rest each time:**

```
 target   final err   w_peak   w_end    settle to 0.8 deg
   +90      -0.03      34.9     0.1        0.68 s
   +90      -0.77      33.7     0.0        0.71 s
  -180      -0.41      52.7     0.0        1.26 s
  -180      +0.25      52.4     0.0        1.00 s
    +5      +0.60       5.7     0.0        0.62 s
   -90      -0.75      33.6     0.0        0.61 s
```

**Constants now in `rtos_main.cpp`:** `A_1 47.9 · A_2 4.97 · a 0.098 · compFrac 0.90 ·
K_θ 216 · K_ω 52 · ffFrac 0.95 · A_static 60 · A_moving 34 · A_viscous 0 ·
deadzone 1.5/0.8 · ALPHA_STALL_MAX 70 · STALL_WW 20 / MS 300 / HOLD 4500 ·
WHEEL_SAT_LIMIT 55`. Full derivation and the four findings are in `CONTROL_README` §12.

**Four things mattered, and three were latent bugs rather than tuning:**

1. **`compFrac` was mildly UNSTABLE** (residual pole `+0.043`), not merely neutral —
   that is why passive desaturation was dead. Ten-run `C`-sweep fixed it.
2. **The feedforward conflated static with kinetic friction.** Splitting into
   `A_static`/`A_moving` took 5° corrections from **5/11 to 8/8**.
3. **`ALPHA_STALL_MAX` failed a third time** and the sweep proved it — not
   `A_FRICTION` — was the binding constraint.
4. **ζ = 0.54, not 0.7.** Coulomb friction already damps heavily, so the textbook
   design brakes twice and the platform stops short. This is what fixed large slews.

**Traps found along the way, all recorded in Appendix A:** the `O` command was clamped
to ±3 V, so the wheel plant had *only ever* been identified to 28 rad/s while slews
reach 50 (T25); persisting five of seven tuned values while leaving the gains at their
placeholder produced a specifically-broken build (T26); and the two-stage deadzone
chatters at the `FINE_WW` boundary, where the obvious latching fix measured *worse*
and was reverted (T27).

## Step 2.3 — Step-response identification — ❌ **DROPPED (B14)**

**Not done, deliberately.** Its goal was *effective mass*, which was only ever a route
to `A(throttle)` in force units. Working in acceleration units gets `A` directly from
Steps 2.1/2.2, so mass is never needed by anything downstream — allocation, the LQR
gains, and the go/no-go all work without it. The original text is kept below for the
record; **do not run it thinking Phase 2 is incomplete without it.**

<details><summary>original Step 2.3 (superseded)</summary>

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

</details>

*(Note on that last trap, since it misled once: for a **body-fixed fan** you should
**not** rotate into the world frame. Thrust and the IMU are both bolted to the
platform, so the measurement is naturally body-frame and heading drops out. Rotating
by a drifting `θ` only injects error. Rotation matters for Phase 7's frame transform,
not for this identification.)*

## Step 2.4 — Yaw coupling

**Concept.** Thrust-line offset from the CoM produces yaw the wheel must reject.
Quantify it now rather than discovering it as a mystery disturbance in Phase 7.

**Do.** Command each fan individually with heading control **active** and log the
wheel speed required to hold heading. That wheel effort is a direct measure of the
yaw disturbance torque.

**Verify.** Record disturbance torque per fan. If the wheel saturates holding
heading against one fan, the thrust line needs mechanical correction — no control
law fixes a persistent torque with a momentum-limited actuator.

**Phase 2 exit:** `git tag trans-p2-plantid`. ✅ **DONE 2026-08-19.**
`CONTROL_README` §12 carries every new constant and the four findings behind the
rotation retune; `claude.md` §2 carries the summary. Raw captures are indexed in
`calibration/runs/INDEX.md`.

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

**⚠️ Trap — the guide text above was WRONG for a Pi 3B+, corrected 2026-08-19.**
"Disable the serial console on `/dev/ttyAMA0`" is generic Pi advice. On a 3B+ the
PL011 (`ttyAMA0`) is wired to the **Bluetooth modem**, and GPIO14/15 get the
**mini-UART** (`ttyS0`) whose baud generator is clocked from the **VPU core
clock** — which the governor scales with CPU load. See **T28**; the fix is
`dtoverlay=disable-bt`, and it is not optional.

**Result (2026-08-19) — PASSED, both directions.**

Wiring as specified: Pi pin 8 → **PC7 = D9** (Arduino header, female socket), Pi
pin 10 → **PC6 = CN10-4** (morpho, needs a female Dupont), Pi pin 6 → GND, run in
one bundle. ⚠️ **`CN10-8` is `U5V`, three positions from the target in the same
column** — landing the Pi's 3.3 V RX there damages the Pi, so the pin was
confirmed with a meter (DC rail presence check, ~0 V vs 5 V) before the Pi was
connected. Counting was done relative to the known `PA11 = CN10-14` from Phase 1
rather than from the header end.

**Pi side** (`disable-bt` applied, `hciuart.service` did not exist on this image —
benign, it means nothing was contending for the UART in the first place):

```
ls -l /dev/serial0  ->  serial0 -> ttyAMA0        <- the check that matters
loopback GPIO14<->GPIO15, 115200  ->  b'HELLO\n'  <- port proven BEFORE the STM32
```

**Firmware:** new `src/pi_link.{h,cpp}` (sole reader *and* sole writer of USART6),
`commands_setAuxPoll()` added to `commands.{h,cpp}` so `pi_poll()` rides
`commsTask`'s existing 2 ms poll with no seventh task, and `piSerial(PC7, PC6)` +
one `pi link:` line in `G` in `rtos_main.cpp`. All six environments compile.

**Measured, read off the terminal:**

```
Pi:     10/10 clean    rtt min/mean/max = 2.61 / 3.52 / 5.32 ms
STM32:  pi link: rx=120  tx=120  txdrops=0  maxburst=12  last=0x49
        ctrl stack free (min words): 485
        power: busV 11.89-11.96   |I| max 44 mA   read fails=0
```

`rx = 120` = 10 × 12 bytes exactly, `last=0x49` = `'I'`, the final character of
`PING-FROM-PI`, so the tail is not being lost. `maxburst=12` means a whole payload
drains in one poll; `txdrops=0` means the echo never outran the 64-byte TX ring.

**Finding carried forward — the RTT distribution is bimodal, and the test caused
it.** Seven samples clustered at 2.6–3.0 ms and three at 4.9–5.3 ms with *nothing
between*, rather than the uniform smear across the 2 ms poll window that was
predicted. Most likely cause: `pi_link_test.py` paces at `time.sleep(0.2)`, and
200 ms is **exactly 100 × the 2 ms poll period**, so every trial lands at nearly
the same phase relative to `commsTask` instead of a random one — the clusters are
aliasing, and the drift between them is the two boards' clocks walking. Pi-side
Linux scheduling jitter is a competing explanation and this data cannot separate
them. **Not chased — 3.1's question was "do bytes cross," and they do.** But see
**T29**: this must be fixed before latency is ever *characterised*, because the
Phase 5 estimator consumes a latency figure and an aliased test measures a phase,
not a distribution.

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

**Result (2026-08-19) — PASSED.**

`src/pi_link.{h,cpp}` carries the byte-level framer, the pose decode, and the
staleness ladder. Wire format, CRC parameters and endianness are specified in
`pi_link.h` and mirrored in `tools/pi_pose_sim.py`; the Python CRC was validated
against the published CCITT-FALSE check vector (`"123456789"` → `0x29B1`) and the
payload offsets were checked field-by-field against the C decode **before**
anything was flashed.

**Tested with synthetic frames and no camera at all** — same argument as B8.
The framer has several independent failure modes (endianness, CRC parameters,
length convention, fragmentation, sequence tracking) and every one of them
presents as `frames=0`; bisecting that with a detector, camera and lighting also
in the picture is strictly harder.

**Measured, read off the terminal:**

```
pi link: DEAD  age=33753ms  frames=62  rx=2508  crc=14  badlen=0  resync=0  seqgaps=26
pi pose: seq=88  tag=7 x3  range=1.507m  bearing=0.25deg  relyaw=0.17deg  q=0.94  piage=12ms
comms:   rx bytes=4          <- the operator port saw ONLY the 'g' presses
fans:    KILLED              <- tier 2 fired
ctrl stack free (min words): 475
```

**The run reconciles exactly**, which is the real result: `rx = 2508 / 33 B =
76.0 frames`, being 62 accepted + 14 CRC-rejected with **zero bytes
unaccounted**; last seq 88 − 76 arrived = 12 never sent; 12 + 14 = **26 gaps**.
`piage=12ms` matches the simulated detection latency, and `resync=0` after a
40-byte garbage preamble means the parser locked on without eating a real frame.

**`comms: rx bytes=4` is the safety property confirmed by measurement**, not by
construction: none of the 2508 Pi bytes reached `commsTask`, so no payload byte
could ever have been read as `X`. That is B18 working.

**Sequence-gap accounting — two structural properties, neither of them bugs.**
Both cost time to understand once and are written down so they don't again (T31):
`seqgaps` **includes CRC-rejected frames**, because a rejected frame's `seq` was
never decoded and is therefore indistinguishable from one never sent; and a gap
counter **cannot see trailing loss**, because an absence needs a *later* frame to
reveal it — so the Pi's count runs 1–2 high at the tail of every run. Observed
26 (STM32) vs 27 (Pi). Correct on both sides.

**One real bug found and fixed in review**, before it could bite: the `uint16`
sequence subtraction wraps correctly for rollover but turns a *backwards* jump
into ~65000 gaps in a single frame — and a backwards jump is exactly what
re-running the Pi sender without resetting the board produces. Bounded by
`PI_SEQ_MAX_GAP`, with `seqrestart` counting restarts as their own event.

**⚠️ Carried to Phase 6 — tier 2 currently uses the WRONG mechanism.** It calls
`fans_stopAll()`, which is the **hard kill and latches until `R`** (B10). That is
right for a fault and wrong for an expected dropout: **B2 says the tag is
routinely lost at close range**, so a 1 s dropout during terminal docking would
permanently disable translation mid-approach and wait for an operator. Phase 6
must give tier 2 a **soft zero with the ESC left armed**, so authority returns the
instant pose does. Faults and expected dropouts deserve different mechanisms.

**Note on the Verify list above.** It asks for round-trip latency in `G`, but
removing the Step-3.1 echo removed the round trip. The requirement is met across
the two steps instead: 3.1 measured RTT at **2.61 / 3.52 / 5.32 ms**, and `G` now
reports the Pi-side `age_us`, which is the number the Phase-5 estimator actually
consumes. Recorded rather than left looking unmet.

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

**Result (2026-08-20) — PASSED. Chosen mode: 1280×720 MJPG, grayscale decode,
`decimate=2.0`, threaded capture.**

```
LATENCY capture->pose   mean 60.9 ms   p95 70.5   max 76.1
decode 31.0 ms (capture thread)   detect 29.4 ms (main thread)
28.6 fps   tags 3/3/3 (mean 3.00, min 3, max 3)
```

**Two settings changed after the first pass and both mattered:**

**Exposure locked manually** (`auto_exposure=1`, `exposure_time_absolute=100`
= 10 ms, plus added light). Barely moved the mean, but killed the tail:
**max latency 680.3 → 89.1 ms**. That 680 ms outlier would have tripped B21's
staleness ladder on a perfectly healthy link.

**`CAP_PROP_BUFFERSIZE` 1 → 4.** With a single V4L2 buffer the driver has nowhere
to capture while userspace holds the frame, so it misses every other one — 15.9
fps against the camera's 30. The shallow-buffer reasoning is correct for a
*serial* loop and obsolete for a threaded one, because the capture thread drains
continuously and always yields the newest frame. **15.9 → 28.6 fps.**

Latency rose 44.5 → 60.9 ms, and that is a deliberate trade, not a regression:
the frames are not aging in the queue (60.9 ≈ 31 decode + 29 detect — it is
processing *contention* from handling every frame). Twice the independent
measurements, a delay that is **measured and therefore compensable** via
`age_us`, and a **tighter** worst case (89 → 76 ms). For a filter, bounded jitter
beats a better mean.

Run on the **3A+ replacement board** (the 3B+ died — see the hardware note below),
at a verified 1400 MHz with the `performance` governor pinned.

**How it got from 443 ms to 45 ms**, in order of contribution:

| change | effect |
|---|---|
| use the reference detector, not `cv2.aruco` | **~4×** — see T32 |
| `decimate=2.0` | **3.5×** on detect (285.6 → 82.0 ms) |
| decode MJPEG straight to grayscale | 50.2 → 15.0 ms (skips colour recon + `cvtColor`) |
| threaded capture, newest-frame-wins | period stops being capture+process serialised |

**720p is not negotiable.** At 800×600 and 640×480 the benchmark detects only
**2 of 3 tags** — the 4 cm flanking tags fall under the detection floor, and those
are the entire mechanism for resolving planar yaw ambiguity (T30). Speed bought by
losing a tag is a broken terminal approach, not speed.

**`decimate=4.0` is too far**: latency drops to 29.4 ms but tags fall to
`mean 2.82, min 1`. 2.0 is the setting that holds 3/3/3. Detect is only 31.8 ms
there, so there is no reason to push it.

**YUYV was tested and REJECTED, opposite to the prediction.** At 720p it is capped
~5.9 fps by **USB bandwidth**, not by the 10 fps the camera advertises: an
uncompressed 720p frame is 1.84 MB and takes ~170 ms to cross the wire. That
transfer is both a rate cap and pure latency, and it is **invisible in the profile
because it hides inside `grab()` as blocking**. MJPEG moves ~100 KB in ~10 ms and
wins on rate *and* latency. See T34.

**⚠️ Still open at the close of this step — auto-exposure.** The camera delivers
only ~15 fps against an advertised 30 for MJPG 720p, which is the signature of an
auto-exposure sitting near 66 ms. That same exposure implies **~15 px of motion
blur at 30 °/s** (0.135°/px), which would destroy corner localisation during
SEARCH. **Locking exposure short is a PREREQUISITE for Step 4.1**, not an
optimisation: auto-exposure changes effective focal length, so a calibration taken
with it enabled is invalid. Set `auto_exposure=1` (manual, in UVC) and
`exposure_time_absolute=100` (10 ms), add light until 3 tags hold at maximum
range, calibrate at that setting, and never touch it again.

**Rolling shutter is a second, separate motion effect** and is not fixed by
exposure: the sensor reads top-to-bottom over the frame period, so during rotation
the tag arrives *sheared* and PnP returns a confidently wrong pose rather than
failing. Mitigations: **step-and-stare** search (rotate, stop, look — what real
spacecraft do, and it removes blur and shear together), and **widen `R` on measured
`ω_p`**, which the STM32 already has from the gyro at 200 Hz.

**Hardware note.** The original Pi 3B+ died mid-phase (PMIC failure — 3.3 V rail
absent, board cold, would not boot from SD *or* USB). Replaced with a **Pi 3A+**:
identical BCM2837B0 at 1.4 GHz so identical detection performance, 512 MB instead
of 1 GB, one USB port, no Ethernet, ~1/5 the current draw, ~$15 cheaper. Nothing in
the architecture changed. Detector is Debian's **`python3-apriltag`** (prebuilt
reference AprilTag 3.4.2) — `pupil-apriltags` cannot be built on 512 MB, it
OOM-kills the compiler even single-threaded with swap.

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
**if `ω_w` walks toward the 55 rad/s abort, the yaw disturbance exceeds what passive
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
| T20 | **`vTaskDelayUntil` schedules from the WAKE time, not from when the task runs** | Release jitter (a higher-prio task holding the CPU, or an equal-prio peer) means two *emissions* can land microseconds apart even though the *schedule* is correct. Cost 1,565 phantom "overruns" at a 2 ms fan period. For anything that emits into hardware with a minimum spacing, gate on a real microsecond timebase, not on ticks — 1 ms granularity structurally cannot see a 50 µs reality. Distinct from T17, which is about catching up *after* starvation. |
| T21 | **A counter that conflates an expected artifact with a fault** | Worse than no counter: it burns a debugging session and then trains you to ignore it. Split benign (`fans_skips`) from real (`fans_overruns`) and state which one is allowed to be non-zero. |
| T22 | **A new capture type added to a single-capture-type code path** | Three separate bugs from one assumption (2026-08-14): `Y`/`I` missing from the `telem_busy()` buffer-lifetime guard; the post-capture transition dropping plant-ID runs into `CTRL_HOLD` (engaging the wheel *and* holding `telem_busy` so the next run was refused); and `stepCount` only incrementing in `O`/`T`/`C`. **Grep every use of the capture state before adding a fourth type.** |
| T23 | **The capture LABEL is the filename, and the script overwrites** | `capture_calibration.py` builds `test{N:02d}_{label}.csv` from the `--- capture start (test N/M: label) ---` marker and opens it `"w"`. Two captures emitting the same marker silently overwrite — presenting as "it only saves the first one" while the script honestly reports N files written. Every capture must emit a **unique N and a descriptive label**. |
| T24 | **Diagnosing a tooling symptom without opening the tool** | Cost two wrong firmware diagnoses before `capture_calibration.py` was actually read; the answer was one line in it. When the user says "the file didn't save", read the writer first. T7, again. |
| T25 | **A diagnostic command's own clamp limiting the identification** | `O<V>` was hard-limited to ±3 V, so the wheel plant had only ever been characterised to 28 rad/s — while a 90° slew takes it to 50. Every constant was an extrapolation to ~2× the measured range, and nobody noticed because the clamp was invisible from the data. **Check the tool's limits before trusting the model's range.** |
| T26 | **Persisting SOME tuned values to source** | Five of seven were written in and the two gains left at their placeholder. The result is not "slower", it is *specifically broken*: the deadband floor scales as `1/K_θ`, so low gains put the deadzones below the floor, feedforward never switches off, and the wheel winds forever. Six T90s ended 21–63° short. **Persist a tuned set atomically, or not at all.** |
| T27 | **"Fixing" the two-stage deadzone chatter** | The `FINE_WW` comparison chatters — crossing it toggles the tolerance, which toggles the controller, which drives `ω_w` back across. Latching the transition measured **worse** (mean 0.97→1.25°, worst 1.47→3.10, wheel peaks 6–17→8–36): the chatter is a *safety valve* that lets the controller give up and the wheel bleed. Reverted. Do not re-fix without n ≥ 8 evidence. |
| T28 | **Pi 3B+: GPIO14/15 are the mini-UART, not the PL011** | The BCM2837 has two UARTs. With Bluetooth enabled the PL011 (`ttyAMA0`) is wired to the **BT modem**, and the header pins get the **mini-UART** (`ttyS0`) — whose baud generator is clocked from the **VPU core clock**, which the governor scales with CPU load and temperature. The failure mode is vicious: the link is perfect at idle and starts throwing framing errors **exactly when AprilTag detection loads the CPU**, so it presents in Phase 4 as a vision bug, an EMI problem, or a bad ground — anything but the wiring session three steps earlier that introduced it. Fix: `dtoverlay=disable-bt` in `/boot/firmware/config.txt` (`/boot/config.txt` pre-Bookworm) + `enable_uart=1` + console off via `raspi-config`. **The check that actually confirms it is `ls -l /dev/serial0` resolving to `ttyAMA0`, not `ttyS0`** — everything else can look right while the port is still the wrong one. Costs Pi Bluetooth, which B1 already chose not to use. `hciuart.service` may not exist on Lite images; that is benign, not a failure. |
| T29 | **Timing a periodic system at a commensurate interval** | `pi_link_test.py` paced at 200 ms against a 2 ms poll — an exact 100× multiple — so every sample landed at the same phase and the RTT came out **bimodal** (7 at 2.6–3.0 ms, 3 at 4.9–5.3, nothing between) instead of spread across the poll window. An aliased test measures **a phase, not a distribution**, and reports a confidently wrong mean. Harmless for a yes/no "do bytes cross" check; **not harmless for the Phase 5 estimator, which consumes a latency figure.** Before latency is characterised rather than merely observed, pace with a non-commensurate or randomised interval (197 ms, or jitter it) and take enough samples to see the actual shape. Same family as T20 — tick-granular reasoning hiding a sub-tick reality. |
| T30 | **Relative yaw from a planar tag is ill-conditioned, and it is worst exactly where docking needs it** | AprilTag gives full 6-DOF from one tag, so range, bearing *and* the dock's face normal all fall out of one detection. But for a **planar** target at small apparent size, two distinct orientations project almost identically — the solver flips between them frame to frame. Range and bearing stay well-conditioned; **yaw is noise at distance and only resolves as the tag grows in frame**, which is the opposite of the error profile an approach wants. Mitigations, in order of value: **multiple tags on the dock at known separation** (turns "read perspective distortion off one square" into "triangulate widely-separated points" — chosen 2026-08-19, and it also survives the close-range dropout B2 warns about), then propagating the PnP ambiguity ratio to the estimator as `quality` + `PI_FLAG_AMBIGUOUS` so `R` is **widened on yaw** instead of a flipped solution being believed. Do NOT let a single-tag yaw drive terminal alignment. |
| T31 | **Sequence-gap counters: two structural properties that both read as bugs** | Cost real confusion on the Step 3.2 verify. **(a) `seqgaps` includes CRC-rejected frames.** A rejected frame's `seq` was never decoded, so at the sequence layer it is indistinguishable from one never sent. That is correct — the estimator wants "measurements I am missing", and the answer does not depend on lost-vs-corrupted. True transport loss is the *derived* `seqgaps − crc`. Do not split them into non-overlapping counters: a CRC failure cannot be attributed to a specific missing `seq`, so the split would have to assume one gap per CRC error, which breaks the moment a corrupt length field desyncs the parser and eats the next frame too. **(b) A gap counter cannot see trailing loss** — an absence needs a *later* frame to reveal it, so the sender's count runs 1–2 high at the tail of every run (observed 26 STM32 vs 27 Pi). Neither is fixable and neither matters. **(c) The one that WAS a bug:** `uint16` subtraction wraps correctly for rollover but turns a *backwards* jump into ~65000 gaps in one frame — and restarting the sender without resetting the board produces exactly that. Bound the jump; count restarts separately. |
| T32 | **A benchmark whose DETECTOR is ambiguous is worthless — and a silent fallback guarantees it** | Cost most of a session (2026-08-19/20). `cv2.aruco` can decode `tag36h11` but uses the ArUco quad detector: **~4× slower than the reference AprilTag detector, and it ignores `decimate` entirely** — so decimated rows silently duplicated the undecimated ones and read as "decimation does nothing". The fallback was hidden by a bare `except: pass`, so a run landed on the slow backend with no indication, was compared against an earlier run on the fast one, and the 4× gap looked like a hardware problem. **Never swallow a backend-selection exception**; print why each preferred option was skipped, print which one ran, and never compare numbers across backends. Related: the Debian binding's constructor kwarg is **`threads`, not the `Nthreads` its own docstring claims**, and it defaults to **1** — getting that wrong costs ~3× on a quad-core and looks exactly like a slow board. |
| T33 | **Reading a clock at idle and assuming it applied during the run** | The old Pi's ARM clock read 600 MHz *after* a benchmark, at idle, and that was taken as proof the run had been throttled to 43% — the basis for a 2.33× extrapolation that predicted 12–14 fps on the replacement. Wrong twice over: **600 MHz at idle is the normal `ondemand` floor**, not throttling, and the real gap between the "throttled" board and a verified-1400 MHz one turned out to be 1.34×, not 2.33×. Governor scaling and thermal/undervolt throttling both produce the same idle reading and only one is a fault. **Poll the clock DURING the workload, in a second session, or the number means nothing** — and pin `performance` before benchmarking so the ramp is not part of what you measure. |
| T34 | **Period is not latency, and blocking hides inside the profile** | Two ways this bit. (a) The serial benchmark reported `grab+decode+detect` as latency, but when the pipeline outruns the camera `grab()` **blocks** — that time is a frame that has not been captured yet, not the age of one that has. It inflates the apparent latency while the real figure is much lower. (b) **YUYV at 720p looked cheap because its "decode" is 8 ms vs MJPEG's 50** — but an uncompressed 1.84 MB frame takes ~170 ms to cross USB 2.0, and that transfer is *both* a rate cap and pure latency, entirely concealed inside `grab()`. Measured: YUYV `decimate=2` showed `grab 74.3 ms`, matching the predicted wait to within 1 ms. **Timestamp the frame the instant `grab()` returns and measure to pose-available; report that separately from the iteration period.** |
| T35 | **Auto-exposure is a control-loop parameter, not a camera convenience** | It silently sets three things at once: **frame rate** (a 66 ms exposure caps you at 15 fps no matter what the camera advertises), **motion blur** (66 ms at 30 °/s ≈ 15 px at 0.135°/px, which destroys corner localisation), and **effective focal length** — so a calibration taken with auto-exposure enabled is invalid the moment the lighting changes. Lock it manually (`auto_exposure=1` is *manual* in UVC; 3 is auto), add light to compensate, calibrate at that setting, and never touch it again. Rolling shutter is a **separate** effect the exposure fix does not address: during rotation the tag arrives sheared and PnP returns a confidently wrong pose rather than failing. |
| T36 | **Deriving geometry from a published FOV instead of a measured focal length** | The camera's spec sheet says 120° DFOV, which gives `f = 424 px` at 1280×720. **Measured `f ≈ 947`** — 2.2× off, implying ~76° diagonal, and self-consistent across two known distances to 2%. Likely 720p is a centre *crop* of the sensor, not a downscale. Everything sized from the published figure was wrong in **both directions**: detection range was badly *under*-estimated (12 cm tag reaches 4.5 m, not 2.0 m), while close-range framing was badly *over*-estimated — view width is `1.35 × range`, not `3.02 × range`. That second error had already produced a dock standoff recommendation (25 cm) which would have pushed both flanking tags out of frame at the docked position, silently removing the yaw solution during terminal alignment. **Measure `f` from a tag of known size at a tape-measured distance before sizing any geometry.** It takes five minutes and needs no chessboard: `f = apparent_px × range / tag_size`, checked at two distances so a disagreement is visible. |

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
| **B11** | **Manual fan commands are RAW THROTTLE; the thrust-vector command moves to Phase 6.3** | 2026-08-13. Step 1.4 asked for "a thrust-vector command", but force→throttle needs `throttle = sqrt(F/F_max)` and `F_max` comes from the Step 2.1 thrust curve, which does not exist. Implementing it now would mean inventing a calibration constant, and would put allocation logic in a bring-up command rather than beside the opposing-pair idle-bias handling in 6.3 where it belongs. Raw throttle (`S<n>` select, `L<pct>` level) is also exactly what Phase 2 needs in order to *measure* the curve. |
| **B12** | **Dead-man timeout on commanded throttle (10 s), added beyond the guide** | 2026-08-13. With the props unguarded (B7), a fan left spinning because the operator was distracted is the specific hazard a guard would have covered — so `fanTask` zeroes all four if nothing calls `fans_setThrottle`/`fans_setAll` for `FAN_CMD_TIMEOUT_MS`. Chosen over a hardware interlock (none available) and over trusting operator discipline. **Free in closed loop**: a controller refreshes every cycle, so it can only fire on an untended manual command. ⚠️ Phase 2.1's thrust sweep must refresh within 10 s per step or raise the constant. |
| **B13** | **`skips` and `overruns` are separate counters; fan period 2 ms → 3 ms** | 2026-08-13, after the Step 1.4 overrun investigation (full write-up under Step 1.4). `vTaskDelayUntil` schedules from the wake time but frames are emitted when the task gets the CPU, so release jitter can put two emissions ~50 µs apart on a correct 3 ms schedule — inside the 60 µs a frame needs to clock out. That is benign (the ESC just gets the next frame), so it is now counted as a **skip**, gated on the **TIM5 microsecond timebase** rather than tick arithmetic — 1 ms granularity cannot see a 50 µs event, which is precisely why it looked mysterious. `overruns` is reserved for "still not drained after 100 µs genuinely elapsed", a real fault, and must be 0. Period raised to 3 ms to turn ~−10 µs of jitter margin into ~600 µs; 333 Hz is still far above the ESC's ~250 ms disarm timeout. **Method note worth keeping:** two competing hypotheses predicted different `NDTR` values, so two register reads settled it in one flash cycle instead of an argument — T7 applied to my own reasoning. |
| **B14** | **Phase 2 identifies the plant in ACCELERATION units; mass and force are never measured** | 2026-08-13, user decision after reviewing four options. The motors cannot easily be dismounted, so the scale rig (2.1 as written), the pendulum, and the string-and-pulley were all impractical. Reformulated: `ẍ = A(throttle) − A_c·sign(v)`, and **`m`, `F` and `F_c` cancel out** — the controller only ever needs `A(throttle)` and `A_c`, both directly measurable from the accelerometer already on the bus. Same reasoning that let `CONTROL_README` §3 work in `A_1`/`A_2`/`A_FRICTION` and deliberately never pin down `J_w` or `J_p`. **Nothing is lost:** the 2.2 go/no-go becomes `A_max/A_c` (identical ratio), allocation becomes `throttle = sqrt(A/A_max)`, and the LQR gains lose the mass term entirely because the control output is an acceleration — exactly as on the wheel axis. Effective mass (old 2.3) is dropped as a goal; it was only ever a route to `A`. **Do not "fix" the missing absolute force numbers.** |
| **B15** | **No table tilt — asserted by the user, not measured** | 2026-08-13. The measured accel bias magnitude (0.177–0.181 m/s², stable to 2% across a power cycle) corresponds to ~1.05° of tilt-equivalent *if* it were tilt, which would be 3× the 0.3° planning figure §18 warns about and comparable to friction itself. A `B` → rotate 180° → `B` test would separate world-fixed tilt from body-fixed sensor bias in 30 s. **The user has confidently asserted the table is level and declined the test; treated as sensor zero-g offset (16 mg, well inside MPU6050 spec) and not pursued further.** Recorded only so that if translation later shows a persistent directional drift that no controller seems to fix, this is the first thing to re-examine. |
| **B16** | **Single-fan (`S1`) thrust steps, not all-four** | 2026-08-13. The four fans are opposing pairs (§18: "only one fan works for N/S/E/W; a diagonal splits across two"), so `S0` + a thrust step is **near-zero net thrust** — the pairs fight each other, producing heat and current and almost no motion. Per-motor curves need one fan at a time anyway. `S0` is kept for a different job: four motors drawing at a known throttle with the platform stationary is the clean way to answer the fuse/current question (T15) before raising `FAN_THROTTLE_MAX` above 30%. |
| **B17** | **Fan 4 spin direction reversed in ESC firmware over DSHOT, not by touching hardware** | 2026-08-14. Three motors spun one way and one the other against 2 CW + 2 CCW props, so one fan had prop handedness mismatched to its rotation — costing ~half thrust (T3), and fan 4 could not break stiction at all. Fixed by sending DSHOT special commands from the firmware: `S4` → `J21` (SPIN_DIRECTION_REVERSED ×10) → `J12` (SAVE_SETTINGS ×10) → power-cycle. Implemented as a **general command primitive** (`fans_sendCommand`, serial `J<cmd>`) rather than a one-shot, because this ESC's DSHOT command table is demonstrably incomplete — the beep command has never worked on it — so being able to try 21, then 8, then 7 and observe was the point. **Command 21 worked.** Protocol details that are load-bearing: the telemetry-request bit must be SET to distinguish a command from a throttle value, the motor must be stopped, and the command must be repeated (~10 frames). Chosen over swapping props (hardware contact, and it only moves the mismatch) and over swapping phase wires (most invasive). **Verified by measurement, not just by eye:** fan 4 went from not moving at 40% to 0.577 ±6% at 55%. |
| **B18** | **The Pi port is NEVER passed to `commands_init()` — `pi_link.*` owns USART6 outright, and `pi_poll()` rides `commsTask` via a registered callback** | 2026-08-19, Step 3.1. **The safety half:** `commsTask`'s contract is line assembly plus *any unrecognised input stops the motor*, with an `X` fast path that fires **before the line is even queued**. The Pi sends binary pose frames, so a payload byte of `0x58` is `'X'` and would stop the wheel mid-slew, a `0x0A` mid-frame is a line terminator, and Pi boot chatter arrives as a scatter of commands. The operator parser and the pose parser must never share a code path — so the Pi port gets its own reader from day one, before there is any protocol to get wrong. **The invariant half:** B15 (sole writer) and the sole-reader rule are stated over `Serial`/`hc05Serial`, but what they actually protect is that **`HardwareSerial` is not reentrant *per port*.** USART6 is touched by nothing else, so `pi_link` is both its sole reader and its sole writer — the same requirement satisfied by ownership rather than by arbitration. **The structural half:** `pi_poll()` is wired in with `commands_setAuxPoll()` rather than `#include "pi_link.h"` inside `commands.cpp`, matching the two existing registered-callback precedents (`faults_setHwKillHook` B10, `safety_setFanMonitor`); this keeps proven code generic and adds **no seventh task and no new stack** — the link rides the 2 ms poll that already exists, which is also where Appendix C puts it. Rejected: a dedicated `piTask` (more RTOS surface for a port that produces ~600 B/s), and extending `commands_init` to three streams (would have put a binary stream one `if` away from the motor-stop path). **Step 3.1's echo is a temporary diagnostic and must be deleted in 3.2** — a framed protocol that echoes its own payload back at the sender is a feedback loop. |
| **B19** | **Pose is sent in POLAR (range, bearing, rel-yaw), not Cartesian — and the Pi does geometry, never time** | 2026-08-19, Step 3.2. **Polar:** identical information to (x, y), different *noise*. AprilTag range error grows roughly with distance squared while bearing error is roughly constant in angle, so in polar the two are axis-aligned and the estimator's measurement covariance `R` is **diagonal**. Converting on the Pi produces the classic banana-shaped correlated uncertainty, which needs a full covariance to represent honestly — and sending it as diagonal anyway would make the filter quietly overconfident. Cost: the EKF measurement model is nonlinear, which it already was. **Geometry not time:** the Pi runs detection + PnP + the 6-DOF→planar collapse and is otherwise **stateless per frame** — no smoothing, no velocity, no filtering. A Kalman filter needs independent measurements with known covariance; pre-filtering on the Pi produces time-correlated noise its model does not include, so it would think it had N independent looks when it had one look smeared over N frames (T10's cousin — information counted twice). Bandwidth was explicitly *not* the reason: 33 B at 30 Hz is ~1 kB/s against 11.5 kB/s, ~9% utilised. |
| **B20** | **The frame carries `age_us`, NOT a Pi timestamp — this DEPARTS from the guide's Step 3.2 text** | 2026-08-19. The guide asked for a "Pi-side timestamp". The two clocks are unsynchronised, so an absolute Pi timestamp is meaningless on the STM32 without a clock-sync protocol to build, debug and keep correct across reboots — a whole subsystem bought for nothing. `age_us` is measured **entirely on the Pi's own clock** (capture → transmit), where the difference is valid regardless of offset, and the STM32 adds the transit it already measured in 3.1 (~2–5 ms). Same information, no shared clock. **Measured from CAPTURE, not detection-complete:** AprilTag on a Pi 3B+ carries 50–150 ms, which at docking closing speeds is the *dominant* position error rather than a rounding one, so stamping at the wrong end would quietly discard most of the latency the field exists to report. Verified end-to-end: a simulated 12 ms detection latency arrived as `piage=12ms`. |
| **B21** | **The staleness ladder is 3-tier and split by axis; tier 3 is deliberately NOT wired to `stopMotor()` in Phase 3** | 2026-08-19. **Why a ladder and not one timeout:** the two axes dead-reckon completely differently. Heading coasts on the gyro at ~0.8 °/min — 0.04° after 3 s, effectively free. Position coasts on double-integrated accelerometer, error ~½·a·t², so it is spent within about a second. Translation authority is therefore withdrawn a full 2 s before attitude control is even questioned: a platform holding heading with fans off is stable and recoverable, one that has also dropped attitude control just drifts. Thresholds **250 ms** (≈2–3 missed detections at 10 Hz — tolerate dropout without twitching) / **1 s** / **3 s**. **Why tier 3 only calls a hook:** nothing consumes pose until Phase 6, so a lost link endangers nothing today — while dumping a spinning flywheel *does* kick the platform at ~42 rad/s² against a 4.24 breakaway, and `safety.cpp`'s own comments warn against nuisance trips. A safe-stop that is not needed is not free on this machine. Tiers 1 and 2 fire for real; tier 3 announces itself so all three timers are exercised, and **Phase 6 repoints it at `stopMotor()`** once there is something to protect. **Why the ladder arms on the first valid frame:** if pose has never been valid its absence is not a loss — without that, every rotation test would trip a timeout at boot merely because no Pi is attached. ⚠️ **Known wrong for Phase 6:** tier 2 calls `fans_stopAll()`, the hard kill, which **latches until `R`** (B10). B2 says the tag is routinely lost at close range, so a 1 s dropout during terminal docking would permanently disable translation and wait for an operator. Phase 6 must give tier 2 a **soft zero with the ESC left armed** — expected dropouts and faults need different mechanisms. |
| **B22** | **Capture mode: 1280×720 MJPG, grayscale decode, `decimate=2.0`, threaded capture** | 2026-08-20, Step 4.2, chosen from measurement. **720p is forced, not preferred:** at 800×600 and 640×480 only **2 of 3 tags** detect — the 4 cm flanking tags fall below the detection floor, and those are the whole mechanism for resolving planar yaw ambiguity (T30), so the lower modes trade away terminal alignment for frame rate. **MJPEG over YUYV**, opposite to the initial prediction: uncompressed 720p is 1.84 MB and USB-bandwidth-capped at ~5.9 fps with ~170 ms of transfer that is *both* rate cap and latency, hidden inside `grab()` (T34). **Grayscale decode** (`CAP_PROP_CONVERT_RGB=0` + `imdecode(IMREAD_GRAYSCALE)`) takes decode+convert from 50.2 → 15.0 ms by not reconstructing colour we immediately discard. **`decimate=2.0` not 4.0**: 4.0 is faster (29.4 ms latency) but drops to `min 1` tag; 2.0 holds 3/3/3 with detect at only 31.8 ms, so there is nothing to buy. Result: **44.9 ms mean capture→pose latency, 14.8 fps**, from a 443 ms starting point. |
| **B23** | **Threaded capture with newest-frame-wins, and latency measured from `grab()` rather than inferred** | 2026-08-20. The serial loop `grab → decode → detect` serialises capture and processing, so once processing got fast `grab()` simply blocked and the period stayed pinned at ~67 ms regardless — every millisecond saved in detection was absorbed by waiting. A capture thread buys two things: **overlap** (decode on the capture thread, detect on the main thread, different cores → period is `max()` not `sum()`) and **freshness**. It **deliberately drops frames**: a queue would preserve every frame at the cost of letting them age, which is exactly backwards here — a pose from three frames ago is worse than no pose because the estimator will believe it (T9). Newest-wins is the correct policy for a measurement feeding a control loop. Also load-bearing: the frame is **timestamped the instant `grab()` returns**, so latency is measured rather than inferred from the iteration period, which conflated the two (T34). That timestamp is what `age_us` carries. Detector threads set to 3, not 4, leaving one core for capture. |
| **B24** | **Full chessboard calibration SKIPPED for now; focal length measured directly from two known distances instead** | 2026-08-20, user decision. Step 4.1's chessboard procedure was judged not worth the time yet. Substitute: the 12 cm tag at two tape-measured distances, solving `f = apparent_px × range / tag_size`. **Measured 115 px @ 1.00 m → 958, and 281 px @ 0.40 m → 937. Mean f ≈ 947 px** at 1280×720, the two agreeing to 2%. This removes the dominant error — the systematic range scale — for five minutes of work. **What is NOT corrected: distortion and principal point.** Distortion is worst at the frame edges, and its failure mode is that PnP returns a confident, plausible, slightly wrong pose rather than failing; for this geometry the tags sit ~26% of the way to the edge at dock, where radial distortion is only ~7% of its edge value, so the exposure is limited but real. **Risk flagged once and accepted:** if terminal alignment later shows a bias no controller tuning fixes, this is the first thing to revisit. `tools/pi_calibrate.py` is written and in the repo, auto-capturing across a 3×3 coverage grid, whenever it is wanted. |
| **B25** | **The camera's real FOV is ~76° diagonal, NOT the 120° on the spec sheet — every geometry number derived from 120° was wrong** | 2026-08-20, forced by the B24 measurement. `f = 947 px` implies **68° H / 42° V / ~76° diagonal**; the advertised 120° would be `f = 424`. Either the spec is marketing or **720p is a centre crop of the sensor rather than a downscale** — consistent with this camera offering 1280×720 (16:9) and then only 4:3 modes, with no 16:9 mode below it. The measurement is trusted over the spec because it is self-consistent across two distances to 2%. **Consequences, both directions:** detection range is *better* than planned (12 cm tag to ~4.5 m, 4 cm tags to ~1.5 m, against a 1.36 m table), but **close-range framing is far tighter** — view width is `1.35 × range`, not `3.02 × range`. Minimum ranges to keep things in frame: centre tag 11 cm, top of the centre tag 23 cm (it sits 8.65 cm above the lens axis), **all three tags 25 cm**. **This invalidated the earlier dock-geometry advice** of a 25 cm wall standoff, which would have put the lens at 21 cm and silently dropped both flanking tags — and with them the yaw solution — exactly during terminal alignment. **Wall moved to ~35 cm behind the magnet**, putting the lens at ~31 cm docked: 42 cm of view against a 34 cm tag span. **Lesson: derive geometry from a measured focal length, never from a published FOV.** |
| **B26** | *(next decision goes here)* | |

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
  │  fanTask    (2) 3 ms    DSHOT via TIM1 + DMA  (sole fan writer)         │
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
