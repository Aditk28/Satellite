# RTOS Migration Guide — Reaction Wheel Attitude Control

Turning the tuned super-loop heading controller into a properly structured
FreeRTOS system, without breaking a controller that is already working at the
edge of its bandwidth ceiling.

**Companion to:** `CONTROL_README.md`. This guide assumes the plant is
identified, the controller is tuned, and the closed-loop envelope has been run.

**No logic analyzer, no oscilloscope.** All timing verification is done in
firmware and dumped over serial. Phase 1 builds a software scheduler tracer that
replaces the analyzer.

---

# ═══════════════════════════════════════════════════════════════════════
# STATUS & RESUME — read this first
# ═══════════════════════════════════════════════════════════════════════

> This block is the single source of truth for *where we are*. A fresh session
> (human or a new Claude window) should be able to read only this section plus
> the named step below and continue without re-deriving anything. Update it at
> every step exit.

## Current position

**PHASES 3 AND 4 SWAPPED, BOTH NOW DONE & VERIFIED ON HARDWARE (2026-08-08).
Phase 5 (safety task + I2C mutex) is NEXT.**

**Phase 3 gate PASSED — the headline number:** `ctrl period` MAX went from
**11,500,760 µs → 5,006 µs** (mean 4999 = 200.04 Hz, i.e. ≤6 µs late worst case). The
~11.5 s capture dump no longer stalls the control loop at all; it runs on `telemTask` at
priority 1 in the gaps. Also now `loopFOC : control law` = 352678/17326 = **20.35 ≈ 20**,
exactly `CTRL_DIVISOR` — the 23.9:1 notification-coalescing artifact is gone. `telem drops: 0`,
telem stack free 1479/1536 words. FOC tick dt 239–260 µs, loopFOC 31/32/46 µs (unchanged).
**This also retroactively satisfies the Step 4.2 verify criterion** ("control release jitter
max < 50 µs") which was unmeetable until telemetry left the control path.

**Phase 4 (4.1–4.2) verified:** FOC flat 4000.2 Hz preempting the blocking MPU read.
Steps 4.4–4.5 (asymmetry, compFrac re-verify) and the dead passive-unwind are deferred to
the CONTROL_README hardware retune (STM32 swap + added mass moved the plant; unwind fails on
OLD firmware too, so pre-existing). Step 4.3 (FOC-inside-I2C trace) optional, skippable —
the `M` counts already prove preemption.

**Task inventory now:** `focTask` prio 4 (commutation, TIM9-notified 4 kHz) · `controlTask`
prio 3 (sense + control law + capture fill + command parse, 200 Hz) · `telemTask` prio 1
(ALL serial output) · idle prio 0. Still unsplit: safety/watchdog + INA219 (Phase 5),
command RX (Phase 6).

Swap reason (kept for the record): preemptive telemetry needs a lower-priority
`telemTask`, which only
gets CPU when the control task blocks — but the monolithic control task is a pure busy-spin
(inline `loopFOC`, never blocks), so a prio-1 `telemTask` would be starved. Making the
control task block requires FOC to be ISR-driven (uniform kHz commutation can't survive a
≥1 ms tick-granular `vTaskDelay`). That FOC-in-ISR restructure *is* Phase 4, so Phase 4 is
the real unlock for ALL concurrency (telem, safety, comms). Telemetry's Option-C design
(Appendix B7) still stands, just sequenced after the split. See Appendix B13.

**Phase 2 is COMPLETE — tag `rtos-p2-single-task`.**
Done: Phase 0, Phase 1 (`rtos-p1-kernel`), Phase 2. Decision: **monolithic port** —
`src/rtos_main.cpp` is `heading_control.cpp` + a single RTOS wrap (`hwSetup()` inside
`controlTask` prio 3 / 1536-word stack / TLS=CTRL; `superLoopBody()`; pre-scheduler
`setup()`). File-splitting deferred to Phases 3–6. `diff` vs baseline = 5 regions, all
the wrap; control law/constants/commands/sensors byte-identical.
**Gate PASSED (2026-08-06), all measured not inferred:** T90/T−90 final err 1.54°/0.68°
(in-deadzone), wheels unwound to 0 (compFrac margin intact); **O2 → ω_w 17.2 rad/s,
K=8.59≈8.51 (velocity estimate undisturbed under FreeRTOS — Trap 1 cleared)**; `M`
timing identical to baseline (loopFOC 39µs, compute ~12µs → zero FreeRTOS overhead).

**Git:** branch `rtos-migration`. Last commit `Phase 1.1-1.2: pin deps, prove FPU
port, FreeRTOSConfig + fault hooks`. The Step 1.3 work (config move to `include/`,
`platformio.ini` env split + `-Iinclude`, `src/p1_test.cpp`, doc updates) may be
uncommitted — check `git status` and commit before continuing if so.

## How to resume (new session checklist)

1. **Read, in order:** `CONTROL_README.md` (the controller and plant),
   `READMEs/claude.md` (conventions, hardware, tuned constants, status), then this
   file's Status block and the "current step" section.
2. **Working style** (from `claude.md`, do not violate): one step at a time,
   interactive. The *user* flashes hardware and reports results — you do not move
   on until they confirm. Explain every new file/concept before its code;
   register-level detail (CMSIS, exception behavior, BASEPRI) is welcome, not to
   be simplified. Port application code **verbatim** in Phase 2. Record decisions
   with their reasoning in Appendix B. Maintain this guide, not a side channel.
3. **Safety** (untethered flywheel stores real energy): `X` stops the motor; any
   unrecognized serial input stops it. `WHEEL_SAT_LIMIT = 45 rad/s` hard abort.
   Every fault path disables the driver FIRST, then latches a reason, then reports.
   Never propose a test with hands near a spinning wheel.
4. Go to the step named under "Current position" and follow Concept→Do→Verify→Trap.

## Build & flash cheat-sheet

- **`pio`** lives at `C:\Users\k28ad\.platformio\penv\Scripts` (on PATH in newly
  opened terminals; a Claude shell can call the full path
  `~/.platformio/penv/Scripts/platformio.exe`).
- **Two build environments** (PlatformIO `build_src_filter`, see below):
  - `pio run -e superloop -t upload` — the working controller (`heading_control.cpp`). **Default env.**
  - `pio run -e p1test -t upload` — the Phase 1 throwaway RTOS test harness (`p1_test.cpp`).
  - (`rtos` env for `rtos_main.cpp` gets added in Phase 2.)
- **Monitor:** `pio device monitor -e <env>` (115200). Quit with Ctrl+C.
- **Serial:** two channels — USB (this board routes `Serial` to the ST-LINK UART,
  no buffering, so open the monitor then press RESET to catch boot output) and
  HC-05. `printBoth()` in the controller writes both; `stat_print` in the timing
  code takes a `Print&` so it can target either.

## Critical environment facts (hard-won — do NOT relearn these)

1. **FreeRTOS config placement is a two-part trap.** Our full override must be at
   **`include/STM32FreeRTOSConfig.h`** (exact filename; not `FreeRTOSConfig.h`,
   not in `src/`) AND `platformio.ini` `build_flags` must contain **`-Iinclude`**.
   The filename is what the library wrapper probes via `__has_include`; the
   `-Iinclude` is what puts it on the *kernel/library* TUs' include path. Miss
   either and you get a SPLIT-BRAIN build: your project files use your config, the
   kernel silently uses `FreeRTOSConfig_Default.h` (whose `configASSERT` is a
   silent `for(;;)` hang). Verify by temporarily adding `#pragma message("X")` to
   the config and clean-building: it must fire for `port.c`, `tasks.c`, `queue.c`,
   `heap.c`, … (we counted 13 TUs when correct, 2 when broken). Do NOT use
   `${platformio.include_dir}` in build_flags — it mangles on Windows.
2. **The STM32duino core defines most `TIMx_IRQHandler` symbols (strong)** in
   `HardwareTimer.cpp`. Defining your own collides at link. Use the `HardwareTimer`
   API + `attachInterrupt`, or a timer whose vector the core leaves free — TIM9,
   whose handler is `TIM1_BRK_TIM9_IRQHandler` (Step 1.5).
3. **SysTick is shared with the Arduino core.** The core owns SysTick at 1 kHz and
   calls a weak `osSystickHandler()`, which the FreeRTOS library overrides to drive
   the kernel tick — guarded by `if (xTaskGetSchedulerState() != NOT_STARTED)`.
   Therefore: `configTICK_RATE_HZ` MUST be 1000; `INCLUDE_xTaskGetSchedulerState`
   MUST be 1 (or the pre-scheduler `setup()` hard-faults); do NOT alias
   `xPortSysTickHandler`.
4. **Heap:** `heap_3` (malloc/free wrapper) selected by `-D configMEMMANG_HEAP_NB=3`.
   Under heap_3, `configTOTAL_HEAP_SIZE` is decorative; the real heap is the newlib
   C-runtime heap. A failed allocation fires `vApplicationMallocFailedHook`.
5. **The `.noinit` black box survives a warm reset on this board** (verified Step
   1.3) even though the linker script has no dedicated `.noinit` section — the
   orphan placement lands above `_ebss`. No linker fragment needed. The magic guard
   makes a cold-boot read return "clean boot".
6. **`pio` PATH:** if a new terminal can't find `pio`, `C:\Users\k28ad\.platformio\penv\Scripts`
   is on the user's persistent PATH (added Step 1.3); reopen the terminal, or in
   the current one `$env:Path += ";$env:USERPROFILE\.platformio\penv\Scripts"`.

## Status checklist

| Step | State | One-line result |
|---|---|---|
| 0.1 baseline frozen | ✅ | run-8 CSVs archived; summary table deliberately skipped |
| 0.2 TIM5 µs timebase | ✅ | PSC=89, 1000 ms → 999,993 µs |
| 0.3 timer audit | ✅ | TIM2/TIM3 = PWM; TIM5 = timebase; TIM9 chosen for FOC tick |
| 0.4 WCET measurement | ✅ | loopFOC 40 µs / MPU read 2.37 ms / superloop MAX 2.67 ms = FOC starvation |
| 0.5 FOC rate | ✅ | 4 kHz FOC / 200 Hz control, CTRL_DIVISOR 20, ~16% CPU (Appendix B4) |
| 0.6 golden re-capture | ✅ | 178° slew → −0.32°, wheel unwinds; tagged `rtos-p0-baseline` |
| 1.1 FPU port proof | ✅ | ARM_CM4F confirmed in `port.c.o` disasm; deps pinned |
| 1.2 FreeRTOSConfig + faults | ✅ | `include/STM32FreeRTOSConfig.h`, `faults.*`; compiles |
| 1.3 assert + NVIC audit | ✅ | prio-0 ISR → kernel assert → LED + black box; fixed split-brain config |
| 1.4 software tracer | ✅ | verified: periodic exactly 1000µs (0 jitter), spinner chopped, idle absent |
| 1.5 FOC tick timer (TIM9) | ✅ | TIM9 @ 3999.98 Hz, jitter 0 µs; HardwareTimer API (core owns the vector) |
| 2.x single-task port | ✅ | monolithic port; golden dataset matches, ω_w 17.2@2V, 0 overhead |
| **4.x FOC split** | **✅ 4.1–4.2 (2026-08-08)** | FOC flat 4000.2 Hz preempting the blocking MPU read. Spurious `WHEEL_SAT` traced to a velocity-estimate glitch (not the split — present on reverted monolith too); guarded by `WW_MAX_JUMP` reject + `wwRejects` in `G`. Diagnostics kept: `E`, `V<volts>`, `enctest` env. 4.3 optional; 4.4–4.5 → hardware retune. |
| **3.x telemetry extraction** | **✅ (2026-08-08)** | Option C, sole-writer `telemTask` prio 1. **`ctrl period` MAX 11,500,760 → 5,006 µs**; ratio now exactly 20:1; drops 0. Step 3.1 first shipped BROKEN (half-applied invariant) — see the Phase 3 result note; the lesson is Trap A9. |
| **5.x safety task + I2C mutex** | **✅ (2026-08-08)** | `safetyTask` prio 2, 50 ms: wheel backstop + time-based heartbeat + INA219 power trips (10.0 V / 2500 mA, debounced, set from measured data). I2C **mutex** (priority inheritance); control `i2c_lock(2)` + degrade, safety `(5)`, bias `(20)`. `ctrl period` unchanged at 5006 µs, `i2c timeouts` 0. |
| **6.x comms task** | **✅ (2026-08-08)** | `commsTask` prio 2 = SOLE serial reader; lines queued, `handleCommand` still executes verbatim on the control task. `X` fast path ≤250 µs. `comms drops` 0 under spam. |
| **7.x consolidation** | **✅ (2026-08-08)** | Stacks resized 5376→2176 words (**12.8 KB reclaimed**, ≥61% margin retained); CPU 42/15/11/2/<1, **27% idle**; schedulability proven by response-time analysis (ctrl R=3599 µs < 5000) despite U=0.868 > RM bound 0.757; `CONTROL_README.md` rewritten. Tracer Gantts + plant-dependent artifacts deferred with reasons (7.5). |

## Files that exist now (Phase 0–1.3)

```
include/
  STM32FreeRTOSConfig.h      full FreeRTOS override (Step 1.2; MUST be here + -Iinclude)
  trace_c.h                  C-side tracer enum + hook decls (Step 1.4; here for the SAME reason)
src/
  heading_control.cpp        the super-loop controller (env: superloop). Instrumented in 0.4.
  p1_test.cpp                Phase 1 throwaway RTOS test harness (env: p1test). Extends in 1.5.
  MagneticSensorMT6701SSI.h/.cpp   encoder driver (always compiled)
  timebase.h/.cpp            TIM5 free-running µs counter (Step 0.2)
  timing_stats.h             header-only min/mean/MAX + TIME_BLOCK macro (Step 0.4)
  hw_timers.h/.cpp           timer audit dump (Step 0.3); FOC tick added in 1.5
  faults.h/.cpp              safeStop, .noinit black box, FreeRTOS hooks, rtRunTimeCounter (Step 1.2)
  trace.h/.cpp               scheduler tracer ring buffer + switch hooks + dump (Step 1.4)
  rtos_main.cpp              THE sketch (env: rtos) — Phase 2 monolith + P4 split + P3 wiring
  telemetry.h/.cpp           Phase 3 — telemTask (prio 1), the SOLE serial writer (B15/B16)
  enc_test.cpp               standalone bare-metal MT6701 SSI test (env: enctest) — no
                             FreeRTOS/SimpleFOC/motor. 30-second hardware-vs-firmware check.
platformio.ini               [env] base + superloop / p1test / rtos / enctest; -Iinclude; heap_3
tools/plot_trace.py          Gantt + inter-arrival plot from a trace dump (Step 1.4)
```
Not yet created: `tasks.*`, `control_law.*`, `sensors.*`, `commands.*` — the remaining
file-splits, deferred into Phases 5–7. `rtos_main.cpp` is still monolithic apart from
`telemetry.*` (the first split, Phase 3).

## Detailed progress log — what actually happened

**Phase 0 (measurement).** TIM5 µs timebase built and verified. Timer audit: motor
PWM owns TIM2 (PA5) + TIM3 (PA6/PA7); TIM5 = timebase; TIM9 reserved for the FOC
tick (chosen over TIM4 because TIM4_CH1 = PB6 = the driver enable). WCET
instrumentation (`timing_stats.h` + `M`/`M!` commands) added to `heading_control.cpp`.
Headline finding: the **MPU6050 gyro read is 2.37 ms** and it stalls FOC
commutation for ~2.4 ms out of every 5 ms control period — the concrete
justification for the whole migration (Phase 4 preemption fixes it). FOC rate set
to 4 kHz (PP=11 → f_elec 78.8 Hz → comfortable 3.15 kHz; 4 kHz costs ~16% CPU at
the 40 µs loopFOC WCET). Golden dataset re-captured with instrumentation in place;
178° slew settled to −0.32° with the wheel unwinding cleanly (the compFrac-margin
substitute check passed). Tagged `rtos-p0-baseline`.

**Step 1.1 (FPU port).** Confirmed the compiled `port.c.o` is the `ARM_CM4F` port:
`PendSV_Handler` disassembly shows `tst.w lr,#16` + `vstmdbeq/vldmiaeq {s16-s31}`
(callee-saved FPU registers preserved across context switch — mandatory because
the control law and SimpleFOC transforms run in different tasks). Pinned all six
libraries to exact installed versions so the LDF can't drift.

**Step 1.2 (config + fault hooks).** Wrote `include/STM32FreeRTOSConfig.h` (full
override), `faults.h/.cpp` (one `safeStop` path: hardware driver-kill first via
enable pin, then optional graceful SimpleFOC stop hook, then latch reason to a
`.noinit` black box, then blink the reason code on the LED forever). Fault hooks
own `vApplicationMallocFailedHook` / `vApplicationStackOverflowHook` — no collision
because the library only defines those when its `*_BLINK` config macros are 1, and
we set them 0. Run-time-stats clock routed through `rtRunTimeCounter()` (reads
`TIM5->CNT`) so the config header needn't include the CMSIS device header.

**Step 1.3 (assert safety net) — the hard one.** Built `p1_test.cpp`: starts the
scheduler, fires a TIM7 interrupt whose ISR calls `vTaskNotifyGiveFromISR`, with
the NVIC priority forced to a test value. At priority 0 (illegal, more urgent than
`configMAX_SYSCALL` = 5) the kernel's `vPortValidateInterruptPriority()` assert
must fire. **It initially did nothing** — board froze, no LED, `clean boot` on
reset. The debugging chain that cracked it:
  1. Proved the machinery at priority 5 (notifications streamed 1:1 with ISR count).
  2. Proved the safe-stop path from *thread* context (a direct `configASSERT(0)`
     blinked the LED and latched the black box → `faults.cpp` and the LED are fine).
  3. Read the ISR priority back — it was correctly 0.
  4. So the assert *should* fire but didn't → the kernel wasn't using our assert.
     A `#pragma message` in the config fired for only **2 TUs** (`faults.cpp`,
     `p1_test.cpp`) — the kernel TUs (`port.c` etc.) were compiling against the
     library default `FreeRTOSConfig_Default.h`, whose `configASSERT` is a silent
     `for(;;)`. **Split-brain config.**
Root cause + fix: the config was in `src/` (invisible even to project cross-includes
from the kernel side), then moved to `include/` (fixed the 2 project TUs only),
then `-Iinclude` added to `build_flags` (fixed all 13 TUs including the kernel).
After the fix, the priority-0 ISR correctly trips the assert: LD2 single-pulse
blink, and reset reports `previous boot died: ASSERT at port.c:756`. Priority 5
(legal) streams notifications with no assert. `PRIGROUP` already 3. `.noinit`
survives a warm reset → no linker fragment needed. Two other gotchas found and
logged: the `TIM7_IRQHandler` link collision (→ HardwareTimer API), and
`${platformio.include_dir}` mangling on Windows (→ relative `-Iinclude`).

---

## Starting state (confirm before beginning)

```
A_1 = 45.5        A_2 = 5.35        a = 0.19        A_FRICTION = 22.3
GYRO_SIGN = -1    compFrac = 0.89
K_θ = 119.3       K_ω = 35.1        ω_n = 4.76      ζ = 0.7
ffFrac = 0.90     deadzone = 2.0°   deadzoneFine = 1.0°   FINE_WW = 5 rad/s
ALPHA_STALL_MAX = 55   STALL_WW = 25   WHEEL_SAT_LIMIT = 45
control rate = 200 Hz, logged at full rate
```

Codebase: `reaction-wheel-control/` (super-loop, working). The proven RTOS
skeleton lives separately in `unflashed_files/rtos_tester.cpp`.

### Three properties of this state that shape everything below

**1. You are at the bandwidth ceiling.** `ω_n = 4.76` against a wheel pole at
`A_2 = 5.35`. Above the pole, the feedback linearization stops cancelling
cleanly. You have no margin to spend on added latency, so the migration must
keep the gyro read inside the control task and must not lengthen the
sense→command path.

**2. Desaturation is passive and the margin is ~15%.** Inside the deadzone
`α = 0`, so `u = (compFrac · A_2 / A_1) · ω_w = 0.105·ω_w` against a hold
voltage of `0.123·ω_w`. The wheel bleeds down because 0.105 < 0.123. Phase 4
changes how `ω_w` is computed. If the estimate shifts by more than ~15%, passive
unwind becomes passive windup and the wheel walks toward the 45 rad/s abort.
**Every phase exit checks this explicitly.**

**3. Known-before anomaly.** Some tests showed the platform rotating back after
the step completed; re-running fixed it. Unexplained as of the migration start.
It is on the list so it doesn't get misattributed to FreeRTOS. If you can
characterize it before Phase 2, do — but don't let it block.

---

## How to use this guide

Each step is **Concept → Do → Verify → Trap**. Concepts are written inline and
assume you're comfortable with C and ARM assembly; they're terse on purpose.

### Ground rules

1. **One change per commit, one commit per verify.** When a step fails,
   `git diff` is one change wide.
2. **Never debug two subsystems at once.** If control behavior changes after an
   RTOS edit, revert the edit and confirm the controller is still good before
   investigating.
3. **The golden dataset is the regression test.** Every phase exit re-runs it. A
   refactor that changes behavior has a bug in it.
4. **Tag every phase exit.** `git tag rtos-pN-name`. Rollback is
   `git reset --hard <previous tag>`.

---

## Codebase layout and build mechanics

### The PlatformIO one-sketch rule

PlatformIO compiles **every** `.cpp` in `src/` and links them together. The
constraint isn't "one file" — it's that exactly one translation unit may define
`setup()` and `loop()`, because the Arduino core's `main()` calls them and two
definitions collide at link time.

So: additional `.cpp` and `.h` files in `src/` are fine and normal. Only
alternate *sketches* have to live in `unflashed_files/`.

`MagneticSensorMT6701SSI.h/.cpp` stay in `src/` permanently — every sketch needs
them. Same will be true of the infrastructure files below.

### Target file layout

Build this up as you go. Everything except the active sketch is infrastructure
that survives all seven phases and that `calibration.cpp` will also want.

Sketches (files that own `setup()`/`loop()`) are selected per PlatformIO
environment via `build_src_filter` — they all live in `src/` and the env picks
one; see "Switching between sketches" below. The FreeRTOS config lives in
`include/` for the reason in the Status block (critical fact #1).

```
include/
  STM32FreeRTOSConfig.h      Step 1.2 — full FreeRTOS override. MUST be here, and
                             platformio.ini MUST carry -Iinclude (Status fact #1).

src/
  ── sketches (one per env, build_src_filter) ──
  heading_control.cpp        the working super-loop     (env: superloop, default)
  p1_test.cpp                Phase 1 RTOS test harness   (env: p1test) — throwaway
  rtos_main.cpp              THE Phase-2 sketch          (env: rtos) — added Phase 2

  ── infrastructure, compiled into every env ──
  MagneticSensorMT6701SSI.h/.cpp
  timebase.h  timebase.cpp   Step 0.2 — TIM5 free-running µs counter
  timing_stats.h             Step 0.4 — min/mean/MAX + TIME_BLOCK (header-only)
  hw_timers.h  hw_timers.cpp Step 0.3 / 1.5 — timer audit + FOC tick timer
  faults.h  faults.cpp       Step 1.2 — safeStop, .noinit black box, hooks
  trace_c.h                  Step 1.4 — C-side tracer enum + hook decls (not yet)
  trace.h  trace.cpp         Step 1.4 — scheduler tracer ring buffer (not yet)

  control_law.h  .cpp        Phase 2 — the §8 control law, lifted verbatim
  sensors.h  .cpp            Phase 2 → 5 — MPU6050 + INA219 reads
  telemetry.h  .cpp          Phase 3 — CSV formatting, stream buffer drain
  commands.h  .cpp           Phase 6 — Commander wiring, parse, dispatch
  tasks.h  tasks.cpp         Phase 2 onward — task bodies and creation

unflashed_files/
  calibration.cpp  full.cpp  rtos_tester.cpp    old sketches, kept for reference

baseline/  heading_control_baseline.cpp,  golden/  (run-8 captures)
tools/     capture_/filter_/plot_calibration.py, make_replay.py, plot_trace.py (1.4)
```

### Building from scratch rather than editing in place

Writing `rtos_main.cpp` new and porting pieces across is the cleaner path, and
it's what this guide assumes from here. But it gives up one safety property, and
you should give it back deliberately:

**Editing in place** guarantees that at Phase 2 the *only* difference from the
working firmware is the environment. If behavior changes, it's FreeRTOS.

**Writing from scratch** means Phase 2 differs from the baseline in the
environment *and* in however you retyped or reorganized the application code.
Two variables, and the guide's whole regression method assumes one.

**Give it back like this:** for Phase 2, port the control law and sensor code
**verbatim** — copy-paste, don't retype, don't clean up, don't rename, don't
"improve while I'm in here." Resist it completely. Keep the exact same constants,
the same ordering, the same branch structure, even oddities you'd rather fix.
The reorganizing happens in Phases 3–6, one subsystem per phase, each with its
own verification.

If you find a genuine bug in the old code while porting, don't fix it in
`rtos_main.cpp`. Fix it in `heading_control.cpp` first, re-run the golden
dataset, confirm the change, *then* carry it across. Otherwise you've changed
the reference and the comparison is meaningless.

### Switching between sketches (build_src_filter — this is what we use)

Only one `.cpp` in `src/` may define `setup()`/`loop()`. Rather than moving files
in and out of `src/` by hand, **each PlatformIO environment filters which sketch
compiles**, so every sketch stays in `src/` and every build stays compilable at
all times — which catches "I broke the old sketch while refactoring" immediately.
This was chosen deliberately over the file-move approach (Step 1.3).

Current `platformio.ini` (shared settings in `[env]`, one section per sketch):
```ini
[platformio]
default_envs = superloop            ; bare `pio run` builds the known-good controller

[env]                               ; inherited by all environments
platform = ststm32
board = nucleo_f446re
framework = arduino
lib_archive = false                 ; required for SimpleFOC under PlatformIO
lib_deps = …                        ; pinned to exact versions (Step 1.1)
build_flags =
    -D configMEMMANG_HEAP_NB=3      ; heap_3 (fixes the FreeRTOS boot hang)
    -Iinclude                       ; kernel TUs must see STM32FreeRTOSConfig.h (fact #1)

[env:superloop]
build_src_filter = +<*> -<p1_test.cpp> -<rtos_main.cpp>
[env:p1test]
build_src_filter = +<*> -<heading_control.cpp> -<rtos_main.cpp>
; [env:rtos] added in Phase 2: -<heading_control.cpp> -<p1_test.cpp>
```
Excluding a not-yet-created file (`rtos_main.cpp`) is harmless. Flash a specific
build with `pio run -e <env> -t upload`; compare against the baseline controller
with `-e superloop`.

### Header discipline

Every new header gets `#pragma once`. Infrastructure headers should include only
what they need — `timebase.h` needs `<Arduino.h>` for the CMSIS register
definitions and nothing else. Keep SimpleFOC out of the infrastructure headers
entirely; it drags in a lot and you want `trace.h` includable from
`FreeRTOSConfig.h`-adjacent code.

**One real gotcha:** `FreeRTOSConfig.h` is included by kernel C sources, so
anything it references (the `traceTASK_SWITCHED_IN` hooks, `rtAssertFail`) must
be declared with `extern "C"` and must be plain C-compatible. Put those
declarations in a small `trace_c.h` with an `#ifdef __cplusplus` guard rather
than pulling a C++ header into the kernel build.

### Files touched by phase

| Phase | Creates | Modifies |
|---|---|---|
| 0 | `timebase.*`, `timing_stats.h`, `hw_timers.*` | `heading_control.cpp` (instrumentation only) |
| 1 | `include/STM32FreeRTOSConfig.h`, `faults.*`, `p1_test.cpp` (throwaway harness), `trace_c.h`, `trace.*`, `plot_trace.py` | `platformio.ini` (env split, `-Iinclude`) |
| 2 | `rtos_main.cpp`, `tasks.*`, `control_law.*`, `sensors.*` | — |
| 3 | `telemetry.*` | `tasks.*`, `control_law.*` |
| 4 | — | `tasks.*`, `hw_timers.*`, `control_law.*` |
| 5 | — | `tasks.*`, `sensors.*`, `faults.*` |
| 6 | `commands.*` | `tasks.*`, `telemetry.*` |
| 7 | — | all, plus `CONTROL_README.md` |

---

## Phase map

| Phase | Name | Days | Tag |
|---|---|---|---|
| **0** | Measure the super-loop | 1–1.5 | `rtos-p0-baseline` |
| **1** | Kernel config + software tracer | 2 | `rtos-p1-kernel` |
| **2** | Single-task port | 0.5–1 | `rtos-p2-single-task` |
| **3** | Telemetry extraction | 0.5–1 | `rtos-p3-telemetry` |
| **4** | FOC split | 2 | `rtos-p4-foc-split` |
| **5** | Safety task + I2C mutex | 1 | `rtos-p5-safety` |
| **6** | Comms task | 0.5–1 | `rtos-p6-comms` |
| **7** | Consolidation and evidence | 1 | `rtos-p7-complete` |

**Total: 8.5–10.5 focused days.** Phase 1 is heavier than it looks because the
tracer is real infrastructure — but it pays for itself from Phase 2 onward and
is the thing that makes up for having no analyzer.

---

## The core mental model

Read this once. Everything in the guide follows from it.

**A task is a stack plus a saved register set.** `xTaskCreate` mallocs a stack
and a TCB. A context switch happens in `PendSV`: push R4–R11 (and S16–S31 if
the FPU context is active) onto the current PSP, store PSP into the current
TCB, pick the next TCB, load its PSP, pop. R0–R3, R12, LR, PC, xPSR and S0–S15
are already stacked by the exception entry hardware. That's the whole
mechanism — no magic.

**The scheduling rule is one sentence.** The highest-priority *Ready* task runs.
Always. Equal priorities round-robin on tick boundaries.

**Four states.** Running, Ready (wants CPU, something higher has it), Blocked
(waiting on a semaphore/queue/delay, with a timeout), Suspended. The critical
corollary: **a task that never enters Blocked starves everything below it
completely.** Not "slows" — starves. Every `for(;;)` body must reach a blocking
call.

**The scheduler is entered from SysTick and from any API call that could change
readiness.** From a task, `xSemaphoreGive` reschedules immediately. From an ISR
it does *not*, unless you call `portYIELD_FROM_ISR`.

---

# Phase 0 — Measure the super-loop

**Goal:** a timebase, a timer audit, real worst-case execution times, and a
frozen baseline. No FreeRTOS yet.

## Step 0.1 — Freeze the baseline

```
git checkout -b rtos-migration
git tag rtos-p0-start
cp src/heading_control.cpp baseline/heading_control_baseline.cpp
```

Archive your run-8 envelope data under `baseline/golden/` and record, from it:

```
T90   rise ___ s   overshoot ___ %   settle ___ s   final err ___ °   end ω_w ___ rad/s
T-90  rise ___ s   overshoot ___ %   settle ___ s   final err ___ °   end ω_w ___ rad/s
T30   settle ___ s   final err ___ °
H0 nudge: returns to within ___ °
Passive unwind: ω_w decays from ___ to ___ over ___ s
```

That last line is the compFrac margin check. It's the one you'll repeat most.

---

## Step 0.2 — Independent microsecond timebase (TIM5)

**Concept.** `micros()` is built on SysTick, and FreeRTOS wants SysTick. Your
measurement instrument must not depend on the thing you're changing. TIM5 on the
F446 is 32-bit; at 1 MHz it wraps every 71.6 minutes.

**Do.** Two new files in `src/`. This is infrastructure — it outlives every
phase, so it does not go inline in a sketch.

`src/timebase.h`
```c
#pragma once
#include <Arduino.h>

/* Free-running 32-bit microsecond counter on TIM5.
   Independent of SysTick, so it survives the FreeRTOS handoff.
   Wraps every ~71.6 minutes; unsigned subtraction handles the wrap. */
void us_init(void);

static inline uint32_t us_now(void) { return TIM5->CNT; }

/* Elapsed microseconds since `t0`, correct across one wrap. */
static inline uint32_t us_since(uint32_t t0) { return TIM5->CNT - t0; }
```

`src/timebase.cpp`
```c
#include "timebase.h"

void us_init(void) {
  __HAL_RCC_TIM5_CLK_ENABLE();
  TIM5->CR1 = 0;                                       /* stop before config */
  TIM5->PSC = (uint32_t)(getTimerClkFreq(TIM5) / 1000000UL) - 1;
  TIM5->ARR = 0xFFFFFFFFUL;
  TIM5->EGR = TIM_EGR_UG;                              /* latch PSC/ARR now  */
  TIM5->SR  = 0;                                       /* clear the UG flag  */
  TIM5->DIER = 0;                                      /* no interrupts      */
  TIM5->CR1 = TIM_CR1_CEN;
}
```

Then in your sketch:
```c
#include "timebase.h"
...
void setup() {
  Serial.begin(115200);
  us_init();              /* before anything you intend to time */
  ...
}
```

Why `us_since` exists rather than writing `us_now() - t0` inline: unsigned
subtraction is correct across the wrap, but only if both operands stay
`uint32_t`. Wrapping it in a function means you can't accidentally promote one
side to `int` or `unsigned long long` in an expression and silently break the
rollover behavior.

**Verify (no scope needed).** Compare against a known-good reference:
```c
uint32_t a = us_now();
delay(1000);
Serial.printf("1000 ms measured as %lu us\n", us_since(a));
```
Expect 1,000,000 ± a few hundred.

| Reading | Meaning |
|---|---|
| ~1,000,000 | correct |
| ~500,000 | prescaler is 2× too large — you assumed the APB1 *bus* clock |
| ~2,000,000 | prescaler 2× too small |
| 0 or frozen | `CEN` not set, or the TIM5 clock isn't enabled |
| wildly unstable | TIM5 is being reconfigured by something else — Step 0.3 |

APB1 timers on the F446 run at **2× the APB1 bus clock** when the APB1 prescaler
is anything other than 1. That's what `getTimerClkFreq` accounts for. Don't
hardcode 89.

**Trap 1.** `getTimerClkFreq` lives in the STM32duino core's `HardwareTimer.h`.
If it doesn't resolve, add `#include <HardwareTimer.h>` to `timebase.cpp`, or
fall back to `HAL_RCC_GetPCLK1Freq() * 2`.

**Trap 2.** Confirm TIM5 isn't claimed by SimpleFOC before trusting any of this
— that's Step 0.3, and if TIM5 is taken, come back here and pick another 32-bit
timer (on the F446 the only other one is TIM2, which is definitely taken).

**Trap 3.** Call `us_init()` *after* `Serial.begin()` but *before*
`driver.init()`. If SimpleFOC does touch TIM5, initializing yours second means
you'd overwrite its config rather than the reverse — which is a louder, easier
failure to spot than a silently wrong PWM period.

---

## Step 0.3 — Timer audit

**Concept.** `CONTROL_README` documents the TIM2/TIM3 PWM collision. But D10 is
**PB6 = TIM4_CH1**, and if SimpleFOC configured it as a timer output rather than
a plain GPIO enable, TIM4 is taken too. Find out rather than guess.

**Do.** New files, `src/hw_timers.h` / `src/hw_timers.cpp`. Step 1.5 will add
the FOC tick timer to the same pair, so give it a home now.

`src/hw_timers.h`
```c
#pragma once
#include <Arduino.h>

void timers_dumpAll(void);          /* Step 0.3 — audit */
/* void focTick_init(uint32_t hz); */  /* Step 1.5 — added later */
```

`src/hw_timers.cpp`
```c
#include "hw_timers.h"

static void dumpTimer(const char* n, TIM_TypeDef* t) {
  Serial.printf("%-6s CEN=%lu ARR=%-6lu PSC=%-6lu CCER=%04lX CCMR1=%04lX DIER=%04lX\n",
    n, (unsigned long)(t->CR1 & TIM_CR1_CEN), (unsigned long)t->ARR,
    (unsigned long)t->PSC, (unsigned long)t->CCER,
    (unsigned long)t->CCMR1, (unsigned long)t->DIER);
}

void timers_dumpAll(void) {
  Serial.println(F("--- timer audit ---"));
  dumpTimer("TIM1",  TIM1);   dumpTimer("TIM2",  TIM2);
  dumpTimer("TIM3",  TIM3);   dumpTimer("TIM4",  TIM4);
  dumpTimer("TIM5",  TIM5);   dumpTimer("TIM8",  TIM8);
  dumpTimer("TIM9",  TIM9);   dumpTimer("TIM10", TIM10);
  dumpTimer("TIM11", TIM11);  dumpTimer("TIM12", TIM12);
  Serial.println(F("-------------------"));
}
```

Call `timers_dumpAll()` at the **end** of `setup()`, after `driver.init()` and
`motor.initFOC()` — before those, nothing is configured and the dump is
meaningless.

**Reading the dump:**

| Pattern | Meaning |
|---|---|
| `CEN=0`, everything zero | free |
| `CEN=1`, `CCER≠0` | driving an output — off limits |
| `CEN=1`, `CCER=0`, `DIER≠0` | someone's using it for interrupts only |
| `CEN=1`, `CCER=0`, `DIER=0` | running but unused — suspicious, investigate |

**Verify.** Fill in Appendix B. Expect TIM2/TIM3 taken. Choose the FOC tick
timer from what's free, preferring **TIM9** (16-bit, APB2, nothing else wants
it), then TIM11, then TIM4 only if confirmed free.

**Trap.** Reading a timer's registers with its peripheral clock disabled returns
garbage or hard-faults on some parts. If a dump line looks nonsensical, that's
usually a disabled clock — which itself means the timer is free.

---

## Step 0.4 — Measure worst-case execution times

**Concept.** Average execution time is nearly useless for real-time work. A loop
that averages 200 µs but occasionally takes 4 ms will miss deadlines, and the
average won't tell you. You need **max**.

**Do.** New header-only file `src/timing_stats.h`. Header-only because it's all
`static inline` — no `.cpp` needed, and the compiler can fold `TIME_BLOCK` down
to a couple of instructions.

```c
#pragma once
#include "timebase.h"

typedef struct { uint32_t n, mn, mx; uint64_t sum; } stat_t;

static inline void stat_reset(stat_t* s) { s->n = 0; s->mn = 0; s->mx = 0; s->sum = 0; }

static inline void stat_add(stat_t* s, uint32_t v) {
  if (!s->n || v < s->mn) s->mn = v;
  if (v > s->mx)          s->mx = v;
  s->sum += v; s->n++;
}

static inline void stat_print(const char* name, const stat_t* s) {
  if (!s->n) { Serial.printf("%-18s (no samples)\n", name); return; }
  Serial.printf("%-18s n=%-8lu min=%-7lu mean=%-7lu MAX=%lu\n",
    name, (unsigned long)s->n, (unsigned long)s->mn,
    (unsigned long)(s->sum / s->n), (unsigned long)s->mx);
}

/* Time a block and fold the result into a stat_t.
   Usage: TIME_BLOCK(st_foc, { motor.loopFOC(); });                    */
#define TIME_BLOCK(st, code) do { uint32_t _a = us_now(); \
                                  code; \
                                  stat_add(&(st), us_since(_a)); } while (0)
```

Then in the sketch, one `stat_t` per block you're measuring, plus an `M` command
that calls `stat_print` for each and a `M!` (or similar) that resets them all.

```c
static stat_t st_foc, st_move, st_mpu, st_ina, st_law, st_telem, st_period;
```

Instrument, with the motor **spinning** (not idle — the feedforward branches and
`sign()` paths only execute under motion):

Instrument, with the motor **spinning** (not idle — the feedforward branches and
`sign()` paths only execute under motion):

| Block | Why it matters |
|---|---|
| `motor.loopFOC()` | sets the FOC task budget |
| `motor.move()` | should be trivial in voltage-torque mode |
| MPU6050 read | the big blocking item in the control path |
| INA219 read (both regs) | moves to the safety task in Phase 5 |
| control law, steps 1–8 | the actual compute |
| one telemetry row `sprintf` + write | moves off the control path in Phase 3 |
| whole super-loop iteration period | your current effective rate |

**Verify.** Fill in:

```
                        n        min      mean      MAX
loopFOC()            375608     31 us    32 us    40 us
move()               375608      3 us     3 us     8 us
MPU6050 read           5616   2372 us  2372 us  2374 us
INA219 read              --       --       --       --   (not read in this sketch)
control law            5616   2383 us  2386 us  2390 us   (incl. MPU; compute ~14 us)
telemetry row           123    227 us   231 us   237 us
super-loop period    375607     38 us    75 us  2666 us
```

Run it for at least 60 seconds including a `T90` and an `H0` so the max is real.

**Results (2026-08-02, ~28 s run w/ HOLD nudges).** Every deterministic block has
min ~= mean ~= MAX, so the numbers are trustworthy despite the run being short of
60 s. Three findings:

- **The MPU read IS the control path.** Pure control-law compute is
  `st_law - st_mpu ~= 14 us`; the gyro read is 2372 us, 170x larger. The LQR /
  feedforward / linearisation math is free by comparison.
- **Super-loop MAX = 2666 us is FOC starvation, measured.** Control fires once per
  ~67 FOC iterations (375608 / 5616 = 66.9); on that iteration `loopFOC()` does not
  run for 2.37 ms while the blocking I2C read holds the CPU. So commutation stalls
  ~2.4 ms out of every 5 ms. This is the number that justifies the FOC split
  (Phase 4): the 4 kHz timer ISR preempts the I2C read, keeping commutation regular.
- **`loopFOC()` at 40 us MAX is well under the 50 us line** — the FOC rate is
  cheap, see Step 0.5.

**Known optimisation, deferred (verbatim rule).** 2.37 ms is ~7x the ~350 us a
14-byte burst should cost at 400 kHz; 100 kHz -> 400 kHz only bought 1.56x, so the
cost is per-transaction overhead in Adafruit_BusIO, not transfer. `getEvent()`
reads accel+gyro+temp but the law only uses `g.gyro.z`; a targeted 2-byte read of
`GYRO_ZOUT_H/L` (0x47/0x48) could cut it several-fold. Phase 4 preemption makes it
non-critical, so this is logged, not done. If revisited: fix in
`heading_control.cpp`, re-run the golden dataset, confirm, then carry across.

**Trap.** `Serial.printf` inside `TIME_BLOCK` will dominate everything it
touches. Measure the `sprintf` and the write separately from the control path.

---

## Step 0.5 — Determine the required FOC rate

**Concept.** FOC commutation rate is set by *electrical* frequency, not by
control bandwidth. Undersampled commutation doesn't degrade gracefully — the
angle used for the inverse Park transform goes stale, torque drops, and the
error scales with speed.

**Do.**
1. Read your pole-pair count from the `BLDCMotor(N)` constructor. Record it.
2. `f_elec = PP × ω_max / (2π)`, with `ω_max = 45 rad/s` (your abort limit).
3. Minimum useful rate = `20 × f_elec`. Comfortable = `40 × f_elec`.

| PP | `f_elec` | min | comfortable |
|---|---|---|---|
| 7 | 50 Hz | 1.0 kHz | 2.0 kHz |
| 11 | 79 Hz | 1.6 kHz | 3.2 kHz |
| 14 | 100 Hz | 2.0 kHz | 4.0 kHz |

4. Check affordability against your measured `loopFOC()` max: utilization is
   `rate × WCET`. 4 kHz × 40 µs = 16%. 4 kHz × 150 µs = 60% and you'd need to
   fix the SPI read first.

**Verify.** Record the chosen rate in Appendix B *with the arithmetic*.

**Trap.** If `loopFOC()` is much slower than ~50 µs, check the MT6701 SPI clock
before lowering the FOC rate. A 24-bit SSI frame at 1 MHz is 24 µs of pure
transfer; at 8 MHz it's 3 µs. Raise the clock, not the period.

---

## Step 0.6 — Re-capture the golden dataset with the current build

**Do.** With the `M` instrumentation in place (it costs a few µs, so the
baseline should include it), run:

```
Z T30    Z T-30
Z T90    Z T-90
Z T180
H0 + three hand nudges
G        (gains and state)
M        (timing)
```

`Z` before every `T` — carried-over heading silently turns a `T-90` into a 180°
slew.

**Verify.** Results match your run-8 numbers. If they don't, something drifted
(temperature, battery, mechanical) and you need to know that *now*, not after
Phase 2.

**Phase 0 exit:** `git tag rtos-p0-baseline`

---

# Phase 1 — Kernel configuration and the software tracer

**Goal:** a FreeRTOS build you trust, assertions that actually fire, and a
scheduler tracer that substitutes for a logic analyzer.

## Step 1.1 — Identify the port and prove the FPU context is saved

**Concept.** Cortex-M4F has 32 single-precision FPU registers. S0–S15 are
caller-saved and the exception entry hardware stacks them automatically (lazy
stacking). **S16–S31 are callee-saved and the kernel must save them across a
context switch.** Only the `ARM_CM4F` port does this. On `ARM_CM3`, floats in
multiple tasks corrupt each other *silently* — no fault, just wrong numbers.

Your control law, feedback linearization, and SimpleFOC's Clarke/Park transforms
are all float-heavy and will live in different tasks. This is not optional.

**Do.** Pin the library in `platformio.ini` rather than letting the LDF choose:
```ini
lib_deps =
    askuric/Simple FOC@2.4.0
    stm32duino/STM32duino FreeRTOS
    ; existing Adafruit deps
```
Then find the compiled port:
```
find .pio -name "port.c" -path "*FreeRTOS*"
```

**Verify.** The path must contain **`ARM_CM4F`**. Second check, in the
disassembly:
```
arm-none-eabi-objdump -d .pio/build/*/firmware.elf | grep -A40 "<PendSV_Handler>"
```
You must see `vstmdb`/`vldmia` on `s16` in `PendSV_Handler`, plus a
`tst lr, #0x10` test (that's the EXC_RETURN bit that says whether FPU context is
active). If those aren't there, you're on the wrong port.

**Trap.** Some ports select by `#define` rather than directory. The objdump
check is authoritative; trust it over the path.

**Result (2026-08-03) — CONFIRMED `ARM_CM4F`.** The STM32duino `src/port.c`
selects the variant at compile time: `__CORTEX_M == 4 && __FPU_PRESENT == 1`
(MPU branch `#if 0`'d) → `#include ".../ARM_CM4F/port.c"`. Objdump of the actual
compiled `port.c.o` (authoritative, per the trap) shows in `PendSV_Handler`:
`tst.w lr, #16` (EXC_RETURN bit-4 test), `vstmdbeq r0!, {s16-s31}` (save
callee-saved FPU regs), `vldmiaeq r0!, {s16-s31}` (restore) — plus the
`vPortEnableVFP` symbol. The conditional (`eq`) is lazy FPU stacking: S16–S31 are
saved only for tasks that actually touched the FPU. VFP instructions being
emitted at all also proves the hard-float ABI (`-mfpu=fpv4-sp-d16
-mfloat-abi=hard`). Float-heavy control law and FOC transforms in separate tasks
are safe.

---

## Step 1.2 — Write `FreeRTOSConfig.h` deliberately

**Concept.** The config file is the contract between you and the kernel.
`configASSERT` in particular is not decoration — FreeRTOS has extensive internal
assertions that catch the interrupt-priority bug in Step 1.3, mutex-from-ISR,
and blocking-before-scheduler-start. Without it those failures are silent
corruption.

**Do.** Set these explicitly:

```c
#define configUSE_PREEMPTION                    1
#define configCPU_CLOCK_HZ                      SystemCoreClock
#define configTICK_RATE_HZ                      1000
#define configMAX_PRIORITIES                    6
#define configMINIMAL_STACK_SIZE                128     /* WORDS not bytes */
#define configTOTAL_HEAP_SIZE                   (32 * 1024)
#define configUSE_16_BIT_TICKS                  0
#define configUSE_MUTEXES                       1
#define configUSE_TASK_NOTIFICATIONS            1
#define configUSE_STREAM_BUFFERS                1
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_TRACE_FACILITY                1
#define configGENERATE_RUN_TIME_STATS           1
#define configUSE_STATS_FORMATTING_FUNCTIONS    1

#define configASSERT(x) if((x)==0){ taskDISABLE_INTERRUPTS(); rtAssertFail(__FILE__,__LINE__); }

#define configPRIO_BITS                              4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY      15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()  /* TIM5 already running */
#define portGET_RUN_TIME_COUNTER_VALUE()          (TIM5->CNT)

/* Tracer hooks — Step 1.4 */
#define traceTASK_SWITCHED_IN()   traceIn((void*)pxCurrentTCB)
#define traceTASK_SWITCHED_OUT()  traceOut()
```

Implement three failure hooks in new files `src/faults.h` / `src/faults.cpp`.
**All three must kill the motor first.**

`src/faults.h`
```c
#pragma once
#include <stdint.h>

typedef enum {
  FAULT_NONE = 0, FAULT_ASSERT, FAULT_STACK_OVERFLOW, FAULT_MALLOC,
  FAULT_SCHEDULER_RETURNED, FAULT_WHEEL_SAT, FAULT_UNDERVOLT,
  FAULT_OVERCURRENT, FAULT_HEARTBEAT, FAULT_I2C_TIMEOUT
} fault_t;

void faults_safeStop(fault_t r);   /* the ONE stop path — used by everything */
void faults_reportLastBoot(void);  /* prints the .noinit record at startup   */

#ifdef __cplusplus
extern "C" {
#endif
void rtAssertFail(const char* file, int line);
#ifdef __cplusplus
}
#endif
```

`rtAssertFail` needs `extern "C"` because `FreeRTOSConfig.h` is included by the
kernel's C sources — a C++-mangled symbol won't link.

The black box:
```c
typedef struct { uint32_t magic; fault_t reason; int line; char file[24]; }
  fault_record_t;
__attribute__((section(".noinit"))) static fault_record_t g_fault;
```
`.noinit` survives a reset because the startup code doesn't zero it. Guard with
a magic value so you don't read garbage on a cold power-up. Print it at the top
of `setup()` — with no debugger attached, this is how you find out what happened.

Every stop path does the same three things in the same order: disable the driver,
zero the target, latch the reason. Then blink a distinctive LED pattern forever.

Each should: `motor.target = 0; motor.disable(); driver.disable();` then store
`file`/`line` in a `__attribute__((section(".noinit")))` struct with a magic
value, then blink a distinctive LED pattern forever. Print the stored reason at
the top of `setup()` on the next boot — that's your black box, and without a
debugger attached it's how you'll find out what happened.

**Trap.** Every stack size in FreeRTOS on ARM is in **words** (4 bytes).
`xTaskCreate(..., 128, ...)` gives you 512 bytes. A large share of "random
crashes" reports are this.

**Result (2026-08-03) — files written, compiles clean.** Three files:
`src/STM32FreeRTOSConfig.h`, `src/faults.h`, `src/faults.cpp`. Build SUCCESS
(RAM 35.5% / 46.5 KB, Flash 14.6%), no warnings, no hook collisions.

STM32duino-specific decisions worth remembering:
- **Config filename must be `STM32FreeRTOSConfig.h`, not `FreeRTOSConfig.h`.** The
  library's wrapper `FreeRTOSConfig.h` does `#if __has_include("STM32FreeRTOSConfig.h")`
  → full override (skips `FreeRTOSConfig_Default.h`). Naming it `FreeRTOSConfig.h`
  would be silently ignored and you'd run the library defaults.
- **It must live in `include/` AND `platformio.ini` must add `-Iinclude` to
  `build_flags`** (fixed 2026-08-03, Step 1.3). Two-part trap:
  - PlatformIO gives the project `src/` dir to *project* TUs only, so a config in
    `src/` is invisible even to your own `#include`s from the kernel side — it must
    be in `include/`.
  - But `include/` is on the path for *project* TUs only too. The FreeRTOS
    **library/kernel** TUs (`port.c`, `tasks.c`, …) do NOT get it unless you force
    it globally with `-Iinclude` in `build_flags`. Without the flag you get a
    SPLIT-BRAIN: project files (`faults.cpp`, the sketch) use your config while the
    kernel silently uses `FreeRTOSConfig_Default.h` — whose `configASSERT` is a
    bare `for(;;)` hang. Symptom that cost us the most time: a thread-context
    `configASSERT` blinked the LED and latched the black box (project TU, our
    config), but the priority-0 ISR froze with NO LED and NO record (kernel TU in
    `port.c`, default config). It is also a latent corruption risk — kernel and app
    disagreeing on TCB layout (`configNUM_THREAD_LOCAL_STORAGE_POINTERS`, etc.).
  - **Verify** with a temporary `#pragma message("...")` in the config + a clean
    build: it must fire for the kernel TUs (`port.c`, `tasks.c`, `queue.c`, `heap.c`,
    …), not just `faults.cpp`/the sketch. We saw 13 TUs once fixed, 2 before.
- **Own the hooks via `_BLINK=0`.** The library defines `vApplicationMallocFailedHook`
  / `vApplicationStackOverflowHook` **strong** (not weak), but only when
  `configUSE_MALLOC_FAILED_HOOK_BLINK` / `configCHECK_FOR_STACK_OVERFLOW_BLINK`
  are `1`. Set both `0` → library stays out, `faults.cpp` defines them, no
  multiple-definition error.
- **Do NOT alias `xPortSysTickHandler`.** STM32duino's core owns SysTick at 1 kHz
  and the library hooks it via `osSystickHandler()`; aliasing `SysTick_Handler`
  double-defines it. Consequence: `configTICK_RATE_HZ` MUST be 1000.
- **Handler aliases required:** `vPortSVCHandler→SVC_Handler`,
  `xPortPendSVHandler→PendSV_Handler`, or the vector table never reaches the port.
- **`INCLUDE_xTaskGetSchedulerState 1` is load-bearing, not optional.** STM32duino
  drives the kernel tick via core SysTick → `osSystickHandler()` → `xPortSysTickHandler()`,
  and `osSystickHandler` guards that call with
  `if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)`. That guard is
  `#if`'d on `INCLUDE_xTaskGetSchedulerState == 1`. With it 1, the super-loop and
  every RTOS `setup()` run safely before `vTaskStartScheduler()` (tick skipped,
  only `HAL_IncTick` runs). With it 0, the guard compiles out and the first
  SysTick before scheduler start calls `xPortSysTickHandler` into uninitialised
  task lists → hard fault. This is why the super-loop still works with FreeRTOS
  now linked in.
- **Run-time-stats clock** routed through `rtRunTimeCounter()` (returns `TIM5->CNT`,
  in `faults.cpp`) so the config header needn't include the CMSIS device header.
- **Decoupled safe-stop:** `faults_init(enablePin, ledPin, stopHook)`. Hardware
  kill (drive DRV8313 enable LOW) runs first, before the SimpleFOC graceful stop,
  so a corrupt motor object can't prevent the kill. `faults.*` stays SimpleFOC-free.

**Open contingency — the `.noinit` black box.** The F446RE variant ldscript has
NO `.noinit` section, so `g_fault` lands as an orphan; the `0x5A1F0B0B` magic
makes a stray zeroing benign (reads as "clean boot"). **Step 1.3 verifies it
empirically:** trip the assert, reset, and confirm `faults_reportLastBoot()`
prints `ASSERT at faults.cpp:NN`. If it prints "clean boot" instead, the orphan
got zeroed — add this project-local fragment (`ldscripts/noinit.ld`) and the flag
`-Wl,-T"${platformio.project_dir}/ldscripts/noinit.ld"` to `platformio.ini`:
```
SECTIONS {
  .noinit (NOLOAD) : { . = ALIGN(4); *(.noinit) *(.noinit*) . = ALIGN(4); } > RAM
} INSERT AFTER .bss;
```
This places `.noinit` above `_ebss` (startup's bss-clear skips it) and below `_end`
(heap starts after it) — guaranteed to survive a warm reset.

---

## Step 1.3 — Interrupt priorities, and proving the assert works

**Concept — the single most important thing in this document.** On Cortex-M,
**lower numeric priority value = higher urgency**. FreeRTOS protects its critical
sections by writing `configMAX_SYSCALL_INTERRUPT_PRIORITY` into `BASEPRI`, which
masks all interrupts at that numeric level *or higher numerically*. An interrupt
that is more urgent than `BASEPRI` is **not masked**, so it can preempt a kernel
critical section. If that ISR then calls a `...FromISR()` API, it corrupts kernel
data structures.

Arduino cores set many peripheral IRQ priorities to 0 — the most urgent possible,
and therefore illegal for any ISR that talks to FreeRTOS.

**Do — part 1, prove your safety net.** Deliberately break it. Configure a spare
timer IRQ at priority 0 and call `vTaskNotifyGiveFromISR()` from it.

**Verify.** `rtAssertFail` runs and your LED pattern blinks. If nothing happens,
your assertion path is broken — fix it now, because Phase 4 relies on it.

Then set the priority to 5 and confirm the assert stops firing.

**Do — part 2, audit everything.** Set every relevant IRQ priority explicitly.
Do not rely on defaults.

| Interrupt | Calls FreeRTOS API | Priority |
|---|---|---|
| SysTick, PendSV | yes (kernel) | 15 |
| FOC tick timer | yes | 5 |
| USART1 RX (USB) | from Phase 6 | 6 |
| USART3 RX (HC-05) | from Phase 6 | 6 |
| Anything SimpleFOC installed | no | record, leave |

**Do — part 3, check priority grouping.** All four priority bits must be
preemption bits. Read it back:
```c
Serial.printf("PRIGROUP=%lu\n", (SCB->AIRCR >> 8) & 7);
```
You want 3 (which on a 4-bit implementation means 4 preempt bits, 0 subpriority)
or lower numerically. If bits are allocated to subpriority, "priority 5" won't
preempt the way you think and `BASEPRI` masking becomes wrong. Fix with
`HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4)`.

**Result (2026-08-03) — PASSED, via a throwaway `p1test` env/sketch.** Harness:
`src/p1_test.cpp` (owns setup/loop under `[env:p1test]`), a TIM7 interrupt via the
HardwareTimer API (the core owns `TIM7_IRQHandler`, so a raw handler collides —
Step 1.5 uses TIM9 raw instead), NVIC priority forced to `TEST_IRQ_PRIO` after
`resume()`.
- **Part 1** (`TEST_IRQ_PRIO = 0`, verified NVIC priority reads back 0): the ISR's
  `vTaskNotifyGiveFromISR` trips `configASSERT` inside `vPortValidateInterruptPriority`
  → `rtAssertFail` → LD2 single-pulse blink; reset shows
  `previous boot died: ASSERT at port.c:756`. Kernel assert + `.noinit` black box
  both proven; **no linker fragment needed** (orphan `.noinit` survives warm reset).
- **Part 2** (`TEST_IRQ_PRIO = 5`): no assert, `notification N` streams at 4 Hz,
  a clean reset reports `clean boot`.
- **Part 3**: `PRIGROUP = 3` already (STM32duino default is group 4). No change.

**The debugging that mattered:** the priority-0 assert first appeared to do
nothing (froze, no LED, `clean boot`) while a *thread-context* `configASSERT`
worked. Root cause was the split-brain config (see Step 1.2 result) — the kernel's
`port.c` was compiled against `FreeRTOSConfig_Default.h` (silent `for(;;)` assert)
until `-Iinclude` put our config on the kernel's include path. The isolation
sequence that found it: (1) prove the machinery at prio 5 (notifications stream),
(2) prove the safe-stop path from thread context (LED + black box), (3) read the
ISR priority back (was correctly 0), (4) discover only 2 of 13 TUs saw our config.

**Full IRQ-priority audit** (SysTick/PendSV 15, FOC tick 5, USART 6) is deferred
to where those interrupts actually exist — 1.5 and Phase 6.

---

## Step 1.4 — Build the software scheduler tracer

**This is the logic analyzer replacement.** Budget half a day. It is the single
highest-leverage thing in the guide given your tooling.

**Concept.** FreeRTOS exposes empty macros at every context switch. Define them
and you capture the exact scheduler timeline — which task ran, when, for how
long, including idle. This is precisely what SEGGER SystemView does; you're
building a minimal version of it.

**Files.** Three, because of the C/C++ boundary:

- `src/trace_c.h` — the ID enum and the two hook function declarations, plain C,
  `extern "C"` guarded. This is what `FreeRTOSConfig.h` pulls in.
- `src/trace.h` — the C++ API (`trace_start`, `trace_stop`, `trace_dump`,
  `traceRec`).
- `src/trace.cpp` — the buffer and implementation.

`src/trace_c.h`
```c
#pragma once
#include <stdint.h>

enum { TRACE_ID_IDLE = 0, TRACE_ID_CTRL = 1, TRACE_ID_FOC = 2,
       TRACE_ID_SAFETY = 3, TRACE_ID_COMMS = 4, TRACE_ID_TELEM = 5 };

#ifdef __cplusplus
extern "C" {
#endif
void traceIn(void);
void traceOut(void);
#ifdef __cplusplus
}
#endif
```
`FreeRTOSConfig.h` then does `#include "trace_c.h"` and defines the two macros in
terms of those. Do **not** pull `trace.h` (or anything touching SimpleFOC or
Arduino C++ headers) into `FreeRTOSConfig.h` — it's included from kernel C files
and will not compile.

**Do — the ring buffer.**

```c
typedef struct __attribute__((packed)) {
  uint32_t t_us;
  uint8_t  id;      /* task index, 0 = idle */
  uint8_t  evt;     /* 0 = out, 1 = in, 2..n = user markers */
} trace_ev_t;

#define TRACE_N 4096                  /* 24 KB */
static trace_ev_t traceBuf[TRACE_N];
static volatile uint16_t traceHead = 0;
static volatile bool traceOn = false;

static inline void traceRec(uint8_t id, uint8_t evt) {
  if (!traceOn) return;
  uint16_t h = traceHead;
  traceBuf[h].t_us = TIM5->CNT;
  traceBuf[h].id   = id;
  traceBuf[h].evt  = evt;
  traceHead = (h + 1) & (TRACE_N - 1);
}
```

Power-of-two size so the wrap is a mask, not a modulo. No locking: the writer is
always the kernel at switch time (interrupts already masked), so it's
effectively atomic with respect to itself.

**Do — the hooks.** `traceTASK_SWITCHED_IN` runs inside the scheduler with
`pxCurrentTCB` valid. You need a task index, not a pointer. Register each task
handle in a small array at creation and look it up, or — simpler and faster —
stash the index in the task's thread-local storage pointer:

```c
vTaskSetThreadLocalStoragePointer(hFoc, 0, (void*)1);
/* then */
static inline void traceIn(void* tcb) {
  traceRec((uint8_t)(uintptr_t)pvTaskGetThreadLocalStoragePointer(NULL, 0), 1);
}
static inline void traceOut(void) {
  traceRec((uint8_t)(uintptr_t)pvTaskGetThreadLocalStoragePointer(NULL, 0), 0);
}
```
Requires `configNUM_THREAD_LOCAL_STORAGE_POINTERS 1`. Idle's pointer is NULL,
which reads as 0 — exactly what you want.

**Do — user markers.** Add explicit events for things a switch trace can't see:
```c
#define TRC_CTRL_I2C_START  10
#define TRC_CTRL_I2C_END    11
#define TRC_STEP_BEGIN      20
#define TRC_SAFE_STOP       30
```
Cost is ~15 cycles per event, ~80 ns at 180 MHz. At 4 kHz FOC (2 events per
iteration) that's 0.06% overhead. Negligible.

**Do — the dump command.** Add `Y`: stop tracing, walk the buffer from
`traceHead` forward (oldest first), emit CSV, restart. Use the same framing your
`capture_calibration.py` already detects:

```
# trace: n=4096 events, wrap=yes
t_us,id,evt
```

**Do — the plot script.** `plot_trace.py`, alongside your existing pipeline.
Reads the CSV, reconstructs intervals by pairing `in`/`out` per task, and draws
a `broken_barh` Gantt chart with one row per task. Add a second panel showing the
histogram of inter-arrival times per task. Roughly 80 lines of matplotlib.

**Verify.** Create two throwaway tasks — one at priority 3 running 100 µs every
1 ms, one at priority 1 spinning continuously. Trace and plot. You should see:
- the high-priority task appearing exactly every 1000 µs
- the low-priority task's bars visibly chopped at those instants
- idle absent (the spinner never blocks)

If the picture matches what the scheduling rule predicts, your tracer is
trustworthy and you can use it as ground truth for the rest of the guide.

**Trap 1.** Reading the buffer while tracing is on gives torn data. `Y` must set
`traceOn = false` first.

**Trap 2.** Dumping 4096 events as CSV over 115200 baud takes ~1.5 seconds. Do
not dump while the controller is running a step. Capture, stop, then dump.

**Trap 3.** `traceTASK_SWITCHED_OUT` runs before the TCB pointer changes, so
both hooks see the *outgoing* task in `traceOut` and the *incoming* one in
`traceIn`. Verify this empirically with the two-task test rather than trusting
the reasoning.

**Result (2026-08-04) — PASSED, tracer is trustworthy.** Files:
`include/trace_c.h` (kernel-visible, in include/ for the same -Iinclude reason as
the config), `src/trace.h`, `src/trace.cpp`, `tools/plot_trace.py`. The switch
hooks read the current task's index from TLS slot 0
(`pvTaskGetThreadLocalStoragePointer(NULL,0)`), set per task at creation; idle's
NULL slot reads as 0 = IDLE for free. `Y` command dumps the 4096-entry ring as
CSV (`t_us,id,evt`; 0=out 1=in). Verified with two throwaway tasks (periodic
prio 3, ~100 µs every 1 ms; spinner prio 1, never blocks), captured over serial,
plotted:
- **TEST (periodic) period: min = med = max = 1000 µs** — zero jitter at µs
  resolution. n=1023.
- **TEST run 103 µs, spinner run 895 µs** (103 + 895 + ~2 µs switch ≈ 1000).
- **IDLE absent** — 2048 events each for id 6/7, zero id-0, exactly as predicted
  (spinner always Ready → idle never runs).
- **Trap 3 confirmed empirically**: `…6,0` immediately followed by `…7,1`
  (TEST out → TEST2 in) — out/in labels are correct, no swap.
The Gantt shows the spinner's bars cleanly chopped every 1 ms by the periodic
task — preemption made visual. Two engineering notes for reuse: the 24 KB
`traceBuf` is compiled into EVERY env (the global config defines the hooks, so
`trace.cpp` must link everywhere) — dead weight in `superloop`, a Phase-7 trim
candidate. And `plot_trace.py` forces the Agg backend so it never opens a
blocking window (savefig only).

---

## Step 1.5 — Configure the FOC tick timer

**Do.** Configure the timer chosen in Step 0.3 at the rate from Step 0.5. IRQ
priority 5. ISR body for now: increment a counter.

**Verify without a scope.** Count ISR invocations over a TIM5-measured interval:
```c
uint32_t c0 = tickCount, t0 = us_now();
vTaskDelay(pdMS_TO_TICKS(5000));
Serial.printf("rate = %.2f Hz\n", (tickCount - c0) * 1e6f / (us_now() - t0));
```
Expect your target ± 0.1%.

For jitter, timestamp inside the ISR and accumulate min/max of the delta. Target
max deviation under 3 µs. Anything larger means a higher-priority ISR is
interfering — find it.

**Trap.** If you used the Arduino `HardwareTimer` API, re-run the Step 0.3 timer
dump afterwards and confirm TIM2/TIM3 registers are unchanged. `HardwareTimer`
can silently reconfigure a timer SimpleFOC owns. If anything moved, drop to
direct register configuration.

**Result (2026-08-04) — PASSED.** In `hw_timers.*`: `focTick_init(hz)`,
`focTick_count/resetStats/jitter`. **Correction to the plan:** raw
`TIM1_BRK_TIM9_IRQHandler` is NOT usable — the STM32duino core strongly defines it
in `HardwareTimer.cpp` (`#if TIM9_BASE`, always true here), so a raw handler
collides at link exactly like TIM7. Used the `HardwareTimer(TIM9)` API +
`attachInterrupt`, forcing `HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 5, 0)` after
`resume()`. The ISR callback timestamps against the independent TIM5 timebase.
Measured over a 5 s window: **rate 3999.98 Hz** (0.0005% error, target ±0.1%),
**inter-fire dt min=max=250 µs → jitter 0 µs**. Zero jitter is expected: at
priority 5 the tick is more urgent than SysTick (15), nothing sits at 0–4, and the
report task is blocked (no serial TX) during the window, so nothing delays ISR
entry. This carries into Phase 4 unchanged — the callback body just gains the
`CTRL_DIVISOR` counter + the FOC/control `...GiveFromISR` notifies. The
`HardwareTimer(TIM9)`-touches-only-TIM9 check is deferred to Phase 4 (p1test has no
SimpleFOC, so TIM2/TIM3 are clock-disabled anyway).

**Phase 1 exit:** `git tag rtos-p1-kernel`. No application code changed, so no
golden dataset re-run needed.

---

# Phase 2 — Single-task port

**Goal:** the entire existing super-loop, **behaviorally unmodified**, inside one
task in a new sketch. Highest value, lowest risk step in the guide — provided you
resist reorganizing while you port.

## Step 2.0 — Port order

**Concept.** You're writing `rtos_main.cpp` from scratch and moving code across.
The risk is changing two things at once: the environment *and* the application.
The mitigation is a fixed port order with a compile-and-flash checkpoint at each
stage, so a break is attributable.

Port in this order. Flash and check at each numbered stop.

| # | Move across | Check |
|---|---|---|
| 1 | Includes, globals, constants, pin defines | Compiles. Diff the constants block against the baseline character by character — this is where a typo does the most damage. |
| 2 | Hardware init (SPI, encoder, driver, `motor.init/initFOC`, Wire, MPU6050, INA219) into `hwSetup()` | Flash. Boot text matches baseline. Encoder reads sane angle. |
| 3 | Sensor reads into `sensors.cpp` | Gyro and INA219 values match baseline at rest. |
| 4 | The §9 control law into `control_law.cpp`, unchanged | Compiles. Not yet called. |
| 5 | Wire it together as a plain super-loop in `loop()` — **no FreeRTOS yet** | **Full golden dataset. Must match.** This proves the port is clean before the scheduler is a variable. |
| 6 | Wrap in one task, start the scheduler (Step 2.1) | **Full golden dataset again.** Any difference now is FreeRTOS. |

Stop 5 is the one that makes this approach safe. Don't skip it to save a flash
cycle — it's the entire reason the from-scratch path is as trustworthy as an
in-place edit.

**Port verbatim.** Copy-paste, don't retype. Don't rename, don't reorder, don't
clean up, don't fix the thing that's been bothering you. If you find a real bug,
fix it in `heading_control.cpp` first, re-run the golden dataset there, confirm
the change, then carry it across. Otherwise you've moved the reference.

**Trap — the constants block.** `A_1 = 45.5`, `A_2 = 5.35`, `compFrac = 0.89`,
`K_θ = 119.3`, `K_ω = 35.1`, `ffFrac = 0.90`, `GYRO_SIGN = −1`, deadzones,
`ALPHA_STALL_MAX`, `STALL_WW`, `WHEEL_SAT_LIMIT`. A single transposed digit here
produces behavior that looks like an RTOS bug and isn't. Diff them mechanically
rather than reading them.

---

## Step 2.1 — Wrap the loop in one task

**Concept.** Every environmental difference between bare metal and running under
a scheduler — stack, FPU, tick, `micros()`, interrupt priorities — hits you here.
Because stop 5 already proved the application code is faithful, anything that
changes now is the environment.

**Do.** `src/rtos_main.cpp`:
```c
#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "timebase.h"
#include "timing_stats.h"
#include "hw_timers.h"
#include "faults.h"
#include "trace.h"
#include "tasks.h"

void setup() {
  Serial.begin(115200);
  us_init();
  faults_reportLastBoot();      /* the .noinit black box from Step 1.2 */
  tasks_createAll();
  vTaskStartScheduler();
  /* unreachable — if we get here, the heap was too small */
  faults_safeStop(FAULT_SCHEDULER_RETURNED);
}

void loop() {}                  /* never runs */
```

`src/tasks.cpp`:
```c
TaskHandle_t hControl;

static void controlTask(void*) {
  hwSetup();                    /* everything setup() did after Serial.begin */
  for (;;) superLoopBody();     /* the ported loop body, verbatim */
}

void tasks_createAll(void) {
  configASSERT(xTaskCreate(controlTask, "ctrl", 1536, NULL, 3, &hControl) == pdPASS);
  vTaskSetThreadLocalStoragePointer(hControl, 0, (void*)TRACE_ID_CTRL);
}
```

Note `hwSetup()` runs **inside** the task, not in `setup()`. SimpleFOC's
`initFOC()` alignment routine spins the motor and takes hundreds of milliseconds;
doing it before the scheduler starts works, but doing it inside the task means it
uses the task's stack and the task's FPU context, which is what you want to be
testing. It also means the failure hooks are live while it runs.

1536 words (6 KB) is deliberately generous — SimpleFOC plus float `sprintf` in
one task. You'll trim in Phase 7.

**Verify — the important one.** Re-run the full Phase 0 golden dataset:

- [ ] `M` super-loop period within 5% of baseline
- [ ] `T90` rise, overshoot, settle, final error within ~10%
- [ ] `T-90` same
- [ ] `H0` nudge response qualitatively identical
- [ ] **Passive unwind: `ω_w` still decays after a slew, at a similar rate**

**Trap 1 — `micros()`, SysTick, and `ω_w`.** SimpleFOC computes
`shaft_velocity` as Δangle/Δt using its internal `_micros()`. Your feedback
linearization (`u = (α + compFrac·A_2·ω_w)/A_1`) depends on `ω_w`, and so does
your 15% passive-unwind margin. If the scheduler disturbed the timebase, `ω_w` is
wrong and the controller is quietly wrong.

**Check it explicitly and quantitatively:** command a known open-loop voltage,
let the wheel settle, and compare reported `shaft_velocity` against your
identified DC gain `K = 8.51 rad/s per volt`. At 2 V expect ~17.0 rad/s. If it's
off by more than a few percent, point SimpleFOC's timebase at TIM5.

**Trap 2 — `vTaskStartScheduler()` never returns.** Anything after it in
`setup()` is dead code. Immediate reset on boot is almost always heap
exhaustion — check that `vApplicationMallocFailedHook` fires and print the
reason.

**Trap 3 — nothing blocks yet.** Your single task never calls a blocking API, so
idle never runs. Expected for now. It also means `configCHECK_FOR_STACK_OVERFLOW`
only checks at switch time and there are no switches — so add a manual
`uxTaskGetStackHighWaterMark` readout to `G` now rather than trusting the hook.

**Verify with the tracer.** Trace and plot. One task, no switches, idle absent.
Boring — which is the point. It confirms the tracer works in the real firmware,
not just the toy test.

**Result (2026-08-06) — PASSED, monolithic port.** `src/rtos_main.cpp` =
`heading_control.cpp` + the RTOS wrap only (`diff` = 5 regions: includes, the `G`
stack-highwater line, `setup`→`hwSetup`, `loop`→`superLoopBody`, appended
`controlTask`+`setup`). One task, prio 3, 1536-word stack, `hwSetup()` (incl.
`initFOC` + `faults_init`) run in-task. Golden-dataset gate, all measured:
- T90 final err **+1.54°**, T−90 **+0.68°** — in-deadzone, baseline spread; both
  wheels **unwound to 0** (compFrac margin intact — the key FreeRTOS-didn't-break-it
  check).
- **O2 → ω_w plateau 17.2 rad/s at 2.00 V, K=8.59 vs 8.51 (<1%)** — `micros()`/
  velocity estimate provably undisturbed (Trap 1 cleared, Appendix B8).
- `M` timing identical to Phase 0.4 (loopFOC 39µs, compute ~12µs, superloop
  min 38µs) — **zero FreeRTOS overhead** on the control path (single task never
  blocks → no switches during normal running). Tracer would be empty for the same
  reason, so its check is moot here.
No behavioral or timing regression. The controller is now a FreeRTOS task.

**Phase 2 exit:** `git tag rtos-p2-single-task`

---

# Phase 3 — Telemetry extraction

**Goal:** bursty output off the control path. Your first real preemption.

## Step 3.1 — Stream buffer between control and telemetry

**Concept.** A stream buffer is a lock-free single-producer/single-consumer byte
FIFO. Lock-free is the point: the producer (control) must **never** block, and a
mutex could make it. There's no mutual exclusion because head and tail are
written by exactly one side each, and on Cortex-M a 32-bit aligned store is
atomic.

**Do.**

1. A fixed-size POD sample. No pointers, no formatting yet:
```c
typedef struct __attribute__((packed)) {
  uint32_t t_us;
  float target_deg, theta_deg, omega_p, omega_w, alpha, u;
} LogSample_t;                                     /* 28 bytes */
```

2. `logStream = xStreamBufferCreate(96 * sizeof(LogSample_t), sizeof(LogSample_t));`

3. Control task, **timeout 0** so it can never block:
```c
if (xStreamBufferSend(logStream, &s, sizeof s, 0) != sizeof s) logDrops++;
```

4. Telemetry task, priority 1, stack 1536 words:
```c
static void telemTask(void*) {
  LogSample_t s;
  for (;;) {
    if (xStreamBufferReceive(logStream, &s, sizeof s, portMAX_DELAY) == sizeof s)
      emitCsvRow(&s);        /* owns Serial AND the HC-05 */
  }
}
```

5. Report `logDrops` in `G`.

**Verify.**
- CSV format byte-identical — your Python pipeline must not need changing
- `logDrops == 0` during a normal `T90`
- Control loop period unchanged during a full capture — compare `M` against
  Phase 2. **This is the payoff**: previously the dump stretched the loop.
- Tracer + `plot_trace.py`: you should now see the telemetry task's bars being
  chopped by the control task. Save this plot; it's the "before" for Phase 4.
- Golden dataset unchanged, including passive unwind

**Trap 1 — serial bandwidth is the real limit.** 28 bytes as CSV is ~70
characters. At 200 Hz that's 14 kB/s = ~140 kbaud, which **exceeds 115200**. You
must decimate in the control task exactly as `LOG_DECIM_B` does now. Don't
discover this as mysterious dropped samples — do the arithmetic first and set
the decimation deliberately.

**Trap 2 — two writers to `Serial`.** Telemetry now owns both output streams.
Your `Commander` instances still echo from the control task, which is a race.
Either route command responses through the stream buffer as a second record
type, or give commands a small dedicated queue to the telemetry task. Pick one
now; don't leave it.

**Trap 3 — telemetry stack.** `sprintf` with `%f` pulls in a large
float-formatting path. This is the task most likely to overflow. 1536 words, and
check the high-water mark.

**Trap 4 — buffer depth.** 96 samples at your decimated rate = how many ms of
slack? Compute it. If telemetry can't drain that fast on average, no buffer size
saves you — decimate harder.

---

## Result (2026-08-08) — PASSED, but only after shipping it wrong once

### What was built (Option C, Appendix B7)

`telemetry.h/.cpp` — the first real file-split. A priority-1 `telemTask` plus a 24-deep
queue of tagged records. **THE INVARIANT: after `telem_activate()`, `telemTask` is the only
task that writes the serial ports. No exceptions.** Two mechanisms cover every writer:

| mechanism | for | how |
|---|---|---|
| `telem_print(String)` | all text: `printBoth`, HOLD 10 Hz stream, `E`, `V` | copies into a queued `K_TEXT` record, **timeout 0** — the producer can never block |
| `telem_run(fn)` | bulk writers: **capture dump**, `M` timing stats | queues a `K_CALL` with a function pointer; telemTask *invokes* it. Since telemTask owns the ports, `fn` may use `Serial`/`hc05Serial` directly |

`telem_run` is the trick that made this cheap: the bulk writers did not have to be rewritten
to build Strings — the *same code* is simply executed on the *right thread*. `dumpCaptureTo(Serial)`
is correct on telemTask and a bug on the control task.

**Buffer-lifetime guard:** the dump READS `cap_*` from telemTask while the control task could
start a new capture and rewrite it. `T`/`O`/`C` therefore refuse with `busy: previous capture
still dumping` while `telem_busy()`. Refuse, don't corrupt.

**Boot path:** before `telem_activate()` (end of `hwSetup`) the control task never blocks, so
telemTask cannot drain; `telem_print`/`telem_run` fall back to direct/inline execution. Single-writer
still holds because telemTask isn't running yet. Activation happens once, immediately before
the control loop starts blocking on its notification.

### The failure, and why it is the most useful part of this phase

Step 3.1 was first shipped **half-applied**: text was routed through `telemTask` while the
capture dump, HOLD stream, `M`, `E` and `V` still wrote serial **directly from the control
task**. `HardwareSerial` is not reentrant. Two tasks inside the driver corrupted it, and the
symptom was nothing like "garbled text":

- board **froze** after a capture dump (unrecoverable, no further commands)
- the platform slewed the **wrong way** under huge torque
- `wp` froze at 11.78 rad/s (impossible — beyond the MPU's ±8.73 rad/s range), `ww` froze,
  and the control period stretched to **~303 ms**, so θ integrated a frozen garbage rate over
  a 60× oversized dt and the controller slammed 10 V

**A partially-applied single-writer design is worse than none, because it looks correct.**

The debugging that cracked it, worth repeating verbatim as method:
1. Reverted to the Phase-4 tag → clean. So it was the new code, not hardware.
2. **Bisect Test A:** re-added *only* the task creation — an idle prio-1 task blocking forever
   on an empty queue, 6 added lines, no routing, no guards. → **CLEAN.** This proved adding a
   low-priority task is safe, which also cleared Phases 5 and 6 (both add tasks) of suspicion.
3. That left the routing as the only candidate → complete the invariant rather than back away
   from it.

Two hypotheses were checked by inspection and **eliminated** before the bisect: the scheduler
tracer (bounds-masked with `& (TRACE_N-1)` and a no-op when off, so an unset TLS id on the new
task cannot corrupt) and the kernel config (prio 1 legal under `configMAX_PRIORITIES` 6, ample
heap headroom).

### Measured (hardware, 2026-08-08)

```
                      before (P4)        after (P3)
ctrl period MAX       11,500,760 us      5,006 us      <- the whole point
ctrl period mean      5,902 us           4,999 us      = 200.04 Hz
loopFOC : ctrl-law    23.9 : 1           20.35 : 1     = CTRL_DIVISOR exactly
FOC tick dt           242-258 us         239-260 us    unchanged
loopFOC               31/32/45 us        31/32/46 us   unchanged
telem drops           -                  0
telem stack free      -                  1479 / 1536 words
```

- **`ctrl period` MAX 11.5 s → 5.006 ms is the phase.** Worst case is now 6 µs *late*, i.e.
  the dump is completely off the control path. Verified with `M` taken right after a `T90`.
- The **20.35:1 ratio** is independent confirmation: control is now released on exactly every
  20th FOC tick with no coalescing, because nothing occupies the control task long enough to
  make it miss releases.
- **This retroactively satisfies Step 4.2's "release jitter max < 50 µs"**, which could not be
  met while telemetry sat on the control path. Phase 4's odd timing figures were this, not a
  Phase-4 defect.
- `telem stack free 1479/1536` — 1536 words is oversized; could drop to ~512 at Phase 7
  consolidation (Appendix B11).

Deliberately NOT done (per Appendix B14, verification relaxed to structure+safety): the
byte-identical-CSV and golden-dataset regression gates. Format is unchanged by construction
(the same `dumpCaptureTo` code runs, just on another thread), and the plant is being
re-identified in the combined retune anyway.

**Phase 3 exit:** `git tag rtos-p3-telemetry`

---

# Phase 4 — FOC split

**Goal:** commutation at kHz rates in its own high-priority task, preempting
blocking I2C. This is the phase with real engineering content, and the one that
can move your `ω_w` estimate — so it's also the riskiest for the compFrac margin.

## Step 4.1 — One timer, two rates

**Concept.** You need FOC at kHz and control at 200 Hz. The obvious approach is
two hardware timers. Better: one timer at the FOC rate whose ISR notifies the FOC
task every tick and the control task every Nth tick.

Why:
- Control is phase-locked to FOC — no drift between two oscillators, no beat
- Control always runs immediately after a fresh commutation update, so `ω_w` is
  as fresh as possible (matters, given your bandwidth ceiling)
- One less timer to collide with SimpleFOC
- Changing the control rate is one constant

**Task notifications** rather than semaphores: a notification is a 32-bit word
in the TCB, no separate object, ~45% faster than a binary semaphore. The
constraint is one notifier per task, which is exactly your case.

**Do.**
```c
extern "C" void FOC_TICK_IRQHandler(void) {
  FOC_TIM->SR = ~TIM_SR_UIF;
  static uint32_t n = 0;
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(hFoc, &woken);
  if (++n >= CTRL_DIVISOR) { n = 0; vTaskNotifyGiveFromISR(hControl, &woken); }
  portYIELD_FROM_ISR(woken);
}
```

With FOC at 4 kHz and control at 200 Hz, `CTRL_DIVISOR = 20`.

**Trap — `portYIELD_FROM_ISR` is mandatory.** Without it the notified task
becomes Ready but doesn't run until the next SysTick — up to 1 ms of latency,
*intermittently*, depending on where in the tick period the ISR landed. It
produces jitter that looks exactly like a hardware problem. `portYIELD_FROM_ISR`
sets the PendSV pending bit so the switch happens on exception return.

**Verify with the tracer.** Trace and check inter-arrival times of both tasks
directly from the CSV: FOC should be your target period, control 20× that. Plot
the histogram — you want a tight spike, not a bimodal distribution. Bimodal with
a second peak near 1 ms means you dropped the yield.

---

## Step 4.2 — Create the FOC task

**Do.**
```c
static void focTask(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    motor.loopFOC();
    motor.move();
  }
}
xTaskCreate(focTask, "foc", 768, NULL, 4, &hFoc);   /* priority 4, highest */
```

Simultaneously: remove `loopFOC()`/`move()` from the control task, drop the
control task to priority 3, and have it wait on its own notification.

**Shared state.** The control task writes `motor.target = u;` — a single aligned
32-bit float store, atomic on Cortex-M4. **No mutex.** Likewise the control task
reads `motor.shaft_velocity`, atomic in the other direction. Being able to
justify *not* synchronizing is as much a skill as knowing when to.

**Verify.**
- FOC rate and jitter from the tracer histogram
- Control rate exactly 200 Hz, release jitter max under 50 µs
- Motor spins smoothly at 4 V open loop, no audible change
- **`ω_w` calibration recheck**: 2 V open loop → ~17.0 rad/s per `K = 8.51`. If
  this has shifted, the compFrac margin has shifted with it.

**Result (2026-08-07) — Steps 4.1–4.2 DONE, split verified.** TIM9 ISR (`focTick_isr`
→ `focTickNotify`, NVIC prio 5) notifies `focTask` (prio 4) every tick and `controlTask`
(prio 3) every 20th; both wait on `ulTaskNotifyTake`. `loopFOC`/`move` removed from the
control path (incl. `stopMotor`/`measureGyroBias` — `focTask` commutates through them).
Measured (`M`, ~148 s run): **FOC tick dt 242–258 µs** (clean 4 kHz, ±8 µs, not bimodal);
**`loopFOC` n = 592,051 = 4000.2 Hz flat**. Preemption proven from the counts: cumulative
MPU-read time was ~62 s (24,811 reads × 2.51 ms); a starved `focTask` would have lost
~249k ticks (→ ~343k), but the full 592k confirms commutation ran *through* the blocking
I2C — the migration's whole point, measured without needing the Step 4.3 trace. `loopFOC`
cost unchanged (31/32/45 µs). Build RAM 54.5%.

Control-rate figures are dump/telem artifacts, NOT a regression: `ctrl period` mean 5902 µs
/ MAX 11.5 s and the 23.9:1 (not 20:1) ratio come from the capture dump + inline HOLD
telemetry still on the control path coalescing control notifications. **This is a 3↔4
interaction**: the guide's "release jitter max < 50 µs" (Step 4.2 Verify) can't be met until
Phase 3 moves telem+dump off the control path. Re-check clean release jitter at Phase 3 exit.

**Deferred to the combined hardware retune** (new STM32 swapped in + translation-motor mass
added, 2026-08-07): Step 4.4 (direction asymmetry) and Step 4.5 (compFrac re-verify). The
passive unwind is currently dead — wheel holds at ~17.7 rad/s in-deadzone instead of
decaying (effective K′≈9.57 vs identified 8.50, ~12%). **Confirmed NOT caused by the split:**
the OLD monolithic firmware on the SAME new board (run `150522`, T−90) also fails to unwind
and stall-retries. So it's a plant/tuning shift from the hardware change, folded into the
CONTROL_README deferred retune, not a Phase-4 defect. Trap 2 (4 kHz velocity quantization)
remains a candidate contributor to be separated during that retune via a clean O2 on the
split binary.

**Diagnosis note (2026-08-07) — spurious WHEEL_SAT on nudges, Phase 4 backed out.**
After the split, small hand-nudges randomly trip `WHEEL_SAT` with a nonsense ω_w (hundreds
of rad/s) while the wheel is barely moving; sometimes fine. Encoder ruled out (added `E`
cmd: raw single-turn angle reads smooth and monotonic by hand). **Leading hypothesis
DISPROVEN:** SimpleFOC computes wheel velocity from `micros()` (SysTick-based —
`_micros()` in `time_utils.cpp`), the clock Phase 0 flagged as unreliable under FreeRTOS,
and the 4 kHz rate would amplify a small-Δt glitch. BUT `Sensor::getVelocity()`
(`Sensor.cpp:30`) guards it: `Ts < min_elapsed_time → return old velocity`, capping any
Δt-glitch overestimate at ~2.5× — cannot manufacture a 200 rad/s reading. So a nonsense
spike must come from a large *Δangle* (angle/full-rotation glitch), which points back at the
encoder read — yet that's verified clean AND Phase 2 exercised SSI *more* often (~13 kHz)
without the symptom. **Unresolved.** Open threads for the redo: (1) instrument raw
`sensor.getVelocity()` + `Ts` + `full_rotations` at 4 kHz to catch the actual spike; (2)
check whether control-notify coalescing makes `dt` in `controlUpdate` occasionally huge →
big Δtheta → real α slam → *real* fast spin (a genuine over-react, not a measurement
glitch) — this fits "overreacting" and is my current best suspect; (3) consider driving
velocity/`dt` off TIM5 `us_now()` instead of SysTick `micros()`. Working tree reverted to
`rtos-p2-single-task` to test whether nudges still saturate on the known-good monolith
(saturates → hardware; clean → the split). Split preserved in `9d6830e`.

**RESOLVED (2026-08-08) — it was NOT the split.** The reverted Phase-2 monolith saturated
too, so the split was exonerated. Dump-on-abort caught the smoking gun: a T90 with normal
gains was converging cleanly (θ 68→79°, ω_w steady −23) then aborted at −76 on the very next
5 ms cycle — a −23→−76 jump = ~10,600 rad/s², physically impossible (wheel max ~455). So the
"saturation" is a **spurious velocity-estimate spike**, not real speed. Open-loop `V` drive
(V1–V4) was perfectly clean (K′≈8.6→9.4, smooth), which rules out the wheel/encoder/commutation
and ground-bounce/buck (all present in `V`); the discriminator is the **MPU I2C read**, which
only closed loop does — and it went marginal after the wire rework (2373→2510 µs, more spread),
coupling noise into the encoder SSI read that follows it. Fix: physical-plausibility reject on
ω_w (`WW_MAX_JUMP`) in the sense stage — a real wheel can't change speed that fast, so a
single-cycle jump is a measurement glitch; hold last-good, streak-capped resync, `wwRejects`
counted in `G`. This guard is architecture-independent and now lives in the Phase-4 code. Real
hardware follow-ups (separate I2C/SSI wiring, MT6701 supply decoupling, and driving SimpleFOC
velocity off TIM5 `us_now()` instead of SysTick `micros()`) are logged for later, not blocking.
Diagnostic commands `E` (encoder readout) and `V<volts>` (manual drive + unlimited stream) kept
in the build permanently — they earned their place.

**Trap 1 — don't split `loopFOC()` from `move()`.** SimpleFOC's internal state
isn't designed for concurrent access from two tasks. Both in the FOC task.

**Trap 2 — velocity quantization, and this one matters for you.** SimpleFOC
computes velocity from Δangle/Δt. At a fixed 4 kHz with a 14-bit encoder, one LSB
per sample is `2π/16384 × 4000 ≈ 1.53 rad/s` of quantization noise. Your
super-loop ran `loopFOC()` at ~27 kHz bare but with irregular gaps, so the
effective velocity baseline was different — probably longer on average.

**This is a real design tension**: commutation wants a high rate, velocity
estimation wants a long baseline. Symptoms of getting it wrong are noisy `u`,
audible buzz, and — most importantly for you — a shifted passive-unwind rate.

If `ω_w` looks noisy, options in order of preference:
1. Increase SimpleFOC's velocity low-pass time constant (`motor.LPF_velocity.Tf`)
2. Compute velocity on a decimated schedule (every Nth FOC iteration) while
   commutating every iteration
3. Lower the FOC rate toward the minimum from Step 0.5

Do **not** just retune `compFrac` to compensate without understanding which of
these is happening — that's papering over a measurement problem with a control
constant.

---

## Step 4.3 — Verify commutation survives the I2C read

**Concept.** This is the payoff for the whole migration. In the super-loop, the
MPU6050 read blocked `loopFOC()` for its full duration. Now the FOC task should
preempt straight through it.

**Do.** Add trace markers around the I2C read in the control task
(`TRC_CTRL_I2C_START` / `TRC_CTRL_I2C_END`). Capture during a `T90`, dump, plot.

**Verify.** FOC switch-in events appear *inside* the I2C interval, spaced at the
FOC period, with no gaps. Count them: with a 600 µs I2C read at 4 kHz you should
see 2 or 3.

Compare against a Phase 3 trace of the same operation, where the FOC work was
inline and there were zero.

**This pair of plots is your headline artifact.** Save both.

---

## Step 4.4 — The direction-asymmetry experiment

**Concept.** Your notes flag an unexplained finding: direction asymmetry real at
4 V (7.2%, 7σ), noise at ≤2.5 V. A speed-dependent asymmetry is what you'd
expect from commutation stalling during blocking I2C — the error scales with how
many electrical revolutions pass while the angle is stale. It's also what you'd
expect from magnetic asymmetry, encoder eccentricity, or alignment offset. Cheap
discriminating experiment.

**Do.** Re-run the run-2 step protocol — ±1.0, ±2.5, ±4.0 V, three repeats —
with Phase 4 firmware. Run `filter_calibration.py` and compare steady-state
velocity asymmetry against the archived numbers.

**Verify — record the outcome either way.**

| Outcome | Interpretation |
|---|---|
| Asymmetry drops substantially at 4 V | Commutation dropout was a contributor. Your run-2 identification data has a known systematic error at high voltage — worth re-checking `A_1` against the run-6 value. |
| Unchanged | Magnetic or mechanical. Rules out a class of explanation, which is still a result. |
| Something else | Write it down. |

**Trap.** Hold everything else fixed — same voltages, same repeats, similar
temperature, same starting orientation. Otherwise the comparison is worthless.

---

## Step 4.5 — Re-verify the compFrac margin, and retune if needed

**Concept.** This is your highest-risk item. `compFrac = 0.89` was measured with
the super-loop's `ω_w` estimate. Phase 4 changed how `ω_w` is computed. The
passive unwind depends on a 15% margin.

**Do.** Re-run the `C3` / `C-3` compensation tests. Sweep `K<val>` to find the
neutral point in each direction, exactly as run 7 did.

**Verify.**

| Result | Action |
|---|---|
| Neutral still ~0.972 (+) / ~0.892 (−) | Nothing changed. Good. |
| Neutral shifted < 5% | Set `compFrac` at or just below the lower one, as before. Note it. |
| Neutral shifted > 5% | Stop. Go back to Step 4.2 Trap 2 — your velocity estimate changed and you should fix the estimate rather than the constant. |
| Wheel now *grows* inside the deadzone | `compFrac` is above neutral. Reduce immediately; this walks toward the 45 rad/s abort. |

Then re-run the full golden dataset. If the step response has changed
materially, retune per your existing procedure, working **down** the gain table.
Archive as `baseline/golden-p4/` — keep both sets, the comparison is the
write-up.

**Phase 4 exit:** `git tag rtos-p4-foc-split`

---

# Phase 5 — Safety task and the I2C mutex

## Step 5.1 — The safety task

**Concept.** `xTaskDelayUntil` takes a pointer to the *previous wake time* and
advances it by the period, so execution time doesn't accumulate as drift.
`vTaskDelay` sleeps for N ticks *from now*, so the period becomes
`N + execution_time` and slowly slides. For anything periodic, use the former.

**Do.** Priority 2, period 50 ms.

| Check | Action |
|---|---|
| `\|ω_w\| > WHEEL_SAT_LIMIT` | safe stop |
| INA219 bus voltage below threshold | safe stop, latch, report |
| INA219 current above threshold | safe stop |
| Control task heartbeat counter not advancing | safe stop |
| `logDrops` climbing | report only |

```c
static void safetyTask(void*) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(50));
    /* checks */
  }
}
```

Implement one `safeStop(reason_t)` used by every path including the three
failure hooks from Step 1.2. It should disable the driver, zero the target,
latch a reason code, emit a `TRC_SAFE_STOP` marker, and require an explicit `R`.

**Verify.** Provoke each fault deliberately:
- drive the wheel past the limit → stop fires
- unplug the encoder mid-run → decide what *should* happen, then confirm it does
- halt the control task (comment out its notify) → heartbeat watchdog fires

**Trap — the saturation check moved from 200 Hz to 20 Hz.** That's up to 50 ms
of extra exposure. At your wheel accelerations that's meaningful. **Keep the
inline check in the control law as well.** Defence in depth: the inline check is
fast, the safety task catches the case where the control task itself has stopped
running. These are different failures.

---

### Result — Step 5.1 (2026-08-08): PASSED after one instructive false trip

Built `safety.h/.cpp` — `safetyTask`, prio 2, 50 ms via `vTaskDelayUntil`. Two checks:
wheel overspeed (reads `motor.shaft_velocity` **straight from the FOC layer**, not from a
value the control task publishes — a wedged control task must not be able to freeze the
watchdog's own input) → recoverable `stopMotor`; and a control-task heartbeat → terminal
`faults_safeStop(FAULT_HEARTBEAT)` (8 LED pulses). Terminal is the honest response: clearing
flags a dead task will never read recovers nothing. The inline 200 Hz `WHEEL_SAT` check
STAYS — defence in depth, per the Trap below. Arming is deferred to `safety_arm()` at the end
of `hwSetup`, because `initFOC` and the gyro-bias measurement legitimately block for seconds.

**THE BUG — a watchdog must measure TIME, not ITERATIONS.** First version tripped after `B`
and `E`, freezing the board (LD2 blinking 8). Chain:
1. Arduino `delay()` **busy-spins on `yield()`** (`wiring_time.c`), and `yield()` only yields
   to equal-or-higher priority. So during `B` (~1 s) and `E` (3 s) the control task spun at
   prio 3 and **starved safetyTask (2) and telemTask (1) completely**.
2. `safetyTask`'s `vTaskDelayUntil` deadline fell behind by 20–60 periods.
3. When control finally blocked, `vTaskDelayUntil` returned **immediately once per missed
   period**, so the 4 heartbeat checks ran back-to-back **in microseconds** instead of over
   200 ms.
4. The control task kicks at 200 Hz (every 5 ms), so within those microseconds the counter
   had not moved — 4 identical reads → false `FAULT_HEARTBEAT` on a perfectly healthy system.

Three fixes, all kept:
- **Heartbeat is wall-clock based** (300 ms since the counter last moved), not N consecutive
  iterations. An iteration count silently assumes the watchdog runs on schedule; time does not.
- **Resync on starvation** — if the deadline has slipped >2 periods, reset it rather than let
  `vTaskDelayUntil` burst through the backlog.
- **`delay()` → `vTaskDelay()`** in `measureGyroBias` and `E`. This is the root-cause fix and
  it also repaired a latent Phase-3 bug: `E`'s own output was being queued and dropped because
  telemTask could never drain while the control task busy-spun.

**`delay()` is now a landmine anywhere in the control task** — it starves every lower-priority
task silently. Audit for it in Phase 6. (The two remaining `while(1) delay(1000)` calls are
`hwSetup` init-failure dead-ends, before arming — harmless.)

Measured after the fix: `B`, `E`, `V2`, `G`, `T90` all clean; `safety checks` climbing ~20/s;
safety stack free 701/768 words; FOC tick 238–261 µs; `loopFOC` 31/32/45 µs unchanged.
**Clean-window timing (`M!` → `Z` → `T90` → `M`): `ctrl period` min 4994 / mean 5000 / MAX
5006 µs — 200.00 Hz at ±6 µs.** The safety task costs nothing measurable. Same run: T90
settled 90.83° vs 90.00° (0.83° error), α=0 in the deadzone, wheel bled −4.32 → −0.01 rad/s
with u → 0 (passive unwind behaving at low speed).

**Reading `ctrl period` after running diagnostics.** `B`/`E`/`V` execute *inside* the control
task, so `controlStep()` does not return for their duration and the period stat logs one
multi-second gap (observed MAX 4.83 s after a `V2` run). That is NOT a Phase-3 regression —
all three disable the controller first, so the control law is not meant to be running. Take
timing on a clean window: `M!` → `Z` → `T90` → `M`. The structural wart (blocking commands
hijack the control task) is what **Phase 6** fixes by moving command handling to its own task.

---

## Step 5.2 — The I2C mutex

**Concept.** Two tasks now touch `Wire`: control (MPU6050, 200 Hz) and safety
(INA219, 20 Hz). Arduino's `Wire` is not reentrant — concurrent access gives
corrupted transactions and sometimes a hung bus with SDA held low.

Use a **mutex**, not a binary semaphore. The difference is ownership and
**priority inheritance**: when a high-priority task blocks on a mutex, the kernel
temporarily raises the holder's priority so it finishes and releases quickly.

The failure it prevents, in your system: telemetry (prio 1) holds the mutex,
control (prio 3) blocks on it, comms (prio 2) wakes and preempts telemetry.
Control is now waiting on a priority-2 task that has nothing to do with the
mutex. That's **priority inversion** — the bug that caused the Mars Pathfinder
resets in 1997. A binary semaphore has no owner, so it cannot fix this.

**Do.**
```c
xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(2));
/* transaction only */
xSemaphoreGive(i2cMutex);
```

Use a **timeout, never `portMAX_DELAY`**, in the control task. If the bus hangs,
the control task must degrade — reuse the previous gyro sample, increment a fault
counter, and let the safety task decide. A control loop that blocks forever on a
peripheral is how you get a runaway.

**Verify.**
- Trace with I2C markers on both tasks: intervals never overlap
- Control loop period max unchanged from Phase 4 (check `M`)
- Force contention: temporarily run the safety task at 200 Hz, confirm stability
  and that the control task's timeout path actually gets exercised
- Golden dataset, including passive unwind

**Trap 1.** Never take a mutex from an ISR. There is no `FromISR` variant, by
design — priority inheritance is meaningless when there's no task to boost.

**Trap 2.** Take the mutex immediately before the transaction, release
immediately after. Never hold it across a computation. Long hold times mean long
inversion windows.

### Result — Step 5.2 (2026-08-08): PASSED, mutex costs nothing

Built `i2c_bus.h/.cpp` — `xSemaphoreCreateMutex()` (NOT a binary semaphore; ownership is
what gives priority inheritance). Two users: `controlTask` MPU6050 @ 200 Hz, `safetyTask`
INA219 @ 20 Hz. INA219 auto-detected at boot with the `setCalibration_32V_2A()` gotcha
handled (CONTROL_README §16 — `begin()` only confirms an I2C ack); **absence is non-fatal**,
monitoring just disables, so a missing sensor can never cause a spurious safe-stop.

**Timeouts are asymmetric on purpose.** Control uses `i2c_lock(2)` — 2 ms of a 5 ms budget is
the most the loop can afford — and on failure **degrades** (reuses the previous gyro sample,
counts it) rather than blocking. Safety uses `i2c_lock(5)`: not deadline-critical, and it
would rather wait behind a 2.5 ms gyro read than miss its sample. `B` (`measureGyroBias`)
uses `i2c_lock(20)` — caught during an audit, it re-runs *after* the watchdog is armed so it
genuinely races safetyTask; a skipped sample there would bias the bias measurement.

**Measured (hardware):**
```
ctrl period      min 4994 / mean 5000 / MAX 5006 us   <- IDENTICAL to pre-mutex
INA219 read      n=529   1372 / 1372 / 1377 us        (safety holds the bus 1.37 ms @ 20 Hz)
i2c timeouts     0                                     (control never lost the bus)
control law MAX  2530 -> 2771 us                       (+240 us = occasional mutex wait)
MPU6050 read     2392 / 2500 / 2591 us
power (idle)     busV 12.07-12.08, |I| 8 mA
power (T90)      busV min 11.65, |I| max 813 mA
```
The headline: safetyTask holds the bus for 1.37 ms every 50 ms and the 200 Hz control loop
still never misses a deadline by more than 6 µs. Priority inheritance + short holds work.

**Trip thresholds set from MEASURED data, not guessed** (deliberately a two-flash sequence:
monitor first, then arm — guessing a threshold on a spinning flywheel invites nuisance
safe-stops). `minBusV = 10.0 V` (~14% below nominal, well under the 11.65 V working sag, so
it catches a collapsing battery not normal load); `maxCurrent = 2500 mA` (~3× the observed
813 mA peak). **Both debounced over 2 consecutive checks (100 ms)** so one noisy INA219 sample
cannot kill a run. Trips use the *recoverable* stop, not `faults_safeStop` — a sagging battery
should let the operator swap it and send `R`, not force a reset. Tighten once T180/stall data
exists.

**Fixed alongside: `telem drops: 97`.** Not random — during the ~11 s dump, telemTask is busy
writing 1201 rows while the control task keeps queueing 10 Hz HOLD rows; ~110 arrive against a
24-deep queue. Dropping is *by design* (never delay control for telemetry) and the dump itself
was complete, but the noise is avoidable: HOLD rows are now suppressed while `telem_busy()`.

**Phase 5 exit:** `git tag rtos-p5-safety`

---

# Phase 6 — Comms task

**Goal:** command handling off the control path, using the pattern the Pi link
will need.

## Step 6.1 — Deferred interrupt processing

**Concept.** ISRs run with interrupts masked and delay everything, so they should
be minimal. The pattern: ISR grabs the byte, pushes to a queue, returns. A task
does the parsing. This is *deferred interrupt processing*, and it's the structure
the framed Pi↔STM32 protocol drops straight into.

Queues copy **by value**, so the producer's local can go out of scope safely.

**Do.**
1. UART RX ISR at priority 6 → `xQueueSendFromISR(rxQ, &byte, &woken)` →
   `portYIELD_FROM_ISR(woken)`.
2. Comms task, priority 2, blocked on `xQueueReceive(rxQ, ..., portMAX_DELAY)`.
   Accumulate, parse on newline, dispatch. Both channels: either two tasks or one
   task selecting on two queues.
3. Parsed commands that change controller state (`P D F W N Z T H C K O X R B G`)
   go into a **second queue** read by the control task at the top of its cycle.
   The comms task must not write controller state directly — that races with the
   control task mid-computation.
4. Command responses go to the telemetry task's output path (Phase 3 Trap 2).

**Verify.**
- Spam commands over both USB and HC-05 during an active `T90`: control period
  unaffected, capture uncorrupted
- Both `Commander` instances still reply on their own stream (the bug you already
  fixed — make sure the refactor doesn't reintroduce it)
- Measure `X` latency: from byte received to `motor.target = 0`. Should be one
  control period plus queue latency, well under 10 ms.

**Trap 1 — `X` must be fast and must not queue behind anything.** Give it a fast
path: the comms task sets `volatile sig_atomic_t emergencyStop` directly, which
the control task checks first thing every cycle. Safety overrides layering.

**Trap 2 — abort-on-any-byte.** Your `waitForKeypress()` fix assumed a
synchronous drain with a 400 ms quiet window. Re-derive it for the queue model:
the queue already buffers, so "quiet" means `xQueueReceive` returning `pdFALSE`
with a 400 ms timeout. Don't port the polling logic verbatim.

**Trap 3 — queue depth.** 64 bytes at 115200 baud is ~5.5 ms of slack. The comms
task at priority 2 drains far faster. Add an overflow counter anyway; silent loss
is worse than a reported one.

### Result — Phase 6 (2026-08-08): PASSED

Built `commands.h/.cpp` — `commsTask` prio 2, polling both channels every 2 ms
(500 Hz; ~23 bytes/poll at 115200, comfortably inside the core's 64-byte RX buffer).

**What moved: RX and line assembly only. Command EXECUTION deliberately did not.**
`handleCommand` writes controller state (target, ctrlMode, gains, capture flags); running
it on commsTask would drop those writes into the middle of `controlUpdate()` — a torn read
of the very state the control law is using. So complete lines are queued and the control
task drains them at the top of its cycle, calling `handleLine()` **verbatim, unchanged**.

**THE SINGLE-READER RULE — the mirror of telemetry's single-writer rule (B15).** Only
commsTask reads the ports. Two readers race the RX ring buffer's tail and one silently eats
the other's bytes. This bit immediately: `V` polled `Serial.available()` itself, so commsTask
would have swallowed its stop key and `V` would never have exited. Fixed by watching
`commands_rxBytes()` instead of reading the port. Audited: zero direct reads remain in
`rtos_main.cpp`. (RX/TX across tasks stays safe — HardwareSerial keeps separate ring buffers
and head/tail per direction, so commsTask reading while telemTask writes touches disjoint
state.)

**`X` fast path:** commsTask zeroes `motor.target` the instant it sees an X, *before* queueing
— an aligned 32-bit float store is atomic on Cortex-M4, so focTask applies it within one tick
(≤250 µs) instead of waiting up to a full 5 ms control period. The line is still queued so
`handleLine` runs the full stop (flags + operator message) normally.

**Measured (hardware):** `V2` streams and stops cleanly on a keypress (reader race avoided);
`X` immediate; **`comms drops: 0`** throughout, including ~12 `G` commands spammed during an
active capture — the capture completed intact (1201 rows) and the heading settled 91.5° vs
90°. **GATE: `ctrl period` min 4416 / mean 4999 / MAX 5005 µs measured DURING a T90 capture with
~10 `G` commands spammed at it** — command traffic no longer perturbs the control loop at
all; capture intact (1201 rows), settled 88.95° vs 90.00°, `comms drops` 0, `telem drops` 0.
comms stack free 714/768.

**Latent interaction, logged not fixed:** `INA219 read` MAX hit **3042 µs** on one sample
(typ. 1370). That exceeds the control task's 2 ms `i2c_lock` timeout, so control *can*
legitimately fail to get the bus and degrade to the previous gyro sample. It did not happen
here (`i2c timeouts: 0`) and degrading is the designed-safe response, but if timeouts ever
start appearing, this is why — the fix would be splitting the INA219 read into two shorter
transactions or lengthening the control timeout, NOT removing the degrade path. Also observed: `wheel-velocity glitches rejected: 3` during a T90 — the Phase-4
plausibility guard suppressing what would previously have been false saturation aborts.

**`telem drops: 22` under command spam is a BANDWIDTH limit, not a bug (Trap 18 observed).**
Each `G` is ~8 lines; a ~60-char line to *two* ports at 115200 costs ~10.4 ms, so sustained
output caps near ~96 lines/s. Spamming `G` faster than ~12/s asks for more than the wire can
carry and the queue correctly sheds load rather than delaying control. **Enlarging the queue
would not help** — it only moves the same ceiling. Left as-is deliberately.

**Known, deliberately not fixed:** `B`/`E`/`V` still execute on the control task, so they
occupy it for their duration (they disable the controller first, so it is safe — but it is
why `ctrl period` MAX reads in seconds after running them; take timing on a clean window).
Moving them to commsTask means they would touch motor state from another task, which is
exactly what the queue design avoids — a Phase 7 decision, not a rush.

**Phase 6 exit:** `git tag rtos-p6-comms`

---

# Phase 7 — Consolidation and evidence

## Step 7.1 — Right-size stacks

**Concept.** `uxTaskGetStackHighWaterMark` returns the minimum free space (in
words) ever seen on that stack. FreeRTOS fills new stacks with `0xA5` and counts
the surviving pattern.

**Do.** Add all five to `G`. Run a punishing session: `T180`, `T-180`, heavy
telemetry, command spam, a deliberate fault trip and recovery, a stall-recovery
event.

**Verify.** Set each stack to `(allocated − highwater) × 1.5`. Re-run and confirm
≥30% margin remains.

**Trap.** High-water is the minimum *ever seen*. An unexercised path can still
blow the stack later. Exercise the error handlers, not just the happy path.

## Step 7.2 — CPU utilization

**Do.** `vTaskGetRunTimeStats()` into a command, using the TIM5 timebase you
configured in Step 1.2.

```
Task      Abs time     %
foc      _________   ___%
ctrl     _________   ___%
safety   _________   ___%
comms    _________   ___%
telem    _________   ___%
IDLE     _________   ___%
```

## Step 7.3 — Schedulability analysis

**Concept.** With rate-monotonic priorities (shorter period → higher priority),
a task set is *guaranteed* schedulable if total utilization stays under
`n(2^(1/n) − 1)`. For n = 5 that's **0.743**. Above the bound it may still be
schedulable, but you'd need response-time analysis to prove it. This is the Liu
& Layland result from 1973 and it's the difference between "it seems to work" and
"here is why it meets deadlines."

**Do.** Using measured WCETs (re-measured under the new structure, not the
Phase 0 numbers):

```
U = Σ (WCET_i / T_i) = _______   vs bound 0.743
```

Write it up. One page. Include the priority assignment and why FOC — the
"less important" task — is highest.

### Result — Steps 7.1–7.3 (2026-08-08)

New `U` command (runs on telemTask, sole-writer rule): `vTaskGetRunTimeStats` + all five
stack high-water marks. The run-time clock is `rtRunTimeCounter()` = `TIM5->CNT` at 1 MHz,
wired back in Step 1.2 and unused until now — **32 bits at 1 MHz wraps every ~71.6 min, so
read `U` well within an hour of boot.**

**7.2 — CPU utilisation** (measured after a punishing session: T180, T−180, H0+nudges, B, E,
V3, M, command spam):

| task | prio | % | note |
|---|---|---|---|
| ctrl | 3 | **41%** | dominated by the blocking 2.4 ms MPU read |
| foc | 4 | **15%** | 4 kHz × ~35 µs mean — matches the Step 0.5 prediction of ~16% |
| telem | 1 | **10%** | the T180 dumps; pushing bytes out two UARTs is real work |
| safety | 2 | **2%** | |
| comms | 2 | **<1%** | |
| **IDLE** | 0 | **30%** | headroom |

**7.1 — Stack high-water and resize.** Used (words): foc 65, ctrl 259, safety 143, comms 54,
telem 150. Resized at **≥3× margin** — more generous than the guide's 1.5×, because the fault
handlers and the stall-recovery path were NOT exercised in that session and high-water only
bounds the paths actually taken:

| task | was | now | used | margin |
|---|---|---|---|---|
| foc | 768 | **256** | 65 | 3.9× |
| ctrl | 1536 | **768** | 259 | 3.0× |
| safety | 768 | **384** | 143 | 2.7× |
| comms | 768 | **256** | 54 | 4.7× |
| telem | 1536 | **512** | 150 | 3.4× |

5376 → 2176 words = **12.8 KB of RAM reclaimed**. `configCHECK_FOR_STACK_OVERFLOW = 2` is the
safety net if any estimate is wrong (pattern check at every switch → `faults_safeStop`).

**7.3 — Schedulability.** Using measured **WCETs**, not means:

```
task    C (WCET)   T        U = C/T
foc       62 us     250 us   0.248     (loopFOC 46 MAX + move 16 MAX)
ctrl    2669 us    5000 us   0.534     (control law MAX, incl. the MPU read)
safety  3042 us   50000 us   0.061     (INA219 read MAX outlier)
comms     ~50 us   2000 us   0.025
                             -------
                        U =  0.868   vs Liu & Layland bound 4(2^(1/4)-1) = 0.757
```

**U exceeds the RM bound — which does NOT mean deadlines are missed.** The bound is
*sufficient, not necessary*; above it you owe a response-time analysis
`R = C + Σ⌈R/T_j⌉·C_j` iterated to a fixed point:

- **ctrl**: 2669 → 3351 → 3537 → 3599 → converges **R = 3599 µs < 5000 µs deadline ✓**
  (72% of its window; measured `ctrl period` MAX 5006 vs 5000 nominal agrees)
- **safety**: converges **R = 14,707 µs < 50,000 µs ✓**
- **foc** (highest prio): **R = 62 µs < 250 µs ✓**

**All deadlines provably met.** This is the difference between "it seems to work" and "here is
why it meets deadlines."

**Deliberate deviation from strict rate-monotonic:** `commsTask` polls every 2 ms — a *shorter*
period than control's 5 ms — so pure RM would rank it ABOVE control. It is deliberately below.
RM assumes deadline = period; comms' real deadline is human-scale (~100 ms), and a command
serviced 5 ms late is meaningless while control missing 5 ms is not. That is deadline-monotonic
reasoning, and it is the correct assignment. Same logic answers "why is FOC — the least
'important' task — highest priority": commutation has the shortest period and the hardest
deadline; a late commutation corrupts torque, and everything else depends on the motor working.

**The single biggest remaining lever:** ctrl's 2669 µs is ~2.5 ms of blocking `mpu.getEvent()`,
which reads accel+gyro+temp when the law uses only `g.gyro.z`. The Step 0.4 deferred
optimisation (targeted 2-byte read of `GYRO_ZOUT_H/L`, 0x47/0x48) would take U from 0.868 to
roughly 0.47 — comfortably under the bound. Still deferred under the verbatim rule: fix it in
`heading_control.cpp` first, re-verify, then carry across.

## Step 7.4 — Jitter report

**Do.** From the tracer CSV: per-task inter-arrival min/max/mean/σ over a
10-minute run under load, plus execution-time distributions. Your `plot_trace.py`
histogram panel does this.

```
FOC   period ____ µs   jitter max ____ µs   WCET ____ µs
CTRL  period 5000 µs   jitter max ____ µs   WCET ____ µs
```

## Step 7.5 — Artifact checklist

- [ ] Tracer Gantt: Phase 3 (telemetry preempted by control)
- [ ] Tracer Gantt: Phase 4 (FOC preempting through the I2C read)
- [ ] The Phase 3 vs Phase 4 pair showing zero → N commutation updates during I2C
- [ ] Golden dataset comparison: Phase 0 vs Phase 7 step responses
- [ ] compFrac neutral point before vs after
- [ ] Direction-asymmetry experiment result (Step 4.4), whichever way it went
- [ ] CPU utilization table
- [ ] Schedulability analysis
- [ ] Stack high-water table
- [ ] Jitter report with histograms
- [ ] `CONTROL_README.md` updated: firmware structure, timer allocation, task
      table, tracer documentation, resolved/unresolved items

### Result — Step 7.1 resize VERIFIED, 7.5 checklist status (2026-08-08)

Re-measured after resizing, same punishing session. Every task retains far more than the
required 30% margin, and the used-words figures reproduced almost exactly:

| task | free / allocated | used | headroom |
|---|---|---|---|
| foc | 191 / 256 | 65 | 75% |
| ctrl | 481 / 768 | 287 | 63% |
| safety | 235 / 384 | 149 | 61% |
| comms | 202 / 256 | 54 | 79% |
| telem | 362 / 512 | 150 | 71% |

CPU after resize: ctrl 42% / foc 15% / telem 11% / safety 2% / comms <1% / **IDLE 27%**.

**7.5 artifact checklist — honest status.** Several items depend on plant measurements that
were deliberately deferred (B14 relaxed verification to structure+safety; B9/B10 folded into
the combined retune after the STM32 swap + added mass). They are marked deferred WITH REASONS
rather than fabricated — a write-up that says "not measured, here is why" is more credible
than one with invented numbers.

- [x] CPU utilisation table — §7.2 above
- [x] Schedulability analysis — §7.3 above, incl. response-time proof and the RM deviation
- [x] Stack high-water table — above, before and after resize
- [x] `CONTROL_README.md` updated — firmware architecture, task table, the two invariants,
      timer allocation resolved, new commands (`M`/`U`/`E`/`V`), RTOS open-item closed
- [x] Phase 3 vs Phase 4 evidence that commutation survives the I2C read — obtained from
      **counts** rather than a Gantt: `loopFOC` held 592,051 samples = 4000.2 Hz flat across
      62 s of cumulative blocking MPU reads, where a starved focTask would have lost ~249k
      ticks. Step 4.3's trace plot would be prettier; the count is the same proof.
- [ ] Tracer Gantt plots (Step 7.4 / 4.3) — NOT done. The tracer (Step 1.4) and
      `plot_trace.py` exist and work; nobody re-enabled them after Phase 2. Optional: the
      timing evidence above already establishes the result.
- [ ] Golden dataset Phase 0 vs Phase 7 — **deferred, B14.** The plant moved mid-migration
      (hardware rework), so a before/after comparison would measure the rework, not the
      migration.
- [ ] compFrac neutral before/after — **deferred, B9.** Passive unwind is currently dead;
      confirmed pre-existing (fails on the reverted monolith too), folded into the retune.
- [ ] Direction-asymmetry experiment — **deferred, B10.** Only meaningful against a
      re-identified plant.

**Phase 7 exit:** `git tag rtos-p7-complete`

---

# Appendix A — Trap quick reference

| # | Trap | Symptom |
|---|---|---|
| 1 | Stack sizes are **words**, not bytes | Random corruption |
| 2 | Wrong port (CM3 not CM4F) | Silently wrong float math, no fault |
| 3 | ISR at priority 0 calling a `FromISR` API | Kernel corruption |
| 4 | Missing `portYIELD_FROM_ISR` | Up to 1 ms intermittent latency |
| 5 | Priority grouping with subpriority bits | `BASEPRI` masking wrong |
| 6 | A task that never blocks | Lower-priority tasks never run |
| 7 | `vTaskDelay` instead of `vTaskDelayUntil` | Period drifts by execution time |
| 8 | Binary semaphore where a mutex belongs | Priority inversion, no inheritance |
| 9 | `portMAX_DELAY` on a mutex in the control path | Control loop hangs on a stuck bus |
| 10 | Mutex from an ISR | No such API; redesign |
| 11 | Holding a mutex across computation | Long inversion windows |
| 12 | TIM2/TIM3 collision with SimpleFOC PWM | PWM period silently rewritten |
| 13 | `HardwareTimer` reconfiguring a SimpleFOC timer | Same, but harder to spot |
| 13b | `TIM9_IRQHandler` doesn't exist on F4 | Compiles, links, never fires. TIM9 shares TIM1's break vector: use **`TIM1_BRK_TIM9_IRQHandler`** and **`TIM1_BRK_TIM9_IRQn`**. (TIM10 → `TIM1_UP_TIM10_*`, TIM11 → `TIM1_TRG_COM_TIM11_*`.) |
| 14 | SysTick handoff breaking SimpleFOC `_micros()` | `ω_w` wrong → linearization wrong → compFrac margin wrong |
| 15 | FOC-rate velocity quantization | Noisy `u`, shifted unwind rate |
| 16 | Telemetry stack too small (`sprintf %f`) | Overflow in the least-tested task |
| 17 | Two tasks writing `Serial` | **NOT merely "interleaved garbage" — that undersells it and cost us a day (2026-08-08).** `HardwareSerial` is not reentrant: concurrent writers corrupt the driver. Observed: board **froze** after a capture dump, platform slewed the **wrong way at full torque**, `wp` froze at an impossible 11.78 rad/s, control period stretched to **303 ms**. Reads like a hardware fault. Cause was a **half-applied** single-writer design (text queued to telemTask; dump/`M`/`E`/`V` still direct from the control task) — worse than none, because it looks correct. Fix = the absolute invariant B15 + `telem_run(fn)` for bulk writers. |
| 18 | Serial bandwidth < telemetry rate | Silent drops; decimate |
| 19 | **Arduino `delay()` in a task** | Busy-spins on `yield()`, which only yields to equal-or-higher priority → **silently starves every lower-priority task** for the whole delay. Use `vTaskDelay()`. Cost us a false watchdog trip + swallowed telemetry (Step 5.1, 2026-08-08). |
| 21 | **Two tasks READING one serial port** | Mirror of Trap 17. They race the RX ring buffer's tail and one silently eats the other's bytes — `V`'s stop key vanished into commsTask, so it could never exit. One reader only; anything else that needs "did the operator press something" watches a byte counter (Phase 6). |
| 20 | **Watchdog counting iterations instead of time** | If the watchdog is ever starved, `vTaskDelayUntil` returns immediately once per missed period, so N "consecutive" checks span microseconds, not N periods — and a healthy task reads as dead. Always compare **wall-clock elapsed** since the monitored counter last moved (Step 5.1). |
| 19 | Reading the trace buffer while tracing | Torn data |
| 20 | High-water from an unexercised path | Overflow appears weeks later |

---

# Appendix B — Decision log

| # | Decision | Step | Value |
|---|---|---|---|
| B1 | Pole-pair count | 0.5 | `PP = 11` — from `BLDCMotor(POLE_PAIRS)`, `#define POLE_PAIRS 11`. Confirmed correct 2026-08-03 (`initFOC` aligns against it, so a wrong value would break commutation). |
| B2 | Timers free after SimpleFOC init | 0.3 | **TIM2, TIM3 taken** (3-phase PWM, PSC=0, ARR=1799, CR1=0x61 → center-aligned mode 3 → **25 kHz**). TIM5 = timebase. **Free: TIM1, TIM4, TIM8, TIM9, TIM10, TIM11, TIM12.** TIM4 confirmed free — PB6/D10 is a plain GPIO driver-enable, not a timer output. |
| B3 | FOC tick timer | 0.3 | **TIM9.** APB2 @ 180 MHz. Chosen over TIM4 deliberately: TIM4_CH1 is PB6, physically wired to the DRV8313 enable, so a stray channel-enable there would toggle the driver at the tick rate. TIM9's channels are PA2/PA3 (ST-LINK VCP) — no such hazard. TIM4 kept as spare. |
| B4 | FOC rate + justification | 0.5 | **4 kHz.** `f_elec = PP·ω_max/(2π) = 11·45/6.283 = 78.8 Hz`; comfortable rate `40·f_elec = 3.15 kHz`. 4 kHz clears that with margin. Affordable: `4000 × 40 µs (loopFOC WCET) = 16%` CPU, 19% including `move()`. Not lowered — `loopFOC` at 40 µs is already cheap (SPI fast, no reason to drop the rate). Not raised — no commutation benefit above what the electrical rate needs, and higher eats margin. Canonical pairing with the tuned 200 Hz control loop. |
| B5 | `CTRL_DIVISOR` | 4.1 | **20** — 4 kHz FOC / 200 Hz control. Control fires every 20th FOC tick, phase-locked. (Set for real in Step 4.1; fixed here by the B4 rate choice.) |
| B6 | Measured WCETs (super-loop) | 0.4 | loopFOC **40 µs** / move **8 µs** / MPU read **2374 µs** / control-law compute **~14 µs** (`st_law − st_mpu`) / telem row **237 µs** / superloop MAX **2666 µs** = FOC starvation during the blocking gyro read. No INA219 in this sketch. Full table in Step 0.4. |
| B7 | Telemetry architecture + decimation | 3.1 | **DONE 2026-08-08 — Option C shipped and verified** (`ctrl period` MAX 11.5 s → 5.006 ms; drops 0). Implemented exactly as decided below, plus the sole-writer invariant B15 that the first attempt violated. **Option C — hybrid (2026-08-06).** Capture dump (`T`/`O`/`C`) keeps the full-rate 200 Hz RAM buffer (`cap_*`) and hands the *frozen* buffer to `telemTask` for the slow serial dump (control task no longer blocks → `loopFOC`/`WHEEL_SAT` stay live during output, the real Phase-3 goal). Live `H` stream moves onto a per-tick `LogSample_t` **stream buffer** drained by `telemTask` (builds the SPSC primitive; removes the 2nd Serial writer from the control task, Trap 2). **Decimation factor: N/A** — C never streams at 200 Hz over serial, so the Trap-1 bandwidth ceiling (≈140 kbaud > 115200) never binds; capture stays byte-identical at 200 Hz and live `H` stays 10 Hz. Chose C over the guide's literal Step 3.1 (Option A, per-tick 200 Hz streaming) because A forces ≤100 Hz decimation, which halves golden-dataset capture density — A's *Do* section contradicts its own *Verify* ("CSV byte-identical, golden dataset unchanged"); C resolves the tension toward the Verify, which is the criterion that actually protects the controller. Rejected Option B (dump-handoff only, no stream buffer) because it defers the SPSC primitive and leaves Trap 2 open. |
| B8 | `ω_w` calibration check @ 2 V | 2.1, 4.2 | **Phase 2 (2026-08-06): 17.2 rad/s** at 2.00 V → K=8.59 vs identified 8.51 (<1%). **Phase 4 (2026-08-07): shifted** — in-deadzone hold implies effective K′≈9.57 (~12% high) at uniform 4 kHz. NOT the split's fault: OLD firmware on the same new board (run 150522) also fails to unwind. Confounded by the hardware swap (new STM32 + added mass); clean O2 on the split binary still owed to separate Trap-2 quantization from a genuine plant shift. Folded into the deferred retune. |
| B9 | compFrac neutral, before / after | 4.5 | **Deferred to hardware retune (2026-08-07).** compFrac=0.89 no longer holds the −0.84 unwind pole after the STM32 swap + added mass; passive desat is dead (wheel holds ~17.7 rad/s in-deadzone). Re-run the C-sweep during the combined retune, not now (user decision — hardware changed, retuning anyway). |
| B10 | Direction-asymmetry outcome | 4.4 | **Deferred to hardware retune (2026-08-07).** The run-2 re-run comparison is only meaningful against a re-identified plant; folded into the combined retune campaign. |
| B11 | Final stack sizes | 7.1 | |
| B12 | Final `U` vs bound | 7.3 | |
| B15 | **Sole-writer serial invariant** | 3.1 | **Decided 2026-08-08, after shipping the violation.** After `telem_activate()`, `telemTask` is the ONLY task that writes `Serial`/`hc05Serial`. Enforced by two mechanisms: `telem_print()` (queue text) and `telem_run(fn)` (execute a bulk writer ON telemTask, so it may use the ports directly). Chose `telem_run(fn)` over rewriting the bulk writers into String builders because it moves the *same* code to the *right thread* — a small diff and no risk of altering the CSV format. Rationale for the invariant being absolute: `HardwareSerial` is not reentrant, and the half-applied version (text queued, dump/`M`/`E`/`V` still direct) did not degrade gracefully — it corrupted the driver, froze the board, and produced frozen sensor reads + a 303 ms control period that looked exactly like a hardware fault. Any future writer (Phase 6 command echo) MUST use one of the two mechanisms. |
| B16 | **telemTask priority + queue depth** | 3.1 | **Prio 1, depth 24.** Prio 1 is below control (3) and foc (4) so telemetry can never delay commutation or the control law — it runs only in the CPU the control task leaves idle while blocked on its 200 Hz notification. Deliberately NOT higher: a ~11 s dump at a priority above control would starve the loop, which is the very failure being fixed. Depth 24 × ~104 B ≈ 2.5 KB; measured `telem drops: 0` in normal use because text is low-rate and the one bulk item (the dump) is a single `K_CALL` record, not 1500 rows. Producers always send with **timeout 0**, so a full queue drops-and-counts rather than blocking the control task — dropping telemetry is always preferable to delaying control. |
| B14 | **Verification policy relaxed to structure+safety** | 4/all | **2026-08-07 (user decision).** Hardware changed mid-migration (STM32 swapped, translation-motor mass added), and the plant will be fully re-identified in the CONTROL_README combined retune after translation regardless. So the migration's original regression-test discipline — golden dataset must match, compFrac margin preserved phase-to-phase — is **relaxed**. Goal is now: prove the RTOS *structure* is correct, not that behavior is byte-identical to the bare-metal baseline. **Consistency gates become informational; SAFETY gates stay blocking** (wheel-can't-reach-45-abort is a safety property, not a consistency one). Practical effect on remaining phases: skip golden-dataset matching and margin-preservation as pass/fail; keep the WHEEL_SAT/X/fault-path checks. Telemetry "CSV byte-identical" (Phase 3) stays a goal only because the Python pipeline is convenient, not as a regression gate. |
| B13 | **Phase 3 ↔ 4 reorder** | 3/4 | **Swapped: Phase 4 (FOC split) before Phase 3 (telemetry), 2026-08-06.** Found while scoping Phase 3: the monolithic control task never blocks (inline `loopFOC` busy-spin, [rtos_main.cpp:803]), so it holds the CPU continuously and any lower-priority task is starved (the "never-block → starves-below" rule, biting in practice). A telemetry task MUST be lower prio than control — else its ~7 s dump would starve `loopFOC` while the wheel spins. So preemptive telemetry needs the control task to yield, which needs FOC off the busy-loop and into a timer ISR (a ≥1 ms tick-granular `vTaskDelay` can't pace uniform kHz commutation — it makes FOC bursty with hundreds-of-µs stale-angle gaps, risking the compFrac margin). That restructure is exactly Phase 4. Conclusion: the guide's 3-before-4 order had the dependency backwards; Phase 4 is the real unlock for telem/safety/comms alike. Rejected the two stopgaps (cooperative in-loop dump; pull only Step 4.1 forward) in favor of doing Phase 4 properly first — same FOC-timing/ω_w risk either way, so do it cleanly with full phase isolation. Cost: lose the Phase-3 tracer plot as Phase-4's "before" (mitigated: Phase-2 tracer baseline exists). Telemetry Option-C (B7) unchanged, just resequenced. |

---

# Appendix C — Target architecture

```
TIM5  ── free-running 32-bit @ 1 MHz ──> us_now(), tracer, run-time stats
TIM2  ── SimpleFOC PWM (PA5)              [do not touch]
TIM3  ── SimpleFOC PWM (PA6/PA7)          [do not touch]
TIMx  ── FOC tick @ N kHz, IRQ prio 5     [B3]
             ├── notify every tick ──────> focTask      prio 4
             └── notify every CTRL_DIV ──> controlTask  prio 3

focTask      4   loopFOC() + move()                    SPI (exclusive)
controlTask  3   sense → estimate → LQR → FF →         I2C (mutex, 2 ms timeout)
                 linearize → motor.target
safetyTask   2   INA219, ω_w sat, heartbeat            I2C (mutex)
commsTask    2   RX queue → parse → cmd queue          UART RX
telemTask    1   stream buffer → CSV → USB + HC-05     UART TX (exclusive)
idle         0

Shared state:
  motor.target          float, atomic  (ctrl W / foc R)
  motor.shaft_velocity  float, atomic  (foc W / ctrl,safety R)
  emergencyStop         volatile sig_atomic_t (comms W / ctrl R)
  i2cMutex              MPU6050 + INA219
  logStream             StreamBuffer, ctrl → telem
  rxQ, cmdQ             Queues
  traceBuf              ring, written by kernel hooks
```

---

# Appendix D — Understanding check

You don't need to answer these on paper, but if any of them feels blank when we
get there, say so and we'll dig into it rather than moving on.

1. What exactly gets pushed and popped in `PendSV`, and what does the hardware do
   for you on exception entry?
2. Why are Cortex-M interrupt priorities numerically inverted, and what does
   `BASEPRI` do?
3. What breaks if an ISR at priority 0 calls `xQueueSendFromISR`?
4. What does `portYIELD_FROM_ISR` do, and what's the symptom of omitting it?
5. What happens to lower-priority tasks if the highest-priority one never blocks?
6. Mutex vs binary semaphore — name the mechanism that differs and give the
   scenario from your own system.
7. Why does `motor.target` need no synchronization, but `Wire` does?
8. Why is the FOC task highest priority even though the control law is "more
   important"?
9. Why is worst-case execution time the number that matters?
10. State the utilization bound. What does it guarantee and what does it not?
11. Why is a stream buffer lock-free, and what would break if two tasks wrote to
    it?
12. What did the migration reveal about your earlier identification data?

---

# Appendix E — What this sets up

The structure absorbs the translation subsystem without rework:

- **Pi UART link** → second comms task, same deferred-interrupt pattern, framed
  packets instead of ASCII
- **Trajectory task** → prio 2, 50–100 Hz, interpolating waypoints so comms
  jitter never reaches the control law
- **Kalman filter** → predict in the control task, correct triggered by vision
  arrival from comms. Multi-rate fusion falls out of the task structure.
- **Fan allocation** → extends the control task; the square-law throttle mapping
  is a static nonlinearity
- **Active desaturation** → low-priority task requesting a slow wheel ramp-down
  when the safety task flags banked momentum, replacing today's passive unwind

None of that requires changing the priority structure. That's the test of whether
the architecture was right.