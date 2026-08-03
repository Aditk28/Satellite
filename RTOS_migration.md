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

**Do.**
```c
static void us_init(void) {
  __HAL_RCC_TIM5_CLK_ENABLE();
  TIM5->PSC = (getTimerClkFreq(TIM5) / 1000000UL) - 1;
  TIM5->ARR = 0xFFFFFFFFUL;
  TIM5->EGR = TIM_EGR_UG;
  TIM5->CR1 = TIM_CR1_CEN;
}
static inline uint32_t us_now(void) { return TIM5->CNT; }
```

Unsigned subtraction handles wrap correctly: `dt = us_now() - t_prev` is right
even across the rollover, as long as you keep everything `uint32_t` and the
interval is under 71 minutes.

**Verify (no scope needed).** Compare against a known-good reference:
```c
uint32_t a = us_now(); delay(1000); uint32_t b = us_now();
Serial.printf("1000ms measured as %lu us\n", b - a);
```
Expect 1,000,000 ± a few hundred. If you get ~500,000 or ~2,000,000, the
prescaler is wrong — APB1 timers on the F446 run at **2×** the APB1 bus clock,
which is the classic off-by-2 here. Don't hardcode 89; derive it.

**Trap.** Confirm TIM5 isn't claimed by SimpleFOC — Step 0.3.

---

## Step 0.3 — Timer audit

**Concept.** `CONTROL_README` documents the TIM2/TIM3 PWM collision. But D10 is
**PB6 = TIM4_CH1**, and if SimpleFOC configured it as a timer output rather than
a plain GPIO enable, TIM4 is taken too. Find out rather than guess.

**Do.** At the end of `setup()`, after `driver.init()` and `motor.initFOC()`:

```c
static void dumpTimer(const char* n, TIM_TypeDef* t) {
  Serial.printf("%-5s CEN=%lu ARR=%lu PSC=%lu CCER=%04lX CCMR1=%04lX\n",
    n, (t->CR1 & 1), t->ARR, t->PSC, t->CCER, t->CCMR1);
}
dumpTimer("TIM2", TIM2); dumpTimer("TIM3", TIM3); dumpTimer("TIM4", TIM4);
dumpTimer("TIM5", TIM5); dumpTimer("TIM9", TIM9); dumpTimer("TIM11", TIM11);
```

A timer with `CEN=1` and non-zero `CCER` is driving an output and is off limits.

**Verify.** Fill in Appendix B. Expect TIM2/TIM3 taken. Choose the FOC tick
timer from what's free, preferring **TIM9** (16-bit, APB2, nothing else wants it),
then TIM11, then TIM4 only if confirmed free.

---

## Step 0.4 — Measure worst-case execution times

**Concept.** Average execution time is nearly useless for real-time work. A loop
that averages 200 µs but occasionally takes 4 ms will miss deadlines, and the
average won't tell you. You need **max**.

**Do.** Add a small stats accumulator and a `M` command that dumps it:

```c
typedef struct { uint32_t n, mn, mx; uint64_t sum; } stat_t;
static inline void stat_add(stat_t* s, uint32_t v) {
  if (!s->n || v < s->mn) s->mn = v;
  if (v > s->mx) s->mx = v;
  s->sum += v; s->n++;
}
#define TIME_BLOCK(st, code) do { uint32_t _a = us_now(); code; \
                                  stat_add(&(st), us_now() - _a); } while (0)
```

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
loopFOC()            ______   ____ us  ____ us  ____ us
move()               ______   ____ us  ____ us  ____ us
MPU6050 read         ______   ____ us  ____ us  ____ us
INA219 read          ______   ____ us  ____ us  ____ us
control law          ______   ____ us  ____ us  ____ us
telemetry row        ______   ____ us  ____ us  ____ us
super-loop period    ______   ____ us  ____ us  ____ us
```

Run it for at least 60 seconds including a `T90` and an `H0` so the max is real.

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

Implement three failure hooks. **All three must kill the motor first:**

```c
extern "C" void vApplicationStackOverflowHook(TaskHandle_t t, char* name);
extern "C" void vApplicationMallocFailedHook(void);
extern "C" void rtAssertFail(const char* file, int line);
```

Each should: `motor.target = 0; motor.disable(); driver.disable();` then store
`file`/`line` in a `__attribute__((section(".noinit")))` struct with a magic
value, then blink a distinctive LED pattern forever. Print the stored reason at
the top of `setup()` on the next boot — that's your black box, and without a
debugger attached it's how you'll find out what happened.

**Trap.** Every stack size in FreeRTOS on ARM is in **words** (4 bytes).
`xTaskCreate(..., 128, ...)` gives you 512 bytes. A large share of "random
crashes" reports are this.

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

---

## Step 1.4 — Build the software scheduler tracer

**This is the logic analyzer replacement.** Budget half a day. It is the single
highest-leverage thing in the guide given your tooling.

**Concept.** FreeRTOS exposes empty macros at every context switch. Define them
and you capture the exact scheduler timeline — which task ran, when, for how
long, including idle. This is precisely what SEGGER SystemView does; you're
building a minimal version of it.

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

**Phase 1 exit:** `git tag rtos-p1-kernel`. No application code changed, so no
golden dataset re-run needed.

---

# Phase 2 — Single-task port

**Goal:** the entire existing super-loop, unmodified, inside one task. Highest
value, lowest risk step in the guide.

## Step 2.1 — Wrap the loop

**Concept.** Every environmental difference between bare metal and running under
a scheduler — stack, FPU, tick, `micros()`, interrupt priorities — hits you here
with zero application changes to confuse the picture. If something breaks, it's
the environment, and you know it.

**Do.**
```c
TaskHandle_t hControl;

static void controlTask(void*) {
  hwSetup();                 /* everything setup() did after Serial.begin */
  for (;;) superLoopBody();  /* the existing loop() body, verbatim */
}

void setup() {
  Serial.begin(115200);
  us_init();
  printLastFaultReason();
  xTaskCreate(controlTask, "ctrl", 1536, NULL, 3, &hControl);
  vTaskSetThreadLocalStoragePointer(hControl, 0, (void*)1);
  vTaskStartScheduler();
}
void loop() {}
```

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
| 14 | SysTick handoff breaking SimpleFOC `_micros()` | `ω_w` wrong → linearization wrong → compFrac margin wrong |
| 15 | FOC-rate velocity quantization | Noisy `u`, shifted unwind rate |
| 16 | Telemetry stack too small (`sprintf %f`) | Overflow in the least-tested task |
| 17 | Two tasks writing `Serial` | Interleaved garbage in CSV |
| 18 | Serial bandwidth < telemetry rate | Silent drops; decimate |
| 19 | Reading the trace buffer while tracing | Torn data |
| 20 | High-water from an unexercised path | Overflow appears weeks later |

---

# Appendix B — Decision log

| # | Decision | Step | Value |
|---|---|---|---|
| B1 | Pole-pair count | 0.5 | `PP = ` |
| B2 | Timers free after SimpleFOC init | 0.3 | |
| B3 | FOC tick timer | 0.3 | `TIM` |
| B4 | FOC rate + justification | 0.5 | ` Hz` |
| B5 | `CTRL_DIVISOR` | 4.1 | |
| B6 | Measured WCETs (super-loop) | 0.4 | table |
| B7 | Telemetry decimation factor | 3.1 | |
| B8 | `ω_w` calibration check @ 2 V | 2.1, 4.2 | ` rad/s` |
| B9 | compFrac neutral, before / after | 4.5 | ` / ` |
| B10 | Direction-asymmetry outcome | 4.4 | |
| B11 | Final stack sizes | 7.1 | |
| B12 | Final `U` vs bound | 7.3 | |

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