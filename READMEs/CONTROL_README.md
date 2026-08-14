# Reaction Wheel Attitude Control

Closed-loop heading control of a free-floating platform using a single reaction
wheel. System identification, control design, firmware, tuning procedure, and
measured performance.

**Status:** working, tuning incomplete. Closed loop validated across a ±30° to
±180° envelope at a mean final error of 1.20° and 1.6 s settling, but the
terminal approach is **inconsistent** — roughly half of large negative slews
stall a few degrees short and need a retry. Cause is understood and documented
in §7 and §17; the fix needs a friction recalibration that has been deliberately
deferred. **Translation hardware is BUILT and TESTED — control not started (§18).
The active build guide is `TRANSLATION_DOCKING.md`.**

**Why deferred:** adding the fan subsystem changes the platform's mass,
inertia, and friction, so every constant here has to be re-identified anyway.
Tuning twice is wasted work — the plan is to finish the RTOS merge, build
translation, then run one full identification and tuning campaign across all
three axes at once.

**Firmware status (2026-08-08): the RTOS migration is complete.** The controller runs as
five FreeRTOS tasks (§12); the control law and constants below were ported verbatim and are
unchanged. Two caveats on the numbers in §2, both from a mid-migration hardware rework
(STM32 swapped, translation-motor mass added, wiring moved) rather than from the migration:
the plant has shifted, so **passive desaturation currently does not complete** — the wheel
holds speed in the deadzone instead of bleeding to zero — and the terminal-approach figures
predate the change. Both fold into the combined retune (§17), which was always the plan.
A physical-plausibility guard now rejects impossible single-cycle jumps in `ω_w` and `ω_p`
before they reach the control law, after wiring noise produced false 150 rad/s saturation
aborts that killed otherwise healthy slews.

---

## Contents

1. [The plant](#1-the-plant)
2. [Measured performance, and where it is inconsistent](#2-measured-performance)
3. [Identified constants](#3-identified-constants)
4. [Model derivation](#4-model-derivation)
5. [Discovery 1 — the system is not fully controllable](#5-discovery-1--the-system-is-not-fully-controllable)
6. [Discovery 2 — friction exceeds the maneuver](#6-discovery-2--friction-exceeds-the-maneuver)
7. [Discovery 3 — the linearization must under-compensate](#7-discovery-3--the-linearization-must-under-compensate)
8. [The control law](#8-the-control-law)
9. [Gain selection and LQR](#9-gain-selection-and-lqr)
10. [Momentum management](#10-momentum-management)
11. [The estimator](#11-the-estimator)
12. [Firmware and commands](#12-firmware-and-commands)
13. [Tuning procedure](#13-tuning-procedure)
14. [Calibration history](#14-calibration-history)
15. [Data formats and analysis pipeline](#15-data-formats-and-analysis-pipeline)
16. [Bugs and gotchas](#16-bugs-and-gotchas)
17. [Open items](#17-open-items)
18. [Translation subsystem — future](#18-translation-subsystem--future)

---

## 1. The plant

A platform rides on three ball transfer units. Bolted to it is a BLDC gimbal
motor whose rotor carries a flywheel. There is no external actuator.

The platform turns itself by trading angular momentum with the wheel: when the
motor accelerates the wheel one way, Newton's third law pushes the platform the
other. This is how spacecraft point themselves.

**The control problem:** given a target heading, compute a motor voltage such
that the platform reaches that heading and stays there.

### Hardware

| part | role |
|---|---|
| STM32 Nucleo-F446RE | control loop, 180 MHz |
| 4015 BLDC hollow-shaft external-rotor motor | reaction wheel |
| MT6701 magnetic encoder (SSI) | wheel angle and velocity |
| SimpleFOC Mini (DRV8313) | 3-phase gate driver, no onboard MCU |
| MPU6050 | platform angular rate (gyro Z) |
| INA219 | bus voltage and current on the motor supply |
| HC-05 | wireless telemetry and command, 115200 baud |

The driver is a bare gate driver, so the Nucleo runs the FOC loop itself
(SimpleFOC library, STM32duino core under PlatformIO). Torque/voltage mode —
true current-sensed FOC is not possible without added hardware, but voltage-mode
torque control is adequate at this torque/speed scale.

---

## 2. Measured performance

Full envelope, 2026-08-02, `Z` before every step (run `165048`, 14 valid tests):

| slew | final error (+) | final error (−) |
|---|---|---|
| 30° | 0.42° | 0.77° |
| 60° | 1.93° | 1.75° |
| 90° | 0.62° | 1.14° |
| 120° | 1.95° / 1.00° | 0.93° |
| 150° | 1.88° | 0.61° |
| 180° | 1.76° | 0.73° / 1.28° |

- **Mean final error 1.20°, worst 1.95°** — every step inside the 2° deadzone
- **Settling to ±2°: mean 1.57 s, worst 4.05 s**
- **Rise time 0.37–0.51 s, flat across all slew sizes** (see below)
- **Peak wheel speed 9–42 rad/s**, scaling with slew size; wheel returns to
  approximately zero on its own after every step
- **Peak `U_q` 1.9–9.4 V** of a 10 V limit
- Direction asymmetry in final error: 1.31° positive vs 1.05° negative —
  negligible. Apparent asymmetry in earlier batches was an artifact of not
  zeroing heading between tests.

### Where it is inconsistent

The envelope figures above are the runs that completed. They are not the whole
picture:

- **Roughly half of large negative slews (−90°, −180°) stall 3–6° short.** The
  stall detector catches it and retries, and the retry usually lands, but not
  always — one observed session cycled stall → retry → stall until stopped
  manually.
- **The cause is `A_FRICTION` being too small**, not the gains. See §7. A
  correction from rest at 5° error was clearing the breakaway threshold by only
  about 10%, which makes success a coin flip on stiction. Raising `A_FRICTION`
  from 22.3 toward 28.3 improves it but has not been swept properly.
- **Small errors are the hard case, not large ones.** At 90° of error the PD term
  alone dwarfs friction. At 5° it contributes almost nothing and feedforward has
  to do all the work, which is exactly where its magnitude error shows.

Everything above ~10° of error is reliable. The last few degrees are not.

**Rise time being flat from 30° to 180° is the most informative number here.**
A linear system's rise time grows with step size. This one's does not, because
the response is dominated by Coulomb feedforward and the deadzone rather than by
the PD law — `alpha` is exactly zero for roughly 83% of a typical capture. The
controller is better described as "break stiction, coast, stop" than as a
second-order system, and the second-order design is what sets the coast rate.

Overshoot runs 2–7%, implying ζ ≈ 0.56 rather than the designed 0.7. Left
alone: 12% overshoot settling in 1.2 s with sub-degree error is a good operating
point.

---

## 3. Identified constants

Everything the controller uses, and where each number came from.

| constant | value | units | provenance |
|---|---|---|---|
| **`A_1`** | **45.5** | (rad/s²)/V | `K'/τ'` from six ±3 V open-loop captures |
| **`A_2`** | **5.35** | 1/s | `1/τ'`, same captures |
| `K'` | 8.51 | (rad/s)/V | wheel plateau ÷ command, 2.3% scatter over 6 tests |
| `τ'` | 0.187 | s | spin-up 0.181, free decay 0.193 |
| **`a = J_w/J_p`** | **0.19** *(nominal)* | — | regression of platform rate vs wheel-speed change, 48 tests, 20% scatter. **Evidence says the true value is 0.15–0.17** — see note below |
| **`A_FRICTION`** | **28.3** *(runtime tunable, `A<val>`)* | rad/s² | `4.24/a`. Was fixed at 22.3 using `a` = 0.19; raised to 28.3 for `a` = 0.15. **Not swept — the main known tuning gap** |
| platform Coulomb friction | 4.24 | rad/s² | breakaway sweep, 0.55 V step |
| **`compFrac`** | **0.89** | — | fraction of `A_2` actually applied; measured neutral point 0.892 |
| `deadzone` / `deadzoneFine` | 2.0° / 2.0° | deg | equal, i.e. a single 2° tolerance. Fine was 1.0° and that was below the reachable floor — see §10 |
| residual wheel pole at `compFrac` | −0.84 | 1/s | from the stall condition, §7 |
| motor deadband | 0.35 | V | staircase run; below this the wheel does not sustain rotation |
| `K_HOLD` | 8.13 | (rad/s)/V | wheel speed sustained per volt at 42 rad/s |
| gyro bias | 0.42–0.46 | dps | at-rest windows; re-measured every boot |
| **`GYRO_SIGN`** | **−1** | — | gyro and encoder use opposite conventions |
| `R`, `K_v` | 0.788 Ω, 0.113 V/(rad/s) | | least-squares over 24 from-rest tests. **Not used by the controller** — kept because absolute torque units need them |

### On `a`, and why it is not being chased

`a` is the only plant parameter in the gain formulas, and it is the least
certain number in the project. Every estimate from motion data is biased low by
Coulomb friction, which eats platform acceleration:

| window | `a` estimate |
|---|---|
| 0.01–0.08 s | 0.159 |
| 0.01–0.13 s | 0.151 |
| 0.02–0.20 s | 0.140 |
| 0.05–0.30 s | 0.122 |

Extrapolating toward t → 0 gives roughly 0.16–0.17. A geometric measurement
(`½mR²`) would settle it but has not been done.

**This is deliberately left alone**, and `A_FRICTION` is tuned empirically
instead (command `A<val>`). Nothing downstream depends on knowing `a` precisely,
because the gains were tuned on hardware rather than computed from it. What `a`
uncertainty does explain, and what matters to keep in mind:

- `A_FRICTION` was originally computed as `4.24/0.19 = 22.3`. If `a` is 0.15 the
  true figure is 28.3, so feedforward was running **~25% light**. Large errors
  had enough PD to cover the gap; small ones did not. This is the direct cause of
  the intermittent terminal stalls in §2. The code only ever uses the product
  `ffFrac × A_FRICTION`, so either can absorb the correction — but the constant
  was wrong, not the fraction.
- Two authority-clamp values (`ALPHA_STALL_MAX` = 28, then 40) were sized against
  `a = 0.19` and both failed on hardware. See §16.

### Why `A_1` and `A_2` rather than `J_w`

They are measured end-to-end from input to output, so they are immune to
convention questions — SimpleFOC's voltage scaling, phase-vs-line resistance,
modulation factors. Backing out an absolute `J_w` gives ~0.0035 kg·m², far larger
than an HDD platter should be, which signals an unaccounted convention factor.
Nothing in the controller uses `J_w` alone, so it does not matter.

---

## 4. Model derivation

### Step 1 — voltage to current

A motor is a resistor and a generator in one package:

```
I_q = (U_q − K_v·ω_w) / R
```

At standstill 1 V pushes `1/0.788 = 1.27 A`. As the wheel speeds up it generates
`0.113·ω_w` volts against you; current and torque fall to zero at
`ω_w = 8.85 rad/s`. That is why wheel speed plateaus, and why the measured 8.51
rad/s per volt sits below 8.85 — friction takes a cut. Inductance is neglected:
`L/R` is microseconds against a 187 ms mechanical time constant.

### Step 2 — current to torque

```
τ = K_t·I_q
```

### Step 3 — torque splits between two bodies

```
J_w·ω̇_w = τ                          (wheel)
J_p·ω̇_p = −τ − T_friction            (platform)
```

The minus sign is the entire mechanism.

### Step 4 — combined wheel dynamics

Chaining the three gives a first-order lag — a push term minus a drag term:

```
ω̇_w = A_1·U_q − A_2·ω_w = 45.5·U_q − 5.35·ω_w
```

`A_1 = K_t/(R·J_w)` and `A_2 = K_t·K_v/(R·J_w)` in principle, but both were
measured directly: steady state gives `A_1/A_2 = K' = 8.51`, and the 63% rise
time gives `1/A_2 = τ' = 0.187 s`.

### Step 5 — full state space

With `x = [θ_p, ω_p, ω_w]` and input `U_q`, friction dropped. Every entry is
`a`, `A_1`, `A_2`, or a 1 — the matrix contains no new information, but it
permits questions the scalar equations cannot answer:

```
      ⎡0   1      0   ⎤        ⎡  0  ⎤
A  =  ⎢0  −c   1.016  ⎥   B =  ⎢−8.65⎥
      ⎣0   0   −5.35  ⎦        ⎣45.5 ⎦
```

The first question asked of it returned a bad answer.

---

## 5. Discovery 1 — the system is not fully controllable

### The evidence

Controllability matrix `[B, AB, A²B]` returns **rank 2 of 3**, determinant
exactly zero — not a numerical artifact. PBH localizes it to the λ = 0 mode. The
left eigenvector identifies the untouchable combination:

```
c·θ_p + ω_p + a·ω_w = constant          (verified: wᵀB = −2.3×10⁻¹⁶)
```

### What it means

**This is angular momentum conservation.** An internal torque can only move
momentum between wheel and platform, never change the total. This is precisely
why real spacecraft carry magnetorquers or thrusters — a reaction wheel cannot
desaturate itself.

**Consequence:** from rest to rest under pure viscous friction, the platform
*cannot hold a new heading with the wheel stopped*. It can only park at an offset
while the wheel keeps spinning — 1 V holds about 20°. Adding an integrator does
not fix it (rank 3 of 4) and the LQR solve fails with "no finite solution."

The voltage staircase measured this before it was derived: platform peak rate
stayed flat at **1.0–2.2 dps from 0.35 V to 0.80 V** while wheel speed climbed
**1.7 → 6.4 rad/s**. Constant voltage produced no sustained rotation.

### The fix — feedback linearization

The platform does not care about voltage. It cares about wheel *acceleration*,
and `ω_w` is measured. So stop treating voltage as the input, treat
`α = ω̇_w` as the input, and solve the wheel equation backwards:

```
U_q = (α + A_2·ω_w) / A_1
```

The `A_2·ω_w` term pre-cancels back-EMF at whatever speed the wheel happens to
be at; whatever remains produces the commanded acceleration. The wheel state
disappears from the model — absorbed into the input transformation — leaving:

```
θ̈_p = −a·α
```

A double integrator. Two states, rank 2 of 2, **controllable**, and the
best-understood plant in control theory.

---

## 6. Discovery 2 — friction exceeds the maneuver

Converting everything to platform angular acceleration:

| quantity | value |
|---|---|
| max authority (10 V) | 86 rad/s² |
| **Coulomb friction** | **4.24 rad/s²** |
| 90° slew in 2 s requires | 1.57 rad/s² |

**Friction is nearly three times the maneuver itself.** Not a correction — the
dominant term.

### The deadband this creates

The controller commands platform acceleration `a·K_θ·e`. Motion only occurs above
4.24 rad/s², so there is a deadband in heading error:

```
|e| > 4.24 / (a·K_θ)
```

| `K_θ` | deadband |
|---|---|
| 19.1 | 67° |
| 43.0 | 30° |
| 119.3 | 11° |

**Pure feedback cannot point this platform at any sane gain.** Reaching 2° by
gain alone would need `K_θ ≈ 640`, wildly unstable.

**This was reproduced exactly on hardware.** With feedforward disabled at
`K_θ = 19.1`, a hand nudge stalled at 28.16° with `alpha = 9.39`, i.e. a
commanded platform acceleration of 1.78 rad/s² against a 4.24 breakaway. The
predicted deadband was 67°; the observed stall sat comfortably inside it.

### Viscous vs Coulomb

Fitting both models to all 48 tests:

| model | mean R² |
|---|---|
| viscous | 0.826 |
| **Coulomb** | **0.918** |
| both terms | 0.951 |

**Coulomb wins in 43 of 48 tests.** Physically sensible — rolling resistance in
ball bearings is closer to constant drag than rate-proportional. The viscous
coefficient came out with **84% scatter** (4.05 ± 3.39), the signature of fitting
the wrong model form.

### Feedforward — the fix

```c
if (|e| > deadzone) {
    ff = (|ω_p| > W_MOVING) ? −A_FRICTION·sign(ω_p)   // MOVING: cancel friction
                            : +A_FRICTION·sign(α);    // STUCK:  push to break free
    α += ffFrac · ff;
}
```

**The two branches have opposite signs.** Derivation: we want the closed loop to
behave as though friction were absent, `θ̈ = −a·α_lqr`. With friction present:

```
−a·(α_lqr + α_ff) − A_f·sign(ω_p) = −a·α_lqr
⟹  α_ff = −(A_f/a)·sign(ω_p)          [MOVING: negative]
```

When stuck there is no velocity to oppose; instead push whichever way the
controller wants, so the sign follows `α`. Getting the moving branch backwards is
**worse than no feedforward at all** — simulated final error 48° versus 27° with
none, versus 0.7° correct.

### The magnitude is the weak point

The *structure* of the feedforward is right and well tested. Its *magnitude* is
not well calibrated, and that is the project's main outstanding tuning gap.

Break-free from rest needs the delivered wheel acceleration to exceed
`4.24/a` — about **28 rad/s²** at `a` = 0.15. What the controller actually
supplies at small error is:

```
ffFrac × A_FRICTION + K_θ·|e|
```

At `ff = 0.90`, `A_FRICTION = 22.3`, and a 5° error that is
`20.1 + 10.4 = 30.5` — clearing 28 by 9%. Stiction varies by more than 9% with
contact point and dwell, so whether it moves is effectively random. Raising
`A_FRICTION` to 28.3 gives `25.5 + 10.4 = 35.9`, a 28% margin.

**Symptoms of each direction of error:**

| symptom | meaning |
|---|---|
| Parks short, does not move, wheel winds up | `ffFrac × A_FRICTION` too **low** |
| Overshoots then creeps back | too **high** — the MOVING branch is over-cancelling friction |

Both knobs (`A<val>` and `F<val>`) act on the same product. Tune one and leave
the other alone.

### Friction is also what makes reorientation possible

Under *pure viscous* friction the conservation law says the platform returns to
its original heading whenever the wheel stops — you could never permanently
reorient. **Coulomb friction breaks that conservation law.** It lets the platform
stick at a new heading with zero stored momentum. So friction simultaneously
limits precision and enables the machine to work at all. On a real satellite
there is no friction, which is exactly why they need separate desaturation
hardware.

---

## 7. Discovery 3 — the linearization must under-compensate

The linearization is only as good as `A_2`, and the failure is **asymmetric**:

```
real plant:    ω̇_w = A₁′·U_q − A₂′·ω_w
commanded:     U_q = (α + compFrac·A_2·ω_w) / A_1
result:        ω̇_w = (A₁′/A₁)·α  +  [ (A₁′/A₁)·compFrac·A_2 − A₂′ ]·ω_w
                                     └──── this must be ≤ 0 ────┘
```

**Over-compensate and that residual pole goes positive: voltage drives speed,
speed demands more voltage through the compensation term, exponential runaway.
Under-compensate and it is merely a stable lag.** Always err low.

### How this was found

The first closed-loop run diverged to wheel saturation. At `ω_w = −22.2` the
controller commanded `α = +2.04` (braking) and the wheel delivered `−24`. Fitting
the residual pole gave **+0.89 /s**, an unstable 1.13 s growth — which matched the
observed runaway once the controller's opposing effort was accounted for.

Cause: the original `A_1 = 40.56`, `A_2 = 5.56` implied a DC gain of 7.3, against
a re-measured **8.51**. A 16% gain error was enough to flip the pole's sign.

### `compFrac`, and how it was measured

The `C<V>` command spins the wheel open-loop, then commands `α = 0` with the
compensation still active. The wheel must coast down; if it holds speed the
compensation is exactly neutral, if it accelerates it is too high.

Sweeping `compFrac` over 7 trials in both directions gave straight lines:

```
positive ω_w:  pole = 3.00·cf − 2.92    →   neutral at cf = 0.972
negative ω_w:  pole = 3.68·cf − 3.28    →   neutral at cf = 0.892
```

`cf = 1.0` was **already unstable** positive-going even with corrected constants,
so the true damping pole is nearer 5.0 than 5.35. Rather than chase `A_2` again,
the margin lives in `compFrac` where it is visible.

### Choosing the value

`compFrac = 0.80` was tried first and was too conservative. Under-compensation is
subtracted from *every* commanded α, so a sustained α decays with the residual
pole — at 0.80 that was 1.9 s, and slews ran out of torque mid-maneuver. **0.89**
sits at the lower measured neutral and gives roughly a 4 s torque hold.

### What under-compensation costs, quantitatively

With `compFrac = 0.89`, residual pole ≈ **−0.84 /s**, so a sustained α drives the
wheel to a steady state rather than accelerating forever:

```
ω_w,steady ≈ α / 0.84
```

This is the single most useful relation in the file. It sets:

- **How long torque lasts.** Roughly 4 s before an α stops producing acceleration.
- **Delivered vs commanded α.** `delivered = commanded − 0.84·ω_w`. A terminal
  correction needs ~28 delivered; arriving at 20 rad/s therefore needs 45
  commanded. Steps arriving below ~15 rad/s landed at 0.31°; those arriving above
  ~17 stalled 2–9° short until the clamp was raised.
- **Passive desaturation.** See §10.

### Direction asymmetry is drift, not a property

Three measurements disagree on which direction is faster: the O-tests said
positive by 3.8%, the C-sweep said negative by 7.5%, the original 4 V campaign
said negative by 7.2%. Magnitude is consistently a few percent; sign is not
stable. Most likely thermal.

**Do not build direction-dependent compensation.** A per-direction value would be
wrong half the time. One conservative value covers the drift.

---

## 8. The control law

Runs at 200 Hz (`CONTROL_PERIOD_US = 5000`) in `heading_control.cpp`.

```c
// 1. SENSE
float w_p = GYRO_SIGN * (readGyroZ() − gyroBias) * PI/180;  // rad/s, platform
float w_w = motor.shaft_velocity;                            // rad/s, wheel

// 2. ESTIMATE heading
theta = wrapPi(theta + w_p * dt);

// 3. ERROR, with two-stage tolerance (see §10)
float dz = (|w_w| < FINE_WW) ? deadzoneFine : deadzone;
float e  = wrapPi(target − theta);
bool outside = |e| > dz;

// 4. LQR / PD → desired WHEEL ACCELERATION
float alpha = −K_theta * e + K_omega * w_p;

// 5. COULOMB FEEDFORWARD (§6)
alpha += ffFrac * ff;

// 6. AUTHORITY CLAMP, only while the platform is stationary
if (notMoving) alpha = constrain(alpha, ±ALPHA_STALL_MAX);

// 7. FEEDBACK LINEARISATION → voltage
float u = (alpha + compFrac * A_2 * w_w) / A_1;

// 8. SATURATE + COMMAND
motor.target = constrainf(u, −VOLTAGE_LIMIT, VOLTAGE_LIMIT);

// 9. SUPERVISE
if (|w_w| > WHEEL_SAT_LIMIT) stopMotor();
```

### Sign warning (a) — the LQR line

```
alpha = −K_theta*e + K_omega*w_p
        ^^^^^^^^^^ minus     ^^^^^^^^^ PLUS
```

Derivation:

```
plant:  θ̈ = −a·α                (minus: wheel one way, platform the other)
error:  e = θ_target − θ_p,  so  ė = −ω_p  and  ë = −θ̈
⟹      ë + a·K_ω·ė + a·K_θ·e = 0
```

**The plus stops being strange once you see `ω_p = −ė`.** In error coordinates it
is an ordinary damping term; it only looks inverted because the gyro reports the
platform's rate and the error's rate is its negative. Writing
`−(K_θ·e + K_ω·ω)` puts negative damping in that equation — in simulation it
diverged, ending 34° short of a 90° target with `α` past 600 rad/s².

### Sign warning (b) — the feedforward branches

Covered in §6. Moving branch negative, stuck branch positive.

### The authority clamp

A sustained α drives the wheel toward `α/0.84`, so `α = 62` (which `K_θ = 119.3`
produces at 30° error) implies 74 rad/s — far past the abort. But this only
matters while the platform is **stationary**: when it is moving the maneuver
finishes first, and clamping there just slows large slews. The pathological case
is pushing hard while stuck, where no motion results and the wheel winds anyway.

`ALPHA_STALL_MAX` must exceed the true breakaway with real margin. Two values
were tried and failed — see §16.

---

## 9. Gain selection and LQR

### Why a matrix formulation gives you the gains

For the 2-state plant `[θ_err, ω_p]`, `u = −Kx` written out is one gain on error
and one on rate. **That is the definition of PD** — the architecture is forced by
the state dimension. LQR's contribution is choosing the two numbers.

Matching the closed-loop equation `ë + a·K_ω·ė + a·K_θ·e = 0` against the
standard second-order form `ë + 2ζω_n·ė + ω_n²·e = 0`:

```
K_θ = ω_n² / a          K_ω = 2ζ·ω_n / a          ω_n = 5.714 / t_settle
```

**You tune in units of "how fast should this settle," not in units of "119.3."**

### LQR vs pole placement

LQR minimizes `J = ∫(xᵀQx + uᵀRu)dt`, giving `K = R⁻¹BᵀP` where `P` solves the
Algebraic Riccati Equation. Solved offline in one line
(`scipy.linalg.solve_continuous_are`) — **never on the STM32**; compute once,
paste two numbers into firmware.

On this plant a weighting sweep lands at ζ ≈ 0.71 regardless of `Q`, a known
signature of double integrators. Given that, pole placement is more direct and is
what the table below uses.

### Gain table (ζ = 0.7, `a` = 0.19 nominal)

| settle | `ω_n` | `K_θ` | `K_ω` |
|---|---|---|---|
| 3.0 s | 1.90 | 19.1 | 14.0 |
| 2.0 s | 2.86 | 43.0 | 21.1 |
| 1.5 s | 3.81 | 76.4 | 28.1 |
| **1.2 s** | **4.76** | **119.3** | **35.1** | ← in use

**Work DOWN the table, not up.** Counterintuitive if you are used to creeping up
cautiously, but slow gains keep the feedforward active longer, which winds the
wheel up more — so faster gains are better on *both* accuracy and saturation.
Simulated 90° slew: 3.0 s gains → 6.30° error / 72.9 rad/s; 1.5 s gains →
0.03° / 22.4 rad/s. **Confirmed on hardware:** at `K_θ = 19.1` a nudge recovery
stalled at 15.6°; at 119.3 the same recovery lands within 2°.

The theoretical ceiling is `ω_n ≈ 4.8` (the wheel pole sits at `A_2`, above which
the linearization stops cancelling cleanly). The 1.2 s row sits at 4.76 and
behaves — plausibly because the corrected `A_2 = 5.35` and partial compensation
moved the effective pole.

### Honest scope

For a 2-state system this **is** a PD controller; LQR is a principled way to
choose PD gains, not a different architecture. Its real value arrives with the
translation fans (MIMO, where hand-tuning a dozen gains is miserable) and when
`ω_w` enters the cost function so momentum management falls out of the
optimization automatically — instead of out of a compensation margin, which is
what currently happens.

---

## 10. Momentum management

Every slew banks wheel speed, and Coulomb friction holds the platform at its new
heading, so the momentum does not return on its own. A reaction wheel cannot
desaturate itself (§5).

### Passive desaturation — the mechanism in use

Inside the deadzone `α = 0`, so the commanded voltage is
`compFrac·A_2·ω_w/A_1 = 0.105·ω_w`, while merely *holding* speed needs
`ω_w/K_HOLD = 0.123·ω_w`. The shortfall bleeds the wheel down:

```
ω̇_w = −0.84·ω_w          (a 1.2 s exponential)
```

**The important property is that it is self-limiting.** Reaction torque during
the unwind is `a·0.84·ω_w = 0.16·ω_w` rad/s² on the platform, which stays under
the 4.24 breakaway for any `ω_w` below 26.5 rad/s. The wheel comes home, friction
holds the heading, and there is no logic and no tuning.

Observed: every step in the envelope run ended with `ω_w ≈ 0`, except those where
the 6 s capture ended first — a repeat of the same command from the same state
reached −1.34 rad/s given more time.

Above 26.5 rad/s the unwind torque does exceed breakaway, so a fast-ending slew
may drag the heading slightly on the way down. Not observed to matter.

### An active ramp was written and removed

Commanding `α = −10` to brake gives `u = (−10 + 0.89·5.35·20)/45.5 = 1.87 V` at
20 rad/s against a 2.46 V hold voltage — so the wheel decelerated at **27 rad/s²,
not 10**. That is 5.1 rad/s² on the platform, above breakaway: the platform broke
free, the heading drifted, the controller fought back, and the wheel wound up.

**The compensation shortfall that starves a positive α ADDS to a negative one.**
Never command braking torque through the linearization at speed. If this is
revisited it must command voltage directly from the hold curve
(`u_hold = ω_w/K_HOLD`), with a P term on wheel speed so it tolerates error in
`K_HOLD`.

### Stall recovery

Passive unwind cannot reach one case: parked **outside** the deadzone with the
wheel already fast, where the wheel cannot deliver the commanded α and pushing
harder only winds toward the abort. Detection is `|e| > dz` AND platform
stationary AND `|ω_w| > STALL_WW` for `STALL_MS`. Response is `α = 0` for 2 s —
the same state as being inside the deadzone — letting the passive unwind restore
authority, then retry.

**Retries are capped at `MAX_STALL_RETRIES` = 3.** Without a cap the controller
cycles stall → unwind → retry → stall indefinitely, which was observed in
practice: the unwind restores authority, but if `ffFrac × A_FRICTION` is below
breakaway the retry fails for the same reason as the original attempt, forever.
After three, it prints `PARKED at <e> deg` and holds until a new target, `Z`, or
`X`. A `PARKED` message means the friction magnitude is too low, not that the
gains are wrong.

### Two-stage deadzone — the terminal approach

A fine correction needs the wheel **slow**, because
`delivered α = commanded − 0.84·ω_w`. So the tolerance is speed-gated:

```c
dz = (|ω_w| < FINE_WW) ? deadzoneFine : deadzone;
```

The slew runs against the coarse 2° tolerance, parks, the wheel unwinds, and as
it drops below 5 rad/s the tolerance tightens to 1° — at which point the
controller re-engages with full authority available and creeps in. This is the
"reduced-speed, tightened-deadband final approach" from Phase 9 of the project
plan, and it is what makes sub-degree pointing reachable.

`deadzoneFine` must stay **above** the achievable Coulomb deadband
`(1 − ffFrac)·A_FRICTION/K_θ`, or feedforward never switches off and the wheel
winds indefinitely. **This was shipped wrong once:** the fine deadzone defaulted
to 1.0° while the floor at `ff = 0.90`, `K_θ = 119.3` was 1.07–1.36°, making it
unreachable and producing exactly the stall it was meant to avoid. Both deadzones
are now 2.0°. `G` prints the floor every time and warns if either is below it:

```
deadband floor=1.07deg  (both deadzones clear it)
```

| `ffFrac` | deadband floor | safe fine deadzone |
|---|---|---|
| 0.90 | 1.36° | 2° |
| 0.95 | 0.68° | 1° |
| 0.97 | 0.41° | 0.75° |

### The deadzone is not optional

With feedforward always active it never switches off once the platform has
stopped, and keeps accelerating the wheel:

| deadzone | final error | wheel peak |
|---|---|---|
| none | 0.68° | **72.9 rad/s** |
| 1° | 0.57° | 31.7 rad/s |
| 2° | 1.53° | 29.9 rad/s |

---

## 11. The estimator

`θ_p` is not measured directly. Two imperfect sources:

| sensor | strength | weakness |
|---|---|---|
| MPU6050 gyro | fast (hundreds of Hz) | measures *rate*; bias integrates into drift |
| AprilTag camera | absolute, drift-free | slow (~30 Hz), latent, drops out |

They fail in complementary ways, which is when fusion helps. **Currently
gyro-only** — vision is not built.

**Gyro bias** is measured at every boot from a 200-sample at-rest average.
Historically 0.42–0.46 dps, stable to ±0.014 dps across a sweep. Uncorrected that
is **25° per minute of pure fiction**; after removal, drift is about
**0.8°/min** — fine for 30-second tests, and the hard floor on absolute accuracy
until vision exists.

**Complementary filter** (start here):
```c
theta = 0.98f*(theta + w_p*dt) + 0.02f*theta_vision;
```

**Kalman filter** (upgrade): state `[θ_p, ω_p, b_gyro]`. Predict with the gyro,
correct with vision. Estimating bias **as a state** makes it self-calibrating
against thermal drift rather than relying on a startup measurement.

### The wheel encoder cannot give you heading

In principle momentum conservation gives `θ_p = −a·θ_w`, and on a frictionless
spacecraft that would work. **It fails here because friction is an external
torque and it is large** — 4.24 rad/s² against maneuvers of ~1.57. An
encoder-derived heading would be badly wrong within one slew.

What the encoder *is* for: `ω_w` for feedback linearization (mandatory),
saturation monitoring, and improving the Kalman prediction step.

---

## 12. Firmware and commands

Only one sketch can be in `src/` at a time (PlatformIO builds one
`setup()`/`loop()`). Inactive ones live in `unflashed_files/`.
`MagneticSensorMT6701SSI.h/.cpp` stay in `src/` always.

| file | purpose |
|---|---|
| `rtos_main.cpp` | **the controller** (env `rtos`) — FreeRTOS, five tasks, 200 Hz control / 4 kHz FOC |
| `heading_control.cpp` | the original super-loop (env `superloop`) — kept as the regression reference |
| `telemetry.*` `safety.*` `commands.*` `i2c_bus.*` | RTOS subsystems, see below |
| `timebase.*` `hw_timers.*` `faults.*` `trace.*` `timing_stats.h` | measurement + fault infrastructure |
| `enc_test.cpp` | standalone bare-metal MT6701 test (env `enctest`) — 30-second hardware-vs-firmware check |
| `calibration.cpp` | system ID: `MODE_STEP` (run 2), `MODE_STAIRCASE` (superseded, kept for motor deadband), `MODE_BREAKAWAY` (run 5) |
| `full.cpp` | earliest open-loop bring-up, superseded |

### Firmware architecture — FreeRTOS, five tasks

The super-loop was migrated to FreeRTOS in seven phases (see `RTOS_migration.md` for the
full record, including the failures). The control law, constants, and sensor code were
ported **verbatim** — the migration changed structure, not behaviour.

| task | prio | period | owns |
|---|---|---|---|
| `focTask` | 4 | 250 µs (4 kHz) | `loopFOC()` + `move()` — commutation only |
| `controlTask` | 3 | 5 ms (200 Hz) | gyro read, control law, capture buffer, command execution |
| `safetyTask` | 2 | 50 ms (20 Hz) | independent watchdog: wheel overspeed, heartbeat, INA219 power, fanTask stall |
| `commsTask` | 2 | 2 ms poll | serial RX + line assembly — **sole reader** |
| `fanTask` | 2 | 3 ms (333 Hz) | DSHOT300 on TIM1 + DMA burst — **sole fan writer** |
| `telemTask` | 1 | event | **all** serial output — **sole writer** |

`fanTask` was added by `TRANSLATION_DOCKING.md` Phase 1 (tag `trans-p1-fans`). It cost
the control loop nothing — measured ctrl period 4999 / 5000 / 5001 µs afterwards, at or
better than the pre-fan baseline — because it sits below control and FOC and shares no
mutex, bus or port with them.

One TIM9 interrupt at 4 kHz notifies `focTask` every tick and `controlTask` every 20th
(`CTRL_DIVISOR`), so control is phase-locked to commutation with no drift between clocks.

**Two invariants hold the design together, both learned the hard way:**
- **One writer.** Only `telemTask` writes the serial ports. `HardwareSerial` is not
  reentrant; a *partially* applied version of this rule corrupted the driver, froze the
  board, and produced frozen sensor reads that looked exactly like a hardware fault.
- **One reader.** Only `commsTask` reads them. Two readers race the RX ring buffer and one
  silently eats the other's bytes.

**Shared state:** `motor.target` and `motor.shaft_velocity` are aligned 32-bit floats —
atomic on Cortex-M4, no mutex needed. The **I2C bus does** need one (MPU6050 on control at
200 Hz, INA219 on safety at 20 Hz): a real mutex, for priority inheritance. The control task
takes it with a **2 ms timeout, never `portMAX_DELAY`**, and on failure degrades to the
previous gyro sample — a control loop blocked forever on a wedged bus with a spinning
flywheel is how runaways happen.

**Measured performance (2026-08-08):** control period 4994 / 5000 / 5006 µs (200.00 Hz,
±6 µs); FOC tick 238–261 µs; CPU 42% ctrl / 15% foc / 11% telem / 2% safety / **27% idle**.
Response-time analysis proves every deadline is met (control converges at R = 3599 µs
against a 5000 µs deadline) even though total utilisation 0.868 exceeds the Liu & Layland
rate-monotonic bound of 0.757 — the bound is sufficient, not necessary.

**What the migration bought, concretely:** a telemetry dump used to stall the control loop
for **11.5 seconds**; it now costs **6 µs**. Commutation used to stop for 2.4 ms during every
gyro read; it now runs at a flat 4 kHz straight through it.

### Commands (115200, newline-terminated, USB or HC-05)

| send | does |
|---|---|
| `O<V>` | **letter O.** Open-loop pulse, 800 ms, no feedback — the sign check |
| `C<V>` | Compensation test: spin up open-loop, then `α = 0` — the `compFrac` check |
| `T<deg>` | Step to target and capture |
| `H<deg>` | Hold heading with slow telemetry |
| `Z` | Zero the heading estimate here |
| `P<val>` / `D<val>` | Set `K_θ` / `K_ω` |
| `K<val>` | Set `compFrac` |
| `A<val>` | Set `A_FRICTION`, the feedforward magnitude |
| `F<val>` | Set feedforward fraction |
| `W<deg>` / `N<deg>` | Set coarse / fine deadzone |
| `G` | Print gains, state, the deadband floor, and `ffFrac × A_FRICTION` |
| `B` | Re-measure gyro bias (platform must be still) |
| `X` / `R` | Stop / resume |
| `M` / `M!` | Print / reset timing stats (per-block min/mean/MAX, FOC tick jitter) |
| `U` | System report: per-task CPU % and stack high-water marks |
| `E` | Encoder diagnostic — motor off, stream shaft angle/velocity to hand-turn the wheel |
| `V<V>` | Manual constant-voltage drive + unlimited encoder stream; any key stops |
| `S<n>` | **Fans:** select channel 1–4, or `S0` for all four |
| `L<pct>` | **Fans:** throttle the selection. `L0` = off but still armed. Clamped to `FAN_THROTTLE_MAX` = 30%, and auto-zeroes after 10 s with no refresh |

**Fan safety.** `X` and every fault path hard-kill the fans **before** the wheel — the
props are unguarded by choice (`TRANSLATION_DOCKING.md` B7), so they are the larger
hazard. A hard kill drives PA8–PA11 low as GPIO rather than trusting the DMA path, and
latches; `R` re-arms (~1 s). The 30% ceiling is applied inside `fans_setThrottle()`, so
a runaway control law is bound by it too.

**Any unrecognised input stops the motor.** Deliberate — a confused operator
should not leave a flywheel spinning.

Safety: hard abort above `WHEEL_SAT_LIMIT` = 45 rad/s. This dumps the wheel
instantly, which spins the platform (roughly 42 rad/s² against a 4.24 breakaway).
Accepted behaviour; with the current clamp it should not be reached.

**Build:** PlatformIO, STM32duino core, Nucleo-F446RE. SimpleFOC 2.4.0, Adafruit
MPU6050 / INA219 / Unified Sensor / BusIO, Wire, SPI.

### Current tuned values

```
A_1 = 45.5    A_2 = 5.35    a = 0.19 nominal (true ~0.15)
GYRO_SIGN = −1              compFrac = 0.89
K_θ = 119.3   K_ω = 35.1    ffFrac = 0.90    A_FRICTION = 28.3  (needs sweeping)
deadzone = 2.0°             deadzoneFine = 2.0°       FINE_WW = 5 rad/s
ALPHA_STALL_MAX = 55        STALL_WW = 25             WHEEL_SAT_LIMIT = 45
MAX_STALL_RETRIES = 3
```

Also in the repo: `live_monitor.py`, a self-contained serial bridge and browser
instrument panel for watching heading, wheel speed, and momentum live. Useful for
filming and for watching a stall happen in real time.

---

## 13. Tuning procedure

Keep a finger on `X` throughout. **`Z` before every `T`** — heading carries over
between tests otherwise, which silently turns a `T-60` into a 120° slew.

### Pre-flight

1. **Confirm the right binary.** Boot text should list `O<V> openloop`.
2. **Gyro bias** 0.35–0.55 dps with the platform dead still. Outside that, re-run `B`.
3. **Estimator sanity.** `G` a few times over 30 s: drift under ~0.5°/min. Rotate
   ~90° by hand, `G` again: should read about ±90.
4. **Sign check — `O1`. Do not skip.**

   | `omega_w` | `theta_deg` | verdict |
   |---|---|---|
   | positive | **negative** | correct |
   | positive | positive | flip `GYRO_SIGN` to `+1.0f`, reflash |

   The open-loop pulse is bounded either way. Closed loop with the sign inverted
   drives error larger until the wheel abort catches it.

5. **Compensation check — `C3` and `C-3`.** The wheel must coast DOWN after the
   pulse. Flat means neutral with no margin; growth means `compFrac` too high.
   Sweep with `K<val>` to find the neutral point in each direction, then set
   `compFrac` at or just below the lower one.

### Then

6. **First closed loop.** Start at the 3.0 s row with `F0`, `Z`, `H0`. Nudge by
   hand — it should push back and settle at a large offset (that is the friction
   deadband, expected). If it accelerates away, `X` immediately.
7. **Enable feedforward** at `F0.85` and repeat the nudge. Should return closer.
8. **Gain progression: work DOWN the table**, re-running `T90` each time. Stop and
   back off one row on ringing, audible buzz (lower `D` only), or the wheel
   climbing toward 45 rad/s.
9. **Friction magnitude (`A`), then feedforward trim (`F`).** These act on the
   same product `ffFrac × A_FRICTION`, so sweep one. Break-free from rest needs
   about 28 delivered. Parks short and the wheel winds → raise; overshoots then
   creeps back → lower. **Test at small errors, not large ones** — a 90° slew
   will succeed with a badly wrong value because PD covers the shortfall. Command
   a 5° correction from rest and repeat it ten times; that is the case that
   discriminates. This sweep has not been done properly and is the main reason
   the terminal approach is inconsistent.
10. **Deadzone (`W`, `N`).** Trades accuracy against wheel windup. `N` must stay
    above the floor in §10's table. Never zero.
11. **Record.** Final `G`, a clean `T90`, a `T-90`, an `H0` nudge run. Then update
    the constants at the top of the source so tuned values survive a power cycle.

### Expected, not faults

- **1–2° final error** — the Coulomb deadband, a measured property. Docking
  magnets tolerate several degrees, which is why they beat a mechanical latch.
- **Wheel at 20–40 rad/s immediately after a large slew**, unwinding over a few
  seconds.
- **~0.8°/min heading drift** — gyro-only integration. Vision fixes this.

---

## 14. Calibration history

Eight runs, ~150 logged trials. Several failed at their stated purpose and
produced the most valuable findings anyway.

### Run 1 — 07-29 (9 tests) — first sweep

Velocity-only logging. Found: ~320 Hz log rate; peak wheel velocity scaling
near-linear at 7.7–8.3 rad/s per volt; gyro bias +0.42 dps; τ declining
monotonically with voltage (a Coulomb signature); wheel and gyro **positively**
correlated → opposite sign conventions.

**Structural limitation found:** velocity-only data cannot separate `R`, `K_t`,
`K_v` and friction — at steady state `U_q = ω·(R·b/K_t + K_v)`, one number from
four unknowns.

### Run 2 — 07-30 (48 tests) — the main dataset

16 conditions × 3 repeats, with INA219 current/voltage and wheel angle added.
Firmware changes that made it work: phase A gated on settle, repeats as the outer
loop, sign-interleaved condition order.

48/48 settled, 48/48 clean phase A, no buffer overruns. **`R` and `K_v` separate**
via power balance (`Iq ≈ V_bus·I_bus/U_q`, since `U_d ≈ 0` in voltage mode) →
`R = 0.788 Ω`, `K_v = 0.113`, RMS residual 4.9%. Also: ±4 V direction asymmetry
real at 7σ, noise at ≤2.5 V; single-slope residuals up to ±9.8%, not explained by
bus sag.

### Run 3 — 07-30 (6 tests) — net vs total rotation

Investigated the platform visibly spinning ~360° while firmware printed ~220°.
**Resolution: net ≠ total.** The firmware integrates *signed* rate over the whole
capture; in a reversal the swings partially cancel. Test 3: phase A +83.6°, phase
B −302.6°, net −219.0°, total path 386.2°.

### Run 4 — 07-30 (staircase) — failed at its purpose

Intended to find platform breakaway by climbing voltage. Two failures: a buffer
overrun (needed ~6,700 samples against a 2,400 cap), and — more fundamentally —
**the test cannot measure what it was designed for.** Reaction torque is
proportional to wheel *acceleration*; at constant voltage the wheel plateaus, so
each tread delivers a torque impulse set by the **step size** (constant 0.05 V),
not by absolute voltage. Confirmed: platform peak rate flat at 1.0–2.2 dps from
0.35 to 0.80 V while wheel speed climbed 1.7 → 6.4 rad/s.

**What it found anyway — the motor deadband.** The wheel does not turn below
~0.35 V, then jumps to 1.72 rad/s. Below that the controller has no authority at
all — a limit-cycle risk for any integral term.

### Run 5 — 07-30 (46 breakaway trials)

Sweeps **step size** from rest — the quantity that actually scales platform
torque. **Result: platform breakaway ≈ 0.55 V** → `0.19 × 40.56 × 0.55 = 4.24
rad/s²`.

**Use net displacement, not peak rate.** The firmware's threshold detector
reported 0.40 V using peak rate, which is misleading: ball transfer units have
compliance, so a torque impulse can deflect the platform elastically and let it
spring back. Net displacement is flat below 0.50 V then jumps, and the net/peak
ratio jumps 5× at the same point — two independent signatures agreeing.

*Caveat: only two repeats, with rep 1 systematically higher than rep 2, cause
unestablished. Thin for a stochastic quantity like stiction.*

### Run 6 — 08-02 `150258` (6 O-tests) — re-identification

Triggered by the first closed-loop run diverging. `K' = 8.51 ± 0.19` against a
modelled 7.3 (**+16%**); `τ' = 0.187` against 0.18. → `A_1 = 45.5`, `A_2 = 5.35`.
Also confirmed `GYRO_SIGN` correct in all six.

### Run 7 — 08-02 `151508` (7 C-tests) — `compFrac` sweep

Straight lines in both directions, neutral at 0.972 (+) and 0.892 (−). See §7.

### Run 8 — 08-02 `165048` / `170424` — closed-loop envelope

15 steps from ±30° to ±180°. Results in §2. One intermittent failure traced to
`ALPHA_STALL_MAX`, see §16.

---

## 15. Data formats and analysis pipeline

```
capture_calibration.py  →  calibration_run_<timestamp>/       (raw)
filter_calibration.py   →  <run>/filtered/                    (corrected + derived)
plot_calibration.py     →  <run>/filtered/plots/*.png
make_replay.py          →  <run>/filtered/replay.html
```

`py -m pip install numpy pandas matplotlib pyserial` (on Windows `pip` alone often
is not on PATH).

### Heading controller capture

```
t_us,target_deg,theta_deg,omega_p,omega_w,alpha,u
```

`omega_p`, `omega_w` in rad/s; `alpha` in rad/s² commanded; `u` in volts. Logged
at the full 200 Hz control rate. **Not the same as the HOLD telemetry stream**,
which is the same six fields at 10 Hz and is far too coarse to fit a 0.19 s time
constant against.

### Raw calibration CSV

```
# test 7/48: step 0 -> +4.0V [rep 1/3]
# mode=... phaseB_start_sample=480 phaseA_clean=yes gyro_bias_dps=0.4611 stop_reason=platform_settled
t_us,targetV,wheel_vel,wheel_angle_rad,gyroZ_dps,current_mA,busV
```

Metadata is free-form `key=value`. **Parsers must not pattern-match the whole
line** — see §16. `t_us` is `micros()`, not zeroed. `gyroZ_dps` is raw, bias not
removed. `wheel_angle_rad` is **sign inverted vs velocity**. `current_mA` carries
a ~−6 mA zero offset.

### Filtered CSV

Adds `t_s`, `phase` (A pre-step / B transient), `wheel_accel` (sets reaction
torque), `gyro_dps` (bias removed, sign flipped), `platform_deg`, `power_mW`,
`iq_est_A` (NaN where |targetV| < 0.15).

`filter_calibration.py` applies three corrections: gyro bias from at-rest phase-A
windows of `from=0.00V` tests only; gyro sign flip; wheel angle sign auto-detected
per test by regressing `d(angle)/dt` against `wheel_vel`.

`make_replay.py` builds a standalone HTML replay. **Momentum bars are relative
within each body, not a conserved total** — `J_w` and `J_p` are not known
absolutely.

---

## 16. Bugs and gotchas

Everything that cost real time.

### Sign conventions (three separate ones)

1. **Gyro vs encoder.** Raw data has wheel and platform *positively* correlated.
   Mounting artifact, not physics. `GYRO_SIGN = −1`.
2. **`sensor.getAngle()` vs `motor.shaft_velocity`.** The encoder reports raw
   direction; `shaft_velocity` has the `initFOC()` alignment correction applied.
   Measured slope: **−0.98 across every test.** Logging `motor.shaft_angle`
   instead would fix it at the source.
3. **The LQR rate term and the two feedforward branches** — §8, §6.

### Not zeroing heading between tests

`theta` carries over, so a `T-60` immediately after a `T60` is a **120° slew**.
This silently corrupted an entire envelope batch and looked like a direction
asymmetry. Always `Z` first.

### The authority clamp was sized against the wrong `a`

`ALPHA_STALL_MAX` failed twice for the same reason — both values were computed
using `a = 0.19` when the effective value is nearer 0.15.

| value | result |
|---|---|
| 28 | `a·28 = 4.2` vs a 4.24 breakaway. Whether the platform moved was a **coin flip on stiction**. It sat at `alpha = 28.0` for 1.8 s, never moved, wound the wheel to 44.5, hit the abort |
| 40 | Slews landed, but terminal corrections needed `28 + 0.84·ω_w` commanded — arriving above ~17 rad/s stalled 2–9° short |
| **55** | Covers arrival up to ~32 rad/s. Works |

Any constant derived from `a` deserves the same scrutiny.

### Braking through the linearization

§10. A commanded −10 rad/s² was delivered as −27, above breakaway. The
compensation shortfall adds to a negative α.

### `A1`/`A2` collide with Arduino macros

`A0`–`A15` are predefined analog pin macros on STM32duino. Use `A_1`/`A_2`. Same
reason the mode enum is `CTRL_*`.

### Device Guard blocks `arm-none-eabi-gcc-ar.exe`

WDAC blocks specific binaries by reputation. Fix: substitute plain
`arm-none-eabi-ar` via `use_plain_ar.py`, registered **without** the `pre:`
prefix:

```ini
extra_scripts = use_plain_ar.py     ; correct
extra_scripts = pre:use_plain_ar.py ; runs too early, platform overwrites AR
```

### Serial prompts vs abort-on-any-byte

A typed character's trailing Enter arrives a few ms *after* the character. A
prompt that drains once and returns immediately misses it — the byte lands moments
later and `checkAbort()` reads it as an abort. Fix: `waitForKeypress()` drains
until the link has been quiet for 400 ms.

### Metadata regex broke capture silently

`capture_calibration.py` originally matched the metadata line exactly. When
repeats and phase flags were added the match failed, the state machine stalled,
and **a full sweep was lost** — no files, no error. Fix: treat metadata as opaque,
detect the CSV header (`t_us,`) instead, add a stall guard.

### Buffer sizing

The staircase needed ~6,700 samples against a 2,400 cap. `MODE_BREAKAWAY` dumps
after each trial so only one trial (~580 samples) is ever in memory — the
structural fix, not just a bigger number.

### INA219 reading `inf`

`begin()` returning true only confirms an I2C ack, not that calibration registers
are set. Call `setCalibration_32V_2A()` explicitly.

### I2C speed dominates the loop

At the default 100 kHz each logged sample cost ~3.7 ms against a ~27 kHz bare FOC
loop. `Wire.setClock(400000)` in all current firmware.

### TIM2/TIM3 are taken — matters for the RTOS merge

Motor PWM uses TIM2_CH1 (PA5) and TIM3_CH1/CH2 (PA6/PA7). The proven RTOS
skeleton uses `HardwareTimer(TIM2)` for its 1 kHz tick — `setOverflow()` rewrites
TIM2's ARR, the same register that sets the PWM period. **Use TIM4, TIM5, or
TIM9 for the control-loop timer.**

**Resolved:** TIM5 is the µs timebase, **TIM9 is the FOC/control tick** (4 kHz). TIM9 was
chosen over TIM4 deliberately — TIM4_CH1 is PB6, which is physically the DRV8313 enable, so
a stray channel-enable there would toggle the driver at the tick rate. Two further traps
found in the process: TIM9's vector is `TIM1_BRK_TIM9_IRQn` (not `TIM9_IRQn`), and the
STM32duino core **strongly defines every `TIMx_IRQHandler`**, so you cannot write your own —
use the `HardwareTimer` API + `attachInterrupt`, then override the NVIC priority.

### Angle wrapping in analysis

A `T180` that lands at −178.24° is 1.76° from target, not 358°. Wrap before
computing error or a good result reads as a catastrophic one.

---

## 17. Open items

### Near-term

0. **Friction magnitude sweep — the one real tuning gap.** `A_FRICTION` was
   raised from 22.3 to 28.3 by calculation, never swept on hardware. Until it is,
   small-error corrections clear breakaway by a margin comparable to stiction's
   own variability, and the terminal approach stays a coin flip. The test is
   repeated 5° corrections from rest, not large slews. **Deliberately deferred to
   the combined retune** (see below) rather than done now.
1. ~~**RTOS merge.**~~ **DONE (2026-08-08).** Five tasks on FreeRTOS, TIM9 at 4 kHz driving
   both FOC and (÷20) control. See §12 and `RTOS_migration.md`. Deadlines proven by
   response-time analysis; 27% CPU idle; 12.8 KB of stack reclaimed by high-water resizing.
2. **Vision.** AprilTag on the Pi, UART protocol, complementary filter, then
   Kalman. This is what removes the 0.8°/min drift and makes heading *accurate*
   rather than merely precise.
3. **`motor.shaft_angle`** instead of `sensor.getAngle()` — fixes the angle sign
   at the source.
4. **Persist tuned gains** after each session; serial-set values do not survive a
   power cycle.

### Known unknowns

- **`a` absolute** — 0.19 nominal, evidence says 0.15–0.17. Deliberately not
  chased (§3); `A_FRICTION` is tuned empirically instead. Any future constant
  derived from `a` needs checking — two have already failed (§16).
- **`J_p` absolute** — only the ratio is known. Would need a geometric or
  pendulum measurement, *not* momentum conservation, since friction corrupts that
  ledger.
- **Direction asymmetry** — a few percent, sign not stable across sessions.
  Likely thermal. Covered by margin rather than modelled.
- **Low-voltage nonlinearity** — single-slope residuals ±9.8%; the two-parameter
  R/K_v fit is 4.9% RMS but worst at 1.0 V (+10%). Coulomb friction is the likely
  cause.
- **Breakaway repeatability** — only 2 repeats, rep 1 systematically higher.

### Deferred to the combined retune

Adding the fan subsystem changes platform mass, inertia, and friction, so `a`,
`A_FRICTION`, `A_1`, `A_2`, and every gain derived from them must be
re-identified regardless. Tuning the rotation axis to perfection first would be
work thrown away. The plan is therefore: finish the RTOS merge, add vision, build
the translation hardware, then run **one** identification and tuning campaign
across x, y, and θ together.

Carried into that campaign:

| item | what to do |
|---|---|
| `a` absolute | Measure geometrically (`½mR²`) once the final chassis exists. It sets the gain formulas and `A_FRICTION`, and every constant sized against the wrong value has failed at least once (§16) |
| `A_FRICTION` | Sweep on hardware with repeated small corrections, per §13 step 9 |
| Coarse/fine deadzone | Re-derive the floor from the new `ffFrac × A_FRICTION` |
| `compFrac` | Re-run the `C` sweep; the wheel is unchanged but the platform inertia is not |
| Gains | Re-run the descent through the table with the new `a` |
| `ω_w` in the LQR cost | Momentum management currently falls out of a compensation margin rather than the design. With three axes to weight anyway, this is the moment to do it properly |

**Do not carry forward:** the assumption that `a` = 0.19, or any constant derived
from it.

### Deliberately not done

- **MPC.** Simulated slews use 18% of voltage with zero saturated timesteps. With
  inactive constraints MPC converges to exactly the LQR solution, so it would cost
  significant complexity for identical behaviour. It becomes worthwhile for
  terminal docking (terminal state constraints), momentum limits as hard
  constraints, and the unidirectional fan allocation problem. Best placed on the
  Pi at 10–50 Hz feeding setpoints to this loop, replacing the trajectory
  generator rather than the controller.
- **Active desaturation ramp.** Passive unwind covers the operating range and is
  self-limiting; the active version is easy to get dangerously wrong (§10).

---

## 18. Translation subsystem

**Hardware built and tested (2026-08-08). Control not started — see
`TRANSLATION_DOCKING.md`, the active build guide.**

### What exists and is proven

| item | state |
|---|---|
| 4× FEICHAO 2204 2300KV + HQProp 4043 3-blade | mounted, all four channels spinning |
| AERO SELFIE 45A 4-in-1 ESC | working — **Bluejay firmware, DSHOT only** |
| Raspberry Pi 3B+ (1 GB) + camera | mounted and wired; link not yet run |
| DSHOT300 drive | proven in `fan_test.cpp` (standalone, env `fantest`) |

**Measured:** unloaded commutation floor **~2%** (1% pulses = desync/retry, 2%
runs). With props, **~50% throttle gives usable translation velocity**. A quick
translation test produced a curved path, attributed to table dust rather than tilt.

**The single most expensive lesson: the ESC runs Bluejay, which removed analog
input entirely.** Servo PWM produces nothing regardless of accuracy — TIM1 at
50.000 Hz with 999.8 µs pulses and MOE set, verified with a meter at the ESC's own
pad, and the ESC never armed. Throttle-range calibration also produces no beeps,
because analog range calibration does not exist in that firmware. DSHOT works on
both Bluejay and BLHeli_S, so it is the correct target regardless.

**Second lesson: a backwards prop costs about half its thrust.** Needing 75%
throttle became a comfortable 50% purely by flipping one prop. Check orientation
and handedness before concluding you need larger props.

**Third: all four DSHOT channels must share one GPIO port** (PA8/PA9/PA10/PA0).
Putting channel 4 on PC7 forced either a port parameter or a duplicated send
function, and every such variant broke the previously-working channels.

### Still to identify — nothing below is measured yet

`TRANSLATION_DOCKING.md` Phase 2 covers all of it: static thrust curve per motor,
translational breakaway force (the five-minute string-and-weights test), effective
mass, and the yaw disturbance from thrust-line/CoM offset. **Rotation constants get
re-identified there too**, since the added mass moved the plant — see §17.

### None of the rotation constants transfer

### None of the rotation constants transfer

Translation is a different plant needing its own campaign: fan thrust vs PWM
curves, platform mass, translational breakaway force, table-tilt disturbance.

**What transfers is the method:** measure the deadband before tuning, check
controllability before designing, test Coulomb vs viscous rather than assuming,
verify signs open-loop first, and re-identify when the closed loop disagrees with
the model.

### Architecture

Because translation is architecturally decoupled from rotation, the clean approach
is a rotation transform (world → body frame using `θ`) then **three independent
double integrators** — x, y, θ — each with its own PD. Three tuning problems, not
one 18-dimensional one.

### Differences from rotation

- **Fans are unidirectional.** Allocation is constrained (thrust ≥ 0). Opposing
  pairs need idle bias for bidirectional authority, which costs power continuously
  — but also keeps motors above the sensorless commutation floor.
- **No actuator feedback.** ESCs are open-loop from the MCU: no encoder, no RPM,
  no thrust. The only feedback is platform position from vision. Contrast the
  wheel, where the MT6701 made feedback linearization possible — **none of the
  §5/§7 machinery has an analogue here.**
- **Square-law nonlinearity.** Thrust ∝ RPM², RPM ≈ linear in throttle, so
  `throttle = sqrt(F_desired/F_max)`. The fan analogue of feedback linearization,
  but static.
- **Table tilt hits translation far harder than rotation.** The 0.3° figure from
  the project plan is a translational disturbance. Level the table first.
- **Prop reaction torque acts about a horizontal axis** (props spin in vertical
  planes), so it is a tipping moment, not yaw. Thrust-line offset from the CoM
  *does* produce yaw, which the reaction wheel must reject.

### Inferred translational friction

Both frictions share the same μ: `F_c = T_c/r_b = J_p·α_break/r_b`.

| mass | friction |
|---|---|
| 1.2 kg | 30–39 gf |
| 1.5 kg | 38–48 gf |
| 2.0 kg | 50–64 gf |

Plus 6–20 gf for acceleration → **roughly 50–100 gf from one fan** on a cardinal
push (only one fan works for N/S/E/W; a diagonal splits across two).

*Caveat: rotation makes each ball roll along a circular arc, so the contact also
spins about the vertical axis. Pure translation is straight rolling with no scrub,
so these numbers likely overstate the translational case.*

**Verify directly:** tie a string to the platform, run it over the table edge, add
weight until it moves. Five minutes, replaces the widest uncertainty in the
sizing.

### Parts selected — all now FITTED

| item | choice | note |
|---|---|---|
| motors | FEICHAO 2204 2300KV ×4 | ~$30; 420 gf claim (likely at 4S; we run 3S) |
| ESC | AERO SELFIE 45A 4-in-1, 2–6S | **Bluejay — DSHOT only.** Channel matching matters because fan control is open-loop |
| props | **HQProp 4043, 4-inch 3-blade** | fitted and working at ~50% throttle. 5-inch would give ~2.4× thrust at the same RPM but needs 52 mm standoff |
| power | 3S ~12 V 2300 mAh, 10 A fuse, 18 AWG | ⚠️ sized for LOW throttle — prop current goes as throttle³ (~1.5 A/motor at 50%, ~5 A at 75%). **15 A is the ceiling for 18 AWG** |

Prop geometry, 8-inch disc (102 mm radius):

| prop | half-span | max mount radius | standoff above disc |
|---|---|---|---|
| 5 in | 63.5 mm | 38 mm | 52 mm |
| 4 in | 50.8 mm | 51 mm | 39 mm |
| 3 in | 38.1 mm | 64 mm | 26 mm |

Also needed: a dedicated **5 V / 5 A** buck for the Pi — ESC BECs (~2 A) and
typical LM2596 modules cannot supply a Pi 5 under OpenCV load.

---

## Appendix — quick reference

```
A_1        = 45.5  rad/s²/V      wheel accel per volt
A_2        = 5.35  1/s           wheel damping pole
a          = 0.19                J_w/J_p nominal (true value likely 0.15-0.17)
A_FRICTION = 28.3  rad/s²        Coulomb feedforward magnitude (tune with A)
compFrac   = 0.89                fraction of A_2 applied; MUST be < neutral
GYRO_SIGN  = −1

feedback linearisation:  U_q = (α + 0.89·5.35·ω_w) / 45.5
control law:             α = −K_θ·e + K_ω·ω_p        (minus, PLUS)
feedforward moving:      α += −ff·A_F·sign(ω_p)      (negative)
feedforward stuck:       α += +ff·A_F·sign(α)        (positive)
gains:                   K_θ = ω_n²/a,  K_ω = 1.4·ω_n/a,  ω_n = 5.714/t_settle

delivered α  = commanded − 0.84·ω_w      ← why terminal corrections need a slow wheel
ω_w,steady   ≈ α / 0.84                  ← why a sustained α stops producing torque
passive unwind: ω̇_w = −0.84·ω_w, self-limiting below ω_w = 26.5

break-free from rest needs   ff·A_FRICTION + K_θ·|e|  >  ~28
   -> at small |e| the feedforward magnitude is doing ALL the work,
      which is why the last few degrees are the unreliable part
deadband floor = (1−ff)·A_FRICTION/K_θ   -> deadzones must exceed it
```