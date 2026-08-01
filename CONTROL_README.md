# Reaction Wheel Attitude Control — Complete Reference

Everything about the rotation control subsystem: system identification, model
derivation, structural findings, control design, firmware, analysis tooling,
and the full history of what was measured and what went wrong.

**Status as of this document:** the plant is fully identified, the controller
is written and flashed, and the first closed-loop test has not yet been run.
Translation (fans) is not started.

---

## Table of contents

1. [Scope and status](#1-scope-and-status)
2. [The plant](#2-the-plant)
3. [Symbol reference](#3-symbol-reference)
4. [Identified constants](#4-identified-constants)
5. [Model derivation](#5-model-derivation)
6. [Discovery 1 — the system is not fully controllable](#6-discovery-1--the-system-is-not-fully-controllable)
7. [Discovery 2 — friction exceeds the maneuver](#7-discovery-2--friction-exceeds-the-maneuver)
8. [Friction in depth](#8-friction-in-depth)
9. [The control law](#9-the-control-law)
10. [LQR](#10-lqr)
11. [The estimator](#11-the-estimator)
12. [Calibration campaign](#12-calibration-campaign)
13. [Data formats](#13-data-formats)
14. [Firmware](#14-firmware)
15. [Analysis pipeline](#15-analysis-pipeline)
16. [Tuning procedure](#16-tuning-procedure)
17. [Bugs and gotchas](#17-bugs-and-gotchas)
18. [Open items](#18-open-items)
19. [Translation subsystem — future](#19-translation-subsystem--future)

---

## 1. Scope and status

### Done

- Reaction wheel driven in torque/voltage mode via SimpleFOC + MT6701 encoder
- Five calibration campaigns, ~110 logged trials
- Full plant identification: motor electrical, wheel dynamics, momentum
  coupling, friction character and magnitude, sensor biases and sign conventions
- Two structural properties discovered that invalidate the naive control approach
- Control law designed: feedback linearization + LQR/PD + Coulomb feedforward
- Firmware written with runtime-tunable gains and an open-loop sign-check mode
- Analysis pipeline: capture → filter → plot → animated replay

### Not done

- First closed-loop run (controller flashed, untested)
- Gain tuning on hardware
- Vision (AprilTag) and Kalman filter — currently gyro-only heading
- RTOS integration (still a super-loop)
- Momentum desaturation routine
- Everything translation-related

---

## 2. The plant

A platform rotates freely on three ball transfer units. Bolted to it is a BLDC
gimbal motor whose rotor carries a flywheel. There is no external actuator.

The platform turns itself by trading angular momentum with the wheel: when the
motor pushes the wheel one way, Newton's third law pushes the platform the
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
| HC-05 | wireless telemetry and command |

The driver is a bare gate driver, so the Nucleo runs the FOC loop itself
(SimpleFOC library, STM32duino core under PlatformIO).

---

## 3. Symbol reference

### State

| symbol | meaning | units | source |
|---|---|---|---|
| `θ_p` | platform heading | rad | integrated gyro |
| `ω_p` | platform angular rate | rad/s | MPU6050 gyro Z |
| `ω_w` | wheel angular rate | rad/s | `motor.shaft_velocity` |
| `θ_w` | wheel angle | rad | `sensor.getAngle()` |

### Control

| symbol | meaning | units |
|---|---|---|
| `α_cmd` | commanded wheel angular acceleration | rad/s² |
| `U_q` | q-axis voltage — what gets written to `motor.target` | V |
| `e` | heading error, `θ_target − θ_p`, wrapped to ±π | rad |

### Motor electrical

| symbol | meaning | units |
|---|---|---|
| `I_q` | q-axis current (torque-producing) | A |
| `R` | winding resistance | Ω |
| `K_v` | back-EMF constant | V/(rad/s) |
| `K_t` | torque constant (= `K_v` numerically in SI) | N·m/A |
| `τ` | motor torque | N·m |

**On the "q-axis":** FOC transforms the three phase currents into two
components — `d` (radial, produces no torque) and `q` (tangential, produces all
of it). SimpleFOC drives `d` to zero and modulates `q`. So `U_q` is the useful
voltage and `I_q` the useful current.

### Plant parameters

| symbol | meaning | units |
|---|---|---|
| `J_w`, `J_p` | wheel and platform moments of inertia | kg·m² |
| `a` | `J_w/J_p`, momentum coupling ratio | — |
| `A_1` | wheel angular accel per volt from rest | (rad/s²)/V |
| `A_2` | wheel damping pole, `1/τ_w` | 1/s |
| `τ_w` | wheel time constant | s |
| `T_c` | Coulomb friction torque | N·m |

### Controller

| symbol | meaning |
|---|---|
| `K_θ` | heading error gain |
| `K_ω` | rate gain |
| `ω_n` | closed-loop natural frequency, rad/s |
| `ζ` | damping ratio (0.7 throughout) |

---

## 4. Identified constants

Everything the controller needs, and where each number came from.

| constant | value | units | provenance |
|---|---|---|---|
| `R` | 0.788 | Ω | least-squares over 24 from-rest tests, using INA219-derived `I_q` |
| `K_v` | 0.113 | V/(rad/s) | same regression |
| `K` (DC gain) | 7.3 | (rad/s)/V | steady-state wheel speed vs command |
| `τ_w` | 0.18 | s | 63% rise time of wheel step response |
| **`A_1`** | **40.56** | (rad/s²)/V | `K/τ_w` — used by firmware |
| **`A_2`** | **5.56** | 1/s | `1/τ_w` — used by firmware |
| **`a = J_w/J_p`** | **0.19** | — | regression of platform rate vs wheel-speed change, 48 tests, 20% scatter |
| platform Coulomb friction | 4.24 | rad/s² | breakaway sweep, 0.55 V step |
| **`A_FRICTION`** | **22.3** | rad/s² | `4.24/0.19` — wheel-accel equivalent, used by firmware |
| motor deadband | 0.35 | V | staircase run; below this the wheel does not sustain rotation |
| gyro bias | 0.42–0.46 | dps | at-rest phase-A windows; re-measured every boot |
| **`GYRO_SIGN`** | **−1** | — | gyro and encoder use opposite conventions |
| max authority | 77 | rad/s² | `a·A_1·10 V` |

### Why `A_1` and `A_2` rather than `J_w`

`A_1` and `A_2` are measured end-to-end from input to output, so they are
immune to convention questions — SimpleFOC's voltage scaling, phase-vs-line
resistance, modulation factors. Trying to back out an absolute `J_w` gives
~0.0035 kg·m², far larger than an HDD platter should be, which signals an
unaccounted convention factor somewhere.

**This does not matter.** Nothing in the controller uses `J_w` alone. Only the
ratio `a` and the end-to-end gains appear.

### `R` and `K_v` are not used by the controller

They were expensive to obtain (the whole INA219 detour) and the firmware never
touches them. They are physically interesting, needed for absolute torque
units, and would be required for any model-based work in SI — but the control
law runs entirely on `A_1`, `A_2`, `A_FRICTION`, and `GYRO_SIGN`.

---

## 5. Model derivation

### Step 1 — voltage to current

A motor is a resistor and a generator in one package:

```
I_q = (U_q − K_v·ω_w) / R
```

Inductance is neglected: the electrical time constant `L/R` is microseconds,
against a 180 ms mechanical time constant. Current is effectively instantaneous.

**The intuition is in the back-EMF term.** At standstill, 1 V pushes
`1/0.788 = 1.27 A`. As the wheel speeds up it generates `0.113·ω_w` volts
against you. At `ω_w = 8.85 rad/s` back-EMF equals your 1 V, current hits zero,
torque hits zero. That is why wheel speed plateaus — and the measured 7.3
rad/s per volt sits slightly below 8.85 because friction takes a cut.

### Step 2 — current to torque

```
τ = K_t · I_q
```

### Step 3 — torque splits between two bodies

```
J_w·ω̇_w = τ                          (wheel)
J_p·ω̇_p = −τ − T_friction            (platform)
```

The minus sign is the entire mechanism.

### Step 4 — combined wheel dynamics

```
ω̇_w = A_1·U_q − A_2·ω_w = 40.56·U_q − 5.56·ω_w
```

### Step 5 — full state space

With `x = [θ_p, ω_p, ω_w]` and input `U_q`, friction dropped:

```
      ⎡0   1      0   ⎤        ⎡  0  ⎤
A  =  ⎢0  −c   1.056  ⎥   B =  ⎢−7.71⎥
      ⎣0   0   −5.56  ⎦        ⎣40.56⎦
```

This formulation is what revealed the first structural problem.

---

## 6. Discovery 1 — the system is not fully controllable

### The evidence

Controllability matrix `[B, AB, A²B]` returns **rank 2 of 3**, determinant
exactly zero — not a numerical artifact. PBH test localizes it to the λ = 0
mode. The left eigenvector identifies the untouchable combination:

```
c·θ_p + ω_p + 0.19·ω_w = constant
```

Verified: `wᵀB = −2.3×10⁻¹⁶`. The input genuinely cannot affect it.

### What it means

**This is angular momentum conservation.** An internal torque can only move
momentum between wheel and platform, never change the total. This is precisely
why real spacecraft carry magnetorquers or thrusters — a reaction wheel cannot
desaturate itself.

**Consequence:** from rest to rest, `c·θ_final = c·θ_initial`. Under pure
viscous friction the platform *cannot hold a new heading with the wheel
stopped*. It can only park at an offset while the wheel keeps spinning — 1 V
holds about 20° with the wheel at 7.3 rad/s.

Adding an integrator does not fix it (rank 3 of 4) and the LQR solve fails
outright with a "no finite solution" error.

### The experiment that showed it first

The voltage staircase measured exactly this before it was derived: platform
peak rate stayed flat at **1.0–2.2 dps from 0.35 V to 0.80 V** while wheel
speed climbed **1.7 → 6.4 rad/s**. Constant voltage produced no sustained
rotation. The theory came after the data.

### The fix — feedback linearization

Platform torque depends on wheel *acceleration*, and `ω_w` is measured. So
invert the wheel equation:

```
U_q = (α_cmd + 5.56·ω_w) / 40.56
```

The `5.56·ω_w` term pre-cancels back-EMF; whatever remains produces exactly the
commanded acceleration. The plant becomes:

```
θ̈_p = −0.19·α_cmd
```

A double integrator — fully controllable, and the best-understood plant in
control theory.

---

## 7. Discovery 2 — friction exceeds the maneuver

Converting everything to platform angular acceleration:

| quantity | value |
|---|---|
| max authority (10 V) | 77 rad/s² |
| **Coulomb friction** | **4.24 rad/s²** |
| 90° slew in 2 s requires | 1.57 rad/s² |

**Friction is nearly three times the maneuver itself.** Not a small correction —
the dominant term.

### The deadband this creates

The controller commands platform acceleration `a·K_θ·e`. Motion only occurs
above 4.24 rad/s², so there is a deadband in heading error:

```
|e| > 4.24 / (0.19·K_θ)
```

| `K_θ` | settle time | deadband |
|---|---|---|
| 3.16 | 7.2 s | 404° |
| 14.1 | 3.5 s | 90° |
| 43.0 | 2.0 s | 30° |
| 100 | 1.3 s | 13° |

**Pure feedback cannot point this platform at any sane gain.** Reaching a 2°
deadband by gain alone would need `K_θ ≈ 640`, wildly unstable.

### Why this matters more than it sounds

Without knowing this, the failure mode during tuning is: raise P, nothing
happens, raise it more, nothing, keep going, hit instability before ever
achieving pointing. That is not resolvable by more tuning. You would have to
independently invent Coulomb feedforward, realize it needs two opposite-signed
branches, and guess its magnitude.

---

## 8. Friction in depth

### Viscous vs Coulomb

- **Viscous:** `τ = b·ω`. Scales with speed. Move slowly, feel almost nothing.
  Like moving through honey.
- **Coulomb:** `τ = T_c·sign(ω)`. Constant magnitude opposing motion,
  independent of speed. Like dragging a box across concrete — the hard part is
  starting.

### Which one this rig has

Fitting both models to all 48 tests:

| model | mean R² |
|---|---|
| viscous | 0.826 |
| **Coulomb** | **0.918** |
| both terms | 0.951 |

**Coulomb wins in 43 of 48 tests.** Physically sensible — rolling resistance in
ball bearings is closer to constant drag than rate-proportional. The viscous
coefficient also came out with **84% scatter** (4.05 ± 3.39), the signature of
fitting the wrong model form.

### Feedforward — the fix

```c
if (fabsf(e) > deadzone) {
    float ff;
    if (fabsf(ω_p) > W_MOVING)
        ff = −A_FRICTION * sign(ω_p);   // MOVING: cancel friction opposing motion
    else
        ff =  A_FRICTION * sign(α);     // STUCK:  push to break free
    α += FF_FRACTION * ff;
}
```

**The two branches have opposite signs.** Derivation: we want the closed loop
to behave as though friction were absent, `θ̈ = −a·α_lqr`. With friction present:

```
−a·(α_lqr + α_ff) − A_f·sign(ω_p) = −a·α_lqr
⟹  α_ff = −(A_f/a)·sign(ω_p)          [MOVING: negative]
```

When stuck there is no velocity to oppose; instead push whichever way the
controller wants, so the sign follows `α`.

Getting the moving branch backwards is **worse than no feedforward at all** —
simulated final error 48° versus 27° with none, versus 0.7° correct.

### Achievable accuracy

| feedforward accuracy | deadband at `K_θ` = 43 |
|---|---|
| none | 29.8° |
| 80% | 6.0° |
| 95% | 1.5° |

This single number dominates your final pointing accuracy. Realistically
80–95% is achievable; Coulomb friction is not perfectly constant (it varies
with contact point, dwell time, temperature), which is why breakaway repeats
scattered.

### Friction is also what makes reorientation possible

The conservation law says that under *pure viscous* friction the platform
returns to its original heading whenever the wheel stops. You could never
permanently reorient.

**Coulomb friction breaks that conservation law.** It lets the platform stick
at a new heading with zero stored momentum. Spin the wheel up (platform
rotates), spin it down (friction absorbs momentum asymmetrically), platform
stays where it got to.

So friction simultaneously limits your precision and enables the machine to
work at all. On a real satellite there is no friction, which is exactly why
they need separate desaturation hardware.

---

## 9. The control law

Runs at 200 Hz (`CONTROL_PERIOD_US = 5000`) in `heading_control.cpp`.

```c
// 1. SENSE
float w_p = GYRO_SIGN * (readGyroZ() − gyroBias) * PI/180;  // rad/s, platform
float w_w = motor.shaft_velocity;                            // rad/s, wheel

// 2. ESTIMATE heading
theta = wrapPi(theta + w_p * dt);

// 3. ERROR
float e = wrapPi(target − theta);

// 4. LQR / PD → desired WHEEL ACCELERATION
float alpha = −K_theta * e + K_omega * w_p;

// 5. COULOMB FEEDFORWARD, gated by deadzone
if (fabsf(e) > deadzone) {
    float ff = (fabsf(w_p) > W_MOVING) ? −A_FRICTION*signf(w_p)
                                       :  A_FRICTION*signf(alpha);
    alpha += ffFrac * ff;
} else {
    alpha = 0.0f;      // inside tolerance: stop, let stiction hold
}

// 6. FEEDBACK LINEARISATION → voltage
float u = (alpha + A_2 * w_w) / A_1;

// 7. SATURATE + COMMAND
motor.target = constrainf(u, −VOLTAGE_LIMIT, VOLTAGE_LIMIT);

// 8. SUPERVISE
if (fabsf(w_w) > WHEEL_SAT_LIMIT) stopMotor();
```

### Sign warning (a) — the LQR line

```
alpha = −K_theta*e + K_omega*w_p
        ^^^^^^^^^^ minus     ^^^^^^^^^ PLUS
```

Derivation:

```
plant:  θ̈ = −a·α                (minus: wheel one way, platform the other)
error:  e = θ_target − θ_p,  ë = −θ̈ = +a·α
want:   ë = −2ζω_n·ė − ω_n²·e,  and  ė = −ω_p
⟹      a·α = 2ζω_n·ω_p − ω_n²·e
⟹      α = −(ω_n²/a)·e + (2ζω_n/a)·ω_p
```

The rate term enters with a **plus**. Writing `−(K_θ·e + K_ω·ω)` is unstable —
in simulation it diverged, ending 34° short of a 90° target with `α` blowing
past 600 rad/s².

### Sign warning (b) — the feedforward branches

Covered in §8. Moving branch negative, stuck branch positive.

### The deadzone is not optional

With feedforward always active, it never switches off once the platform has
stopped and keeps accelerating the wheel:

| deadzone | final error | wheel peak |
|---|---|---|
| none | 0.68° | **72.9 rad/s** |
| 1° | 0.57° | 31.7 rad/s |
| 2° | 1.53° | 29.9 rad/s |
| 3° | 2.51° | 28.9 rad/s |

---

## 10. LQR

### What problem it solves

You want `u = −Kx`, but which `K`? High gains respond fast but saturate
actuators and amplify noise. PID resolves this by hand-tuning. LQR resolves it
by optimization.

### The mechanism

Declare what you care about via a cost:

```
J = ∫ (xᵀQx + uᵀRu) dt
```

`Q` penalizes state error, `R` penalizes effort. The minimizing gain is
`K = R⁻¹BᵀP`, where `P` solves the Algebraic Riccati Equation:

```
AᵀP + PA − PBR⁻¹BᵀP + Q = 0
```

A matrix quadratic, solved offline in one line
(`scipy.linalg.solve_continuous_are`). **Never solved on the STM32** — compute
once, paste two numbers into firmware.

### Weighting sweep on this plant

Plant is `A = [[0,1],[0,0]]`, `B = [[0],[−0.19]]`, state `[θ_err, ω_p]`.

| `q_θ` | `K_θ` | `K_ω` | ζ | settle | peak `U_q` at 90° |
|---|---|---|---|---|---|
| 1 | 1.00 | 3.32 | 0.72 | 12.7 s | 0.04 V |
| 10 | 3.16 | 5.81 | 0.71 | 7.2 s | 0.12 V |
| 50 | 7.07 | 8.66 | 0.71 | 4.9 s | 0.27 V |
| 200 | 14.14 | 12.22 | 0.71 | 3.5 s | 0.55 V |
| 1000 | 31.62 | 18.26 | 0.71 | 2.3 s | 1.22 V |

ζ lands at 0.71 regardless of weighting — LQR's characteristic behavior on a
double integrator.

### Recommended gains via pole placement

More direct than guessing `Q`. For damping ζ = 0.7:

```
ω_n = 5.714 / t_settle
K_θ = ω_n² / 0.19
K_ω = 1.4·ω_n / 0.19
```

| settle | `ω_n` | `K_θ` | `K_ω` | predicted deadband (80% FF) |
|---|---|---|---|---|
| 3.0 s | 1.90 | **19.1** | **14.0** | 13.4° |
| 2.0 s | 2.86 | **43.0** | **21.1** | 6.0° |
| 1.5 s | 3.81 | **76.4** | **28.1** | 3.4° |
| 1.2 s | 4.76 | **119.3** | **35.1** | 2.1° |

**The ceiling is ω_n ≈ 4.8.** The wheel pole sits at `A_2` = 5.56 rad/s; above
that the feedback linearization stops cancelling it cleanly.

### Faster gains are better on *both* axes

Counterintuitive if you are used to creeping up cautiously. Simulated 90° slew
with a 2° deadzone:

| gains | final error | wheel peak |
|---|---|---|
| 3.0 s | 6.30° | 72.9 rad/s |
| 2.0 s | 1.53° | 29.9 rad/s |
| 1.5 s | **0.03°** | **22.4 rad/s** |

Slow gains keep the feedforward active longer, which winds the wheel up more.
So work **down** the table.

### Honest scope

For a 2-state system, `u = −K_θ·e + K_ω·ω` **is a PD controller**. LQR here is
a principled way to choose PD gains, not a different architecture.

Its real value arrives with the translation fans (MIMO, where hand-tuning a
dozen gains is miserable) and when `ω_w` enters the cost function so momentum
management falls out of the optimization automatically.

---

## 11. The estimator

`θ_p` is not measured directly. Two imperfect sources:

| sensor | strength | weakness |
|---|---|---|
| MPU6050 gyro | fast (hundreds of Hz) | measures *rate*; bias integrates into drift |
| AprilTag camera | absolute, drift-free | slow (~30 Hz), latent, drops out |

They fail in complementary ways, which is when fusion helps.

### Gyro bias

Measured at every boot from a 200-sample at-rest average. Historically
0.42–0.46 dps, stable to ±0.014 dps across a whole sweep. Uncorrected that is
**25° per minute of pure fiction**. After removal, drift is about **0.8°/min** —
fine for 30-second tests before vision exists.

### Complementary filter (start here)

```c
theta = 0.98f*(theta + w_p*dt) + 0.02f*theta_vision;
```

Trust the gyro short-term, let vision slowly pull out drift.

### Kalman filter (upgrade)

State `[θ_p, ω_p, b_gyro]`. Predict with the gyro, correct with vision.
Estimating bias **as a state** makes it self-calibrating against thermal drift
rather than relying on a startup measurement.

### The wheel encoder cannot give you heading

In principle momentum conservation gives `θ_p = −0.19·θ_w`, and on a
frictionless spacecraft that would work. **It fails here because friction is an
external torque and it is large.** With friction at 4.24 rad/s² against
maneuvers of ~1.57 rad/s², the conservation assumption is broken worse than it
is satisfied. An encoder-derived heading would be badly wrong within one slew.

What the encoder *is* for: `ω_w` for feedback linearization (mandatory),
saturation monitoring, and improving the Kalman prediction step since commanded
torque is known.

---

## 12. Calibration campaign

Five runs. Two of them failed at their stated purpose and produced the most
valuable findings anyway.

### Run 1 — 2026-07-29 11:29:29 (9 tests)

First sweep. Velocity-only logging: `t_us, targetV, vel_raw, vel_filtered, gyroZ_dps`.

**Findings:**
- ~320 Hz realized log rate, jitter under 100 µs
- All 9 tests ended on `platform_settled`
- Peak wheel velocity scaled near-linearly: 7.76 / 8.32 / 8.30 / 7.66 rad/s per
  volt across 1.0–4.0 V
- Gyro bias +0.42 dps, spread 0.008 dps across five at-rest tests
- τ declined monotonically with voltage: 196.7 → 190.6 → 184.2 → 177.9 ms
  (Coulomb friction signature)
- `vel_raw` and `vel_filtered` are near-perfect mirrors (correlation −0.97 to
  −0.997) — SimpleFOC applies the `initFOC()` direction correction only to
  `shaft_velocity`
- Wheel and gyro **positively** correlated (+0.33 to +0.84) → opposite sign
  conventions

**Structural limitation found:** velocity-only data cannot separate `R`, `K_t`,
`K_v`, and friction. At steady state `U_q = ω·(R·b/K_t + K_v)` — one number
from four unknowns.

### Run 2 — 2026-07-30 11:24:00 (48 tests) — the main dataset

16 conditions × 3 repeats. Added INA219 current/voltage and measured wheel angle.

**Firmware changes that made it work:**
- Phase A gated on settle (min hold AND wheel steady AND platform settled)
- Repeats as the *outer* loop so drift spreads across conditions
- Sign-interleaved condition order
- Split log rate: phase A ~168 Hz, phase B ~224 Hz

**Results:**
- 48/48 `platform_settled`, 48/48 `phaseA_clean=yes` — the settle gating
  completely solved the contaminated-initial-condition problem
- No buffer overruns (peak 894 of 2400 samples)
- INA219 range 0–722 mA, bus 12.03 → 11.32 V under load; idle −6 mA confirms
  it is on the motor supply, not total system draw
- Repeatability: 0.16–4.6% CV on steady-state wheel velocity

**The headline result — `R` and `K_v` separate:**

```
Iq ≈ V_bus·I_bus / U_q          (power balance, since Ud ≈ 0 in voltage mode)
U_q = R·Iq + K_v·ω              (one equation, two unknowns, per test)
⟹  R = 0.788 Ω, K_v = 0.113 V/(rad/s), RMS residual 4.9%
```

**Also found:**
- ±4 V asymmetry is real: negative direction 7.2% faster, **7σ separation**.
  At 1.0–2.5 V it is only 0.7–1.8σ (noise). So asymmetry grows with command.
- A single slope through the origin has residuals up to ±9.8%. Bus sag
  correction only improves it to 8.8%, so sag is not the explanation.
- τ noisier this run (156–215 ms); the clean monotonic decline did not reproduce

### Run 3 — 2026-07-30 11:34:50 (6 tests)

3.0 V → −2.0 V reversals. Investigated a discrepancy: the platform visibly spun
near 360° but firmware printed ~220°.

**Resolution: net ≠ total.** The firmware integrates *signed* rate over the
whole capture including phase A. In a reversal the platform swings one way
during spin-up, then back during reversal, and those partially cancel.

For test03: phase A **+83.6°**, phase B **−302.6°**, net **−219.0°**, total path
386.2°. Net/total ratio 0.32–0.57 across the six tests — bracketing the "about
2/3" observed.

**Ruled out first:** gyro saturation (peaks 144–350 dps against a ±500 dps
range) and undersampling (168/224 Hz against a 21 Hz DLPF).

### Run 4 — 2026-07-30 12:20:30 (staircase) — failed at its purpose

0.05 V treads up to 1.20 V, 800 ms dwell, intended to find platform breakaway.

**Two failures:**

1. **Buffer overrun.** 50 treads × 800 ms at 168 Hz needs ~6,700 samples against
   a 2,400 cap. Stopped at 0.85 V, never descended.

2. **The test cannot measure what it was designed for.** Reaction torque is
   proportional to wheel *acceleration*. At constant voltage the wheel
   accelerates briefly then plateaus, so each tread delivers a torque impulse
   set by the **step size** (constant 0.05 V), not by absolute voltage.
   Climbing the staircase applies the identical impulse at every rung.

   Confirmed directly: platform peak rate flat at **1.0–2.2 dps** from 0.35 V to
   0.80 V while wheel speed climbed **1.7 → 6.4 rad/s**.

**What it found anyway — the motor deadband.** The wheel does not turn below
~0.35 V (0.001–0.056 rad/s, i.e. noise), then jumps to 1.72 rad/s. Below that
command the controller has **no authority at all** — a genuine limit-cycle risk
for any integral term.

Also: low-voltage slope 10.25 rad/s/V vs 7.3 at high voltage, with a −1.71
offset — more Coulomb evidence.

### Run 5 — 2026-07-30 19:26:35 (46 of 48 breakaway trials)

Sweeps **step size** from rest — the quantity that actually scales platform
torque. Each trial: settle at 0 V, step to S, fixed 1500 ms observation window,
return, dump.

**Result: platform breakaway ≈ 0.55 V.**

| step | net displacement | net/peak ratio |
|---|---|---|
| ≤ 0.50 V | 0.005–0.15° | ~0.05 |
| **0.55 V** | **1.53°** | **0.26** |
| 0.60 V | 2.46° | 0.22 |
| 1.20 V | 22.6° | 0.56 |

**Use net displacement, not peak rate.** The firmware's threshold detector
reported 0.40 V using peak rate, which is misleading: ball transfer units have
compliance, so a torque impulse can deflect the platform elastically and let it
spring back. Peak rate sees that wiggle as motion. Net displacement is flat
below 0.50 V then jumps — and the net/peak ratio jumps 5× at the same point.
Two independent signatures agreeing.

`4.24 rad/s² = 0.19 × 40.56 × 0.55`

**Caveats:** two trials missing (0.35 V and 0.75 V, both rep 2). Rep 1 ran
systematically higher than rep 2 across most of the range — cause not
established. Cable twist was ruled out (the rig is fully wireless); most likely
motor thermal drift or the platform ending rep 1 at a different position on a
not-perfectly-level surface. Two repeats is thin for a stochastic quantity like
stiction.

---

## 13. Data formats

### Raw calibration CSV (current firmware)

```
# test 7/48: step 0 -> +4.0V [rep 1/3]
# mode=... from=0.00V to=4.00V rep=1 phaseB_start_sample=480 phaseA_clean=yes gyro_bias_dps=0.4611 stop_reason=platform_settled
t_us,targetV,wheel_vel,wheel_angle_rad,gyroZ_dps,current_mA,busV
```

Metadata is free-form `key=value`. **Parsers must not pattern-match the whole
line** — see §17.

| field | units | notes |
|---|---|---|
| `t_us` | µs | `micros()`, not zeroed |
| `targetV` | V | commanded `U_q` |
| `wheel_vel` | rad/s | `motor.shaft_velocity`, direction-corrected |
| `wheel_angle_rad` | rad | `sensor.getAngle()`, absolute, **sign inverted vs velocity** |
| `gyroZ_dps` | dps | raw, bias NOT removed |
| `current_mA` | mA | motor supply, ~−6 mA zero offset |
| `busV` | V | motor supply |

Older format (run 1) used `vel_raw, vel_filtered` and no current/angle columns.
`filter_calibration.py` handles both.

### Filtered CSV

```
t_s,phase,targetV,wheel_vel,wheel_angle_rad,wheel_accel,gyro_dps_raw,gyro_dps,platform_deg,current_mA,busV,power_mW,iq_est_A
```

| added field | meaning |
|---|---|
| `t_s` | seconds from test start |
| `phase` | `A` (pre-step hold) or `B` (transient) |
| `wheel_angle_rad` | zeroed per test, sign corrected |
| `wheel_accel` | `d(wheel_vel)/dt` — sets reaction torque |
| `gyro_dps` | bias removed, sign flipped |
| `platform_deg` | integrated `gyro_dps` |
| `iq_est_A` | `busV·current/targetV`; **NaN** where \|targetV\| < 0.15 |

### Heading controller capture

```
t_us,target_deg,theta_deg,omega_p,omega_w,alpha,u
```

`omega_p` rad/s, `omega_w` rad/s, `alpha` rad/s² commanded, `u` volts.

### Summary files

- `summary.csv` — one row per test
- `repeatability.csv` — mean/std across repeats per condition
- `breakaway_summary.csv` — per step size, with motor and platform thresholds

---

## 14. Firmware

Only one sketch can be in `src/` at a time (PlatformIO builds one
`setup()`/`loop()`). Inactive ones live in `unflashed_files/`.
`MagneticSensorMT6701SSI.h/.cpp` stay in `src/` always — every sketch needs them.

### `full.cpp` — open-loop bring-up

Torque/voltage mode with SimpleFOC Commander over USB or HC-05. `M<volts>` sets
target. Burst capture on `B`. Used for the earliest sanity checks.

**Known bug, not fixed:** the gyro integration lives inside the
`PRINT_INTERVAL_MS` block (10 s), so `zAngleDeg` integrates with `dt ≈ 10` and
blows up. Left alone deliberately since the file was superseded.

### `calibration.cpp` — system ID, three modes

```c
#define SWEEP_MODE  MODE_STEP | MODE_STAIRCASE | MODE_BREAKAWAY
```

| mode | purpose | status |
|---|---|---|
| `MODE_STEP` | step/reversal system ID | produced run 2 |
| `MODE_STAIRCASE` | voltage-level staircase | **superseded** — cannot measure platform breakaway (§12 run 4). Kept for motor deadband. |
| `MODE_BREAKAWAY` | step-size sweep | produced run 5 |

Key parameters: `TEST_SEQUENCE[]`, `N_REPEATS`, `LOG_DECIM_A/B`,
`MAX_LOG_SAMPLES`, `GYRO_SETTLE_DPS`, `SETTLE_DEBOUNCE_N`, `MIN_CAPTURE_MS`,
`BREAK_MIN_V/MAX_V/STEP_V`.

Safety: any serial byte aborts. `WAIT_FOR_START_SIGNAL` gates the start.

### `heading_control.cpp` — the controller

200 Hz control loop, runtime-tunable gains, capture and dump in the same CSV
framing as `calibration.cpp`.

**Commands** (115200, newline-terminated, USB or HC-05):

| send | does |
|---|---|
| `O1` | **letter O.** Open-loop 1 V pulse, 800 ms, no feedback — the sign check |
| `T90` | step target to 90° and capture |
| `H0` | hold heading at 0° with slow telemetry |
| `Z` | zero the heading estimate here |
| `P43` | set `K_θ` |
| `D21.1` | set `K_ω` |
| `F0.9` | set feedforward fraction |
| `W2` | set deadzone, degrees |
| `G` | print gains and state |
| `B` | re-measure gyro bias (platform must be still) |
| `X` / `R` | stop / resume |

**Any unrecognised input stops the motor.** Deliberate — a confused operator
should not leave a flywheel spinning.

Safety: aborts above `WHEEL_SAT_LIMIT` = 45 rad/s.

### Build

PlatformIO, STM32duino core, Nucleo-F446RE. Libraries: SimpleFOC 2.4.0,
Adafruit MPU6050 / INA219 / Unified Sensor / BusIO, Wire, SPI.

---

## 15. Analysis pipeline

```
capture_calibration.py  →  calibration_run_<timestamp>/       (raw)
filter_calibration.py   →  <run>/filtered/                    (corrected + derived)
plot_calibration.py     →  <run>/filtered/plots/*.png
make_replay.py          →  <run>/filtered/replay.html
```

`py -m pip install numpy pandas matplotlib pyserial` (on Windows `pip` alone
often is not on PATH).

### `capture_calibration.py`

Serial terminal + automatic CSV capture. Two-way: typed input is forwarded to
the board. Detects capture blocks by the CSV header line (`t_us,`) rather than
parsing metadata — see §17.

### `filter_calibration.py`

Three corrections:

1. **Gyro bias** — measured from at-rest phase-A windows of `from=0.00V` tests
   only. Tests that spin up first are excluded (their phase A contains real
   motion). Cross-checked against the firmware's own startup value.
2. **Gyro sign flip** — so wheel and platform share a convention.
3. **Wheel angle sign + zeroing** — auto-detected per test by regressing
   `d(angle)/dt` against `wheel_vel`; flips when the slope is negative.

Derived columns and summaries per §13. Flags `phaseA_clean=no` tests explicitly.

### `plot_calibration.py`

Per-test 4-panel figures, `repeats_*.png` overlays, `overview_steps.png`,
`overview_scaling.png` (with error bars across repeats), `staircase_*.png` with
breakaway/re-stick marked. `--no-per-test` skips the slow part.

### `make_replay.py`

Standalone HTML: top-down platform and wheel turning from recorded data, plus
telemetry readouts, momentum bar, and scrubbable traces. Frame budget adapts to
test count. Skips staircase files (no step to replay).

**Momentum bars are relative within each body, not a conserved total** — `J_w`
and `J_p` are not known absolutely, so the two channels cannot share units.

### `use_plain_ar.py`

Build workaround, see §17.

---

## 16. Tuning procedure

Keep a finger on `X` throughout.

**Step 0 — boot.** Platform completely still. Bias should read 0.35–0.55 dps.
Confirm the startup text lists `O<V> openloop` (otherwise an old binary is
flashed).

**Step 1 — check the estimate.** `G` a few times over 30 s: drift under
~0.5°/min. Rotate ~90° by hand, `G` again: should read about ±90.

**Step 2 — verify the gyro sign. Do not skip.**

```
O1
```

| `omega_w` | `theta_deg` | meaning |
|---|---|---|
| positive | **negative** | correct |
| positive | positive | flip `GYRO_SIGN` to `+1.0f`, reflash |

With the sign inverted the closed loop drives error *larger* and the platform
accelerates until the wheel abort catches it. The open-loop pulse is bounded
either way.

**Step 3 — first closed loop.**

```
Z
H0
```

Nudge by hand. It should push back. If it accelerates away, `X` immediately —
sign problem.

**Step 4 — step response.** `T90`, then plot rise time, overshoot, settling,
final error, and end-of-run `omega_w`.

**Step 5 — gain progression.** Work **down** the table (§10), re-running `T90`
each time. Stop on: ringing (back off a row), audible buzz (lower `D` only),
wheel climbing toward 45 rad/s, or `ω_n` approaching 4.8.

**Step 6 — feedforward trim.** `F`, from 0.85. Parks short → raise; overshoots
then creeps back → lower. Realistic best 0.85–0.95.

**Step 7 — deadzone.** `W`, trades accuracy against wheel windup (§9). Never
zero.

**Step 8 — record.** Final `G`, a clean `T90`, a `T-90` (checks the measured
7.2% direction asymmetry), and an `H0` nudge run. Then update the constants at
the top of the source so tuned values survive a power cycle.

### Expected, not faults

- **A few degrees of final error** — the Coulomb deadband, a measured property.
  Docking magnets tolerate several degrees, which is why they beat a mechanical
  latch.
- **20–30 rad/s left in the wheel after each slew** — Coulomb friction holds the
  platform at its new heading, so momentum stays banked. Three or four slews in
  one direction approaches saturation.
- **~0.8°/min heading drift** — gyro-only integration. Vision fixes this.

---

## 17. Bugs and gotchas

Everything that cost real time.

### Sign conventions (three separate ones)

1. **Gyro vs encoder.** Raw data has wheel and platform *positively* correlated.
   Mounting artifact, not physics. `GYRO_SIGN = −1`.
2. **`sensor.getAngle()` vs `motor.shaft_velocity`.** The encoder reports raw
   direction; `shaft_velocity` has the `initFOC()` alignment correction applied.
   Measured `d(angle)/dt` vs `wheel_vel` slope: **−0.98 across every test.** Test
   1 ended with velocity +8.01 rad/s and angle −5.76 rad — physically
   impossible. `filter_calibration.py` auto-detects and corrects. Logging
   `motor.shaft_angle` instead would fix it at the source.
3. **The LQR rate term and the two feedforward branches** — §9.

### `A1`/`A2` collide with Arduino macros

`A0`–`A15` are predefined analog pin macros on STM32duino. Use `A_1`/`A_2`.
Same reason the mode enum is `CTRL_*` rather than `MODE_*`.

### Device Guard blocks `arm-none-eabi-gcc-ar.exe`

WDAC blocks specific binaries by reputation. `g++.exe` ran fine from the same
folder — `gcc-ar` is a rarely-invoked LTO wrapper with no reputation.

**Fix:** substitute plain `arm-none-eabi-ar` via `use_plain_ar.py`, registered
**without** the `pre:` prefix:

```ini
extra_scripts = use_plain_ar.py     ; correct
extra_scripts = pre:use_plain_ar.py ; runs too early, platform overwrites AR
```

You lose only the LTO plugin, which this project does not use.

### Serial prompts vs abort-on-any-byte

A typed character's trailing Enter arrives a few ms *after* the character. A
prompt that drains once and returns immediately misses it — the byte lands
moments later and `checkAbort()` reads it as an abort, killing the sweep the
keypress just resumed.

**Fix:** `waitForKeypress()` drains until the link has been quiet for
`START_FLUSH_MS` (400 ms). Always use it for prompts; never hand-roll
wait-then-drain-once.

### Metadata regex broke capture silently

`capture_calibration.py` originally matched the metadata line exactly
(`from=...V to=...V hold=...ms stop_reason=...`). When repeats and phase flags
were added, the match failed, the state machine stalled in `AWAIT_META`, and
**a full sweep was lost** — no files written, no error.

**Fix:** treat metadata as opaque, detect the CSV header (`t_us,`) instead, add
a stall guard and an exit summary.

### Buffer sizing

The staircase needed ~6,700 samples against a 2,400 cap. `MODE_BREAKAWAY` dumps
after each trial so only one trial (~580 samples) is ever in memory — the
structural fix, not just a bigger number.

### INA219 reading `inf`

`begin()` returning true only confirms an I2C ack, not that calibration
registers are set. Call `setCalibration_32V_2A()` explicitly. Also carries a
~−6 mA zero offset worth subtracting.

### I2C speed dominates the loop

At the default 100 kHz each logged sample cost ~3.7 ms (MPU6050 + two INA219
reads) against a ~27 kHz bare FOC loop — why phase B logged at 224 Hz rather
than the 320 Hz its decimation implied. `Wire.setClock(400000)` in all current
firmware.

### TIM2/TIM3 are taken — matters for the RTOS merge

Motor PWM uses TIM2_CH1 (PA5) and TIM3_CH1/CH2 (PA6/PA7). The proven RTOS
skeleton in `unflashed_files/rtos_tester.cpp` uses `HardwareTimer(TIM2)` for its
1 kHz tick — `setOverflow()` rewrites TIM2's ARR, the same register that sets
the PWM period. **Use TIM4, TIM5, or TIM9 for the control-loop timer.**

### Net rotation is not total rotation

§12 run 3. The printed figure is signed net over the whole capture including
phase A. Integrate `|gyro|` if you want distance travelled.

### Peak rate is a bad breakaway detector

§12 run 5. Ball-transfer compliance lets the platform deflect and spring back.
Use net displacement.

---

## 18. Open items

### Immediate

1. **Run the closed loop.** `O1` sign check, then `H0`, then `T90`. This is the
   next action.
2. **Tune** per §16.
3. **Persist tuned gains** into the source.

### Near-term

4. **`motor.shaft_angle`** instead of `sensor.getAngle()` — fixes the angle sign
   at the source.
5. **Vision:** AprilTag on the Pi, UART protocol, complementary filter, then
   Kalman.
6. **RTOS merge** — remember the TIM2 conflict.
7. **Desaturation.** Each slew banks 20–30 rad/s. The fix is to ramp the wheel
   down slowly enough that reaction torque stays below breakaway, so friction
   holds the platform while the wheel unwinds. The ramp-rate experiment that
   sizes this was deferred and never run.

### Known unknowns

- **`J_p` absolute** — only the ratio is known. Measure geometrically
  (`½mR²` or a pendulum test), *not* from momentum conservation, since friction
  corrupts that ledger.
- **Direction asymmetry** — real at 4 V (7.2%, 7σ), noise at ≤2.5 V. Unmodelled.
- **Low-voltage nonlinearity** — single-slope residuals ±9.8%; two-parameter
  R/K_v fit is 4.9% RMS but worst at 1.0 V (+10%). Coulomb friction is the
  likely cause.
- **Breakaway repeatability** — only 2 repeats, rep 1 systematically higher than
  rep 2, cause unestablished.

### Deliberately not done

- **MPC.** Simulated slews use 18% of voltage with zero saturated timesteps.
  With inactive constraints MPC converges to exactly the LQR solution, so it
  would cost significant complexity for identical behaviour. It becomes
  worthwhile for terminal docking (terminal state constraints), momentum limits
  as hard constraints, and the unidirectional fan allocation problem. Best
  placed on the Pi at 10–50 Hz feeding setpoints to this loop, replacing the
  trajectory generator rather than the controller.

---

## 19. Translation subsystem — future

Not started. Notes carried forward.

### None of the rotation constants transfer

Translation is a different plant needing its own campaign: fan thrust vs PWM
curves, platform mass, translational breakaway force, table-tilt disturbance.

**What transfers is the method:** measure the deadband before tuning, check
controllability before designing, test Coulomb vs viscous rather than assuming,
verify signs open-loop first.

### Architecture

Because translation is architecturally decoupled from rotation, the clean
approach is a rotation transform (world → body frame using `θ`) then **three
independent double integrators** — x, y, θ — each with its own PD. Three tuning
problems, not one 18-dimensional one.

### Differences from rotation

- **Fans are unidirectional.** Allocation is constrained (thrust ≥ 0). Opposing
  pairs need idle bias for bidirectional authority, which costs power
  continuously — but also keeps motors above the sensorless commutation floor.
- **No actuator feedback.** ESCs are open-loop from the MCU: no encoder, no RPM,
  no thrust. Back-EMF is used *inside* the ESC for commutation only. The only
  feedback is platform position from vision. Contrast the wheel, where the
  MT6701 made feedback linearization possible.
- **Square-law nonlinearity.** Thrust ∝ RPM², RPM ≈ linear in throttle, so
  `throttle = sqrt(F_desired/F_max)`. The fan analogue of feedback
  linearization, but static.
- **Table tilt hits translation far harder than rotation.** The plan's 0.3°
  figure is a translational disturbance. Level the table first.
- **Prop reaction torque acts about a horizontal axis** (props spin in vertical
  planes), so it is a tipping moment, not yaw. Thrust-line offset from the CoM
  *does* produce yaw, which the reaction wheel must reject.

### Inferred translational friction

Both frictions share the same μ:

```
T_c = μ·m·g·r_b      F_c = μ·m·g      ⟹  F_c = T_c/r_b = J_p·α_break/r_b
```

For an 8-inch disc (R ≈ 0.102 m), `J_p = ½mR²`, ball radius 70–90 mm:

| mass | friction |
|---|---|
| 1.2 kg | 30–39 gf |
| 1.5 kg | 38–48 gf |
| 2.0 kg | 50–64 gf |

Plus 6–20 gf for acceleration → **roughly 50–100 gf from one fan** on a cardinal
push (only one fan works for N/S/E/W; a diagonal splits across two).

**Caveat:** rotation makes each ball roll along a circular arc, so the contact
also spins about the vertical axis. Pure translation is straight rolling with no
scrub, so these numbers likely *overstate* the translational case.

**Verify directly:** tie a string to the platform, run it over the table edge,
add weight until it moves. Five minutes, replaces the widest uncertainty in the
sizing.

### Parts selected

| item | choice | note |
|---|---|---|
| motors | FEICHAO 2204 2300KV ×4 | ~$30; 420 gf claim corroborated by an EMAX MT2204 bench test (455 gf on 5×4.5 at 3S) |
| ESC | 4-in-1, 2–6S, PWM/DSHOT | channel matching matters because fan control is open-loop |
| props | **4-inch preferred** | 5-inch needs 52 mm standoff and must mount within 38 mm of centre to stay inside an 8-inch disc |

Prop geometry, 8-inch disc (102 mm radius):

| prop | half-span | max mount radius | standoff above disc |
|---|---|---|---|
| 5in | 63.5 mm | 38 mm | 52 mm |
| 4in | 50.8 mm | 51 mm | 39 mm |
| 3in | 38.1 mm | 64 mm | 26 mm |

Also needed: a dedicated **5 V / 5 A** buck for the Pi — ESC BECs (~2 A) and
typical LM2596 modules cannot supply a Pi 5 under OpenCV load.

---

## Appendix — quick reference

```
A_1        = 40.56 rad/s²/V      wheel accel per volt
A_2        = 5.56  1/s           wheel damping pole
a          = 0.19                J_w/J_p
A_FRICTION = 22.3  rad/s²        Coulomb feedforward magnitude
GYRO_SIGN  = −1

feedback linearisation:  U_q = (α_cmd + 5.56·ω_w) / 40.56
control law:             α = −K_θ·e + K_ω·ω_p     (minus, PLUS)
feedforward moving:      α += −22.3·sign(ω_p)     (negative)
feedforward stuck:       α += +22.3·sign(α)       (positive)
gains:                   K_θ = ω_n²/0.19,  K_ω = 1.4·ω_n/0.19
                         ω_n = 5.714/t_settle,  ceiling ω_n ≈ 4.8
```