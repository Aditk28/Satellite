# Calibration run index

Raw 200 Hz captures. `capture_calibration.py` writes new runs here as
`run_<timestamp>/`. The dated folders below were renamed by hand so the name
says what the run was.

CSV format for all runs from 2026-08-14 on:

```
t_us,target_deg,theta_deg,omega_p,omega_w,alpha,u,ax,ay,iadc
```

The `# mode=...` metadata line holds gains, deadzones, `A_static`/`A_moving`/
`A_visc`, `compFrac`, `alpha_stall_max`, accel bias, `open_volts` and
`fan_pct`/`fan_sel`. Parsers should key on the CSV header `t_us,` and treat
the metadata line as opaque, since its fields change between firmware builds.

`stepCount` resets on reboot, so test numbers restart at `test01` mid-session.
Several folders below contain duplicate numbers for that reason.

## Original rotation campaign (July-Aug 2026, lighter platform)

These predate the translation hardware. Every constant derived from them was
superseded by the 2026-08-15 re-identification (`a` moved from 0.19 to
0.098). Kept for provenance and as a reference for the method. Some contain
a `filtered/` subfolder from the Python pipeline.

| folder | result |
|---|---|
| `2026-07-29_run1_first_velocity_sweep` | 9 tests, velocity only. Gyro and encoder sign conventions are opposite; tau falls with voltage (Coulomb friction). |
| `2026-07-30_run2_main_48test_dataset` | 48 tests, the main historical dataset. `R` and `K_v` separated via power balance. |
| `2026-07-30_run4_stiction_staircase` | Did not measure breakaway: reaction torque scales with wheel acceleration, so a constant-voltage staircase cannot. Found the 0.35 V motor deadband. |
| `2026-07-30_run5_breakaway_46trials` | Breakaway ~0.55 V, judged by net displacement, not peak rate (ball-transfer compliance distorts peak rate). |
| `2026-08-02_run6_reidentification_Otests` | `K' = 8.51` vs a modelled 7.3. That +16% error had made the residual pole positive and caused the first closed-loop runaway. |
| `2026-08-02_run7_compFrac_Csweep` | Neutral at 0.972 (+) / 0.892 (-). |
| `2026-08-02_closed_loop_*` | First closed-loop steps. |
| `2026-08-02_run8_envelope_15steps` + `_continued` | +/-30 to 180 deg envelope: 1.20 deg mean error, 1.57 s settling. Superseded by 2026-08-19 (0.47 deg mean) on a heavier platform. |

## RTOS migration spot-checks (Aug 2026)

`2026-08-02to08_rtos_migration_spotchecks/`: 11 small runs (1-5 files each)
taken as checks during the FreeRTOS migration. Nothing here feeds a current
constant.

## Translation plant ID

| folder | result |
|---|---|
| `2026-08-14_trans_thrust_first_probe` | 1 file, first `I` thrust step. Old-style filename. |
| `2026-08-14_trans_ladder_fan1_25to50` | 19 runs. First ladder: breakaway ~35%, square law confirmed. |
| `2026-08-14_trans_ladder_allfans_35to60` | 31 runs. Main dataset: fan 1 at 35-60% x3, fans 2/3 at 40/50% x3, fan 4 x1. Source of `A(pct) = 2.1e-4 * pct^2`. Also where fan 4's reversed direction was found. |

Result: `A(throttle) = 2.1e-4 * pct^2` m/s^2, `A_c ~ 0.26` m/s^2, go/no-go
margin 2.9x (pass). Fans 1-3 matched within 7%. Mobility drifts about +30%
within a session, so channel comparisons across sessions are not valid.

## Wheel plant re-identification

| folder | result |
|---|---|
| `2026-08-15_wheel_ID_Otests_1to3V` | 12 runs. `A_2 = 4.97` from free decay and from spin-up t63 (two methods, 0.5% apart). `K' = 9.64` incremental. `a = 0.105`. |
| `2026-08-15_wheel_ID_Otests_repeat3V` | 2 runs, 3 V repeat. |
| `2026-08-15_wheel_ID_Otests_3to5p5V` | 11 runs, 3.5-5.5 V, after the `O` command's +/-3 V clamp was lifted. Plateau is linear to 48 rad/s, so gain does not collapse at high speed. `a = 0.098` pooled over 11 rising-phase fits. |
| `2026-08-15_breakaway_sweep_0p3to0p9V` | 23 runs. Breakaway 0.65-0.85 V, varying ~1.3x run to run. Judge by excursion during spin-up, not net displacement: an `O` pulse is symmetric so net is ~0 either way. |
| `2026-08-15_compFrac_Csweep` | 10 runs, +/-2 V, cf 0.70-0.95. `pole = 3.905*cf - 4.453`. Showed cf 0.89 was mildly unstable (+0.04), which is why passive desaturation did not work. |

## Rotation retune

| folder | result |
|---|---|
| `2026-08-15_gain_descent_T90` | 4 runs, gain rows from 3.0 s down to 1.2 s. Error 23 deg -> 3.14 deg; the fastest gains gave the lowest wheel peak. |
| `2026-08-15_Afriction_sweep_clamp55` | 16 runs. A36/40/44/48 at 5 deg. `ALPHA_STALL_MAX` was the binding constraint, not `A_FRICTION`: A44 and A48 gave identical clamped authority. |
| `2026-08-15_Afriction_sweep_clamp70` | 16 runs, clamp raised. 12/16 closed, no aborts. Two opposite failure modes (never breaks free, or breaks free and runs away 28 deg), which led to splitting static from kinetic friction. |
| `2026-08-15_split_feedforward_8of8` | 8 runs. `A_static 60` + `A_moving 34`: 8/8, mean 1.12 deg. |
| `2026-08-15_deadzone_1p5_0p8` | 8 runs. Tighter deadzones: mean 0.97 deg, worst 1.47 deg. |
| `2026-08-19_Ksweep_FINAL_TUNE` | 8 runs. `K_omega` 64 -> 45 -> 30 -> 52. zeta = 0.54, not 0.7. Final: 0.47 deg mean, +/-180 deg in one move, wheel returns to rest. |

## Failed runs kept for reference

| folder | notes |
|---|---|
| `2026-08-15_deadzone_latch_REVERTED` | 4 runs. Latching the `FINE_WW` transition to stop deadzone chatter was worse (mean 0.97 -> 1.25 deg, wheel peaks 6-17 -> 8-36). The chatter acts as a safety valve; don't change it without n >= 8 runs of evidence. |
| `2026-08-15_BROKEN_gains_not_persisted` | 6 runs, all 21-63 deg short. The build had `K = 34.4` with deadzones sized for `K = 216`, so they sat below the deadband floor and feedforward never switched off. Persist a tuned gain set as a whole or not at all. |
| `2026-08-15_envelope_T90_T180_D64` | 9 runs at zeta = 0.66. 3/9 clean, mid-slew stalls. The baseline for the `K_omega` sweep. |

## Yaw coupling

| folder | result |
|---|---|
| `2026-08-14_yaw_coupling_PRE_retune*` | 3 folders, 4 runs. Pre-retune: 24 deg excursions, wheel to 45 rad/s (one abort). Measures the untuned controller, not the disturbance. Kept as the baseline. |
| `2026-08-19_yaw_coupling_POST_retune` | 9 runs. Sign reverses between fan 1 and fan 4, so the cause is thrust-line offset, not ball-transfer steering. Magnitude 2-3 rad/s^2, excursion 4-13 deg, wheel peaks 4-12 rad/s. No mechanical correction needed. The board reset mid-session (duplicate test numbers) and fan 2 produced no thrust in either run. |
