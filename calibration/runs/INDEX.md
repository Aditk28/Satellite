# Calibration run index

Raw 200 Hz captures. `capture_calibration.py` writes new runs here automatically
(`calibration/runs/run_<timestamp>/`); these were renamed by hand at the Phase 2 close
so a folder name says what the run *was* rather than when it happened.

**CSV format** (all runs from 2026-08-14 onward):

```
t_us,target_deg,theta_deg,omega_p,omega_w,alpha,u,ax,ay,iadc
```

Metadata lives in the `# mode=...` line: gains, deadzones, `A_static`/`A_moving`/
`A_visc`, `compFrac`, `alpha_stall_max`, accel bias, `open_volts`, `fan_pct`/`fan_sel`.
Parsers must key on the CSV header `t_us,` and treat the metadata line as opaque —
pattern-matching it silently lost a full sweep once (`CONTROL_README` §16).

⚠️ **`stepCount` resets on reboot**, so test numbers restart at `test01` mid-session.
Several folders below contain duplicate numbers for that reason.

---

## Original rotation campaign (July–Aug 2026, the LIGHTER platform)

⚠️ **These predate the translation hardware. Every constant derived from them was
superseded by the 2026-08-15 re-identification** — `a` alone moved 0.19 → 0.098. Kept
for provenance and because the *method* is still the reference (`CONTROL_README` §14).
Some contain a `filtered/` subfolder from the old Python pipeline.

| folder | what it established |
|---|---|
| `2026-07-29_run1_first_velocity_sweep` | 9 tests. Velocity-only. Found gyro/encoder sign conventions oppose; τ falls with voltage (a Coulomb signature). |
| `2026-07-30_run2_main_48test_dataset` | 48 tests, the main historical dataset. `R` and `K_v` separated via power balance. |
| `2026-07-30_run4_stiction_staircase` | Failed at its stated purpose — reaction torque scales with wheel *acceleration*, so a constant-voltage staircase cannot measure breakaway. Found the 0.35 V motor deadband anyway. |
| `2026-07-30_run5_breakaway_46trials` | Breakaway ≈ 0.55 V by **net displacement, not peak rate** (ball-transfer compliance fools peak rate). |
| `2026-08-02_run6_reidentification_Otests` | `K' = 8.51` vs a modelled 7.3 — the +16% error that had flipped the residual pole positive and caused the first closed-loop runaway. |
| `2026-08-02_run7_compFrac_Csweep` | Neutral at 0.972 (+) / 0.892 (−). |
| `2026-08-02_closed_loop_*` | First closed-loop steps. |
| `2026-08-02_run8_envelope_15steps` + `_continued` | The ±30–180° envelope: 1.20° mean, 1.57 s settling. **Superseded by 2026-08-19 (0.47° mean) on a heavier platform.** |

## RTOS migration spot-checks (Aug 2026)

`2026-08-02to08_rtos_migration_spotchecks/` — 11 small runs taken as phase gates during
the FreeRTOS migration (`RTOS_migration.md`). Mostly 1–5 files each. Kept for
provenance; nothing here feeds a current constant.

---

## Translation plant ID (Phase 2.1 / 2.2)

| folder | what it established |
|---|---|
| `2026-08-14_trans_thrust_first_probe` | 1 file, first `I` thrust step. Old-style filename (pre label fix). |
| `2026-08-14_trans_ladder_fan1_25to50` | 19 runs. First ladder — breakaway ~35%, square law confirmed. |
| `2026-08-14_trans_ladder_allfans_35to60` | 31 runs. Main dataset: fan 1 ladder 35–60% ×3, fans 2/3 at 40/50 ×3, fan 4 ×1. **Source of `A(pct) = 2.1e-4·pct²`.** Also where fan 4's reversed direction was found. |

**Result:** `A(throttle) = 2.1e-4·pct²` m/s², `A_c ≈ 0.26` m/s², go/no-go **2.9× PASS**.
Fans 1–3 matched within 7%. ⚠️ Within-session mobility drift of ~+30% makes any
*cross-session* channel comparison invalid.

## Wheel plant re-identification (Phase 2.3′)

| folder | what it established |
|---|---|
| `2026-08-15_wheel_ID_Otests_1to3V` | 12 runs. `A_2 = 4.97` from free decay *and* spin-up t63 (two independent methods, 0.5% apart). `K' = 9.64` incremental. **`a = 0.105`.** |
| `2026-08-15_wheel_ID_Otests_repeat3V` | 2 runs, 3 V repeat. |
| `2026-08-15_wheel_ID_Otests_3to5p5V` | 11 runs, 3.5–5.5 V — only possible after the `O` command's ±3 V clamp was lifted. **Plateau is linear to 48 rad/s**, killing the "gain collapses at high speed" theory. **`a = 0.098`** pooled over 11 rising-phase fits. |
| `2026-08-15_breakaway_sweep_0p3to0p9V` | 23 runs. Breakaway 0.65–0.85 V, varying ~1.3× run to run. Judge by **excursion during spin-up**, not net displacement — an `O` pulse is symmetric so net ≈ 0 either way. |
| `2026-08-15_compFrac_Csweep` | 10 runs, ±2 V, cf 0.70–0.95. `pole = 3.905·cf − 4.453`. **Proved cf 0.89 was mildly UNSTABLE (+0.04)** — the cause of dead passive desaturation. |

## Rotation retune (Phase 2.3′)

| folder | what it established |
|---|---|
| `2026-08-15_gain_descent_T90` | 4 runs, the 3.0→1.2 s gain rows. Error 23° → 3.14°; **fastest gains gave the lowest wheel peak**, confirming §9. |
| `2026-08-15_Afriction_sweep_clamp55` | 16 runs. A36/40/44/48 at 5°. **Proved `ALPHA_STALL_MAX` was the binding constraint, not `A_FRICTION`** — A44 and A48 delivered identical clamped authority. |
| `2026-08-15_Afriction_sweep_clamp70` | 16 runs, clamp raised. 12/16 closed, no aborts. Showed **two opposite failure modes** (never breaks free / breaks free and runs away 28°), which is what motivated splitting static from kinetic friction. |
| `2026-08-15_split_feedforward_8of8` | 8 runs. `A_static 60` + `A_moving 34` → **8/8, mean 1.12°.** |
| `2026-08-15_deadzone_1p5_0p8` | 8 runs. Tightened deadzones → **mean 0.97°, worst 1.47°.** |
| `2026-08-19_Ksweep_FINAL_TUNE` | 8 runs. `K_ω` 64→45→30→**52**. ζ = 0.54, not 0.7. **Final: 0.47° mean, ±180° in one move, wheel returns to rest.** |

## Runs kept because they document a *failure*

| folder | why it's here |
|---|---|
| `2026-08-15_deadzone_latch_REVERTED` | 4 runs. Latching the `FINE_WW` transition to stop deadzone chatter measured **worse** (0.97→1.25° mean, wheel peaks 6–17→8–36). The chatter is a safety valve. **Trap T27 — don't re-fix without n ≥ 8 evidence.** |
| `2026-08-15_BROKEN_gains_not_persisted` | 6 runs, all 21–63° short. Build had `K = 34.4` with deadzones sized for `K = 216`, putting them below the deadband floor so feedforward never switched off. **Trap T26 — persist a tuned set atomically or not at all.** |
| `2026-08-15_envelope_T90_T180_D64` | 9 runs at ζ = 0.66. 3/9 clean; mid-slew stalls. The "before" for the `K_ω` sweep. |

## Yaw coupling (Phase 2.4)

| folder | what it established |
|---|---|
| `2026-08-14_yaw_coupling_PRE_retune*` | 3 folders, 4 runs. Pre-retune: 24° excursions, wheel to 45 (one abort). **Measured the untuned controller, not the disturbance.** Kept as the "before". |
| `2026-08-19_yaw_coupling_POST_retune` | 9 runs. **Sign reverses between fan 1 and fan 4 → thrust-line offset, not ball-transfer steering.** Magnitude 2–3 rad/s², excursion 4–13°, wheel peaks 4–12 rad/s. Mechanical correction NOT needed. ⚠️ Board reset mid-session (duplicate test numbers) and fan 2 produced no thrust in either run. |
