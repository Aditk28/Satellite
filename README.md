# Autonomous Vision-Guided Docking Platform

A tabletop robot that autonomously navigates back to a fixed docking station from an
arbitrary starting position — a benchtop analogue of spacecraft rendezvous and docking.
Rotation is actuated by a reaction wheel; translation by four fans. Position is tracked
by fusing an onboard IMU with periodic AprilTag vision fixes, and control runs in real
time on an STM32 under FreeRTOS.

Slide the platform anywhere on the table by hand, and it finds its way back to the dock
and re-mates on its own.

## Here's a short demo video showing it in action

https://github.com/user-attachments/assets/00179884-edae-40fa-b7e2-14f87d2f1fc0

---

## How it works

**Rotation** — A BLDC reaction wheel exchanges angular momentum with the platform (Newton's
third law, not friction). Commanding wheel *acceleration* rather than voltage linearizes
the plant into a clean double integrator, closed with an LQR-derived PD law plus a
split Coulomb-friction feedforward (separate constants for breaking static stiction vs.
canceling kinetic drag — one value can't do both). Result: 0.47° mean final error across
slews from ±5° to ±180°, every slew completed in a single move, with the wheel returning
fully to rest afterward (no residual momentum to manage).

**Translation** — Four fixed fans in an X-pattern provide unconstrained planar thrust.
Because fans can only push, opposing pairs run an idle throttle bias so the effective
range spans both directions of each axis. The plant was identified directly in
acceleration units (mass and force never appear), so the same feedback-linearization and
Coulomb-feedforward structure from the rotation axis carries over unchanged.

**State estimation** — Vision position is only reliable when the platform is stationary
(motion blur and a rotating camera lever arm make it noisy in transit), so position is
dead-reckoned from the IMU between fixes and corrected against an averaged multi-frame
AprilTag reading whenever the platform is at rest. Heading is fused continuously — fast
gyro integration corrected by a slow-blended absolute vision heading — because unlike
position, yaw is well-conditioned by the gyro on its own.

**Docking approach** — Because dead-reckoning error grows with the square of elapsed
time, the platform docks in a sequence of short hops rather than one continuous
approach: move a few centimeters, stop, take a vision fix, repeat. Each leg only has to
survive a fraction of a second of open-loop drift instead of the whole trip.

**Safety** — A divergence guard monitors whether commanded thrust is actually reducing
error; if it isn't (a wiring fault, a stalled fan, a bad sign), all four fans cut
immediately rather than continuing to push. Every fault path kills the fans before the
flywheel, since the wheel stores real rotational energy.

---

## Architecture

Six FreeRTOS tasks on an STM32 Nucleo-F446RE, phase-locked to a single hardware timer:

| Task | Priority | Rate | Owns |
|---|---|---|---|
| `focTask` | 4 | 4 kHz | Motor commutation (SimpleFOC) |
| `controlTask` | 3 | 200 Hz | Sensor fusion, control law, command parsing |
| `safetyTask` | 2 | 20 Hz | Independent watchdog — overspeed, power, heartbeat |
| `commsTask` | 2 | 2 ms poll | Serial command RX + the Pi pose link |
| `fanTask` | 2 | 333 Hz | DSHOT motor drive over DMA |
| `telemTask` | 1 | event-driven | Sole serial writer (keeps I/O off the control path) |

The control loop holds 200 Hz timing to within ±6 µs under full system load, verified by
response-time analysis rather than by assumption. A Raspberry Pi runs AprilTag detection
and streams range/bearing/relative-yaw pose estimates to the STM32 over a framed,
CRC-checked UART link.

---

## Hardware

| Component | Part |
|---|---|
| MCU | STM32 Nucleo-F446RE, FreeRTOS |
| Reaction wheel motor | BLDC hollow-shaft external-rotor, driven via FOC |
| Wheel encoder | MT6701 magnetic encoder (SSI) |
| Motor driver | SimpleFOC-compatible 3-phase gate driver (DRV8313) |
| Translation motors | 4× brushless motors on 3-blade props, DSHOT ESC |
| IMU | MPU6050 |
| Power monitor | INA219 |
| Vision | Raspberry Pi + camera, AprilTag |
| Wireless telemetry | HC-05 Bluetooth |
| Chassis support | 3 ball-transfer casters |

---

## Repository layout

```
src/            Firmware — RTOS tasks, control law, estimator, fan allocation
calibration/    System-ID and data-capture tooling (Python)
tools/          Raspberry Pi vision pipeline and pose-link scripts
test/           PlatformIO test scaffold
```

`src/` builds several PlatformIO environments from the same tree — `rtos` is the real
controller; the others (`superloop`, `fantest`, `fandma`, `enctest`, `p1test`) are
standalone hardware bring-up and regression targets used during development:

```bash
pio run -e rtos -t upload
```

## License

MIT — see [LICENSE](LICENSE).
