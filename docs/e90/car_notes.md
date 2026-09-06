# E90 EV car brand — parsers, fingerprint, TX

Owner: car (opendbc `opendbc/car/bmw/`). Spec: `E90-SUNNYPILOT-PORT-PLAN.md` §3, §5, §6 Phase 3–4, §11.

Platform: `BMW_E90_EV`. Safety model: `CarParams.SafetyModel.bmw` (37).

## Parsers (all on panda bus 0 = can2)

| Bus enum | DBC | Messages used |
|---|---|---|
| `Bus.pt` | `bmw_e9x_e8x` | `WheelSpeeds` 0xCE, `SteeringWheelAngle` 0xC4, `CruiseControl` 0x194, `TerminalStatus` 0x130, `TurnSignals` 0x1F6 |
| `Bus.alt` | `bmw_can2_powertrain` | `PEDAL_Status` 0x199, `LeverCommand_From_VCU` 0x3FD, `EPB_Status` 0x3FE, `SendAbsDscMessages` 0x0A8, `ID108DIR_torque_From_Motor` 0x107 |
| `Bus.body` | `ocelot_controls` | `STEERING_STATUS` 0x22F (RX). Packer for `STEERING_COMMAND` 0x22E |
| `Bus.loopback` | raw cache | 0x0A8 / 0x3FE if a DBC signal is missing |

Missing DBC files: that parser is omitted; carstate degrades (no pedal/gear/servo torque). PT DBC is required for speed/angle/stalk.

## CarState

| Field | Source | Notes |
|---|---|---|
| vEgo | `Wheel_RL`/`Wheel_RR` avg, 0.0625 km/h → m/s, Kalman | **Never 0x1A0** (gateway ≥ 10 km/h) |
| standstill | `vEgoRaw < 0.3 m/s` | |
| steeringAngleDeg | 0xC4 `SteeringPosition` | |
| steeringTorqueEps | 0x22F `STEERING_TORQUE` | Actuator torque, **not** a torsion bar. `steeringPressed` left false |
| steerFaultTemporary | 0x22F `CONTROL_STATUS` bit2 (SoftOff) | |
| gear | 0x3FD `Direction` 32/64/96/128 → P/R/N/D | 129 = Sport |
| gas / gasPressed | 0x199 `PedalInPct` | pressed if In>20 **and** In>Out+5 |
| brakePressed | 0x0A8 `BrakeMagic` == `0x64` | raw byte7 fallback. **Not** e9x `EngineAndBrake` / 0x19E |
| parkingBrake | 0x3FE `ParkingBrake` == 2 | |
| cruise buttons | 0x194 plus1/minus1/plus5/minus5/resume/cancel/cancel_lever_up | plus1→accelCruise, minus1→decelCruise, resume→resumeCruise, cancel→cancel, cancel_lever_up→**lkas** (MADS lat toggle) |
| cruiseState.available | ignition-ish + gear D | |
| cruiseState.enabled | always False from stock | DCC dead. `pcmCruise = False` |

Do **not** parse e9x `CruiseControlStatus` 0x200 — that ID is the pedal interceptor on this bus.

## Fingerprint

Static FPv1, no UDS (`FW_VERSIONS = {}`):

`0xA8:8, 0xC4:7, 0xCE:8, 0x107:8, 0x194:4, 0x199:3, 0x22F:8, 0x3FD:5, 0x3FE:7`

Flags on the platform (PEDAL_INTERCEPTOR **off**): `STEPPER_SERVO_CAN | NORMAL_CRUISE_CONTROL` (VO540).

## What is sent (v1)

| ID | When | Payload |
|---|---|---|
| **0x22E** `STEERING_COMMAND` | 100 Hz | mode 1 TorqueControl while `CC.latActive`; SoftOff (3) for 1 s on disable; Off (0) after / on fault. Torque = `actuators.torque * 6 Nm`, rate 10 Nm/s up. Checksum: sum-fold init=addr, two-pass pack |

## What is **not** sent

- **0x194** stalk — RX UI buttons only. Never spoof DCC.
- **0x200** `CRUISE_Gas` — long agent later. Flag `E90Flags.PEDAL_INTERCEPTOR` is off; `alpha_long` does not enable interceptor; carcontroller skips 0x200 even if long is somehow true.
- **0x201** torque-cut — another device owns it.
- BMW DSC/ABS brake torque. No `SAFETY_TESLA` / `SAFETY_ALLOUTPUT`.

## Params

- mass 1570 kg, wheelbase 2.76 m, steerRatio **16 (re-measure)**, centerToFrontRatio 0.44 RWD
- minSteerSpeed 10 km/h, minEnableSpeed 8.3 m/s (30 km/h)
- `STEER_MAX = 6.0 Nm` (half; later 12). `openpilotLongitudinalControl = False`
- torque tune: placeholder `BMW_E90_EV` in `torque_data/override.toml` until logs
- no radar (`RadarInterface` dummy)
- no test route yet (`non_tested_cars`)
