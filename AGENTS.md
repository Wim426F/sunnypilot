# BMW E90 EV × sunnypilot — agent rules

Private fork of sunnypilot + **Wim426F/opendbc**. Not an upstream PR candidate.
Spec: `E90-SUNNYPILOT-PORT-PLAN.md`. Treat §0 safety and §11 closed decisions as law.

## What this car is

2006 BMW 320i E90, Tesla Model 3 RWD DU, RDW NL. Mass 1570 kg, wheelbase 2.76 m.
Lateral = StepperServoCAN on the column (0x22E TX / 0x22F RX).
Longitudinal = Wim pedal-interceptor (`CRUISE_Gas` 0x200). **No friction-brake actuator.**
Decel = Tesla one-pedal regen only (0 % interceptor output). Driver always covers the brake.

Panda **CAN0 = SavvyCAN bus 0** (gateway CAN2 / VCU vehicle / BMW PT after the gateway). One hardware bus. Pedal interceptor and servo are moved onto that bus. Tesla/PCS (log bus 1) is **not** on panda — do not splice (ID collisions).

## Hard safety rules

- **Never TX BMW DSC/ABS brake torque. Never TX stalk 0x194.** Stalk is UI buttons only.
- **Never TX 0x201** in v1 (another device owns torque-cut). Panda allowlist = `{0x22E len 5, 0x200 len 4}` on bus 0 only.
- **Do not use `SAFETY_TESLA` or `SAFETY_ALLOUTPUT` on the road.** Custom `SAFETY_BMW` = cereal `SafetyModel.bmw` = **37**.
- **Do not copy Toyota interceptor payload.** Same IDs 0x200/0x201, **different layout**. Toyota 0x200 is 6 bytes; Wim is **4 bytes**. Toyota `crc8_pedal` iterates **backwards**; Wim CRC iterates **forwards** over bytes 0..2. Do not call `create_gas_interceptor_command`.
- **Do not copy BMW-E8x cruise-stalk actuation.** No DCC. Long = interceptor only.
- **vEgo** = 0xCE mean of FL, FR, RL, RR. **aEgo** = 0x1A0 AccY (0.025 m/s²). Do not use 0x1A0 VehicleSpeed (gateway ≥ 10 km/h). Do not use AccX.
- **Do not use e9x `EngineAndBrake` 0x168 / 0x19E for brake.** Live 0x0A8 is VCU `SendAbsDscMessages`: byte7 `0x64` pressed, `0x04` released.
- **Do not enable stop-and-go / `minEnableSpeed = -1`.** v1 long: enable ≥ 30 km/h, drop below 25 km/h. Accel clip **+1.5 / −1.5 m/s²**.
- **Do not reuse `longitudinal_interceptor_checks()`.** It treats byte1 as gas. Wim byte1 is enable magic `0xA5`.

## 0x200 `CRUISE_Gas` (firmware is law)

4 bytes, ≥ 20 Hz, 100 ms timeout → passthrough.

| Byte | Field |
|---|---|
| 0 | GasReq 0–255 = 0–100 % |
| 1 | Enable **`0xA5`**; anything else ignored |
| 2 | Counter, **low nibble**, must increment |
| 3 | CRC8 poly **0xD5**, init **0xFF**, over bytes 0..2, **forward** |

Semantics: `out = max(bmw_pedal, can_gas)` unless cut. CAN cannot reduce below the human pedal. **0 % = Tesla home = max regen available now.** Reducing pedal toward home increases regen.

Engage: hold `PedalOutPct`, then slew (down 25 %/s, up 50 %/s). When long not active: **stop sending** 0x200 immediately (timeout → passthrough → max regen if foot off). Never send enable without `controls_allowed`.

## 0x22E `STEERING_COMMAND`

5 bytes, 10 ms. TorqueControl (mode 1) in v1. SoftOff (3) on disable, Off (0) on fault. Checksum: sum-fold init=addr. Start `STEER_MAX = 6 Nm`.

## Product modes

- Default daily: **MADS lat-only**. Remain-active on brake. Long opt-in later.
- Long: interceptor gas/regen only, vEgo ≥ 30 km/h, no full stop.
- `regenLimited` alert: `a_cmd` at −1.5 m/s² and pedal ≤ 2 % for 0.5 s. Warning only. Cover brake.

## Layout

Car/safety live in submodule `opendbc_repo` (origin = https://github.com/Wim426F/opendbc.git).
Brand folder: `opendbc/car/bmw/`. Safety: `opendbc/safety/modes/bmw.h`.
Parent pin bumps after opendbc commits.

## File ownership (parallel agents)

| Owner | May edit | Must not |
|---|---|---|
| DBC | `opendbc_repo/opendbc/dbc/bmw_can2_powertrain.dbc`, `ocelot_controls.dbc`; notes in `docs/e90/dbc_notes.md` | firmware IDs; safety; car python |
| Safety | `opendbc_repo/opendbc/safety/modes/bmw.h`, `tests/test_bmw.py`, `safety.h`, `declarations.h` | `SAFETY_ALLOUTPUT` in release; TX 0x194/0x201; car python |
| Car | `opendbc_repo/opendbc/car/bmw/**`, `opendbc/car/values.py` (BMW import only) | 0x200 TX; 0x194 TX; safety hooks |
| MADS | `openpilot/sunnypilot/mads/**`, UI regen string | panda long-through-brake |
| Docs | `docs/can_map.md`, `docs/e90/**` | production code |
| Long (later) | `carcontroller.py` 0x200 path only | stalk spoof; stop-and-go |

Do not start Phase 5 long TX until Phase 4 lat exists and panda tests pass.
