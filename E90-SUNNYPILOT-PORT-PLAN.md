# BMW E90 EV × sunnypilot — port plan (hand-off)

**Car:** 2006 BMW 320i E90, Tesla Model 3 RWD DU, RDW NL. Mass **1570 kg**, wheelbase **2.76 m**. No iDrive, analog cluster, no hydraulic OP brake.
**Goal:** sunnypilot on comma four. Lateral = dzid26 StepperServoCAN (NEMA23). Longitudinal = Wim Pedal-interceptor (`CRUISE_Gas` 0x200). MADS: steer without long.
**Repo to work in:** https://github.com/Wim426F/sunnypilot.git (fork of sunnypilot, **not** stock openpilot).
**This document is the spec.** Local agents execute phases in order. Do not skip safety gates.

---

## 0. Safety (read before any TX)

This is **not** a stock Model 3 openpilot port. Tesla OP talks to Autopilot/EPS on Party CAN. This car talks to a **custom VCU + analog Tesla pedal + stepper on the column**. Wrong safety mode = uncontrolled steer or unintended accel.

| Rule | Why |
|---|---|
| **No friction-brake authority.** Panda must **never** TX a BMW DSC/ABS brake torque request. Decel = Tesla one-pedal regen only (0 % interceptor output). | Hydraulic brakes stay human-only. |
| **Do not enable stop-and-go / `minEnableSpeed = -1` in phase 1–4.** Regen cannot emergency-stop from highway. | Driver must always cover the pedal. |
| **Do not copy Toyota interceptor TX/RX 1:1.** Same IDs 0x200/0x201, **different payloads.** Toyota 0x201 = GAS_SENSOR. Wim 0x201 = torque-cut. Mixing them is a safety bug. | |
| **Do not copy BMW-E8x-E9x longitudinal.** That fork **spoofs cruise stalk 0x194** (plus1/minus1) into stock DCC/CC. This EV has no working BMW PCM cruise for speed. Long = pedal interceptor only. | Stalk is **buttons for OP UI only**. |
| **Do not use `SAFETY_TESLA`.** Wrong buses, wrong IDs, angle EPS. | |
| **Do not use `SAFETY_ALLOUTPUT` on the road.** Bench/cabana only, `#ifdef ALLOW_DEBUG`. | comma policy + your feet. |
| **Panda `controls_allowed` for long must clear on: brake press, cancel, interceptor timeout, CRC fail, driver gas override (policy), steer fault.** Lateral (MADS) may remain if panda lat flag set — **only after** dedicated safety tests. | |
| **comma.ai / sunnypilot safety policy:** no “keep long through brake”, no auto-reengage long, no DM nerf. Custom safety mode stays **private fork**. Do not PR interceptor+stepper as if it were stock ACC. | |
| **ISO 15622-ish jerk:** start `accel` clip **+1.5 / −1.5 m/s²**, jerk ±2.5 m/s³ until logs prove Tesla DU mapping. Tesla stock OP uses +2.0 / −3.48 m/s² — **too much decel to claim** without brakes. | |
| **RDW:** this is an aftermarket L2 experiment. Driver is the system. | |

**Operating modes (product):**

| Mode | Steering | Speed | When |
|---|---|---|---|
| MADS / steer-only | servo | human + one-pedal | default daily |
| Limited long | servo | interceptor gas/regen only, **vEgo ≥ 30 km/h**, no full stop | highway follow, driver covering brake |
| Full OP long to 0 | — | — | **out of scope** until a brake actuator exists |

---

## 1. Architecture

```
BMW PT-CAN 500 kbit  ──CAN1──►  Teensy gateway  ◄──CAN2──  can2_powertrain 500 kbit
  0x0C4 SAS, 0x0CE wheels,                         VCU, Tesla DU, T2C, Superman,
  0x194 stalk, 0x1D6 buttons,                      Pedal-interceptor, (StepperServoCAN)
  0x19E DSC, 0x1A0 speed (MUTATED)                 CRUISE_Gas 0x200, cut 0x201, status 0x199
        │                                                │
        │  gateway: all CAN1→CAN2 except 0x1A0           │
        │  0x1A0 rewritten every 5 ms, min 10 km/h       │
        │  all CAN2→CAN1 forwarded                       │
        ▼                                                ▼
                              comma four panda
                         bus0 = can2_powertrain (actuators)
                         bus1 = optional extra (do not need K-CAN for v1)
```

**Tesla Model 3 in upstream OP:** HW3/HW4, Party CAN, **angle** EPS, stock long via AP (`LONG_CONTROL` flag). **Useless as a copy-paste** for this conversion. Useful only as “regen EVs exist; they still have friction brake in panda.”

**Your long path is comma-pedal-class, not Tesla-class:**

1. OP computes `actuators.accel` (m/s²).
2. Map to Tesla pedal percent 0–100.
3. TX `CRUISE_Gas` 0x200 @ ≥ 20 Hz (firmware timeout **100 ms**).
4. Interceptor does `out = max(bmw_pedal, can_gas)` unless cut.
5. DAC to Tesla DU analog pedal. 0 % ≈ Tesla unpressed = **one-pedal regen**. 100 % = full.

---

## 2. Inventory of sources (what to steal vs ignore)

| Source | Use | Do not |
|---|---|---|
| sunnypilot/sunnypilot + vendored sunnypilot/opendbc | Base. Brand port lives in `opendbc/car/<brand>/`. MADS in `sunnypilot/mads/`. | Don’t fork commaai/openpilot as primary. |
| BMW-E8x-E9x/openpilot `selfdrive/car/bmw/` | **Reference only:** stalk parse, SAS 0xC4, wheels 0xCE, stepper TX helper, flags `STEPPER_SERVO_CAN`. Old layout (pre-opendbc-car). | Don’t copy `carcontroller` cruise-stalk **actuation**. Don’t copy `values.py` mass (ICE). |
| BMW-E8x-E9x/opendbc `opendbc/dbc/bmw_e9x_e8x.dbc` + `ocelot_controls.dbc` | BMW PT parse + servo DBC. | Merge with Wim DBC; don’t replace. |
| dzid26/StepperServoCAN + hardware | Firmware, NEMA23 notes, IDs 0x22E/0x22F, modes, 10 ms. | Recalibrate `motor_rated_hold_torque` after fw **0.4.3** (`/√2`). |
| Wim426F/Pedal-interceptor `pedal_arduino.ino` | **Ground truth** for 0x200/0x201/0x199. | Attached DBC disagrees on 0x201 — see §3. |
| Wim426F/CAN-gateway `src/main.cpp` | Forwarding + **0x1A0 min 10 km/h trap**. | Do not use 0x1A0 for OP speed. |
| Attached `BMW_can2_powertrain.dbc` | VCU/PCS/Superman/T2C/pedal. **Needs cleanup** (dup BO_ 409/512/513, broken BO_ 932, dup BO_ 690). | Don’t feed dirty DBC into cantools until cleaned. |
| comma Toyota interceptor | Pattern: enable bit, counter, CRC8 poly 0xD5, panda rx 0x201… **IDs only, not layout.** | |
| opendbc port guide / sunnypilot brand port | File list for a new brand. | |
| MADS docs | Independent lat/long; MAIN vs LKA button; panda lat flags. | |

**Old E8x constants (reference):**

- `STEER_MAX = 12 Nm` (stepper command scale, DBC ±16 Nm).
- `STEER_DELTA_UP = 0.10 Nm / 10 ms` → 10 Nm/s.
- `STEER_STEP = 1` → 100 Hz (servo wants **10 ms**).
- `STEPPER_SERVO_CAN = 1<<0`.
- E90 wheelbase 2.76 m, steerRatio 16 (hydraulic EPS era — **re-measure** with column stepper + gearbox).
- Cruise options `$540/$544/$541` — **not used for actuation** here.

---

## 3. Protocol bible (implement exactly)

### 3.1 Pedal interceptor — **firmware is law**

Source: `pedal_arduino.ino`. Bus: can2, 500 kbit.

**0x200 `CRUISE_Gas` (decimal 512) — OP → pedal, 4 bytes, ≥ 20 Hz (timeout 100 ms)**

| Byte | Field | Value |
|---|---|---|
| 0 | GasReq raw | 0–255 = 0–100 %. DBC factor `100/255 = 0.39215686` |
| 1 | GasEnable | **`0xA5` (165)** enable; anything else = ignore CAN gas |
| 2 | GasCounter | **low nibble** must increment every frame (0–15) |
| 3 | CRC8 | poly **0xD5**, init **0xFF**, over bytes 0..2 |

CRC (same as comma pedal):

```c
uint8_t crc = 0xFF;
for (i = 0; i < len; i++) {
  crc ^= dat[i];
  for (j = 0; j < 8; j++)
    crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
}
```

**Semantics:** interceptor `cmd = max(physical_bmw_pedal, can_norm)`. CAN **cannot reduce** throttle below the human pedal. Foot off → OP owns 0–100 % Tesla pedal. 0 % = Tesla unpressed voltages (397 mV / 800 mV) = **regen**.

**0x201 `VEHICLE_TorqueCut` (decimal 513) — 3 bytes, timeout 200 ms**

**BLOCKER: DBC vs firmware disagree. Resolve before panda TX of 0x201.**

| Source | Meaning of byte 0 |
|---|---|
| Attached DBC `VAL_ TorqueCut 90 "CUT"` | magic **90** |
| `pedal_arduino.ino` | **bitfield:** bit0 `REGEN_CUT` (ignored), bit1 `0x02` `ALL_TORQUE_CUT` → force 0 % |

Firmware cut: `dat[0] & 0x02`, counter nibble in byte 1, CRC over bytes 0..1, poly 0xD5.

**Recommendation:** treat firmware as truth. OP/panda **do not need to TX 0x201 in v1** (VCU already can). If OP later sends cut on brake, match **firmware bits**, then fix DBC.

**0x199 `PEDAL_Status` (decimal 409) — pedal → OP, 3 bytes, ~10 Hz**

| Byte | Field |
|---|---|
| 0 | PedalInPct 0–100 (BMW pedal) |
| 1 | PedalOutPct 0–100 (after max/cut) |
| 2 | State: 0 PASSTHROUGH, 1 THROTTLEGATING (cut/fault), 2 DRIVEBYWIRE |

**Analog (already in interceptor, OP does not DAC):** BMW 665–3688 mV / 315–1834 mV → Tesla 397–1900 / 800–3820 mV.

### 3.2 StepperServoCAN

ocelot_controls.dbc (RetroPilot Ocelot-steering-dev)

**0x22E (558) `STEERING_COMMAND` 5 bytes, 10 ms, panda TX**

```
STEER_TORQUE  : 32|8@1- (0.125,0)  [-16|15.875] Nm
STEER_ANGLE   : 16|16@1- (0.125,0) [-4096|4095.875] deg
STEER_MODE    : 12|2@1+  (1,0)     0 Off, 1 TorqueControl, 2 AngleControl, 3 SoftOff
COUNTER       : 8|4@1+
CHECKSUM      : 0|8@1+   sum-fold: init=addr, add bytes, (s&0xFF)+(s>>8)
```

Old helper (port to `bmwcan.py` / `e90can.py`):

```python
# BMW-E8x bmwcan.create_steer_command — bus was 2; we use panda bus 0 if servo is on can2
values = {COUNTER: frame % 0xF, STEER_MODE: mode, STEER_ANGLE: 0, STEER_TORQUE: tq_nm}
# checksum = calc_checksum_8bit(dat, addr) after first pack
```

v1: **TorqueControl only** (mode 1). SoftOff (3) on disable. Off (0) on fault/cancel/over-torque.

**0x22F (559) `STEERING_STATUS` 8 bytes, 10 ms, RX**

Angle, speed, torque, CONTROL_STATUS, TEMPERATURE, DEBUG_STATES, COUNTER, CHECKSUM.

**NEMA23:** hardware README — example motors Nema23 104 mm / 56 mm, dense windings; 3.3 A peak, 32 V; above ~2 A torque tapers. Set `firmware/src/BSP/actuator_config.c`: `motor_rated_current`, `motor_rated_hold_torque` (N·cm), `motor_gearbox_ratio`, `final_drive_ratio`. **fw 0.4.3:** hold torque / √2 — retune OP `STEER_MAX` after flashing.

IDs offset by solder jumpers ×3 if collision. Default 0x22E/0x22F must not clash with Tesla DU `0x108` (263) — they don’t.

### 3.3 BMW PT messages (gateway CAN1→CAN2, **unmodified except 0x1A0**)

From `bmw_e9x_e8x.dbc` (decimal IDs):

| ID | Name | Use in OP |
|---|---|---|
| **206 (0xCE)** | `WheelSpeeds` FL/FR/RL/RR, 0.0625 km/h/bit | **vEgo** (average driven axle). **Do not use 0x1A0.** |
| **196 (0xC4)** | `SteeringWheelAngle` 0.04395 deg/bit | SAS for model; compare to 0x22F |
| **404 (0x194)** | `CruiseControlStalk` plus1, plus5, minus1, minus5, cancel, resume, cancel_lever_up, Counter, Checksum | **Engage/set/cancel UI only.** Never TX this in v1. |
| **470 (0x1D6)** | `SteeringButtons` vol/seek/voice | optional MADS extra |
| **168 (0xA8)** | VCU `SendAbsDscMessages` | **brake_pressed.** Byte 7 = `0x64` pressed, `0x04` released. Period ~10 ms. Source: [Stm32-vcu BMW_E90.cpp](https://github.com/Wim426F/Stm32-vcu/blob/master/src/BMW_E90.cpp). Do **not** use 0x19E or ICE `EngineAndBrake` 0x168. |
| **0x3FE** | parking brake | byte0 == 2 → engaged. Optional `parkingBrake`. |
| **414 (0x19E)** | stock DSC | ignore for OP; VCU does not emit it. |
| **416 (0x1A0)** | `Speed.VehicleSpeed` 0.103 km/h/bit | **POISON on CAN2:** gateway forces **≥ 10 km/h**. Ignore for OP. |

**Gateway 0x1A0 (must not fight):**

- CAN1 RX 0x1A0 stored, **not** forwarded immediately.
- Every **5 ms** TX to CAN2 with 12-bit speed field **clamped to min 10 km/h**.
- If comma later needs true standstill on VCU, change gateway in a **dedicated ticket** — not in the OP port.

### 3.4 Power / gear on can2 (Wim DBC)

| ID | Signal | Use |
|---|---|---|
| 263 (0x107) | `DIR_axleSpeed`, `DIR_torqueActual`, `DIR_torqueCommand` | power meter later; speed cross-check |
| 1686 | `MaxDischargePower`, `MaxRegenPower` | clip OP accel to pack limits |
| 1021 | `Direction` Park/R/N/D | gear; OP disable in R/P |
| 409 / 512 / 513 | pedal | §3.1 |
| 1761 | Superman faults | optional mute HVAC, not OP |

**Pack power for cluster (separate from OP):**
`fake_rpm = ((power_kW + 100) / 300) * 7000` with power = `V_pack * I_pack` from MCU mux 0x293 m2 (659). Not blocking for sunnypilot.

---

## 4. ID collision checklist (can2)

| ID | Who | Action |
|---|---|---|
| 0x200 | Pedal gas **and** BMW `CruiseControlStatus` (gateway comment MB21 0x200) | **Verify on cabana.** If BMW 0x200 still appears on CAN2, interceptor and OP will fight. Gateway must **block BMW 0x200** or pedal/OP must move IDs. **Do this before any OP TX.** |
| 0x201 | Pedal cut vs Toyota GAS_SENSOR | unique on this bus if Toyota not present — OK |
| 0x22E/0x22F | servo | OK |
| 512 decimal | DBC `CRUISE_Gas` = 0x200 | same |

---

## 5. Target tree (current sunnypilot / opendbc)

Brand name: `bmw` (or `bmw_e90ev` if you want zero clash with a future OEM BMW port). Prefer `bmw`.

```
opendbc/dbc/bmw_e9x_e8x.dbc              # from E8x opendbc, as-is
opendbc/dbc/ocelot_controls.dbc          # servo
opendbc/dbc/bmw_can2_powertrain.dbc      # CLEANED Wim DBC
opendbc/car/bmw/
  __init__.py
  values.py
  fingerprints.py
  bmwcan.py
  carstate.py
  carcontroller.py
  interface.py
  radar_interface.py                     # dummy, no radar
opendbc/safety/modes/bmw.h               # NEW
opendbc/safety/tests/test_bmw.py         # NEW
opendbc/safety/safety.h                  # register hooks
opendbc/safety/safety_declarations.h     # SAFETY_BMW enum
sunnypilot/mads/                         # hook cruise MAIN + remain-active
```

Register platform in `opendbc/car/car_helpers.py` / brand import list (follow whatever master uses for a small brand, e.g. `body` or `mazda`).

**Fingerprint v1:** static fingerprint of live can2 (set of addresses). Include 0x199, 0xCE, 0xC4, 0x194, 0x107. No ECU UDS required.

**CarSpecs:** mass=1570, wheelbase=2.76, steerRatio=**start 16, measure**, centerToFrontRatio=0.44 (RWD).
`minSteerSpeed = 0` for MADS parking-lot steer only after panda tests; start **minSteerSpeed = 10 km/h**.
`openpilotLongitudinalControl = True` only behind a flag `E90Flags.PEDAL_INTERCEPTOR`.
`minEnableSpeed = 8.3 m/s` (30 km/h) until phase 5.
`safetyConfigs` = custom `SAFETY_BMW`.

Panda alternativeExperience: MADS lat-without-long bit as sunnypilot already sets for other cars. Copy from current sunnypilot hyundai/toyota MADS wiring, do not invent.

---

## 6. Phased TODOs (local agents)

Each phase has **exit criteria**. Do not start the next phase without them.

### Phase 0 — Repo + hygiene (1 sitting)

- [ ] Confirm local clone tracks `sunnypilot/sunnypilot` `master` + your fork remote.
- [ ] Vendored opendbc submodule: `opendbc_repo`. All car code goes **there**, then bump submodule in sunnypilot.
- [ ] Add `AGENTS.md` at fork root with: regen-only, no 0x1A0, 0x200 format, MADS default.
- [ ] Copy cleaned DBC into `opendbc/dbc/bmw_can2_powertrain.dbc`.
- [ ] **Clean DBC:** remove duplicate `BO_ 409/512/513` blocks; fix truncated `BO_ 932`; resolve duplicate `BO_ 690`; align 0x201 with firmware bits; comment units metric.
- [ ] Copy `bmw_e9x_e8x.dbc` + `ocelot_controls.dbc`.
- [ ] `cantools convert` / opendbc generator so packer names exist.

**Exit:** `pytest opendbc/can` (or project equivalent) parses all three DBCs.

### Phase 1 — Listen-only on the car (no panda TX)

**Hardware:** comma four on **can2 only**, panda `SAFETY_SILENT` / `SAFETY_NOOUTPUT`. Custom harness: panda CAN0 to gateway CAN2. Do **not** splice PT-CAN independently until logs prove 0xCE/0x194 present on can2.

- [ ] Cabana route: 10 min mixed drive, park, D, R, stalk +/−, brake, full regen, full accel.
- [ ] Confirm RX on panda bus0: `0x199, 0xCE, 0xC4, 0x194, 0x107, 0x22F` (if servo already powered), `0x1A0` (should never show <10 km/h).
- [ ] Plot: `0x1A0` vs `0xCE` vs `DIR_axleSpeed`. Document the 10 km/h floor.
- [ ] **Collision test:** is BMW `0x200` on can2? If yes → **gateway filter ticket** before Phase 4.
- [ ] Decode 0x194 bits vs physical stalk (plus1 vs plus5). Write a 10-line table in the repo.
- [ ] Decode brake: which of 0x19E `BrakeStates` / pressure actually tracks the pedal (ICE DME 0x168 may be missing).
- [ ] Pedal: 0x199 PedalInPct vs foot; state 0 at idle.

**Exit:** a checked-in `docs/can_map.md` with bus, ID, rate, signal used, and 0x200 collision result.

### Phase 2 — Safety firmware (bench, wheels off or car in air)

- [ ] New `opendbc/safety/modes/bmw.h` modeled on Toyota interceptor **structure**, not IDs:
  - TX allowlist: `{0x22E, bus 0, len 5}`, `{0x200, bus 0, len 4}`. **No 0x194. No DSC. No 0x201 unless firmware-aligned and required.**
  - RX checks: 0x199 @ ~10 Hz, 0xCE @ ~50 Hz, 0xC4 @ ~100 Hz, 0x22F @ 100 Hz when lat enabled, 0x194 @ ~5–25 Hz.
  - Checksum 0x200 = CRC8 D5; counter increment.
  - Checksum 0x22E = bmwcan 8-bit sum.
  - `gas_pressed` iff PedalInPct > **20** (tune) **and** PedalInPct > PedalOutPct + 5 (human override).
  - `brake_pressed` iff CAN `0x0A8` byte 7 == `0x64`. Rising edge → `controls_allowed = 0` for **long**. Lat per MADS param. Do not TX 0x201.
  - Torque limits: max **12 Nm** cmd, rate up **10 Nm/s**, rt_delta conservative (copy E8x then tighten). Driver torque: if `|0x22F torque|` vs cmd disagree > limit → disable.
  - Max gas raw 255; **enable byte must be 0xA5** or TX rejected.
  - If 0x199 missing > 200 ms → drop long TX. If 0x22F missing > 50 ms → SoftOff then Off.
- [ ] `test_bmw.py` in `opendbc/safety/tests/` covering: spam torque, checksum fail, counter stall, gas without enable, brake disengage, 0x194 not injectable.
- [ ] Register `SAFETY_BMW` in declarations + `safety_hook_registry`. Pick unused enum; **do not reuse TESLA/BODY.**

**Exit:** `pytest opendbc/safety/tests/test_bmw.py` green. Flash panda from this tree. Cabana: confirm **TX blocked** in silent, allowed only in BMW mode after fake cruise engage sequence.

### Phase 3 — carstate + interface (still no actuation)

- [ ] `carstate.py` parsers (PT DBC + powertrain DBC + ocelot):
  - vEgo from `Wheel_RL/RR` average, 0.0625 km/h → m/s. Kalman as stock.
  - standstill: vEgo < 0.3 m/s (will work; 0x1A0 will not).
  - steeringAngleDeg from 0xC4; steeringTorque from 0x22F.
  - gear from 0x3FD Direction.
  - gas = PedalInPct/100; gasPressed threshold.
  - brakePressed from `0x0A8` byte7 == `0x64` (VCU `din_brake`).
  - cruise buttons from 0x194, **VO540 / NORMAL_CRUISE_CONTROL**: `plus1`/`minus1` speed, `resume`/`cancel`/`cancel_lever_up`. Parse `plus5`/`minus5` if they ever appear; do not require them. Never TX 0x194.
  - `cruiseState.available` = ignition on + gear D. `enabled` = OP state machine, **not** stock DCC (likely dead).
- [ ] `interface.py`: CarInterface, no radar.
- [ ] Dummy `carcontroller.py` that **sends nothing**.
- [ ] Fingerprint + `values.py` platform `BMW_E90_EV`.
- [ ] Process replay on Phase 1 route: CarState matches cabana.

**Exit:** on-device, UI shows speed, gear, angle, pedal %. No CAN TX except panda default.

### Phase 4 — Lateral only (MADS)

- [ ] Mechanical: NEMA23 + StepperServoCAN on column, CAN on can2, 8–32 V, calibrate encoder (blue LED). Set current/torque/gearing in firmware. Confirm 0x22F live.
- [ ] `bmwcan.create_steer_command(mode, torque_nm, frame)` + checksum.
- [ ] `carcontroller.py`: 100 Hz, mode 1 when `CC.latActive`, else mode 3 then 0. Scale `actuators.steer * STEER_MAX`. Rate limit.
- [ ] Start `STEER_MAX = 6 Nm` (half), then 12.
- [ ] MADS: MAIN/cancel_lever or stalk resume toggles **lat only**. Long flag off. `mads` remain-active on brake.
- [ ] panda: lat controls_allowed without long.
- [ ] Tests: parking lot 10 km/h, then 50, then 100. Driver override (hand on wheel) must win. SoftOff on panda heartbeat loss.
- [ ] Tune `lateralTuning` PID/torque; measure steerRatio from SAS vs wheel.

**Exit:** 30 min highway **steer-only**, no long TX (grep logs: zero 0x200). No panda rejects. Written torque plot.

### Phase 5 — Longitudinal mapping (interceptor, high speed only)

- [ ] Confirm interceptor DRIVEBYWIRE with a **USB/Teensy test sender** (`sender_example.md`) **before** OP: 0x200 enable 0xA5, counter++, CRC, 20 % gas, wheels off or stands.
- [ ] Map `actuators.accel` → gas percent. **First version: feedforward only, no braking beyond 0 %.**

Proposed v1 map (metric, clip later with 1686 limits):

```
# a in m/s², v in m/s
# 0 % pedal ≈ max regen (measure a_regen(v) on a straight, foot off, no OP)
# 100 % ≈ ~+3 to +5 m/s² (measure)
gas = interpolate(a, [a_regen(v), 0, a_max(v)], [0, gas_coast, 100])
```

Measure `a_regen(v)` and `a_max(v)` in logs (DIR_torqueActual, PackCurrent). Do **not** assume Tesla OP −3.48 m/s².

- [ ] `carcontroller.py` TX 0x200 at 50 Hz (every 2nd 100 Hz loop) when `CC.longActive`; when not: **stop sending** (timeout → passthrough). Never send enable without controls_allowed.
- [ ] Clip gas so `PedalOut` cannot exceed human+margin when override.
- [ ] `minEnableSpeed = 30 km/h`; disengage long below 25 km/h with chime. **No stopping behind leads — alert, driver brakes.**
- [ ] Accel limits: `ACCEL_MAX = 1.5`, `ACCEL_MIN = -1.5` m/s² v1.
- [ ] Panda: long allowed only if lat safety OK optional; **brake → long off**.
- [ ] Tune with `longitudinalManeuvers` **only above 40 km/h**, empty road.

**Exit:** lead follow 80–120 km/h, cut-in, driver override gas and brake. Never reaches 0 km/h under OP. Log PedalIn vs PedalOut vs GasReq.

### Phase 6 — MADS product + polish

- [ ] Default: lat on with MAIN, long opt-in (sunnypilot alpha long toggle analogue).
- [ ] Stalk: set/resume = set cruise speed; +/− = 1/5 km/h; cancel = long off, lat per MADS setting.
- [ ] UI: “regen only — cover brake”.
- [ ] Optional: TX 0x201 bit1 on hard OP disable if firmware matches.
- [ ] Optional: clip GasReq using `MaxDischargePower`/`MaxRegenPower` (1686).
- [ ] Do **not** enable experimental long stopping.

### Phase 7 — Explicitly later / out of scope

- [ ] Hydraulic or iBooster / DSC torque for true AEB.
- [ ] Radar (none). Vision-only long.
- [ ] Stock Tesla AP.
- [ ] Gateway 0x1A0 true-zero (needed for some DU/EPS, **not** for OP if 0xCE used).
- [ ] Cluster RPM→kW faceplate (separate).
- [ ] Upstream PR to sunnypilot (policy: custom actuator + no brakes = likely never official).

---

## 7. Hardware BOM (lateral)

- StepperServoCAN PCB + TLE5012, firmware ≥ 0.4.3.
- NEMA23 (56 or 104 mm), current ≤ rated; gearbox ~5:1 if needed; **measure backlash**.
- Column coupling: follow BMW-E8x wiki / dzid26 mechanical notes.
- CAN: same 500 kbit can2, terminate 120 Ω if the servo is at a stub.
- comma four: **custom** harness, **not** Tesla A/B and **not** stock BMW OBD-only. Panda CAN0 = can2. Power from always-on + ignition sense.
- Pedal interceptor already in car.
- Cruise stalk: keep physical BMW stalk; parse 0x194.

---

## 8. Agent work split (parallel after Phase 0)

| Agent | Owns | Must not |
|---|---|---|
| A DBC | Clean powertrain DBC, merge packer tests | Change firmware IDs |
| B Safety | `bmw.h` + `test_bmw.py` + enum | Enable ALLOUTPUT in release |
| C State | carstate, fingerprint, values | TX |
| D Lat | bmwcan steer, carcontroller lat, servo config | 0x200 |
| E Long | accel→gas map, 0x200 TX, limits | stalk spoof, stop-and-go |
| F MADS | button mapping, remain-active, UI string | panda long-through-brake |
| G Gateway | 0x200 collision filter if needed; **do not** touch 0x1A0 unless DU requires | OP code |

---

## 9. First commands on the machine (human)

```bash
cd /path/to/sunnypilot
git remote -v   # origin = Wim426F/sunnypilot, upstream = sunnypilot/sunnypilot
git submodule update --init --recursive
# work in opendbc_repo for car/safety, then commit submodule pin

# never:
# SAFETY_ALLOUTPUT on public roads
# minEnableSpeed = -1
# TX 0x194
```

Cabana: `tools/cabana` on a route, filters `199,200,201,206,196,404,558,559,107`.

---

## 10. Resource URLs

- https://github.com/Wim426F/sunnypilot.git
- https://github.com/Wim426F/Pedal-interceptor/blob/main/pedal_arduino.ino
- https://github.com/Wim426F/CAN-gateway/blob/main/src/main.cpp
- https://github.com/BMW-E8x-E9x/openpilot (legacy `selfdrive/car/bmw/*`)
- https://github.com/BMW-E8x-E9x/opendbc/blob/master/opendbc/dbc/bmw_e9x_e8x.dbc
- https://github.com/BMW-E8x-E9x/openpilot/wiki
- https://github.com/dzid26/StepperServoCAN
- https://github.com/dzid26/StepperServo-hardware
- https://github.com/RetroPilot/opendbc/blob/Ocelot-steering-dev/ocelot_controls.dbc
- https://github.com/commaai/opendbc#how-to-port-a-car
- https://github.com/sunnypilot/opendbc/blob/master/opendbc/safety/modes/toyota.h
- https://github.com/sunnypilot/user-docs/blob/master/docs/technical/gas-interceptor.md

---

## 11. Closed decisions (2026-09-06) — agents treat as spec

| # | Question | Decision | Agent action |
|---|---|---|---|
| 1 | BMW 0x200 on can2? | **ID is free.** VCU does not TX 0x200. Pedal `CRUISE_Gas` owns 0x200. | TX 0x200 only as interceptor gas. Still **log once** in Phase 1 that no other 0x200 exists; if a stranger 0x200 appears, stop. |
| 2 | 0x201 cut encoding? | **Not OP’s message.** Another device torque-cuts. Do not TX, do not checksum-enforce as OP actuator. | Panda TX allowlist: **0x200 + 0x22E only.** RX 0x201 optional ignore. |
| 3 | brake_pressed without DME? | VCU [BMW_E90.cpp](https://github.com/Wim426F/Stm32-vcu/blob/master/src/BMW_E90.cpp) `SendAbsDscMessages(din_brake)` → **0x0A8** 8 bytes, ~10 ms. Byte 7 **`0x64` = pressed, `0x04` = released**. Park brake **0x3FE** byte0 == 2. | `carstate` + panda: `brake_pressed = (0x0A8 dat[7] == 0x64)`. Do not use 0x19E / 0x168. |
| 4 | Stalk $540 vs $544? | **Normal CC / VO540.** Speed up/down on stalk. | Flag `NORMAL_CRUISE_CONTROL`. Parse 0x194 `plus1` `minus1` `resume` `cancel` `cancel_lever_up`. plus5/minus5 optional. **Never spoof stalk.** |
| 5 | 0x22E free? | **Yes. Servo installed and ready.** | Phase 4 is hardware-ready. TX 0x22E / RX 0x22F on can2. |
| 6 | a_regen(v), a_max(v)? | **Not a fixed table.** 0 % interceptor = current DU regen (SOC/temp/speed/your setting). Neighborhood log ~−0.4 m/s² foot-off is observation only. | LongControl **I-term on aEgo**. Pedal scale monotonic 0–100 %. Clip +1.5/−1.5. Saturated 0 % = cannot regen more (no friction brake). |
| 7 | Harness? | **One panda CAN.** Panda CAN0 = SavvyCAN **bus 0** = gateway CAN2 / VCU vehicle / BMW PT after GW (wheels, SAS, stalk, `0x0A8`, gear). **Do not** splice Tesla/PCS (log bus 1): collisions `0x1D6`/`0x2B2`/`0x398`. Pedal `0x199`/`0x200` and servo `0x22E`/`0x22F` are **moved onto bus 0** (human). | `CanBus.pt = 0`. Three DBC parsers, one hardware bus. |

**Accel assumptions (1570 kg, ~980 DU, gauge ±100 / +200 kW) — FLAG MEASURE:**

At speed \(v\) (m/s), rough \(a \approx P / (m v)\):

- \(P_\mathrm{dis,max} \approx 200\,\mathrm{kW}\) → \(a_\mathrm{max}(30\,\mathrm{m/s}) \approx 4.2\,\mathrm{m/s^2}\); OP v1 clip **+1.5 m/s²**
- \(P_\mathrm{regen,max} \approx 100\,\mathrm{kW}\) → \(a_\mathrm{regen}(30\,\mathrm{m/s}) \approx -2.1\,\mathrm{m/s^2}\); OP v1 clip **−1.5 m/s²**
- Standstill / low \(v\): torque-limited, do not claim these numbers below 30 km/h (long disabled there anyway)

Phase 5 test ticket: log `DIR_torqueActual`, pack V/I, vEgo, PedalOutPct on a straight: foot-off regen sweep + full-pedal accel sweep. Replace assumptions.

**Phase 1 is no longer blocked on Q1/Q2.** Remaining Phase 1 work is listen-only confirmation (0x0A8 byte7 tracks foot, 0x194 bits match VO540, 0x22F alive, no unexpected 0x200). Then Phase 2 safety + Phase 4 lat may proceed.
