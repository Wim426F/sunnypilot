# E90 EV — Phase 1 listen-only CAN map

Human-on-car checklist. This machine cannot cabana the vehicle. Tick boxes from a comma four on **can2 only**. Spec: `E90-SUNNYPILOT-PORT-PLAN.md` §3, §4, §6 Phase 1, §11. Safety: `AGENTS.md`.

**Phase 1 TX = none.** Panda must stay `SAFETY_SILENT` (0) or `SAFETY_NOOUTPUT` (19). Do not use `SAFETY_TESLA` or `SAFETY_ALLOUTPUT`. Do not start a car-port `controlsd` that would set `SAFETY_BMW`.

Closed decisions that this log must confirm, not reopen:

| # | Decision | Phase 1 check |
|---|---|---|
| 1 | 0x200 ID is free. VCU does not TX it. Pedal `CRUISE_Gas` owns it. | Log once: **no other 0x200** on can2. Stranger 0x200 → **stop**. |
| 2 | 0x201 is **not OP**. Another device torque-cuts. | RX optional. **Never TX.** |
| 3 | Brake = VCU `0x0A8` byte7 `0x64` / `0x04`. | Foot tracks byte7. Do **not** use `0x19E` / ICE `EngineAndBrake`. |
| 4 | Stalk = VO540 / NORMAL_CC. UI only. | Fill stalk table. **Never TX 0x194.** |
| 5 | 0x22E free. Servo installed. | 0x22F alive if servo powered. **No 0x22E TX** in Phase 1. |
| 7 | Single bus: panda CAN0 = can2_powertrain. | All rows below are **bus 0**. Do not splice PT-CAN until 0xCE + 0x194 are proven here. |

---

## 0. Hardware / panda (do this first)

- [ ] Custom harness: **panda CAN0 ↔ Teensy gateway CAN2** only. 500 kbit. No isolated servo bus. No PT-CAN splice.
- [ ] Panda boot safety is silent. Confirm `pandaStates[0].safetyModel` is `silent` (0) or `noOutput` (19). **Not** `tesla` (10), **not** `allOutput` (17).
- [ ] Cabana / loggerd / dashcam-style record only. Do not enable openpilot longitudinal or lateral.
- [ ] After the route: grep / cabana TX: **zero** frames from panda on 0x22E, 0x200, 0x194, 0x201.

Record **≥ 10 min** mixed: park, D, R, stalk +/− / resume / cancel, brake on/off, full regen (foot off), full accel, standstill, >30 km/h.

---

## 1. Cabana filters

Cabana **Address** filter is **hex** (`messageswidget.cc` `parseRange(..., 16)`). Plan §9 string `199,200,201,206,196,404,558,559,107` mixes hex pedal/T2C IDs with **decimal** BMW/servo IDs (`206` = 0xCE, `404` = 0x194, `558` = 0x22E). **Do not paste that string into Address.**

**Use this Address filter (hex, comma-separated or one-at-a-time):**

```
199, 200, 201, CE, C4, 194, 1D6, A8, 19E, 1A0, 22E, 22F, 107, 3FD, 3FE, 696
```

Plan §9 names the same set as decimal `199,200,201,206,196,404,558,559,107` plus the extra IDs above. Source/bus filter: `0`.

Load DBC when present: `bmw_e9x_e8x.dbc` (BMW PT), `ocelot_controls.dbc` (servo), `bmw_can2_powertrain.dbc` (pedal + T2C + VCU). The last two may still be missing in-tree (Phase 0 DBC). Firmware / this table is law until they land.

---

## 2. Tick list (Phase 1 exit)

Fingerprint v1 = set of live addresses on bus 0. Must include **0x199, 0xCE, 0xC4, 0x194, 0x107**.

### Present on panda bus 0

- [ ] `0x199` pedal status (~10 Hz, 3 bytes). Idle: `PedalInPct` ≈ 0, state **0 PASSTHROUGH**.
- [ ] `0xCE` wheel speeds (~50 Hz). `Wheel_RL` / `Wheel_RR` move from true 0.
- [ ] `0xC4` SAS (~100 Hz). Angle tracks wheel; compare later to `0x22F`.
- [ ] `0x194` stalk (event, ~5–25 Hz). See §5 table. **UI only — never TX.**
- [ ] `0x107` T2C `DIR_*` (Tesla 0x108 remapped). Speed cross-check only — **not vEgo**.
- [ ] `0x22F` servo status @ 100 Hz **if servo powered**. Closed: hardware ready.
- [ ] `0x1A0` present and **never < ~10 km/h** (POISON — do not use for vEgo).
- [ ] `0x0A8` VCU ABS/DSC emu @ ~10 ms. Byte7 tracks foot (see §6).
- [ ] `0x3FD` gear from F30 lever @ ~10 Hz.

### Collision / poison / do-not-TX

- [ ] **0x200 collision test (mandatory even though ID is closed as free):**
  - [ ] **Pass:** no 0x200 on bus 0 for the whole route.
  - [ ] **Fail — stop:** any 0x200 appears (8-byte BMW `CruiseControlStatus`, 4-byte interceptor, or other). Note DLC + first bytes. Gateway filter ticket before any OP TX.
- [ ] `0x201` if present: VCU torque-cut (3 bytes, ~20 Hz). **Not OP. Do not TX. Do not checksum-enforce as OP actuator.**
- [ ] `0x22E` **absent** (Phase 1 has no steer TX). If present → another sender; stop and identify.
- [ ] `0x194` RX only. Confirm panda cannot inject it in SILENT/NOOUTPUT.
- [ ] `0x19E` optional/ignore. Stock DSC; VCU does not emit it. **Not `brake_pressed`.**
- [ ] ICE DME `EngineAndBrake` layout on 0x0A8 **must not** be used (`BrakePressed` bit61 in `bmw_e9x_e8x.dbc`). Live payload is VCU `SendAbsDscMessages`.

### Plots (save screenshots / note values)

- [ ] Standstill: `0xCE` RL/RR ≈ 0, `0x1A0 VehicleSpeed` ≈ **10 km/h floor**, `0x107 DIR_axleSpeed` ≈ 0.
- [ ] Rolling 20 / 50 / 100 km/h: `0xCE` vs `0x1A0` vs `DIR_axleSpeed`. Document the floor.
- [ ] Brake: foot on/off vs `0x0A8` byte7 vs (if present) `0x19E BrakeStates` / pressure — expect only byte7 to track.
- [ ] Pedal: foot vs `0x199` PedalInPct; state stays 0 at idle with no CAN gas.

**Route id / date:** _______________  
**0x200 collision result:** pass / fail _______________  
**0x1A0 floor (measured km/h at standstill):** _______________

---

## 3. Expected CAN map (panda bus 0 = can2_powertrain)

All actuators and forwarded BMW PT live on this one bus. Gateway: CAN1→CAN2 all **except** 0x1A0 (rewritten every 5 ms, min 10 km/h). CAN2→CAN1 all forwarded.

| bus | ID hex (dec) | name | rate (expected) | DBC file | signal used by OP | notes |
|---|---|---|---|---|---|---|
| 0 | **0x199 (409)** | `PEDAL_Status` | ~10 Hz | `bmw_can2_powertrain.dbc` (fw) | `PedalInPct`, `PedalOutPct`, `State` | Pedal → OP. 3 bytes. b0 in 0–100 %, b1 out 0–100 %, b2: 0 PASSTHROUGH / 1 THROTTLEGATING / 2 DRIVEBYWIRE. Idle state **0**. RX check ~10 Hz; missing >200 ms drops long (later). |
| 0 | **0x200 (512)** | `CRUISE_Gas` | ≥20 Hz **when long active**; else **silent** | `bmw_can2_powertrain.dbc` (fw) | `GasReq`, enable `0xA5`, counter nibble, CRC8 | **OP TX interceptor only (Phase 5+).** 4 bytes. Toyota 6-byte 0x200 is a different layout — do not copy. CRC poly 0xD5 init 0xFF **forward** over bytes 0..2. 100 ms timeout → passthrough. **Phase 1: must be absent.** Closed: ID free; still log once. Stranger 0x200 → stop. BMW DME `CruiseControlStatus` is also 0x200/8 B — must not appear. |
| 0 | **0x201 (513)** | `VEHICLE_TorqueCut` | ~20 Hz (VCU 50 ms) | `bmw_can2_powertrain.dbc` (fw) | *(ignore)* | **Not OP.** VCU (`EvControlsT2C::SetTorque`) TXes 3 bytes: bit0 regen-cut ignored by pedal, bit1 `0x02` all-torque-cut, counter, CRC. DBC vs fw disagreed historically; fw is law. Panda allowlist later = `{0x22E, 0x200}` only. RX optional. |
| 0 | **0xCE (206)** | `WheelSpeeds` | ~50 Hz | `bmw_e9x_e8x.dbc` | **`Wheel_RL`, `Wheel_RR` → vEgo** | DSC, 8 B, 0.0625 km/h/bit, signed. Average **driven axle** (RWD). **Do not use 0x1A0.** Standstill = vEgo < 0.3 m/s works here. Gateway forwards unmodified. |
| 0 | **0xC4 (196)** | `SteeringWheelAngle` | ~100 Hz | `bmw_e9x_e8x.dbc` | `SteeringPosition` → `steeringAngleDeg` | SAS, 7 B. E8x DBC 0.04395 deg/bit (vendored copy still has 0.04283 — verify on car). Compare to 0x22F angle. |
| 0 | **0x194 (404)** | `CruiseControlStalk` | event ~5–25 Hz | `bmw_e9x_e8x.dbc` | `plus1`, `minus1`, `resume`, `cancel`, `cancel_lever_up` (`plus5`/`minus5` optional) | **UI only. Never TX.** VO540 / `NORMAL_CRUISE_CONTROL`. 4 B, counter + checksum. Higher rate while pressed. |
| 0 | **0x1D6 (470)** | `SteeringButtons` | event | `bmw_e9x_e8x.dbc` | vol / seek / voice (optional MADS extra) | 2 B SZL. Not required for v1. |
| 0 | **0x0A8 (168)** | VCU `SendAbsDscMessages` | ~10 ms (100 Hz) | `bmw_can2_powertrain.dbc` (fw); **not** ICE `EngineAndBrake` | **`brake_pressed` = byte7 == `0x64`** | 8 B. Byte7 **`0x64` pressed, `0x04` released**. Same *ID* as ICE DME `EngineAndBrake` (dec 168) — **different payload**. Do not parse DBC `BrakePressed` bit61. Rising edge → long `controls_allowed = 0` later. |
| 0 | **0x19E (414)** | `StatusDSC_KCAN` | ~20 Hz if DSC still talks | `bmw_e9x_e8x.dbc` | **none** | Stock DSC. VCU does not emit it. May be forwarded from PT. Ignore for OP brake. |
| 0 | **0x1A0 (416)** | `Speed` | gateway TX every **5 ms** | `bmw_e9x_e8x.dbc` | **none — POISON** | `VehicleSpeed` 12-bit, 0.103 km/h/bit. Gateway stores CAN1 0x1A0 and rewrites CAN2 with **min 10 km/h**. **Do not use for vEgo.** True-zero is a gateway ticket, not this port. |
| 0 | **0x22E (558)** | `STEERING_COMMAND` | 10 ms when lat on | `ocelot_controls.dbc` | `STEER_TORQUE`, `STEER_MODE`, `COUNTER`, `CHECKSUM` | **Panda TX later (Phase 4).** 5 B. Mode 1 TorqueControl, 3 SoftOff, 0 Off. Checksum sum-fold init=addr. Start `STEER_MAX = 6 Nm`. **Phase 1: must be absent.** |
| 0 | **0x22F (559)** | `STEERING_STATUS` | 10 ms (100 Hz) | `ocelot_controls.dbc` | `STEERING_ANGLE`, `STEERING_TORQUE`, `CONTROL_STATUS`, … | Servo RX. Missing >50 ms → SoftOff then Off (later). Default IDs; jumper offset ×3 if collision (none expected vs Tesla `0x108`). |
| 0 | **0x107 (263)** | T2C `DIR_torque` (Tesla 0x108 remapped) | ~100 Hz | `bmw_can2_powertrain.dbc` | `DIR_axleSpeed`, `DIR_torqueActual`, `DIR_torqueCommand` | Speed **cross-check** + power meter later. **Not vEgo.** T2C renamed 0x108→0x107 to miss PCS. Tesla scale axle RPM 0.1; VCU converts rotor with ×0.9 — measure, do not treat as kph. |
| 0 | **0x3FD (1021)** | F30 `Direction` | ~10 Hz | `bmw_can2_powertrain.dbc` | `Direction` Park/R/N/D | VCU `F30_Lever` TX, 5 B. Byte2: `0x20` P, `0x40` R, `0x60` N, `0x80` D, `0x81` Sport. OP disable in R/P. Not Tesla AP 0x3FD. |
| 0 | **0x3FE (1022)** | parking brake | ~? (EMF) | `bmw_can2_powertrain.dbc` / VCU RX | optional `parkingBrake` | Byte0 **== 2 → engaged**. VCU `handle3FE`. Optional. |
| 0 | **0x696 (1686)** | pack power limits | ~2 Hz (VCU 500 ms, MOD_RUN) | `bmw_can2_powertrain.dbc` | `MaxDischargePower`, `MaxRegenPower` | VCU → T2C `0x696`. uint16 BE, 0.01 kW/bit, bytes 0–1 discharge, 2–3 regen. Phase 1: log presence. Phase 5+ optional accel clip. |

Panda **future** TX allowlist (not Phase 1): `{0x22E len 5, 0x200 len 4}` on bus 0 only. No 0x194, no DSC, no 0x201.

---

## 4. 0x200 / 0x201 / 0x199 layouts (firmware is law)

**0x200 `CRUISE_Gas`** — 4 bytes, OP → pedal, timeout 100 ms:

| Byte | Field |
|---|---|
| 0 | GasReq 0–255 = 0–100 % |
| 1 | Enable **`0xA5`**; anything else ignored |
| 2 | Counter, **low nibble**, must increment |
| 3 | CRC8 poly **0xD5**, init **0xFF**, bytes 0..2 **forward** |

`out = max(bmw_pedal, can_gas)` unless cut. CAN cannot reduce below the human pedal. 0 % = Tesla unpressed = regen. When long not active: **stop sending** (timeout → passthrough).

**0x201** — 3 bytes, **VCU owns**. Pedal: `dat[0] & 0x02` = all-torque-cut → force 0 %.

**0x199** — 3 bytes, pedal → OP @ 10 Hz. State 0 at idle with foot off and no 0x200.

---

## 5. Stalk 0x194 decode (human fills)

VO540 / NORMAL_CC. Parse `plus1` / `minus1` / `resume` / `cancel` / `cancel_lever_up`. `plus5` / `minus5` optional — do not require. DBC: `bmw_e9x_e8x.dbc` `CruiseControlStalk` (E8x GitHub names; vendored copy may still say `plus1mph_request`).

Idle frame (no press): _______________ (hex, 4 bytes)

| # | physical action | plus1 | plus5 | minus1 | minus5 | cancel | resume | cancel_lever_up | raw 4 B hex | notes |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | released / idle | | | | | | | | | |
| 2 | + click (faster 1) | | | | | | | | | plus1 vs plus5 |
| 3 | + hold ~1 s | | | | | | | | | |
| 4 | + hold ~2 s | | | | | | | | | plus5 ever? |
| 5 | − click (slower 1) | | | | | | | | | |
| 6 | − hold ~1 s | | | | | | | | | |
| 7 | − hold ~2 s | | | | | | | | | minus5 ever? |
| 8 | pull toward driver | | | | | | | | | expect `resume` |
| 9 | end-button press | | | | | | | | | expect `cancel` / set |
| 10 | push lever up / away | | | | | | | | | expect `cancel_lever_up` |

---

## 6. Brake 0x0A8 byte7 (human confirms)

VCU `BMW_E90::SendAbsDscMessages(din_brake)` @ 10 ms. **Not** 0x19E, **not** ICE bit61.

| foot | expect byte7 | observed byte7 | 0x19E BrakeStates (if any) | tracks foot? |
|---|---|---|---|---|
| released | `0x04` | | | |
| pressed | `0x64` | | | |
| pressed + regen (foot off throttle) | `0x64` if brake pedal | | | |
| park brake only (0x3FE) | still `0x04` unless foot | | | 0x3FE byte0==2 separate |

---

## 7. Still unknown / not this log

- `bmw_can2_powertrain.dbc` and `ocelot_controls.dbc` may not be in the opendbc tree yet (Phase 0 DBC). Use firmware + this map.
- Vendored `bmw_e9x_e8x.dbc` SAS factor 0.04283 vs E8x/plan **0.04395** — measure 0xC4 vs 0x22F / steering wheel.
- `DIR_axleSpeed` → kph (T2C 0.1 axle RPM vs VCU ×0.9 rotor). Cross-check only.
- `a_regen(v)` / `a_max(v)`: closed as unknown; measure Phase 5. Not a Phase 1 blocker.
- 0x3FE rate; 0x19E presence; whether DSC still talks on forwarded PT.
- 0x1D6 usefulness for MADS.

Phase 1 is **not** blocked on Q1/Q2. Remaining: 0x0A8 byte7 tracks foot, 0x194 matches VO540, 0x22F alive, **no unexpected 0x200**. Then Phase 2 safety + Phase 4 lat.
