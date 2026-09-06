# E8x / StepperServoCAN reference (steal vs ignore)

Owner: docs. Production `bmwcan.py` is **car**. Safety hooks are **safety**.
Spec law: `E90-SUNNYPILOT-PORT-PLAN.md` §0, §2, §11 and `AGENTS.md`.

Fetched 2026-09-06 from public GitHub (see [Files fetched](#files-fetched)). This is **reference only** — do not vendor E8x `selfdrive/car/bmw/` into `opendbc/car/bmw/`.

| This car (E90 EV) | E8x ICE fork |
|---|---|
| Panda CAN0 = can2_powertrain. Servo + pedal + VCU + forwarded BMW PT. | PT-CAN bus 0, F-CAN bus 1, servo often bus 1 (Python) / bus 2 (panda). |
| Long = Wim `CRUISE_Gas` 0x200. No DCC. | Long = **spoof cruise stalk 0x194** into stock CC/DCC. |
| Brake = VCU 0x0A8 byte7 `0x64`. No DSC torque TX. | Brake = ICE `EngineAndBrake` 0xA8 `BrakePressed`. |
| vEgo = `Wheel_RL/RR` on 0xCE. **Never 0x1A0.** | vEgo = `Speed.VehicleSpeed` 0x1A0 (gateway poisons this here). |
| MADS lat-only default. Start `STEER_MAX = 6 Nm`. | `STEER_MAX = 12 Nm`, lat gated on `CC.enabled` (pcm cruise). |

---

## Steal vs ignore

| Item | Steal | Ignore / do not copy |
|---|---|---|
| `bmwcan.calc_checksum_8bit` / firmware `Msg_calc_checksum_8bit` | **Yes.** 0x22E TX and 0x22F RX. Same algorithm. Paste below. | DBC comment on RetroPilot `ocelot_controls.dbc` that says “CRC8 J1850” for 0x22E — **wrong**. BMW-E8x-E9x/opendbc comment is correct: “8bit sum of all bytes and message id”. |
| `bmwcan.create_steer_command` | **Yes, structure.** Pack with CHECKSUM=0, checksum, pack again. TorqueControl (1) in v1. | Bus number (`CanBus.SERVO_CAN = 1` / panda `{558, 2}`). **We TX 0x22E on panda bus 0.** Old 0.74 packer `msg[2]` for data — current opendbc is `msg[1]`. |
| `SteeringModes` Off/Torque/Angle/SoftOff | **Yes.** | AngleControl (2) in v1. |
| 0x22E / 0x22F DBC layout | **Yes.** `ocelot_controls.dbc` from BMW-E8x-E9x/opendbc (checksum comment fixed) or RetroPilot Ocelot-steering-dev (layout identical). | Ocelot **pedal** 0x200/0x201 (Toyota 6-byte). Ocelot brake 0x20E/0x20F. |
| `values.py` `STEER_STEP`, deltas, `STEPPER_SERVO_CAN` | **Yes as starting numbers**, then retune. See [STEER_MAX](#steer_max-and-rate-limits). | `STEER_MAX = 12` as first flash. ICE `CarSpecs.mass` (3300 lb). `$540/$544/$541` cruise flags for **actuation**. Fingerprint `0x200 → NORMAL_CRUISE_CONTROL` (0x200 is **our pedal**). |
| `carstate` stalk 0x194 named signals | **Yes, RX only.** plus1/minus1/plus5/minus5/resume/cancel/cancel_lever_up, `Counter_0x194`. | **Never TX 0x194.** Do not use stalk as DCC speed actuator. |
| `carstate` SAS 0xC4 | **Yes.** `SteeringPosition` / `SteeringSpeed`, 0.04395 deg/bit. | F-CAN `SteeringWheelAngle_DSC` 0xC9 unless we add F-CAN. |
| `carstate` wheels 0xCE | **Yes, and this is vEgo.** `Wheel_*` 0.0625 km/h/bit. Average **RL/RR** (RWD). | E8x `vEgoRaw = Speed.VehicleSpeed` (0x1A0). E8x `standstill` from 0x1A0 `MovingForward/Reverse`. |
| `carstate` 0x22F | **Yes.** `STEERING_TORQUE`, `STEERING_ANGLE`, `CONTROL_STATUS bit2` = SoftOff → `steerFaultTemporary`. | Gating lat on `CC.enabled` / stock cruise. |
| `carcontroller` lat path | **Pattern:** 100 Hz, `actuators.steer * STEER_MAX`, `apply_dist_to_meas_limits`, TorqueControl when lat active, SoftOff then Off. | Gate on `CC.enabled` (use `CC.latActive` for MADS). Immediate Off on brake/gas (MADS remain-active on brake). `print()` debug. |
| `carcontroller` `create_accel_command` / stalk spoof | — | **Entire function and all `cruise_cmd(...)`.** No DCC. Long = interceptor only (later phase). |
| `bmwcan.calc_checksum_cruise` (0x194, init=0) | Only if we ever **validate RX** 0x194. | Any TX of 0x194. |
| `bmwcan.calc_checksum_4bit` (0x130 TerminalStatus) | Optional later for KL15. | v1. |
| E8x `interface.py` `_get_params` | steerRatio 16 start, wheelbase 2.76, `centerToFront = wheelbase * 0.44`, torque tune kp/ki/kf scaled by `STEER_MAX`, `steerActuatorDelay = 0.4`. `0x22F in fingerprint → STEPPER_SERVO_CAN`. | `pcmCruise` / stalk long. `minEnableSpeed` tied to DCC. `openpilotLongitudinalControl = True` always. `SafetyModel.bmw` from **their** panda (stalk TX). `0x200` as CC flag. ICE transmission 0xB8/0xB5. XI 0xBC steerRatio 18.5. |
| E8x `detect_stepper_override` | Idea only (no SAS torque sensor on E-series). | Prefer `\|0x22F torque − cmd\|` for override. Do not fake `steeringPressed` from gas/voice. |
| E8x panda `safety_bmw.h` | Torque scale 0.125 Nm/bit, mode in byte1 bits 4–5, SAS 0xC4 layout, cancel bits on 0x194 **RX**. | **TX allowlist `{404,0},{404,1}` (0x194).** Speed from 0x1A0. Brake from ICE 0xA8 bits 61–62. Gear from 0x1D2. GMLAN enable pin. Master panda `{252,2}` TMCL 0xFC. `BMW_MAX_TQ_RATE` 16 Nm/10 ms at 0 m/s (do not copy; too loose). `controls_allowed` from stock CC 0x193/0x200. |
| StepperServoCAN firmware ≥ 0.4.3 | IDs, 10 ms, modes, checksum, timeout SoftOff, `actuator_config.c` current/hold/gearing. | Default NEMA17 1.3 A / 39 N·cm numbers unless that is the motor. AngleControl. |
| StepperServoCAN ocelot pedal/brake frames | — | Toyota-layout 0x200/0x201. |

---

## 8-bit sum-fold checksum (0x22E / 0x22F)

Firmware law: `dzid26/StepperServoCAN` `firmware/src/BSP/can.c` `Msg_calc_checksum_8bit`.

**Algorithm:** `checksum = msg_id`; add every payload byte; fold `(s & 0xFF) + (s >> 8)`; keep low byte.

**On TX:** pack with `CHECKSUM = 0`, compute over the **full DLC including the zero checksum byte**, write checksum, pack again.

**On RX:** copy payload, **zero byte 0**, compute, compare to original byte 0. Firmware does this for 0x22E. Same function is used when packing 0x22F.

`ocelot_controls.dbc` CHECKSUM is `0|8@1+` → **byte 0**.

### Python (paste into `bmwcan.py`)

```python
def calc_checksum_8bit(work_data: bytes | bytearray, msg_id: int) -> int:
  # E8x bmwcan.py + StepperServoCAN Msg_calc_checksum_8bit
  # work_data must have CHECKSUM byte already 0
  checksum = msg_id
  for byte in work_data:
    checksum += byte
  checksum = (checksum & 0xFF) + (checksum >> 8)
  checksum &= 0xFF
  return checksum
```

### C (paste into `opendbc/safety/modes/bmw.h`)

```c
// StepperServoCAN firmware/src/BSP/can.c — 0x22E TX check and 0x22F RX check
static uint8_t bmw_checksum_8bit(const uint8_t *data, uint8_t len, uint16_t msg_id) {
  uint16_t checksum = msg_id;
  for (uint8_t i = 0U; i < len; i++) {
    checksum += data[i];
  }
  checksum = (checksum & 0xFFU) + (checksum >> 8U);
  checksum &= 0xFFU;
  return (uint8_t)checksum;
}

static bool bmw_compute_checksum_ok(const CANPacket_t *msg) {
  // Caller: only 0x22E (len 5) or 0x22F (len 8)
  uint8_t buf[8];
  const uint8_t len = GET_LEN(msg);
  if (len > sizeof(buf)) {
    return false;
  }
  for (uint8_t i = 0U; i < len; i++) {
    buf[i] = GET_BYTE(msg, i);
  }
  const uint8_t got = buf[0];
  buf[0] = 0U;
  return bmw_checksum_8bit(buf, len, GET_ADDR(msg)) == got;
}
```

Adapt `CANPacket_t` / `GET_*` to whatever current opendbc safety uses (E8x panda still had `CAN_FIFOMailBox_TypeDef`).

### Worked examples

All-zero 0x22E (addr 558, 5 zero bytes): `558 + 0 = 558` → `(558 & 0xFF) + (558 >> 8) = 46 + 2 = 48 = 0x30`.

Firmware unit test 0x22F (addr 559): payload `{0xBE, 0x12, 0x00, 0xFF, 0x4C, 0x11, 0x00, 0x1E}`, zero byte0 → sum `559+0x12+0xFF+0x4C+0x11+0x1E = 955` → `187+3 = 190 = 0xBE`.

### Packer gotcha (current sunnypilot opendbc)

`CANPacker.make_can_msg` returns `(addr, bytes(dat), bus)`:

```python
addr, dat, _bus = packer.make_can_msg("STEERING_COMMAND", 0, values)
```

E8x **master** uses `msg[1]` for dat (correct for this tree). E8x **0.74** used `msg[2]`. Do not copy the 0.74 index.

Do not rely on packer auto-checksum (`SignalType` checksum) unless a `calc_checksum` is registered for ocelot. E8x always two-pass packs.

---

## `create_steer_command` (paste into `bmwcan.py`)

E8x source: `selfdrive/car/bmw/bmwcan.py`. Bus changed to **0**. `STEER_ANGLE = 0` in v1 TorqueControl.

```python
from enum import IntEnum
from opendbc.can.packer import CANPacker


class SteeringModes(IntEnum):
  Off = 0
  TorqueControl = 1
  AngleControl = 2
  SoftOff = 3


def calc_checksum_8bit(work_data: bytes | bytearray, msg_id: int) -> int:
  checksum = msg_id
  for byte in work_data:
    checksum += byte
  checksum = (checksum & 0xFF) + (checksum >> 8)
  checksum &= 0xFF
  return checksum


def create_steer_command(packer: CANPacker, frame: int, mode: SteeringModes,
                         steer_tq: float = 0.0, steer_delta: float = 0.0):
  """0x22E STEERING_COMMAND, 5 bytes, 10 ms. panda bus 0."""
  values = {
    "COUNTER": frame % 16,
    "STEER_MODE": int(mode),
    "STEER_ANGLE": steer_delta,
    "STEER_TORQUE": steer_tq,
    "CHECKSUM": 0,
  }
  addr, dat, _bus = packer.make_can_msg("STEERING_COMMAND", 0, values)
  values["CHECKSUM"] = calc_checksum_8bit(dat, addr)
  return packer.make_can_msg("STEERING_COMMAND", 0, values)
```

Keep one `CANPacker('ocelot_controls')` on the controller; do not construct a new packer every frame (E8x did; skip that).

### Do not paste — stalk spoof

```python
# E8x bmwcan.create_accel_command — NEVER TX on this car
def create_accel_command(packer, action: CruiseStalk, bus: int, cnt):
    values = {
        "setMe_0xFC": 0xFC,
        "requests_0xF": 0xF,
        "Counter_0x194": cnt % 0xF,  # 0..14
    }
    values[action.value] = 1
    dat = packer.make_can_msg("CruiseControlStalk", bus, values)[1]
    values["Checksum_0x194"] = calc_checksum_8bit(dat, 0)  # init 0, not addr
    return packer.make_can_msg("CruiseControlStalk", bus, values)
```

0x194 checksum is the same fold **but `msg_id = 0`**. RX parse does not need it if the DBC decoder is used.

---

## 0x22E / 0x22F layout (ocelot)

IDs default **0x22E command / 0x22F status**. Solder jumpers offset both by `3 * jumper_config` (up to four actuators). Confirm cabana IDs before panda allowlist freeze.

Expect **10 ms** both ways (100 Hz). `STEER_STEP = 1`.

### `STEERING_COMMAND` 0x22E, 5 bytes, panda TX

| Signal | DBC | Wire |
|---|---|---|
| CHECKSUM | `0\|8@1+` | byte 0 |
| COUNTER | `8\|4@1+` | byte 1 low nibble, 0–15 |
| STEER_MODE | `12\|2@1+` | byte 1 bits 4–5 |
| STEER_ANGLE | `16\|16@1- (0.125,0)` | bytes 2–3 LE int16, ±4096 deg |
| STEER_TORQUE | `32\|8@1- (0.125,0)` | byte 4 int8, **−16 … +15.875 Nm** |

```
STEER_MODE: 0 Off (instant 0 torque)
            1 TorqueControl   ← v1
            2 AngleControl    ← not v1
            3 SoftOff         ← ramp to 0 in ~1 s; then command Off to re-enable
```

Firmware applies torque/mode **before** checksum compare, then only increments the “valid RX” counter on checksum match. Panda must still reject bad checksum / bad counter. After **5** 10 ms service ticks with no valid checksum (`CHECK_RX_FAIL_LIM`), firmware forces SoftOff (~50 ms). Plan: 0x22F missing > 50 ms → SoftOff then Off.

### `STEERING_STATUS` 0x22F, 8 bytes, RX

| Signal | DBC | Wire |
|---|---|---|
| CHECKSUM | byte 0 | same algorithm, init=`addr` (0x22F) |
| COUNTER | byte 1 low nibble | |
| CONTROL_STATUS | byte 1 high nibble | bit0 enabled, bit1 sensored, **bit2 SoftOff**, bit3 close-loop |
| STEERING_TORQUE | byte 2 int8 × 0.125 Nm | use as `steeringTorqueEps` |
| STEERING_SPEED | byte 3 int8 × 0.015625 rev/s | |
| TEMPERATURE | byte 4 uint8, offset −60 °C | |
| STEERING_ANGLE | bytes 5–6 LE int16 × 0.125 deg | compare to SAS 0xC4 |
| DEBUG_STATES | byte 7 | bit5 = CAN RX checksum error seen |

E8x: `steerFaultTemporary = CONTROL_STATUS & 0x4`.

---

## STEER_MAX and rate limits

E8x `CarControllerParams` (`selfdrive/car/bmw/values.py`):

```python
class CarControllerParams:  # 100 Hz
  STEER_STEP = 1                 # 100 Hz = 10 ms (servo contract)
  STEER_MAX = 12                 # Nm  (DBC full-scale ±16 / 15.875)
  STEER_DELTA_UP = 10 / 100      # 0.10 Nm / 10 ms = 10 Nm/s
  STEER_DELTA_DOWN = 1000 / 100  # 10 Nm / sample → no practical down limit
  STEER_ERROR_MAX = 999          # no meas-error clip
  # STEPPER_SERVO_CAN = 2 ** 0
```

**This port:** start `STEER_MAX = 6 Nm` (half E8x), then 12 after logs. DBC clip remains ±16; panda max cmd must match the Python clip.

Rate-limit helper already in this tree: `opendbc/car/lateral.py` `apply_dist_to_meas_limits` (E8x carcontroller calls it with EPS torque from 0x22F).

E8x lat loop (steal pattern, not the cruise gate):

```python
new_steer = actuators.steer * CarControllerParams.STEER_MAX
apply_steer = apply_dist_to_meas_limits(
    new_steer, self.apply_steer_last, CS.out.steeringTorqueEps,
    CarControllerParams.STEER_DELTA_UP, CarControllerParams.STEER_DELTA_DOWN,
    CarControllerParams.STEER_ERROR_MAX, CarControllerParams.STEER_MAX)
# lat active → TorqueControl(apply_steer)
# disable → SoftOff while last torque != 0, else Off
```

E8x panda `BMW_MAX_TQ_RATE` `{0,5,15} m/s → {16,8,1} Nm/10 ms` is **not** the carcontroller limit. Do not copy as-is; start panda rate near **10 Nm/s up**.

E8x torque tune (scale with whatever `STEER_MAX` we ship):

```python
ret.lateralTuning.torque.kp = 1.5 / STEER_MAX
ret.lateralTuning.torque.ki = 0.5 / STEER_MAX
ret.lateralTuning.torque.kf = 5.0 / STEER_MAX
ret.lateralTuning.torque.friction = 0.23
ret.lateralTuning.torque.latAccelFactor = 1.41
ret.lateralTuning.torque.useSteeringAngle = False
ret.steerActuatorDelay = 0.4
ret.steerLimitTimer = 0.4
```

Retune after fw **0.4.3** (next section). `steerRatio` 16 is hydraulic-EPS-era; re-measure with column stepper + gearbox.

---

## Firmware 0.4.3 hold torque / √2

Release 0.4.3 (`96d2649`, 2024-06-13):

> The scaling of motor rated torque has changed!!!  
> Adjust by **1.41** your `motor_rated_hold_torque` or PID gains in Openpilot.

```c
// firmware/src/BSP/actuator_config.c  (fw ≥ 0.4.3)
motor_k_torque = (motor_rated_hold_torque / (SQRT2 * 100)) / motor_rated_current;
// SQRT2 1.41421f; hold torque is N·cm; /100 → N·m
// Datasheet hold torque = both phases at rated current (and often inflated)
```

Set in firmware (not OP): `motor_rated_current`, `motor_rated_hold_torque` (N·cm), `motor_gearbox_ratio`, `final_drive_ratio` (negative reverses). CAN Nm is **output** torque after gearing.

Hardware: 3.3 A peak / 32 V; NEMA23 104 mm or 56 mm examples; dense windings. Above ~2 A on the small AliExpress 17HS4401S, torque tapers. Calibrate encoder (blue LED / F1). Require fw **≥ 0.4.3**.

If OP `STEER_MAX` was tuned on **pre-0.4.3** firmware, either multiply firmware `motor_rated_hold_torque` by 1.41 or retune kp/ki/kf. Do not assume 12 Nm at the rim equals 12 Nm electrical after a flash.

---

## Carstate parsers to steal (RX)

DBC: `bmw_e9x_e8x.dbc` (BMW-E8x-E9x/opendbc).

### 0xCE `WheelSpeeds` (206) — **vEgo**

```
BO_ 206 WheelSpeeds: 8 DSC
 SG_ Wheel_FL : 0|16@1- (0.0625,0)  "kph"
 SG_ Wheel_FR : 16|16@1- (0.0625,0) "kph"
 SG_ Wheel_RL : 32|16@1- (0.0625,0) "kph"
 SG_ Wheel_RR : 48|16@1- (0.0625,0) "kph"
```

~50 Hz on PT. Signed 16, 0.0625 km/h/bit. E8x parses all four into `get_wheel_speeds` then **throws them away** for `vEgoRaw` (uses 0x1A0). We average **RL/RR**, convert km/h → m/s, Kalman as stock. Standstill: `vEgo < 0.3 m/s`, not 0x1A0 flags.

### 0xC4 `SteeringWheelAngle` (196) — SAS

```
BO_ 196 SteeringWheelAngle: 7 DSC
 SG_ SteeringPosition : 0|16@1- (0.04395,0) [-600|600] "deg"
 SG_ SteeringSpeed    : 24|16@1- (0.04395,0) "deg/s"
```

~100 Hz. E8x panda uses `0.0439453125` (= 360/8192). Carstate: DBC 0.04395. Safety: pick one and test.

### 0x194 `CruiseControlStalk` (404) — **buttons only**

```
BO_ 404 CruiseControlStalk: 4 SZL
 SG_ Checksum_0x194   : 7|8@0+
 SG_ Counter_0x194    : 11|4@0+          # 0..14
 SG_ requests_0xF     : 15|4@0+          # 0xF when any request
 SG_ plus1            : 16|1@0+
 SG_ plus5            : 17|1@0+
 SG_ minus1           : 18|1@0+
 SG_ minus5           : 19|1@0+
 SG_ cancel           : 20|1@0+
 SG_ resume           : 22|1@0+
 SG_ cancel_lever_up  : 23|1@0+
 SG_ setMe_0xFC       : 31|8@0+          # 0xFC
```

Stock rate: **5 Hz** idle, **20 Hz** while held (`carcontroller.py` comments). E8x spoofs extra 5 Hz / 100 Hz frames — **we do not**.

E8x parse (copy named-signal logic):

```python
msg = cp.vl["CruiseControlStalk"]
if msg["plus1"]:
  cruise_stalk_speed = 1
elif msg["minus1"]:
  cruise_stalk_speed = -1
elif msg["plus5"]:
  cruise_stalk_speed = 5
elif msg["minus5"]:
  cruise_stalk_speed = -5
else:
  cruise_stalk_speed = 0
cruise_stalk_resume = msg["resume"] != 0
cruise_stalk_cancel = msg["cancel"] != 0
cruise_stalk_cancel_up = msg["cancel_lever_up"] != 0
cruise_stalk_counter = msg["Counter_0x194"]
```

E8x panda cancel RX: `GET_BYTE(msg, 2) & 0x90` (bit4 cancel, bit7 cancel_lever_up on byte 2). Use DBC names in Python; raw mask is OK in C if it matches cabana.

VO540: plus1/minus1 are the speed clicks. Parse plus5/minus5 if they appear; do not require them.

### Do not steal from E8x carstate

| E8x | Why |
|---|---|
| `vEgoRaw = Speed.VehicleSpeed` 0x1A0 | Gateway clamps ≥ 10 km/h on can2 |
| `brakePressed = EngineAndBrake.BrakePressed` | ICE DME; live brake is VCU 0x0A8 byte7 `0x64` |
| `gas = AccPedal` 0xAA | Use interceptor 0x199 `PedalInPct` |
| `cruiseState.enabled` from 0x200 / 0x193 DCC | No working BMW PCM cruise for speed |
| `steeringPressed` faked from voice/gas | Use 0x22F vs cmd |
| `CanBus.SERVO_CAN = 1` sharing F-CAN | Servo is on can2 = panda bus 0 |

---

## E8x `carcontroller` TX inventory (do not copy long)

| TX | ID | Bus in E8x | This port |
|---|---|---|---|
| `create_steer_command` TorqueControl / SoftOff / Off | 0x22E len 5 | SERVO_CAN (1) / panda tests bus 2 | **Yes, bus 0** |
| `create_accel_command` plus1/plus5/minus1/minus5/cancel | **0x194** len 4 | PT 0 or F-CAN 1 | **Forbidden** |
| DSC/ABS brake torque | 0xB6 etc. | — | **Forbidden** |
| Pedal interceptor | — | E8x has none | 0x200 later, not this file |

E8x long: hold plus5 ≈ +1.2 m/s², minus5 ≈ −1.4, plus1 ≈ +0.8, minus1 ≈ −0.8 by **clicking stock CC**. Irrelevant here.

---

## Flags and fingerprint

```python
class BmwFlags(IntFlag):
  STEPPER_SERVO_CAN = 2 ** 0          # steal: 0x22F seen
  NORMAL_CRUISE_CONTROL = 2 ** 1      # E8x: 0x200 on PT — DO NOT use; 0x200 is pedal
  DYNAMIC_CRUISE_CONTROL = 2 ** 2     # $544 DCC — no actuation
  # ACC / no-LDM / SERVOTRONIC — ignore
```

E8x: `if 0x22F in fingerprint[SERVO_CAN]: flags |= STEPPER_SERVO_CAN`. We fingerprint 0x22F on bus 0.

E8x E90 `CarSpecs`: mass `3300 * LB_TO_KG + STD_CARGO_KG`, wheelbase **2.76**, steerRatio **16**. Mass is ICE — use **1570 kg**.

`minEnableSpeed = 30 km/h` in E8x is DCC minimum. Keep a 30 km/h floor for **long** only; MADS lat uses `minSteerSpeed` (plan: start 10 km/h).

---

## E8x panda (reference, mostly ignore)

`BMW-E8x-E9x/panda` **master** TX `{404,0},{404,1},{252,2}` is the **old TMCL 0xFC** actuator. Ignore.

`dzid26/panda_bmw` **openactuator_dev** is the stepper-era file:

```c
const AddrBus BMW_TX_MSGS[] = {{404, 0}, {404, 1}, {558, 2}};
// 0x194 stalk spoof + 0x22E on bus 2
```

Our allowlist: **`{0x22E len 5, 0x200 len 4}` on bus 0 only.** No 0x194. No 0x201. No 0xFC.

Useful fragments from openactuator `safety_bmw.h` (re-implement, do not copy file):

- 0x22E torque: `(int8_t)GET_BYTE(msg, 4) * 0.125f`
- 0x22E mode: `(GET_BYTE(msg, 1) >> 4) & 0x3`
- 0x22F torque: `(int8_t)GET_BYTE(msg, 2) * 0.125f`
- 0x22F SoftOff: `((GET_BYTE(msg, 1) >> 4) >> 2) & 1`
- 0xC4 angle: `to_signed((b1 << 8) | b0, 16) * 0.04395`

Do not copy: 0x1A0 speed, ICE 0xA8 brake bits `0x60`, gear 0x1D2 `lever == 8`, cruise engage from 0x193/0x200, GMLAN pin.

Their `test_bmw.py` checksum for 0x22E is **not** the firmware fold (it sums cnt+mode+angle+tq without addr fold). Write new tests from the C/Python snippets above.

---

## Files fetched

### BMW-E8x-E9x/openpilot `master` (`selfdrive/car/bmw/`)

| File | URL |
|---|---|
| `bmwcan.py` | https://raw.githubusercontent.com/BMW-E8x-E9x/openpilot/master/selfdrive/car/bmw/bmwcan.py |
| `carstate.py` | https://raw.githubusercontent.com/BMW-E8x-E9x/openpilot/master/selfdrive/car/bmw/carstate.py |
| `carcontroller.py` | https://raw.githubusercontent.com/BMW-E8x-E9x/openpilot/master/selfdrive/car/bmw/carcontroller.py |
| `values.py` | https://raw.githubusercontent.com/BMW-E8x-E9x/openpilot/master/selfdrive/car/bmw/values.py |
| `interface.py` | https://raw.githubusercontent.com/BMW-E8x-E9x/openpilot/master/selfdrive/car/bmw/interface.py |
| `fingerprints.py` | https://raw.githubusercontent.com/BMW-E8x-E9x/openpilot/master/selfdrive/car/bmw/fingerprints.py |
| `radar_interface.py` | listed (dummy); not needed beyond “no radar” |

Also listed: `__init__.py` empty, `tests/`.

Older `create_steer_command` (packer `msg[2]`, hard-coded bus 2): https://github.com/dzid26/openpilot-for-BMW-E8x-E9x/blob/0.74_openactuator_testdev/selfdrive/car/bmw/bmwcan.py

### BMW DBC

| File | URL |
|---|---|
| `bmw_e9x_e8x.dbc` | https://raw.githubusercontent.com/BMW-E8x-E9x/opendbc/master/opendbc/dbc/bmw_e9x_e8x.dbc |
| `ocelot_controls.dbc` (checksum comment **correct**) | https://raw.githubusercontent.com/BMW-E8x-E9x/opendbc/master/opendbc/dbc/ocelot_controls.dbc |
| RetroPilot Ocelot-steering-dev (layout same; 0x22E checksum comment **wrong** “J1850”) | https://raw.githubusercontent.com/RetroPilot/opendbc/Ocelot-steering-dev/ocelot_controls.dbc |

### E8x panda

| File | URL |
|---|---|
| openactuator `safety_bmw.h` (0x22E + **0x194 TX**) | https://raw.githubusercontent.com/dzid26/panda_bmw/openactuator_dev/board/safety/safety_bmw.h |
| openactuator `test_bmw.py` | https://raw.githubusercontent.com/dzid26/panda_bmw/openactuator_dev/tests/safety/test_bmw.py |
| master `safety_bmw.h` (TMCL 0xFC — ignore) | https://raw.githubusercontent.com/BMW-E8x-E9x/panda/master/board/safety/safety_bmw.h |

### dzid26/StepperServoCAN `main` (fw 0.4.3+)

| File | URL |
|---|---|
| README (IDs, 10 ms, modes) | https://raw.githubusercontent.com/dzid26/StepperServoCAN/main/README.md |
| `firmware/src/BSP/can.c` (checksum + 50 ms SoftOff) | https://raw.githubusercontent.com/dzid26/StepperServoCAN/main/firmware/src/BSP/can.c |
| `firmware/src/BSP/actuator_config.c` (hold / √2) | https://raw.githubusercontent.com/dzid26/StepperServoCAN/main/firmware/src/BSP/actuator_config.c |
| `firmware/src/BSP/utils.h` (`SQRT2`) | https://raw.githubusercontent.com/dzid26/StepperServoCAN/main/firmware/src/BSP/utils.h |
| `firmware/src/BSP/control_api.c` (mode map, CONTROL_STATUS bits) | https://raw.githubusercontent.com/dzid26/StepperServoCAN/main/firmware/src/BSP/control_api.c |
| `firmware/src/APP/main.c` (10 ms status TX, comm-loss SoftOff) | https://raw.githubusercontent.com/dzid26/StepperServoCAN/main/firmware/src/APP/main.c |
| `firmware/src/OP/Msg.h` / `Msg.c` (0x22E/0x22F pack) | https://raw.githubusercontent.com/dzid26/StepperServoCAN/main/firmware/src/OP/Msg.h |
| `firmware/test/test_checksum/test_checksum.c` | https://raw.githubusercontent.com/dzid26/StepperServoCAN/main/firmware/test/test_checksum/test_checksum.c |
| `StepperServoCanGUI.py` (`msg_calc_checksum_8bit`, 10 ms loop) | https://raw.githubusercontent.com/dzid26/StepperServoCAN/main/StepperServoCanGUI.py |
| Release 0.4.3 | https://github.com/dzid26/StepperServoCAN/releases/tag/0.4.3 |
| Hardware README (NEMA23, 3.3 A, 32 V) | https://raw.githubusercontent.com/dzid26/StepperServo-hardware/master/README.md |

Repo roots: https://github.com/BMW-E8x-E9x/openpilot · https://github.com/dzid26/StepperServoCAN
