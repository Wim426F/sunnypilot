# E90 EV longitudinal — interceptor research (no TX)

**Owner:** Agent E (research only). Do **not** implement `carcontroller.py` 0x200 TX until Phase 4 lat exists and panda `test_bmw.py` is green.

**Law:** `pedal_arduino.ino` (`/home/wimboone/Documents/pedal_arduino/pedal_arduino.ino`). Spec: `E90-SUNNYPILOT-PORT-PLAN.md` §0, §3.1, §6 Phase 5, §11. Rules: `AGENTS.md`.

**Actuator:** Wim pedal-interceptor `CRUISE_Gas` **0x200**, 4 bytes, panda CAN0 = can2_powertrain. Decel = Tesla one-pedal regen at 0 % interceptor output. **No friction-brake actuator. Never TX 0x194, 0x201, or DSC/ABS torque.**

---

## 1. Exact 0x200 pack (`CRUISE_Gas`)

Firmware comments + `handleGasFrame()`:

| Byte | Field | Value |
|---|---|---|
| 0 | `GasReq` raw | `0..255` = `0..100 %`. Scale `100/255 ≈ 0.39215686`. `51` = **20 %** exactly. |
| 1 | `GasEnable` | **`0xA5` (165)** to enable. Anything else: `gas_enabled = false`, `gas_request = 0`. |
| 2 | `GasCounter` | **Low nibble only** (`dat[2] & 0x0F`). Must **differ** from last accepted (frozen/replay ignored; timeout then disables). Sender policy: **increment** `0..15`, wrap 15→0. High nibble **0**. |
| 3 | CRC8 | poly **`0xD5`**, init **`0xFF`**, over **bytes 0..2**, iteration **forward** (`i = 0 .. len-1`). |

DLC **4**. Rate **≥ 20 Hz** (firmware `CAN_GAS_TIMEOUT_MS = 100`). OP v1: **50 Hz**. Bus **0**.

CRC is comma’s poly, but **not** `crc8_pedal()` (that walks **backwards**). C from firmware:

```c
uint8_t crc = 0xFF;
for (i = 0; i < len; i++) {          // FORWARD — bytes 0, 1, 2
  crc ^= dat[i];
  for (j = 0; j < 8; j++)
    crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);  // uint8_t wrap
}
```

Semantics (firmware join, after analog normalize):

```
can_norm = gas_enabled ? gas_request * 10000 / 255 : 0
cmd      = max(bmw_pedal_norm, can_norm)     # CAN cannot cut below the human pedal
if torque_cut or pedal_fault: cmd = 0        # 0x201 / sensor fault; OP does not TX 0x201
DAC      = Tesla unpressed..full (397/800 mV .. 1900/3820 mV)
```

0 % CAN = Tesla unpressed voltages = **one-pedal regen**, not 0 mV (0 mV is a sensor fault to the DU).

0x199 `PEDAL_Status` (~10 Hz, RX only): byte0 PedalIn %, byte1 PedalOut %, byte2 state `0 PASSTHROUGH / 1 THROTTLEGATING / 2 DRIVEBYWIRE`. DRIVEBYWIRE iff `gas_enabled && can_norm > pedal`. This is the interceptor RX; **not** Toyota `GAS_SENSOR` 0x201.

### 1.1 Paste into `opendbc/car/bmw/bmwcan.py` (later long agent)

Firmware-accurate pack. Do **not** call `opendbc.sunnypilot.car.create_gas_interceptor_command`. Do **not** use `crc8_pedal`. `CRC8BODY` in `opendbc/car/crc.py` is poly 0xD5 but body helpers iterate **backwards** — only reuse the table if you walk **forward** with init `0xFF`.

```python
# Wim pedal-interceptor CRUISE_Gas 0x200. Firmware is law (pedal_arduino.ino).
# 4 bytes, bus 0. Do not use create_gas_interceptor_command / crc8_pedal.

from opendbc.car.can_definitions import CanData

GAS_ENABLE_MAGIC = 0xA5
CRUISE_GAS_ADDR = 0x200
CRUISE_GAS_BUS = 0  # panda CAN0 = can2_powertrain


def crc8_wim_pedal(data: bytes, poly: int = 0xD5) -> int:
  """CRC8 poly 0xD5, init 0xFF, FORWARD over data (bytes 0..2 of 0x200)."""
  crc = 0xFF
  for b in data:
    crc ^= b
    for _ in range(8):
      crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc


def create_pedal_command(gas_raw: int, idx: int) -> CanData:
  """Pack 0x200. Call only when CC.longActive (and controls_allowed).
  Do not pack a disabled frame — stop sending instead.
  gas_raw: 0..255. idx: rolling counter; low nibble is sent.
  """
  dat = bytearray(4)
  dat[0] = int(max(0, min(255, gas_raw)))
  dat[1] = GAS_ENABLE_MAGIC
  dat[2] = int(idx) & 0xF
  dat[3] = crc8_wim_pedal(bytes(dat[:3]))
  return CanData(CRUISE_GAS_ADDR, bytes(dat), CRUISE_GAS_BUS)


# Worked example: gas_raw=51 (~20 %), enable=0xA5, counter=1 → CRC 0xD5
# CAN payload: 33 A5 01 D5
assert crc8_wim_pedal(bytes([51, 0xA5, 1])) == 0xD5
assert create_pedal_command(51, 1) == CanData(0x200, bytes([0x33, 0xA5, 0x01, 0xD5]), 0)
```

Packer-based variant (once DBC agent names the signals). Checksum still computed in Python over the packed bytes — do not trust a DBC checksum mux until it is this algorithm:

```python
def create_pedal_command(packer, gas_raw: int, idx: int):
  values = {
    "GasReq": int(max(0, min(255, gas_raw))),  # raw 0..255; or pack percent with DBC factor 100/255
    "GasEnable": 0xA5,
    "GasCounter": int(idx) & 0xF,             # SG_ start bit 19, 4 bits, Motorola → byte2 low nibble
  }
  addr, dat, bus = packer.make_can_msg("CRUISE_Gas", 0, values)
  values["GasCrc"] = crc8_wim_pedal(dat[:3])
  return packer.make_can_msg("CRUISE_Gas", 0, values)
```

Suggested DBC (DBC agent; do **not** copy Toyota `GAS_COMMAND`):

```
BO_ 512 CRUISE_Gas: 4 NEO
 SG_ GasReq : 7|8@0+ (1,0) [0|255] "" XXX
 SG_ GasEnable : 15|8@0+ (1,0) [0|255] "" XXX
 SG_ GasCounter : 19|4@0+ (1,0) [0|15] "" XXX
 SG_ GasCrc : 31|8@0+ (1,0) [0|255] "" XXX
VAL_ 512 GasEnable 165 "ENABLE" ;
```

Percent display: `GasReq * 100 / 255`. Optional parallel `SG_` with factor `0.3921568627` — do not double-pack.

### 1.2 Worked example (gas raw = 51 ≈ 20 %, enable = 0xA5, counter = 1)

`51 / 255 * 100 = 20.0 %`. Payload bytes 0..2 = `33 A5 01`.

| Step | `crc` |
|---|---|
| init | `FF` |
| xor byte0 `33` | `CC` |
| 8 shifts | `A5` |
| xor byte1 `A5` | `00` |
| 8 shifts | `00` |
| xor byte2 `01` | `01` |
| 8 shifts (`uint8_t`: last shift `80 << 1` then `^ D5`) | **`D5`** |

**Frame: `33 A5 01 D5`.** Same 3 bytes through Toyota `crc8_pedal` (reverse) = `0xA4` — wrong, interceptor drops it.

Extra vectors (forward CRC):

| gas | enable | counter | CRC |
|---|---|---|---|
| 0 | `A5` | 0 | `7B` |
| 51 | `A5` | 1 | `D5` |
| 51 | `A5` | 2 | `7F` |
| 255 | `A5` | 15 | `44` |
| 51 | `00` | 1 | `EC` (firmware would accept CRC then **disable**) |

Pre-OP bench sender: USB/Teensy at ≥ 20 Hz, counter++, CRC as above, 20 % (`51`), wheels off / stands. **`sender_example.md` is not this message** (see §3).

---

## 2. Why Toyota / Honda interceptor helpers must not be used

Same CAN IDs (`0x200` TX, `0x201` RX) are a **coincidence**. Layout, length, CRC direction, enable encoding, and product role are all different. Mixing them is a safety bug.

### 2.1 `create_gas_interceptor_command` / `crc8_pedal` / 6-byte `GAS_COMMAND`

Source: `opendbc_repo/opendbc/sunnypilot/car/__init__.py`, DBC `opendbc/dbc/generator/toyota/_community.dbc` (Honda/GM community copies are the same 6-byte shape).

| | Toyota / Honda comma interceptor | Wim E90 interceptor |
|---|---|---|
| ID 0x200 DLC | **6** | **4** |
| Byte 0–1 | 16-bit Motorola `GAS_COMMAND` (DBC scale e.g. `0.159375, -75.555`) | **uint8** `GasReq` 0–255 linear % |
| Byte 2–3 | 16-bit `GAS_COMMAND2` (second pot, different scale) | nibble counter + CRC |
| Enable | **1-bit bool** `ENABLE` at bit 39 (`gas_amount > 0.001`) | **whole byte `0xA5`** |
| Counter | nibble in **byte 4** (`PEDAL_COUNTER`) | nibble in **byte 2** |
| CRC | byte 5, poly 0xD5 init 0xFF, iterate **`range(size-1, -1, -1)` (backwards)** | byte 3, same poly/init, iterate **forward 0..2** |
| 0x201 | RX `GAS_SENSOR` **6 bytes** (user pots + state enum) | **TX torque-cut 3 bytes**, not OP’s (see §3) |
| User pedal RX | 0x201 `INTERCEPTOR_GAS` | **0x199** PedalIn/Out/state |
| When inactive | **keep TX** 50 Hz with gas=0, `ENABLE=False` | **stop sending** (100 ms → passthrough) |
| Role | Low-speed supplement; Honda/Toyota set **`minEnableSpeed = -1`** (stop-and-go) + stock **friction brakes** | **Only** long actuator; regen floor 0 %; enable **≥ 30 km/h** |
| Detect | `0x201 in fingerprint` | **Do not.** 0x201 is VCU cut, always on this bus. Flag `E90Flags.PEDAL_INTERCEPTOR`. |

`crc8_pedal`:

```python
for i in range(size - 1, -1, -1):  # BACKWARDS — Wim firmware does not do this
  crc ^= data[i]
  ...
```

`create_gas_interceptor_command`:

- packs DBC name **`GAS_COMMAND`** (6 bytes) on bus 0
- `ENABLE` bool, not `0xA5`
- `GAS_COMMAND = GAS_COMMAND2 = gas_amount * 255` then DBC inverse-scale into 16-bit fields — **not** Wim byte0
- checksum via `crc8_pedal(dat[:-1])`

Panda allowlist v1 is `{0x200, bus 0, len 4}`. A Toyota 6-byte frame is rejected on length (good) **if** the allowlist stays 4. Do not widen it.

### 2.2 Honda / Toyota carcontroller anti-patterns (do not copy)

- **Honda** `GasInterceptorCarController.update`: TX **every** interceptor path when `enableGasInterceptor`, including `CC.longActive == False` with `gas = 0.0`. Wim: silence = disable. Honda also still TX **Nidec brake** `0x1FA`. We have no brake TX.
- **Toyota** `create_gas_command`: TX `frame % 2 == 0` whenever interceptor + `openpilotLongitudinalControl`, **including** `interceptor_gas_cmd = 0` when not `longActive`. Maps `actuators.accel` through a **low-speed** `PEDAL_SCALE` that **ramps to 0 above `MIN_ACC_SPEED`** (PCM takes over). Our interceptor is the highway actuator, not a creep helper.
- Both brand `interface.py`s: `minEnableSpeed = -1` when interceptor present. **Forbidden** here (`AGENTS.md`: no stop-and-go).
- Both detect interceptor with **`0x201 in fingerprint`**. On can2 that ID is torque-cut from another device.
- Honda Nidec interceptor still uses stock **brake** commands. Copying Honda long = inventing a brake we do not have.
- Tesla `CarControllerParams.ACCEL_MIN = -3.48` is stock AP **with friction brakes**. Spec: too much decel to claim. v1 clip **−1.5**.

E8x BMW long (stalk spoof `0x194`) is a different anti-pattern: this EV has no working PCM DCC. Stalk is UI only.

---

## 3. Why 0x201 is not OP’s message — do not TX

Closed decision §11 #2: **another device owns torque-cut.** Panda TX allowlist = `{0x200 len 4, 0x22E len 5}` on bus 0 only. RX 0x201 optional ignore.

Firmware `0x201` (`VEHICLE_TorqueCut`, DLC **3**, timeout **200 ms**):

| Byte | Firmware (`pedal_arduino.ino`) |
|---|---|
| 0 | **Bitfield:** bit0 `0x01` `REGEN_CUT` (**ignored** — forcing 0 % would *increase* regen), bit1 **`0x02` `ALL_TORQUE_CUT`** → force Tesla output 0 %. |
| 1 | Counter, low nibble, must change |
| 2 | CRC8 poly 0xD5 init 0xFF over bytes 0..1, **forward** |

Fail-safe: **silence → do not cut** (fall back to pedal). Cut sender, if any, must keep TX even when not cutting.

Attached DBC (`VAL_ TorqueCut 90 "CUT"`) and local `sender_example.md` (`TORQUE_CUT_MAGIC = 0x5A` = 90) describe a **magic byte**, not a bitfield. `0x5A` happens to have bit1 set, so firmware would still cut — that does **not** make 90 the protocol. Firmware is law. OP must not pick a side by transmitting.

Toyota/Honda `0x201` is 6-byte `GAS_SENSOR`. Honda safety `HONDA_GAS_INTERCEPTOR_ADDR_CHECK` / Toyota `TOYOTA_GAS_INTERCEPTOR_ADDR_CHECK` treat 0x201 as user-gas RX. Using those hooks here would parse a 3-byte cut frame as dual 16-bit pots.

v1: **never TX 0x201.** Optional later (Phase 6): cut bit1 on hard OP disable **only** if firmware-aligned and the other device is not already doing it.

---

## 4. When to send 0x200

| Condition | 0x200 |
|---|---|
| `CC.longActive` **and** panda `controls_allowed` (long) **and** `openpilotLongitudinalControl` / `E90Flags.PEDAL_INTERCEPTOR` | TX **50 Hz** (`frame % 2 == 0` on the 100 Hz loop), enable **`0xA5`**, counter++ each TX, valid CRC |
| Long not active, lat-only MADS, disengage, fault, brake, cancel, interceptor timeout, CRC fail, driver gas override | **Stop sending.** 100 ms firmware timeout → `gas_enabled = false` → passthrough |
| `controls_allowed == 0` | **Never** send enable. Do not send a “disabled” 0x200 either: panda must reject `byte1 != 0xA5` |

Do **not** keep a 50 Hz heartbeat of zeros (Toyota/Honda). Firmware: a valid `0xA5` + `GasReq=0` is **enabled 0 %** (`max(pedal, 0)` = passthrough of the foot, regen if foot off) and keeps the timeout refreshed. That is still “enable without `controls_allowed`” if long is off. Gate is **presence of the frame**, not gas=0.

Panda long `controls_allowed` must clear on: brake press (`0x0A8` byte7 `0x64`), cancel, interceptor / 0x199 timeout, CRC fail, driver gas override, steer fault (policy). No auto-reengage long. No long-through-brake.

`CC.longActive` is the carcontroller proxy; still do not TX if the flag is up but you cannot know panda would allow it (e.g. do not send enable while `CS.out.brakePressed` or gas-override).

Rate: 50 Hz leaves ~5 missed frames before the 100 ms timeout. 100 Hz is allowed by firmware but unnecessary; steer already owns 100 Hz.

---

## 5. Accel → gas map v1 (feedforward only)

No braking beyond **0 % interceptor**. 0 % is Tesla unpressed = regen. There is no hydraulic OP brake and no 0x201 from OP.

```
clip a to [ACCEL_MIN, ACCEL_MAX] = [-1.5, +1.5] m/s²
gas_pct = interp(a, [a_regen(v), 0, a_max(v)], [0, gas_coast, 100])
gas_raw = clip(round(gas_pct * 255 / 100), 0, 255)
```

Do **not** assume Tesla OP −3.48 / +2.0. Do **not** enable stop-and-go.

### 5.1 Speed gates

| | km/h | m/s |
|---|---|---|
| Engage long (`minEnableSpeed`) | **30** | **8.33** |
| Drop long (hysteresis, chime) | **25** | **6.94** |
| Stop behind lead / `minEnableSpeed = -1` | **out of scope** | — |

vEgo = `Wheel_RL/RR` on **0xCE**. Never 0x1A0 (gateway floor 10 km/h).

`CarParams.minEnableSpeed` is a single threshold — 30/25 hysteresis is **not** stock. Needs a BMW-specific dropout (interface / controlsd / carcontroller). Below 25 km/h: drop long, alert, **driver brakes**. Never command a full stop.

Longitudinal maneuvers / first tune: **only above 40 km/h**, empty road. Exit: 80–120 km/h lead follow; never 0 km/h under OP.

### 5.2 FLAG MEASURE — §11 accel assumptions (1570 kg, ~980 DU)

At speed \(v\) (m/s), rough \(a \approx P / (m v)\). **Do not claim these below 30 km/h** (long is disabled there anyway; low-v is torque-limited).

| | Assumption | At \(v = 30\,\mathrm{m/s}\) | OP v1 clip |
|---|---|---|---|
| \(P_\mathrm{dis,max}\) | **200 kW** (gauge +200) | \(a_\mathrm{max} \approx 200\,\mathrm{kW}/(1570\times 30) \approx \mathbf{4.25\,m/s^2}\) | **+1.5 m/s²** |
| \(P_\mathrm{regen,max}\) | **100 kW** (gauge ±100) | \(a_\mathrm{regen} \approx \mathbf{-2.12\,m/s^2}\) | **−1.5 m/s²** |
| `gas_coast` | Pedal % for \(a \approx 0\) (one-pedal: 0 % is **not** coast) | **unknown** | FLAG MEASURE |
| jerk | ISO 15622-ish | — | **±2.5 m/s³** (§0) until logs |

Placeholder plant curves (replace from logs):

```python
MASS_KG = 1570.0
P_DIS_MAX_W = 200e3    # FLAG MEASURE
P_REGEN_MAX_W = 100e3  # FLAG MEASURE
ACCEL_MAX = 1.5
ACCEL_MIN = -1.5
GAS_COAST_PCT = None   # FLAG MEASURE — do not invent a production number

def a_max_v(v_ego: float) -> float:
  v = max(float(v_ego), 8.33)
  return P_DIS_MAX_W / (MASS_KG * v)

def a_regen_v(v_ego: float) -> float:
  v = max(float(v_ego), 8.33)
  return -P_REGEN_MAX_W / (MASS_KG * v)
```

Phase 5 test ticket (straight, no OP): log `DIR_torqueActual`, pack V/I, vEgo, `PedalOutPct`:

1. Foot-off regen sweep → `a_regen(v)`.
2. Full-pedal accel sweep → `a_max(v)`.
3. Hold several pedal % until `a ≈ 0` → `gas_coast` (and whether it moves with `v`).

Optional later (Phase 6): clip `GasReq` with `MaxDischargePower` / `MaxRegenPower` (msg 1686). Not v1.

### 5.3 Conservative v1 until `gas_coast` is measured

The spec interpolator **needs** `gas_coast`. Mapping `a = 0` → `0 %` is **max regen**, not coast — planner would see a persistent negative `aEgo` at zero request.

Until the sweep exists:

- Still **clip the request** to ±1.5.
- **Negative `a`:** `gas_raw = 0` (full one-pedal regen). Plant may deliver ~−2.1 m/s² at 30 m/s, **steeper than the clip**. Accept only on empty-road measurement runs; do not treat this as a shipped tracking map.
- **Positive `a`:** `gas_pct = interp(a, [0, min(ACCEL_MAX, a_max(v))], [0, 100])` is also wrong at the origin (0 % ≠ coast). Better: do not ship long until `gas_coast` is a measured constant (or `gas_coast(v)` table).

Feedforward only: no long PID / PCM windup copy from Toyota. Tesla DU + analog pedal will need a later measured `gas_pct(a, v)` table.

Human override: firmware `max(pedal, can)` so CAN **cannot** reduce below the foot. Panda `gas_pressed` (proposed): `PedalInPct > 20` **and** `PedalInPct > PedalOutPct + 5`. When pressed, drop long TX (stop sending).

---

## 6. Proposed signatures (do not implement production code in this change)

Later long agent, `opendbc/car/bmw/bmwcan.py` + `carcontroller.py` + `values.py`. Phase 5 gate: lat exists, panda tests pass, Phase 1 log shows no stranger 0x200.

```python
# bmwcan.py
def crc8_wim_pedal(data: bytes, poly: int = 0xD5) -> int: ...
def create_pedal_command(gas_raw: int, idx: int) -> CanData: ...
# or: def create_pedal_command(packer, gas_raw: int, idx: int) -> tuple[int, bytes, int]: ...

# values.py / CarControllerParams
ACCEL_MAX = 1.5          # m/s²
ACCEL_MIN = -1.5         # m/s²
JERK_LIMIT = 2.5         # m/s³  (§0; not in Phase 5 bullets — leftover)
PEDAL_TX_STEP = 2        # 100 Hz loop → 50 Hz 0x200
MIN_ENABLE_SPEED = 30.0 * CV.KPH_TO_MS   # 8.33  → CarParams.minEnableSpeed
MIN_ENABLE_SPEED_DROPOUT = 25.0 * CV.KPH_TO_MS  # 6.94  hysteresis; not a stock field
# CarParams.openpilotLongitudinalControl = True only behind E90Flags.PEDAL_INTERCEPTOR
# Do not set minEnableSpeed = -1.

# bmwcan.py or carcontroller.py
def accel_to_gas_raw(accel_mps2: float, v_ego: float) -> int:
  """Clip ±1.5, feedforward interp onto FLAG MEASURE a_regen/a_max/gas_coast, return 0..255.
  Floor 0 (no brake actuator). Never used below dropout speed.
  """
  ...

# carcontroller.py — long path only (lat already sending 0x22E)
def _update_longitudinal(self, CC, CS, actuators, frame: int) -> list[CanData]:
  """50 Hz 0x200 iff CC.longActive and not brake/gas-override.
  Else return [] (stop sending). Never 0x194 / 0x201.
  """
  ...
```

Sketch of the long path (reference only):

```python
def _update_longitudinal(self, CC, CS, actuators, frame: int) -> list[CanData]:
  if frame % CarControllerParams.PEDAL_TX_STEP != 0:
    return []
  if not (self.CP.openpilotLongitudinalControl and CC.longActive):
    return []
  if CS.out.brakePressed or CS.out.gasPressed:  # panda will also clear long
    return []
  gas_raw = accel_to_gas_raw(float(actuators.accel), float(CS.out.vEgo))
  return [bmwcan.create_pedal_command(gas_raw, frame // CarControllerParams.PEDAL_TX_STEP)]
```

Do **not** subclass `opendbc.sunnypilot.car.toyota.gas_interceptor.GasInterceptorCarController` or Honda’s. Do not import `create_gas_interceptor_command`.

Pre-OP: Teensy/USB sender using `create_pedal_command(51, n)` at 50 Hz, confirm 0x199 state 2 DRIVEBYWIRE and PedalOut ~20 % with foot off.

---

## 7. Safety interaction

Toyota/Honda:

```c
// longitudinal.h — WRONG for Wim
bool longitudinal_interceptor_checks(const CANPacket_t *msg) {
  return !get_longitudinal_allowed() && (msg->data[0] || msg->data[1]);
}
```

`get_longitudinal_allowed()` = `controls_allowed && !gas_pressed_prev`.

Why this helper is wrong on byte meaning:

- Toyota: bytes 0–1 **are** the 16-bit gas command. Inactive TX is `0,0` (ENABLE lives in byte 4). Check ≈ “no non-zero gas unless allowed”.
- Wim: **byte1 is enable magic `0xA5`**, byte0 is gas. A legal enabled 0 % frame is `00 A5 nn crc`. Under Toyota’s check, `data[1] == 0xA5` is treated as **gas**, so:
  - `!allowed` + valid Wim enable → **reject** (accidentally good for “never enable without controls_allowed”).
  - `allowed` → **no check at all**: no `0xA5` requirement, no CRC, no nibble counter, no max. You can TX `byte1 = 0x00` (firmware ignores) or a garbage CRC.
- Toyota also allows `ENABLE=1` with zero gas when `!allowed` (ENABLE is not in bytes 0–1). Wim cannot express “enable” without putting `A5` in byte1.

Panda v1 for 0x200 (safety agent; research only here):

- Allowlist: `{0x200, bus 0, len 4}` only. **Not** len 6.
- Reject unless **`data[1] == 0xA5`** (even when `controls_allowed` — so inactive = **no frame**, not a disabled frame).
- Reject unless `get_longitudinal_allowed()` (never enable without `controls_allowed`; gas_pressed blocks long).
- Reject bad CRC (forward D5/FF over 0..2 vs `data[3]`).
- Reject frozen counter nibble (`data[2] & 0xF`).
- `data[0]` is gas 0..255; no extra max needed beyond 255. Rate-limit optional (accel/jerk live in carcontroller clips).
- **Do not** call `longitudinal_interceptor_checks()`.
- **Do not** TX-check 0x201. **Do not** use Toyota/Honda 0x201 RX as user gas.
- RX 0x199 @ ~10 Hz; missing > 200 ms → drop long TX.
- `brake_pressed` from `0x0A8` `dat[7] == 0x64` (not 0x19E / 0x168). Rising edge clears **long** `controls_allowed`. Lat per MADS.
- `gas_pressed` from 0x199 as above, not 0x201.

`SAFETY_BMW` = cereal `SafetyModel.bmw` = **37**. Not `SAFETY_TESLA`. Not `SAFETY_ALLOUTPUT` on the road.

---

## Leftover questions

1. **`gas_coast` (and `gas_coast(v)`)** — blocking for a tracking map. Sweep before shipping long. Until then, 0 % at `a ≤ 0` overshoots the −1.5 clip if plant regen is ~−2.1 m/s².
2. **Measure `a_regen(v)` / `a_max(v)`** from `DIR_torqueActual` + pack V/I + vEgo + PedalOut. Replace the 200/100 kW FLAG MEASURE numbers. Pack limits vs inverter vs one-pedal map may not match the cluster gauge.
3. **Jerk ±2.5 m/s³** is in §0 but not in the Phase 5 bullet list. Apply as a carcontroller rate limit on `actuators.accel` / `gas_raw`?
4. **30 / 25 km/h hysteresis** — where (CarInterface vs controlsd vs carcontroller)? Stock `minEnableSpeed` has no dropout field.
5. **`sender_example.md` is 0x201** (magic `0x5A`), not a 0x200 sender. Pre-OP gas test must use §1.1, not that file. Firmware 0x201 is bitfield `0x02`; DBC/sender say 90 — ignore for OP (do not TX).
6. **Phase 1 stranger 0x200** — §11 says ID is free (VCU does not TX 0x200; BMW PT `CruiseControlStatus` 0x200 is 8-byte DME and must stay off can2). Still log once. If a 4-byte or 8-byte 0x200 appears, stop.
7. **Clip 1686 MaxDischarge/MaxRegen in v1?** Spec: Phase 6 optional. Recommend no for first TX.
8. **Driver gas override threshold** (20 % + 5 % delta) is Phase 2 proposal — confirm on 0x199 vs foot before relying on it to stop 0x200.
9. **Does `CC.longActive` already imply panda `controls_allowed`?** Treat both as required. Never send enable on brake/gasPressed even if `longActive` glitches.
10. **DBC signal names** (`GasReq` vs percent factor) — DBC agent. Pack-by-bytes in `bmwcan.py` does not need to wait, but panda tests / cabana decode will.
11. **0x199 during enabled 0 %** looks like PASSTHROUGH (`can_norm > pedal` is false). Do not use state==DRIVEBYWIRE as the only “long is acting” flag; compare PedalOut vs GasReq when GasReq > 0.
12. **Firmware counter** requires *change*, not strict +1. OP should still increment. Skipping nibble values is OK; freezing is not.
13. **Do not fingerprint interceptor via 0x201.** Use `E90Flags.PEDAL_INTERCEPTOR`. Default product is MADS lat-only; long is opt-in.

**Not leftover (closed):** no 0x201 TX; no 0x194 TX; no DSC brake TX; no `minEnableSpeed = -1`; no Toyota helper; no `SAFETY_TESLA` / `SAFETY_ALLOUTPUT` on the road; stop sending when long is off; clip ±1.5; enable ≥ 30 km/h.
