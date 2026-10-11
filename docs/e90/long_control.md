# E90 EV longitudinal control

One-pedal interceptor. No friction-brake actuator. `E90Flags.PEDAL_INTERCEPTOR` is off until lat is proven.

## Plant

Tesla analog pedal, 0–100 % via `CRUISE_Gas` 0x200.

- 0 % = home = maximum regen the DU will give at that instant (SOC, temperature, speed, regen setting).
- Reducing pedal toward home increases regen.
- Coast / deadband ≈ a = 0.
- Pressing past coast = drive torque.

Interceptor: `out = max(bmw_pedal, can_gas)` unless torque-cut. CAN cannot lift the driver’s foot. Timeout 100 ms without 0x200 → passthrough.

## Signals

| Quantity | Source |
|---|---|
| `vEgo` | 0xCE mean(FL, FR, RL, RR). RWD driven rears slip first; do not use RL/RR only. |
| `aEgo` | 0x1A0 `AccY`, factor 0.025 m/s². Not AccX. Not 0x1A0 `VehicleSpeed` (gateway floor 10 km/h). Not dv/dt of wheels. |

## Loop

Stock `LongControl`: P = 0. Feedforward `aTarget`. I-term on `(aTarget − aEgo)`.

```
kiBP = [0, 5, 35] m/s
kiV  = [0.8, 0.5, 0.35] 1/s
a_cmd = clip(aTarget + ki * ∫(aTarget − aEgo), −1.5, +1.5)
```

Monotonic FF (not an inverted plant):

| a_cmd (m/s²) | Pedal % |
|---|---|
| −1.5 | 0 |
| 0 | 10 |
| +1.5 | **40** (`PEDAL_MAX_PCT`) |

**Hard cap 40 %** (raw 102/255). I-term cannot wind to 100 % because AccY lags `aTarget`. Neighborhood log: ~30 % ≈ +1.5 m/s². Panda rejects `GasReq > 102` even if Python is wrong. Raise `PEDAL_MAX_PCT` and `BMW_GAS_MAX_RAW` together if highway needs more; never 100 %.

Saturated 0 % = cannot regen more. Saturated 40 % = will not press more throttle.

## Pedal command (this car’s smoothness)

Control loop 100 Hz. TX 0x200 at 50 Hz.

`a_cmd` is **not** rate-limited. Spec §0 ISO-ish ±2.5 m/s³ jerk was a starting idea; it is **not** in the loop. Ride quality is **Tesla pedal %/s** in `CarControllerParams` for this E90 EV. Not ISO 15622. Not panda. Retune here; do not invent a second limiter.

| Constant | Value | This car |
|---|---|---|
| `PEDAL_MAX_DOWN_PCT_S` | 25 %/s | Toward home (more regen). 40 %→0 % in 1.6 s. |
| `PEDAL_MAX_UP_PCT_S` | 50 %/s | Away from home (press). 0 %→40 % in 0.8 s. |

Asymmetric on purpose: slower into regen than out of it.

Engage: `gas_pct` = `PedalOutPct` (0x199), then slew toward `map(a_cmd)`.

Disable (`longActive` false, speed drop below 25 km/h, cancel): **stop TX immediately**. No slew. Firmware passthrough. Foot off → home → max regen.

Panda does **not** enforce %/s. Panda only rejects `GasReq > 102`.

## Alerts

`regenLimited` (sunnypilot `OnroadEventSP`): warning only, does not disengage.

Trigger: `longActive`, `a_cmd ≤ ACCEL_MIN + 0.05`, interceptor command `gas ≤ 2 %`, held **0.5 s**.

HUD: “Take Control” / “Regen at limit — cover brake”, same class as stock `steerSaturated`.

Meaning: pedal is at home; AccY cannot meet `aTarget`. Driver must use the hydraulic brake.

Steering limit: stock `steerSaturated` (“Turn Exceeds Steering Limit”) is unchanged.

0x200: byte0 GasReq 0–255, byte1 `0xA5`, byte2 counter nibble, byte3 CRC8 poly 0xD5 init 0xFF forward over bytes 0–2. Never TX 0x194 or 0x201.
