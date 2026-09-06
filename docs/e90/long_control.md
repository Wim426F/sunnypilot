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
| +1.5 | 100 |

I-term moves `a_cmd` when SOC/regen/power change the gain. Saturated 0 % = cannot regen more.

## Pedal command

Control loop 100 Hz. TX 0x200 at 50 Hz.

| Constant | Value | Reason |
|---|---|---|
| `PEDAL_MAX_DOWN_PCT_S` | 25 %/s | Toward home (more regen). 100 %→0 % in 4 s while tracking. Prevents a single-frame step to max regen. |
| `PEDAL_MAX_UP_PCT_S` | 50 %/s | Away from home. |

Engage: `gas_pct` initialized to `PedalOutPct` (0x199). Then slew toward `map(a_cmd)`.

Disable (`longActive` false, speed drop below 25 km/h, cancel): **stop TX immediately**. No slew. Firmware passthrough. Foot off → home → max regen.

0x200: byte0 GasReq 0–255, byte1 `0xA5`, byte2 counter nibble, byte3 CRC8 poly 0xD5 init 0xFF forward over bytes 0–2. Never TX 0x194 or 0x201.
