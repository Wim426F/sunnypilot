# SAFETY_BMW (37) — Phase 2 notes

Panda safety for the 2006 E90 EV: StepperServoCAN on 0x22E/0x22F and Wim pedal interceptor on 0x200. Enum matches cereal `CarParams.SafetyModel.bmw` = **37**. Not TESLA, not BODY, not `SAFETY_ALLOUTPUT`.

## TX allowlist (bus 0 only)

| ID | Len | Relay | Role |
|---|---|---|---|
| 0x22E | 5 | yes (actuator) | `STEERING_COMMAND` |
| 0x200 | 4 | no (interceptor, like Toyota) | `CRUISE_Gas` |

Not injectable: **0x194** (stalk), **0x201** (torque-cut, another device), DSC/ABS, **0x1A0**.

## RX (bus 0)

| ID | Len | Hz | Use |
|---|---|---|---|
| 0x199 | 3 | 10 | PedalIn/Out %; long TX needs this live (&lt; 200 ms) |
| 0xCE | 8 | 50 | `Wheel_RL/RR` vEgo, 0.0625 km/h/bit. **Not 0x1A0.** |
| 0xC4 | 7 | 100 | SAS liveness |
| 0x22F | 8 | 100 | Servo torque + checksum/counter; lat TX needs this live (&lt; 50 ms) |
| 0x194 | 4 | 5–25 | Stalk buttons (RX only) |
| 0x0A8 | 8 | 100 | VCU brake: byte7 `0x64` pressed, `0x04` released |

## Checksums / counters

**0x200 CRC8:** poly `0xD5`, init `0xFF`, **forward** over bytes 0..2, compare byte 3. Do **not** use Toyota `crc8_pedal` (reverse loop) or `longitudinal_interceptor_checks()` (byte1 is enable `0xA5`, not gas).

**0x200 counter:** byte2 low nibble must increment mod 16.

**0x22E/0x22F checksum:** BMW 8-bit sum-fold: init=`addr`, add all bytes with checksum byte 0 during calc, `(s & 0xFF) + (s >> 8)`. Matches E8x `calc_checksum_8bit` / StepperServoCAN GUI. (ocelot DBC comment saying J1850 CRC8 on 0x22E is wrong vs firmware.)

**0x22E counter:** byte1 low nibble, must increment mod 16.

## Actuation policy

- Steer: TorqueControl (mode 1) only. SoftOff (3) / Off (0) must carry 0 torque. AngleControl (2) rejected.
- Torque scale: 0.125 Nm/bit, **max 12 Nm = 96 counts**. Rate up 1 count/10 ms (8-bit floor on 10 Nm/s). `rt_delta` 25 counts / 250 ms. Meas vs cmd error 2 Nm (16 counts) from 0x22F.
- Long: TX 0x200 only if `get_longitudinal_allowed()` (controls_allowed and not gas override). **All** 0x200 blocked otherwise, including 0% / enable 0. Enable byte must be `0xA5`.
- `gas_pressed` iff PedalInPct &gt; 20 **and** PedalInPct &gt; PedalOutPct + 5.
- `brake_pressed` iff 0x0A8 byte7 == `0x64`. Rising edge clears `controls_allowed` (long). MADS lat remain-active uses stock `mads_state_update` + `ENABLE_MADS` / not `DISENGAGE_LATERAL_ON_BRAKE`.
- Virtual cruise (no stock PCM): 0x194 plus1/resume latches engage, cancel (up or up/down) clears. Idle holds. Tests: `_pcm_status_msg(True)` = resume bit, `False` = cancel bit.

## Tests

`python -m pytest opendbc/safety/tests/test_bmw.py -q` from `opendbc_repo` (builds libsafety).

Result: **82 passed, 8 skipped** (skips: no regen paddle, no MADS LKAS button, no second speed source). Release `-DALLOW_DEBUG` off still compiles `SAFETY_BMW`.

Covered: torque spam / rate, 0x22E checksum and counter stall, 0x200 without `0xA5`, CRC fail, blocked without controls, Toyota 6-byte 0x200 rejected, Toyota inactive 4-byte (byte0=byte1=0) rejected, 0x0A8 brake disengages long, 0x194/0x201/0x19E/0x1A0 TX false, MADS lat-without-long, 0x199/0x22F timeouts.

## Not done

- No `SAFETY_ALLOUTPUT` on the road.
- No panda TX of 0x201 (v1).
- Flash panda and cabana TX-blocked-in-silent is a human Phase 2 exit step.
