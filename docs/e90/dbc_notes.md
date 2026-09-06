# E90 EV — DBC notes (Agent A)

Sources: dirty `/home/wimboone/Downloads/BMW_can2_powertrain.dbc`, in-repo `opendbc_repo/opendbc/dbc/bmw_e9x_e8x.dbc`, pedal firmware `pedal_arduino.ino`, VCU `Stm32-vcu/src/BMW_E90.cpp` + `i3LIM.cpp`, PCS `stm32-M3-PCS/src/PCSCan.cpp`. ocelot from RetroPilot `Ocelot-steering-dev`.

## Files

| File | Action |
|---|---|
| `opendbc_repo/opendbc/dbc/bmw_can2_powertrain.dbc` | cleaned Wim DBC |
| `opendbc_repo/opendbc/dbc/ocelot_controls.dbc` | copied from RetroPilot Ocelot-steering-dev |
| `opendbc_repo/opendbc/dbc/bmw_e9x_e8x.dbc` | **unchanged** |

Downloads `bmw_e9x_e8x.dbc` is 18 lines longer than the in-repo copy (whitespace / extra empty messages, not a protocol win). Kept the in-repo file.

## What was cleaned in `bmw_can2_powertrain.dbc`

- Duplicate `BO_ 409 / 512 / 513` (second copy started at the truncated `BO_ 932` line). One of each remains.
- Truncated `BO_ 932 PCS_AlertMatrix_From_PCS: 8 PCSO_ 409 PEDAL_Status...`. Reconstructed as 8-byte PCS: `PCSAlertPage` (byte0 low nibble, the only original signal that survived) + `PCSAlertBits` (bits 4..63). Matches PCS `handle3A4` (page mux, bits 4..63 = alerts). Per-alert Tesla names were not in the truncated paste.
- Duplicate `BO_ 690`: dropped Tesla `ChargePowerRequest_From_PCS_Controller` (5 bytes). Kept `Measurement_From_i3LIM` (8 bytes). See below.
- `VAL_` / `CM_` moved out of `BO_` blocks. Hex `VAL_ 919` `0x03`/`0x02` converted to `3`/`2` so `CANDefine` `int()` works. Dangling `VAL_ 920 SystemState` (no `SG_`) dropped.
- Mux `SG_` spacing on `MCU_PDO_from_MCU` normalized so opendbc `SGM_RE` matches.
- Unit strings metric (`V`, `A`, `C`, `s`, `min`, `kW`). Firmware CAN IDs unchanged.

## 0x201 `VEHICLE_TorqueCut` (firmware alignment)

Dirty DBC had `SG_ TorqueCut : 0|8` and `VAL_ 513 TorqueCut 90 "CUT"`. Pedal firmware is a bitfield:

| Bit | Name | Firmware |
|---|---|---|
| byte0 bit0 | `REGEN_CUT` (0x01) | `din_brake`; **ignored** by pedal |
| byte0 bit1 | `ALL_TORQUE_CUT` (0x02) | `neutralPending`; force 0 % |
| byte1 low nibble | `CutCounter` | must increment |
| byte2 | `CutChecksum` | CRC8 poly 0xD5 init 0xFF over bytes 0..1, **forward** |

Magic 90 is gone. Panda still must **not** TX 0x201 in v1.

## 0x200 / 0x199 (kept)

`CRUISE_Gas` 4 bytes, `GasEnable` VAL 165 (`0xA5`). CRC8 poly 0xD5 init 0xFF over bytes 0..2, **forward** (Toyota interceptor iterates backwards; do not call `create_gas_interceptor_command`). `PEDAL_Status` 3 bytes, `PedalState` 0/1/2.

## 0x0A8 `SendAbsDscMessages` (added)

`BO_ 168` 8 bytes VCU. `BrakeMagic` at `56|8`: `0x64` pressed, `0x04` released (`BMW_E90.cpp`). Checksum is BMW sum-fold of bytes 1–7 + `0xA8`. This is live brake. **Do not** use e9x `EngineAndBrake` 0x168 (ICE DME) or 0x19E.

## 690 conflict

Same ID 0x2B2 cannot have two layouts.

| Layout | DLC | Who | Kept? |
|---|---|---|---|
| `Measurement_From_i3LIM` | 8 | VCU `i3LIM.cpp` `handle2B2` (CCS V/I) | **yes** — this car’s VCU RX |
| `ChargePowerRequest_From_PCS_Controller` | 5 | PCS `Msg2B2`: `ChargerPower` 0\|16@1+ factor 0.001 kW, `ChargerEnable` byte2 `0x02`/`0x00` | **no** — documented only |

If PCS 5-byte 0x2B2 and i3LIM 8-byte 0x2B2 ever share can2, that is a hardware collision, not a DBC mux.

## ocelot_controls.dbc

Upstream RetroPilot file kept as-is, including extra IDs 512/513 (Toyota-style 6-byte pedal) and 526/527 (Ocelot brake). Servo IDs used here:

- `STEERING_COMMAND` 0x22E (558) 5 bytes: `STEER_TORQUE` 32\|8@1- 0.125 Nm, `STEER_ANGLE` 16\|16@1-, `STEER_MODE` 12\|2, `COUNTER` 8\|4, `CHECKSUM` 0\|8
- `STEERING_STATUS` 0x22F (559) 8 bytes

**Checksum comment drift:** upstream CM_ on 558 CHECKSUM says “CRC8 per J1850”. Plan §3.2 / StepperServoCAN is **sum-fold init=addr**. Packer must implement sum-fold, not the DBC comment. 559 CHECKSUM comment (“8bit sum of all bytes and message id”) matches.

**Do not load ocelot 512/513 as the Wim pedal.** Wim pedal is `bmw_can2_powertrain` 4-byte 0x200 / 3-byte 0x201.

## bmw_e9x_e8x.dbc collisions (separate parser)

Same numeric IDs, different meaning. Load as a **separate** CANParser, never concatenate with the powertrain DBC.

| ID | e9x | powertrain |
|---|---|---|
| 168 (0xA8) | `EngineAndBrake` ICE DME | `SendAbsDscMessages` VCU brake |
| 512 (0x200) | `CruiseControlStatus` | `CRUISE_Gas` interceptor |

Closed decision Q1: VCU does not TX 0x200. Pedal owns it. Gateway still forwards BMW PT 0x200 if a DME/ghost ever emits `CruiseControlStatus` — Phase 1 must log that no stranger 0x200 exists.

## Parse result

`PYTHONPATH=opendbc_repo python -c "from opendbc.can import CANParser, CANPacker, CANDefine; ..."`:

| DBC | msgs | VAL_ | result |
|---|---|---|---|
| `bmw_can2_powertrain` | 70 | 53 | ok |
| `bmw_e9x_e8x` | 326 | 6 | ok |
| `ocelot_controls` | 6 | 4 | ok |

opendbc `unittest` `opendbc.can.tests` (parser + define + packer + checksums): 49 tests OK.

cantools `strict=False` loads all three. `strict=True` fails only on source overlap `PCS_LineStatus_From_PCS` (`ACAmps` vs `ACVolts`) in the powertrain DBC — leftover from the Wim file, not introduced here.

## Leftover / not fixed

- Several PCS/i3LIM frames remain ID-only shells (`BO_` with no `SG_`).
- Overlapping signals on some PCS mux frames (`BO_ 708` mux vs `LVVolts`) as in the source DBC.
- `BO_ 920` still 7 bytes / incomplete vs a dropped `SystemState` VAL.
- `BO_ 932` is a reconstructed mux+bitmap, not the full Tesla per-alert matrix.
- ocelot 558 CHECKSUM comment vs sum-fold (above).
