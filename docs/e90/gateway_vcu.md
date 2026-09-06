# E90 EV — gateway + VCU (can2)

Source-cited notes for panda on **can2 only**. No gateway or VCU code was changed in this ticket. Do **not** change the 0x1A0 rewrite here.

Local trees (not in this repo):

- `/home/wimboone/Documents/CAN-gateway` — origin `https://github.com/Wim426F/CAN-gateway.git` (`src/main.cpp` HEAD `8d08aac` “Pass all BMW to VCU messages through”).
- `/home/wimboone/Documents/Stm32-vcu` — `BMW_E90` + `F30_Lever` + `EvControlsT2C`. Live params: `Parameters/paramsE90VCU_v223wim.json`.

Spec: `E90-SUNNYPILOT-PORT-PLAN.md` §1, §3.3, §4, §11 Q1/Q3/Q7. Phase 1 human checklist: [`docs/can_map.md`](../can_map.md).

---

## Verdicts (read this first)

| Item | Result |
|---|---|
| **0x200 collision** | **ID is free on can2 as closed in §11 Q1.** VCU **does not TX 0x200** (no `Send(0x200)` in `Stm32-vcu`). Gateway **does not originate** 0x200. **But** the gateway **will forward BMW PT 0x200 CAN1→CAN2** if a DME/ghost ever emits `CruiseControlStatus` — there is **no ID drop** on that path. Pedal `CRUISE_Gas` (4 B) owns the ID. Phase 1 must still log **zero stranger 0x200**. |
| **Brake** | VCU `SendAbsDscMessages` → **0x0A8** 8 B @ **10 ms**. Byte7 **`0x64` pressed / `0x04` released**. Panda: `brake_pressed = (dat[7] == 0x64)`. Do **not** use 0x19E or ICE `EngineAndBrake` 0x168. |
| **0x1A0** | Gateway **stores** CAN1 0x1A0, **does not forward the raw frame**, and **TX a mutated copy to CAN2 every 5 ms** with 12-bit speed **clamped ≥ 10 km/h**. **Do not use for vEgo.** Do **not** change this rewrite in the OP port. Speed = `Wheel_RL/RR` on **0xCE**. |
| **Harness** | **Panda CAN0 = gateway CAN2 = can2_powertrain only.** One `CanBus.pt = 0`. Servo + pedal + VCU vehicle CAN + forwarded BMW PT all on that bus. Do not splice PT-CAN until 0xCE and 0x194 are proven on can2. |

---

## 1. Architecture

```
BMW PT-CAN 500 kbit  ──CAN1──►  Teensy 4.0 gateway  ◄──CAN2──  can2_powertrain 500 kbit
  DSC 0xCE / 0xC4 / 0x1A0                         VCU (STM32 CAN2 = VehicleCan),
  SZL 0x194 / 0x1D6                               T2C + Tesla DU (STM32 CAN1 = InverterCan,
  CAS 0x130, EMF 0x3FE, …                         same physical bus per §11 Q7),
                                                  Pedal-interceptor, StepperServoCAN
        │                                                │
        │  app: all CAN1→CAN2 except 0x1A0               │
        │  0x1A0 rewritten every 5 ms, min 10 km/h       │
        │  app: all Can2.read() → CAN1                   │
        ▼                                                ▼
                              comma four panda
                         bus0 = can2_powertrain
```

Gateway buses (`CAN-gateway/src/main.cpp:7-8`):

```7:8:/home/wimboone/Documents/CAN-gateway/src/main.cpp
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_64> Can1; // BMW-E90 side
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_64> Can2; // VCU side
```

VCU CAN assignment (`Stm32-vcu/src/stm32_vcu.cpp:1353-1370`, params):

| Param | Value | Meaning |
|---|---|---|
| `Vehicle` | 9 | `vehicles::vBMW_E90` |
| `GearLvr` | 1 | `ShifterModes::BMWF30` |
| `Inverter` | 9 | `InvModes::T2C` |
| `VehicleCan` | 1 | STM32 CAN2 — `BMW_E90` + `F30_Lever` |
| `InverterCan` | 0 | STM32 CAN1 — `EvControlsT2C` |
| `ShuntType` | 4 | HVCU, **not** BMW SBOX (SBOX RX of 0x200 is unused) |

`selectedVehicle` / `selectedShifter` share `vehicle_can` (`stm32_vcu.cpp:1113-1118`).

---

## 2. What the gateway forwards / blocks

### 2.1 Application logic (this is law for OP)

`loop()` (`main.cpp:103-115`):

```103:115:/home/wimboone/Documents/CAN-gateway/src/main.cpp
void loop() {
  if (Can1.read(rxMsg)) {
    lastCan1Time = millis(); // Any message resets
    if (rxMsg.id == 0x1A0) {
      last1A0Msg = rxMsg;
    } else {
      Can2.write(rxMsg);
    }
  }
  
  if (Can2.read(rxMsg)) {
    Can1.write(rxMsg);
  }
```

- **CAN1 → CAN2:** every 11-bit frame **except 0x1A0** is written to can2 unmodified.
- **0x1A0:** stored only; a **rewritten** copy is TX on CAN2 on a 5 ms timer (below).
- **CAN2 → CAN1:** every frame `Can2.read()` returns is written to PT-CAN. No software ID list.

Commit `8d08aac` (2026-06-15) commented out **all Can1 mailbox filters** (“Pass all BMW to VCU messages through”). The leftover comments still name the old allowlist, including `MB21, 0x200 // Cruise control status` (`main.cpp:52`). That comment is **not** a block — Can1 now accept-all.

### 2.2 0x1A0 rewrite (do not change)

Timer and clamp (`main.cpp:18-19`, `96-100`, `122-145`):

```18:19:/home/wimboone/Documents/CAN-gateway/src/main.cpp
const unsigned long X1A0_SEND_INTERVAL = 5; //ms
unsigned long last1a0SendTime = 0;
```

```122:145:/home/wimboone/Documents/CAN-gateway/src/main.cpp
  if (millis() - last1a0SendTime >= X1A0_SEND_INTERVAL) {
    last1a0SendTime = millis();
    txMsg = last1A0Msg;

    // Extract raw VehicleSpeed (12-bit, little-endian, unsigned)
    uint16_t raw = ((uint16_t)(txMsg.buf[1] & 0x0F) << 8) | txMsg.buf[0];

    // Calculate physical speed
    float speed_kph = raw * 0.103f;

    // Apply minimum limit
    if (speed_kph < 10.0f) {
      speed_kph = 10.0f;
    }

    // Calculate new raw value
    uint16_t new_raw = (uint16_t)roundf(speed_kph / 0.103f);

    // Pack back into buffer, modifying only the VehicleSpeed bits
    txMsg.buf[0] = (uint8_t)(new_raw & 0xFF);
    txMsg.buf[1] = (txMsg.buf[1] & 0xF0) | (uint8_t)((new_raw >> 8) & 0x0F);

    Can2.write(txMsg);
  }
```

Facts:

- Period **5 ms** (200 Hz) on **CAN2 only**. Stock DSC 0x1A0 still lives unmodified on PT-CAN (CAN1).
- Scale **0.103 km/h/bit**, 12-bit, little-endian in `buf[0]` + low nibble of `buf[1]`. Matches `bmw_e9x_e8x.dbc` `BO_ 416 Speed` `VehicleSpeed : 0|12@1- (0.103,0)`.
- **Floor 10.0 km/h.** Standstill on can2 still shows ~10 km/h. That is why OP **must not** use 0x1A0 for vEgo / standstill.
- Only the speed nibble-field is rewritten. **Checksum_416 (byte 7) is not recomputed.** VCU `handle1A0` does not verify it (`BMW_E90.cpp:140-147`).
- Before the first CAN1 0x1A0, `last1A0Msg` is seeded `id=0x1A0, len=8, buf[0]=0x6F` (`main.cpp:96-100`) → raw 111 ≈ **11.4 km/h**, then still clamped ≥ 10.
- VCU also consumes this mutated frame: `handle1A0` uses **`* 0.1` not 0.103** (`BMW_E90.cpp:144`). OP must not copy that; use 0xCE.

True-zero 0x1A0 (if a DU/EPS ever needs it) is a **dedicated gateway ticket**, not this port (`E90-SUNNYPILOT-PORT-PLAN.md` §3.3 / Phase 7).

### 2.3 Can2 mailbox filters (do not confuse with the can2 wire)

`initCanFilters()` still programs **Can2** RX mailboxes (`main.cpp:57-73`):

| MB | ID | Comment in source |
|---|---|---|
| 0 | 0x0AA | ABS/DSC info |
| 1 | 0x0A8 | Brake pedal status |
| 2 | 0x0A9 | Counter |
| 3 | 0x0BA | Gear (VCU TX of 0x0BA is **commented out**) |
| 4 | 0x1D0 | Dashboard engine info |
| 5 | 0x332 | Instrument cluster init |
| 6 | 0x1D2 | Shift position (VCU TX **disabled**) |
| 7 | 0x592 | Error lights |
| 8 | **0x200** | “Cruise control status” |
| 9–14 | 0x1B6, 0x381, 0x3B3, 0x3B4, 0x3B0, 0x1B5 | cluster/power |

These filters affect **only what the Teensy reads from can2 to copy onto PT-CAN**. They do **not** hide frames from panda. Panda is a peer on the can2 wire and sees gateway TX + every other node on that bus.

Layout caveat (no `setMaxMB()` in `main.cpp`): FlexCAN_T4 `begin()` → `disableFIFO()` splits hardware mailboxes half RX / half TX; the first quarter of the RX half is STD, the rest of that half EXT (`FlexCAN_T4.tpp:204-214`). i.MX RT106x MCR `MAXMB` reset is 16 boxes → **MB0–3 STD RX, MB4–7 EXT RX, MB8–15 TX**. Consequences:

- `Can2.setMBFilter(MB8, 0x200)` sits on a **TX** mailbox → **no-op**.
- MB4–7 filters (0x1D0, 0x332, …) are STD IDs programmed onto **EXT** mailboxes → will not collect 11-bit frames.
- MB0–3 collect **0x0AA / 0x0A8 / 0x0A9 / 0x0BA** only.

So CAN2→CAN1 is **not** “every can2 ID” in hardware, even though the C++ looks that way. Brake **0x0A8 is in MB1** and **is** expected to reach PT-CAN (cluster brake lamps). Pedal/OP **0x200 is not expected to leak onto PT-CAN** through this RX path. That leak-prevention is accidental (mailbox layout), not an explicit `if (id==0x200) drop`.

CAN1→CAN2 is the collision path that matters for OP: Can1 filters commented out, accept-all STD RX, software only special-cases 0x1A0.

### 2.4 Other gateway behaviour

- Sleep: 60 s with no CAN1 RX → hibernate on pin 23 (`main.cpp:24-25`, `147-149`). Irrelevant to OP while driving.
- Baud 500 kbit both sides (`main.cpp:88-91`).

---

## 3. Collision checklist (can2, from source)

Stock BMW `CruiseControlStatus` is **DME**, 8 bytes, ID 0x200 (`opendbc_repo/opendbc/dbc/bmw_e9x_e8x.dbc` `BO_ 512`). This conversion has no DME; VCU emulates the DME-ish ABS/engine set (0x0AA/0x0A8/0x0A9/0x1D0) instead of 0x200.

| ID | On can2 according to code? | Who | Collision vs OP |
|---|---|---|---|
| **0x200** | **Only if BMW PT still TX it** (gateway **forwards CAN1→CAN2**). VCU **does not TX**. Gateway **does not originate**. | Pedal/OP `CRUISE_Gas` 4 B (Phase 5+). Stock DME 8 B if resurrected. | **Closed Q1: free.** Still log once. Stranger 0x200 → stop. See §7 ticket. |
| **0x201** | **Yes** (T2C). | `EvControlsT2C::SetTorque` 3 B @ 50 ms (`EvControlsT2C.cpp:169-191`). Regen-cut on `din_brake`; all-torque-cut in N/P until DU reports N. | **Not OP.** Never TX. Panda allowlist later = `{0x22E, 0x200}` only. |
| **0x194** | **Yes** (forwarded). | SZL on PT-CAN. Gateway CAN1→CAN2 unmodified. VCU does **not** TX 0x194. | **UI only. Never TX.** |
| **0xCE** | **Yes** (forwarded). | DSC `WheelSpeeds`. Unmodified. VCU does **not** TX. | **vEgo** = `Wheel_RL/RR`. |
| **0xC4** | **Yes** (forwarded). | DSC/SAS `SteeringWheelAngle`. Unmodified. VCU does **not** TX. | SAS for model. |
| **0x0A8** | **Yes** (VCU TX). | `BMW_E90::SendAbsDscMessages` on VehicleCan. | **brake_pressed.** Same *ID* as ICE `EngineAndBrake` — **different payload**. |

VCU `Send(0x200)` grep: **no matches** in `Stm32-vcu/src`. BMW SBOX `RegisterUserMessage(0x200)` is current-sensor **RX** and is not selected (`ShuntType=4`).

---

## 4. VCU `SendAbsDscMessages` 0x0A8 (brake)

### 4.1 Period

`BMW_E90::Task10Ms` (`BMW_E90.cpp:201-207`) calls `SendAbsDscMessages` whenever `CANWake`:

```201:207:/home/wimboone/Documents/Stm32-vcu/src/BMW_E90.cpp
void BMW_E90::Task10Ms()
{
    if(CANWake)
    {
        SendAbsDscMessages(Param::GetBool(Param::din_brake));
    }
}
```

Scheduler: `s.AddTask(Ms10Task, 10)` (`stm32_vcu.cpp:1399`); `Ms10Task` → `selectedVehicle->Task10Ms()` (`stm32_vcu.cpp:615`).

`CANWake` is 0x480 NM: `bytes[1] == 0x32` → false, else true (`BMW_E90.cpp:172-184`). 0x480 is registered RX (`BMW_E90.cpp:57`).

**Period: 10 ms (100 Hz) while the PT network is awake.**

### 4.2 0x0A8 payload, checksum, byte7

```354:378:/home/wimboone/Documents/Stm32-vcu/src/BMW_E90.cpp
    uint8_t a8_brake;

    if(Brake_In)
    {
        a8_brake=0x64;
    }
    else
    {
        a8_brake=0x04;
    }

    int16_t check_A8 = (A81+0x21+0xe0+0x21+0x1f+0x0f+a8_brake+0xa8);
    check_A8 = (check_A8 / 0x100)+ (check_A8 & 0xff);
    check_A8 = check_A8 & 0xff;

    bytes[0]=check_A8;  //checksum
    bytes[1]=A81; //counter byte
    bytes[2]=0x21;
    bytes[3]=0xe0;
    bytes[4]=0x21;
    bytes[5]=0x1f;
    bytes[6]=0x0f;
    bytes[7]=a8_brake;  //brake off =0x04 , brake on = 0x64.

    can->Send(0x0A8, bytes, 8); //Send on CAN
```

| Byte | Content |
|---|---|
| 0 | BMW fold checksum: `sum(A81 + 0x21 + 0xE0 + 0x21 + 0x1F + 0x0F + brake + 0xA8)`, then `(sum/256)+(sum&0xFF)` |
| 1 | Counter `A81` (increments every 10 ms; reload with the ABS set when `BA5==0x5C`, `BMW_E90.cpp:412-429`) |
| 2–6 | Constants `21 E0 21 1F 0F` |
| **7** | **`0x64` pressed, `0x04` released** |

OP / panda: **`brake_pressed = (0x0A8 dat[7] == 0x64)`**. Do not parse e9x DBC `EngineAndBrake` bit61 on this ID.

### 4.3 Where `din_brake` comes from

`SendAbsDscMessages(Param::GetBool(Param::din_brake))`. `GetBool` is true for any non-zero int.

1. **Tesla DU 0x118** `DI_brakePedalState` bits 19–20 (`EvControlsT2C.cpp:117-121`):

```117:121:/home/wimboone/Documents/Stm32-vcu/src/EvControlsT2C.cpp
    case 0x118: { // ID118DriveSystemStatus
        uint8_t brakePedalState = (bytes[2] >> 3) & 0x03;
        Param::SetInt(Param::din_brake, brakePedalState);
```

Tesla DBC (`tesla_model3_party.dbc` `BO_ 280`): `DI_brakePedalState` 0=OFF, 1=ON, 2=INVALID. **INVALID (2) is also bool-true** → 0x0A8 byte7 `0x64`. Phase 1: confirm foot tracks `0x64`/`0x04`; treat INVALID as a VCU quirk if it ever latches.

2. **Digital OR** if `din_brake == 0`: `utils::GetDigInputs` (`utils.cpp:68-71`) sets `din_brake` from `DigIo::brake_in` / CAN_IO_BRAKE. Cannot *clear* a T2C-set value.

### 4.4 Other frames in the same 10 ms function (not OP brake)

Same `SendAbsDscMessages` also TX:

| ID | Len | Role |
|---|---|---|
| **0x0AA** | 8 | DME-style engine/RPM. Checksum in byte0 over bytes1–7+`0xAA`. Pedal byte **forced 0**. RPM from `Param::speed` when `Ready()` (KL15). **Not** `brake_pressed`. |
| **0x0A9** | 8 | Companion counters `A90/A91`, constants. No brake bit. |
| **0x0BA** | — | **Disabled** (commented; “never appeared in original logs”). |

`Task100Ms` / `Task200Ms` (not brake): `0x1D0` engine temps/fuel @ 100 ms; `0x332` dash-init burst once at T15; `0x592` on `errlights` change; **0x1D2 gear display disabled** (manual cluster, F30 owns gear).

VCU does **not** emit stock DSC **0x19E**. If 0x19E appears on can2 it is forwarded PT.

---

## 5. Gear 0x3FD / EPB 0x3FE

### 5.1 0x3FD — VCU **TX** (F30 GWS)

`F30_Lever::sendcan` @ **100 ms** (`F30_Lever.cpp:310-346`, `Task100Ms` `353-380`), on **VehicleCan** (same bus as 0x0A8):

```326:332:/home/wimboone/Documents/Stm32-vcu/src/F30_Lever.cpp
    bytes[1] = Cnt3FD;
    bytes[2] = DirOut;
    bytes[3] = 0x00;
    bytes[4] = SportNum;
    bytes[0] = get_crc8(bytes, 5, 0x70, 1);

    can->Send(0x3FD, bytes, 5);
```

| Byte | Field |
|---|---|
| 0 | CRC8 (poly 0x1D, init 0, xor final 0x70, skip byte0) |
| 1 | Counter nibble, 0…0xE |
| **2** | **Direction:** `0x20` Park, `0x40` Reverse, `0x60` Neutral, `0x80` Drive, `0x81` Sport (`F30_Lever.cpp:22-27`, `314-325`) |
| 3 | 0x00 |
| 4 | SportNum |

Forced Park (`DirOut = 0x20`) in every `opmode` except `MOD_RUN` and `MOD_OFF` (`F30_Lever.cpp:322-325`) — comment: engages EPB. OP: **disable long in R/P** from this byte. Same ID as plan “1021 Direction”.

Also TX **0x202** 2 B backlight (`F30_Lever.cpp:340-346`). Not used by OP.

RX for the lever is 0x197 / 0x55E / 0x65E (`F30_Lever.cpp:89-92`), not 0x3FD.

### 5.2 0x3FE — VCU **RX only** (BMW EPB status)

Registered (`BMW_E90.cpp:60`). **No `Send(0x3FE)`.**

```194:199:/home/wimboone/Documents/Stm32-vcu/src/BMW_E90.cpp
void BMW_E90::handle3FE(uint32_t data[2])
{
    uint8_t* bytes = (uint8_t*)data;
    bool parkingbrake_engaged = bytes[0] == 2 ? 1 : 0;
    Param::SetInt(Param::handbrk, parkingbrake_engaged);
}
```

Source: BMW EMF/EPB on **PT-CAN**, forwarded CAN1→CAN2 (unfiltered). Optional OP `parkingBrake` when **byte0 == 2**. Not in vendored `bmw_e9x_e8x.dbc` as `BO_ 1022`.

---

## 6. Recommended panda harness

Closed §11 Q7 + `AGENTS.md`:

- **Single bus.** Panda **CAN0** ↔ Teensy **CAN2** (can2_powertrain), 500 kbit.
- Nodes on that wire: gateway (forwarded BMW PT + mutated 0x1A0), VCU vehicle CAN (0x0A8/0x0AA/0x0A9/0x1D0/0x3FD), T2C/DU (0x107/0x118/0x201), pedal (0x199, RX 0x200/0x201), servo (0x22E/0x22F).
- **Do not** put panda on PT-CAN (gateway CAN1). You would see raw 0x1A0 (true zero) **and** miss pedal/servo/T2C.
- **Do not** add a second isolated servo bus unless cabana shows errors.
- Phase 1: `SAFETY_SILENT` / `SAFETY_NOOUTPUT`. Confirm 0xCE + 0x194 on bus 0 before any other splice.

`CanBus.pt = 0` only.

---

## 7. Ticket (docs only) — gateway filter BMW 0x200

**Not implemented.** Gateway is **not** currently TX’ing 0x200 of its own. VCU is **not** TX’ing 0x200. **No loud “already colliding” warning.** Closed Q1 stands. Phase 1 must still prove the wire.

### If Phase 1 sees a stranger 0x200 on can2 — stop OP TX

Open a **CAN-gateway** ticket (do **not** mix with 0x1A0 work):

**Title:** Drop BMW PT `0x200` on CAN1→CAN2 (protect pedal `CRUISE_Gas`)

**Why:** Pedal interceptor and (later) OP own **0x200** as 4-byte `CRUISE_Gas` (`enable=0xA5`, CRC8 D5). Stock DME `CruiseControlStatus` is the same ID, 8 bytes. `loop()` forwards **every** CAN1 ID except 0x1A0 (`main.cpp:106-110`). Can1 filters were deliberately removed in `8d08aac`. If DME/DSC/a logger ever emits 0x200 on PT-CAN, can2 will have two 0x200 senders and the interceptor will fight.

**Do:**

1. In the CAN1 RX path, **do not** `Can2.write` when `rxMsg.id == 0x200`. Drop, do not rewrite.
2. Optional, same ticket: drop `0x200` on CAN2→CAN1 so OP/pedal gas never appears on PT-CAN. Today that leak is probably already blocked by Can2 mailbox layout (`MB8` TX no-op) — make it an explicit `if`.
3. **Do not** touch the 0x1A0 5 ms / min-10 km/h rewrite.
4. Keep forwarding 0xCE, 0xC4, 0x194, 0x0A8, 0x3FE.

**Verify:** cabana on panda CAN0, long off → **zero** 0x200. Interceptor test sender → only 4-byte `CRUISE_Gas`. BMW PT (CAN1) must not show 4-byte 0x200 if step 2 is done.

**If Phase 1 is clean (no 0x200):** leave the gateway as-is until a stranger frame appears. Do not pre-emptively filter.

---

## 8. OP mapping (from this research)

| Need | Use | Do not use |
|---|---|---|
| vEgo | 0xCE `Wheel_RL/RR` | 0x1A0 (10 km/h floor); VCU `Veh_Speed` (*0.1 on mutated 0x1A0*) |
| `brake_pressed` | 0x0A8 byte7 `== 0x64` | 0x19E; ICE 0x168 bit61; Tesla 0x118 directly |
| `parkingBrake` | 0x3FE byte0 `== 2` (optional) | — |
| gear | 0x3FD byte2 P/R/N/D | VCU 0x1D2 (disabled); 0x0BA (disabled) |
| stalk UI | 0x194 RX | TX 0x194 |
| long actuator | 0x200 4 B interceptor, Phase 5+ | BMW 8 B 0x200; Toyota 6 B 0x200 |
| torque-cut | ignore / other device 0x201 | OP TX 0x201 |
| panda bus | CAN0 = can2 | PT-CAN / Tesla Party / `SAFETY_TESLA` |
