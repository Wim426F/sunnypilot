# E90 EV — MADS (Agent F)

Owner: MADS. Spec: `E90-SUNNYPILOT-PORT-PLAN.md` §0 modes, §6 Phase 4/6, §11. Rules: `AGENTS.md`.

Product default: **steer-only MADS**, long off, **remain-active on brake**.

## Panda `alternativeExperience`

Set in `openpilot/sunnypilot/mads/helpers.py` (`set_alternative_experience`), same wiring as hyundai/toyota:

| Bit | BMW |
|---|---|
| `ENABLE_MADS` (1024) | set when MADS param is on |
| `MADS_DISENGAGE_LATERAL_ON_BRAKE` (2048) | **never** |
| `MADS_PAUSE_LATERAL_ON_BRAKE` (4096) | **never** |

`read_steering_mode_param` forces `MadsSteeringModeOnBrake.REMAIN_ACTIVE` for `brand == "bmw"`. Brake rising edge still clears **long** `controls_allowed` in panda; lat stays requested.

Do **not** change panda long-through-brake policy here.

## MAIN mapping (0x194 stalk)

BMW **has** a cruise stalk. Do **not** add `bmw` to `MADS_NO_ACC_MAIN_BUTTON` (tesla/rivian). `MadsMainCruiseAllowed` stays.

There is no stock DCC. `mads.py` sets `allow_always = True` so lat can engage without pcm cruise.

| Stalk (carstate, RX only, never TX) | ButtonType | MADS |
|---|---|---|
| `resume` | `resumeCruise` | MAIN-**on**: `lkasEnable` if lat is off |
| `cancel` / `cancel_lever_up` | `cancel` | MAIN-**off**: `lkasDisable` if lat on and long already off |
| `plus1` / `minus1` (later Phase 6) | `accelCruise` / `decelCruise` | set-speed only; not lat toggle |

Car agent should emit those button events from 0x194. Optionally also latch `cruiseState.available` from resume/cancel (stock MAIN). Existing MADS MAIN edges then match:

- `available` False→True + `MadsMainCruiseAllowed` → `lkasEnable`
- `available` True→False → `lkasDisable`

If cancel fires while long just dropped (`enabled_prev` and not `enabled`), lat **stays** (`manualLongitudinalRequired`). That is remain-active vs long-off, not MAIN-off.

`mads` strips `buttonCancel` / `pcmDisable` so cancel never takes down lat through the stock selfdrive path.

## UI

On-road HUD (tici + mici comma four): **`regen only — cover brake`** when `CarParams.brand == "bmw"`.

- helper: `openpilot/selfdrive/ui/sunnypilot/onroad/bmw_regen_hint.py`
- tici: `openpilot/selfdrive/ui/sunnypilot/onroad/hud_renderer.py` (under current speed)
- mici: `openpilot/selfdrive/ui/sunnypilot/mici/onroad/hud_renderer.py`

MADS settings lock steering-on-brake to Remain Active for bmw. MAIN cruise toggle stays selectable.

## Out of scope

- Experimental long / stop-and-go
- TX 0x194 / 0x200 / 0x22E
- `opendbc/car/bmw` (car agent)
- panda `bmw.h` long-on-brake
