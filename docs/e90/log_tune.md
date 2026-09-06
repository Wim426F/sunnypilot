# E90 EV — tune from `smalldrivearoundtheblock.csv`

SavvyCAN ~172 s neighborhood drive. **No comma four.** Pedal (`0x199`) was on log **bus 1**; speed/brake/gear/SAS on log **bus 0**. Same timestamps, so PedalIn vs 0xCE accel is valid. Going forward panda CAN0 = **bus 0**; interceptor will be moved there.

## Bus decision

Do **not** splice log bus 0 and bus 1 (collisions: `0x1D6`, `0x2B2`, `0x398`). Panda sees only bus 0. Tesla/PCS stays off panda.

`0x200`: **zero frames on both buses**. Collision check passed.

`0x1A0` at standstill: **9.99 km/h** (97 × 0.103). Tracks 0xCE within ~0.55 km/h when rolling. Poison confirmed.

## Longitudinal (D, `0x0A8` byte7 = `0x04`)

0xCE RL/RR average, Δt ≈ 0.4 s.

**Foot-off regen** (PedalIn = 0), few samples (n=103):

| km/h | median a (m/s²) |
|---|---|
| 15–20 | −0.39 |
| 25–30 | −0.31 |
| 35–40 | −0.37 |
| 50–55 | −0.58 |

Not pack-limited 100 kW (−2.1 at 30 m/s). Neighborhood / torque-limited. Highway regen may be stronger. **0 % pedal still means Tesla unpressed.** Planner clip stays **−1.5**.

**PedalIn vs accel** (max pedal in this log = 35 %):

| PedalIn % | median a | median v |
|---|---|---|
| 0–5 | −0.29 | 9 kph |
| 5–10 | −0.06 | 8 kph |
| 10–15 | +0.21 | 15 kph |
| 15–20 | +0.43 | 17 kph |
| 20–25 | +0.82 | 20 kph |
| 25–30 | +1.24 | 26 kph |
| 30–35 | +2.19 | 33 kph |

Coast (a ≈ 0) ≈ **8 %**. **+1.5 m/s² ≈ 30 %**. Encoded in `CarControllerParams.ACCEL_TO_PEDAL_BP` / `PEDAL_PCT_V`. Do not extrapolate to 100 % — not in the log.

## Lateral

SAS 0xC4 while “straight” (|yaw from axle| small): about **−6 deg** offset.

steerRatio from SAS vs rear-axle yaw: **median ~10, n=7** — too few. Keep **16** until a servo + highway log.

No `0x22F` in this capture. No stalk bits (0x194 byte2 always 0).

## Still off

`E90Flags.PEDAL_INTERCEPTOR` is **off** → no 0x200 TX until 0x199 is on panda CAN0 and lat is proven.
