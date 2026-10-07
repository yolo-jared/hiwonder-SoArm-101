# P16 vs P32 lift comparison (2026-10-07 ~14:24-14:26, user-approved P change)

Same session, same start pose (shoulder_lift 2166, elbow_flex 2451), 12.3 V, side camera. Each run: 45 deg lift (511 ticks,
"minus" = up), 2 s ramp, 3 s hold, return. Run A: P16 (as configured). Run B: `--set-p 32 --set-p-joints elbow_flex,shoulder_lift`
written with torque off before enable; P restored to 16 after torque-off, readback `{'elbow_flex': 16, 'shoulder_lift': 16}`.
Writes: A [40, 42]; B [21, 40, 42]. Torque-off verified both runs. D=32, I=0 unchanged.

| Joint | Run | Hold error (ticks) | Hold drive (per mille) | Slope (drive-16)/err | Peak error in ramp | Hold jitter p-p (ticks) |
|---|---|---|---|---|---|---|
| elbow_flex | P16 | 55 (4.8 deg) | 210 | 3.5 | 91 | 3 |
| elbow_flex | P32 | 27 (2.4 deg) | 206 | 7.0 | 51 | 1 |
| shoulder_lift | P16 | 45 (4.0 deg) | 174 | 3.5 | 95 | 0 |
| shoulder_lift | P32 | 19 (1.7 deg) | 150 | 7.1 | 51 | 0 |

Settle peak-to-peak at start pose: 0 ticks on all joints in both runs. Contact sheets (1 fps) show clean lift, steady hold, return.

## Wrist flex (runs C/D, ~14:40, +/-30 deg, 1.5 s ramp, 2 s hold)

| Run | "+" hold error | "+" drive | "-" hold error | Hold jitter p-p | Peak error |
|---|---|---|---|---|---|
| C P16 | 59 (5.2 deg) | 224 | 22 | 0-4 | 85 |
| D P32 | 32 (2.8 deg) | 241 | 12 | 0-2 | 49 |

Same law: 16 + 3.5 x err at P16, 16 + 7 x err at P32. P restored to 16 (readback), torque off verified.
## Shoulder pan, wrist roll, gripper (runs E/F, ~14:50, +/-20 deg, gripper in free air)

| Joint | P16 hold error +/-/return | P32 hold error +/-/return | P16 peak error | P32 peak error | Jitter |
|---|---|---|---|---|---|
| shoulder_pan | 13 / 14 / 10 | 6 / 7 / 5 | 37 | 21 | none |
| wrist_roll | 5 / 5 / 5 | 1 / 4 / 3 | 32 | 18 | none (load p-p 58 on entering hold, settled at 1 tick, load 0) |
| gripper | 14 / 12 / 16 | 6 / 5 / 6 | 39 | 23 | none |

Hold drive unchanged (friction): pan ~60, roll ~35, gripper ~60. P restored to 16 (readback), torque off verified.
Not tested: gripper squeezing an object. Max_Torque_Limit 500 caps grip drive at either P; P32 reaches the cap at about
half the leader-follower gap (inference from the drive law, not measured).

## Conclusion

- Prediction confirmed: P is the per-tick slope (3.5 at P16, 7.0 at P32); the constant 16 is unchanged. Doubling P halved the
  gravity offset (elbow 55 -> 27, shoulder 45 -> 19 ticks) at the same holding drive.
- No oscillation or jitter observed at P32 in this 45 deg lift-and-hold (data and video). Not tested: fast teleop motion,
  gripper contact, long holds/heat.
- Persistence: Lock=0, so a P32 write survives power-off; `SOFollower.connect()` -> `configure()` rewrites P16 on every
  LeRobot connect regardless.
