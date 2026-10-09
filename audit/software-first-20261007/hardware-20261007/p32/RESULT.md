# P16 vs P32 lift comparison (2026-10-07 ~14:24-14:26, user-approved P change)

Raw data (samples, summaries, probes, contact sheets) is local only, in the main checkout's gitignored
`test-output/hiwonder-evidence-20261007/hardware-20261007/p32/`.

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

## Gripper free close, jaw on jaw (runs G/G2/H, ~14:45-14:55, `gripper_grasp.py`, gripper-only torque)

Closing = decreasing ticks. Target range_min+40 = 1456, but the jaws meet at 1488-1491: calibration range_min (1416)
is ~75 ticks past contact, so a "fully closed" command always stalls jaw on jaw.

| Run | Contact pos | Stall drive | Stall current (raw) | Overload flagged after contact |
|---|---|---|---|---|
| G P16 | 1491 | 139 (= 16 + 3.5 x 35) | 115 | ~1.95 s (cleanup failed, see incident) |
| G2 P16 | 1491 | 139 | 115 | 2.02 s |
| H P32 | 1488 | 241 (= 16 + 7 x 32) | 204 | 1.96 s |

- Overload trips ~2.0 s after the jaw stalls regardless of drive (139 vs 241): a stall timer, matching
  Protection_Time 200 (x10 ms). Gripper protection regs (read-only, `probe-gripper.json`): Max_Torque_Limit 500,
  Torque_Limit 500, Protection_Current 250, Protective_Torque 20, Protection_Time 200, Overload_Torque 25.
- After overload every reply carries the error flag and LeRobot `bus.write()` raises. A Goal write still applied
  (jaw reopened); Torque_Enable=0 writes did NOT apply (run G: 4 tries, readback 1) until the flag cleared.
- P32 squeezes ~1.7x harder on a free close (drive 241 vs 139, current 204 vs 115).
- Incident (run G, 14:46): old cleanup raised, torque stayed on, the spoken cue still said "Torque off". Manual
  write at ~14:48 succeeded (readback 0). Fix: `torque_safety.torque_off_verified()` (Goal=Present, retry, readback),
  used by both scripts; cue now spoken only after readback 0. Checked by `check_torque_safety.py` (negative control
  reproduces run G's error).

## Torque recurrence (runs T1/T2, Wa, recurrence trials 1-5, 15:10-15:35)

| Shutdowns after overload | Torque-off method | Torque found on later |
|---|---|---|
| G2, H, T1 | single write + single readback (no input flush) | 1 of 3 (T1: verified 0 at 15:11:11, ON at 15:12:34; no other port user, no writes) |
| Wa, trials 1-5 | input flush before reads, two 0 readbacks 0.5 s apart, every attempt logged; then 90-180 s read-only watch | 0 of 6 |

- Wa: first Torque_Enable=0 write replied without error yet readback was 1; the retry took. A torque-off write can be
  silently ignored.
- Leading hypothesis (not proven; 1/3 vs 0/6 is a small sample): T1's torque never went off. Its torque-off write was
  ignored (as in Wa) and the single readback returned a false 0 (stale reply; bundled SDK has no reply LEN check,
  FINDINGS F3). Alternative: the servo re-enabled torque on its own; not seen in 6 watched shutdowns (~10 min total).
- Relevance: LeRobot `disable_torque()` writes Torque_Enable=0 once per motor with no readback, so a silently ignored
  write would leave torque on after a "clean" disconnect. Matches the historical torque-recurrence symptom; unconfirmed.
- Tissue (T1): jaw stopped at 1491, same as empty; a single tissue is below position resolution. No P32 tissue run.

### Verified torque-off, issue #2 (2026-10-08, `lerobot_disconnect_trials.py --cycles 9`)

Each cycle: gripper stalls on the tissue until a write reply carries Overload, the method under test shuts the bus
down in a child process, then a separate exclusive read-only watch reads Torque_Enable on IDs 1-6 every 0.5 s for 90 s.

| Method | Cycles with Overload | Shutdown raised | Torque found on in watch | Result |
|---|---|---|---|---|
| new: `HiwonderMotorsBus.disconnect()` (AC7) | 9 / 9 (at 3.53-3.59 s) | 0 / 9 (0.54-0.56 s each) | 0 / 9 (177-178 samples each) | PASS; 95% upper bound on failure rate 0.283 |
| old: `FeetechMotorsBus.disable_torque(num_retry=5)` (AC8) | 9 / 9 (at 3.55-3.58 s) | 9 / 9: `Failed to write 'Torque_Enable' on id_=6 with '0' after 6 tries. [ServoStatus] Overload error!` | 0 / 9 (harness safety net turned the gripper off, verified, before the watch) | negative control failed 9 / 9 |

- What AC8 shows: under Overload the inherited method raises on every shutdown, so LeRobot's disconnect stops there
  with gripper torque unverified and the port open. Whether its write applied is not observable here (the safety net
  ran before any readback). Torque turning itself back on was not seen in 18 watched shutdowns today (27 min) plus 6
  yesterday.
- Limit of this proof: the hardware watch has never been observed returning FAIL (AC8 failed by raising, and the safety
  net turned torque off before the watch). Partial evidence that it can see torque on: the same `read_raw` path read
  TE 1 on hardware in the goal probe, the watch bus read Status 32 from hardware, and the FAIL rule is covered by the
  offline checks. Not a live negative control.
  UPDATE 2026-10-09, live negative control (`lerobot_disconnect_trials.py --watch-check`, 1 run): gripper torque turned
  on deliberately (Goal_Position=Present, no motion), read TE 1; the read-only watch returned FAIL on its first sample,
  first_on id 6 TE 1 (ids 1-5 read 0); gripper then turned off and verified; final read all six TE 0. PASS. The watch
  has now been seen failing on hardware when torque is on, so its earlier PASS results are not vacuous.
- Today did not reproduce the historical torque recurrence (T1, 2026-10-07): 0 of 18 watched shutdowns.
- Goal probe (FA-05): with TE 0, writing Goal_Position = Present turned torque on (readback 1, 1, 1).
- Overload latch: a stall cut off with torque off leaves Status 0x20 set (>5 min observed) and every LeRobot read
  raises, so the first AC7 attempt stopped at cycle 2 (kept as `ac-new-attempt1`). Goal=Present cleared it within 2 s
  in all 18 resets; the harness now clears it before every cycle and after the last.
- Raw: main checkout `test-output/verified-torque-off-20261008/` (`ac-new`, `ac-old`, `goal-probe`, `ac-new-attempt1`).

## Conclusion

- Prediction confirmed: P is the per-tick slope (3.5 at P16, 7.0 at P32); the constant 16 is unchanged. Doubling P halved the
  gravity offset (elbow 55 -> 27, shoulder 45 -> 19 ticks) at the same holding drive.
- No oscillation or jitter observed at P32 in this 45 deg lift-and-hold (data and video). Not tested: fast teleop motion,
  gripper contact, long holds/heat.
- Persistence: Lock=0, so a P32 write survives power-off; `SOFollower.connect()` -> `configure()` rewrites P16 on every
  LeRobot connect regardless.
