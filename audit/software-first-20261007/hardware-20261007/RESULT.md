# Powered follower joint sweeps (2026-10-07, user-authorized, area cleared by user)

Port `/dev/cu.usbmodem5C821076571` (follower: P=16 on all six, calibration matched file, 12.2-12.4 V, status 0).
Script `../follower_sweep.py`: writes only Goal_Position (42) and Torque_Enable (40); goal preloaded to present before
enable (all six goal registers read 0 beforehand); returns to start; torque-off read back 0 on all six after every run.
No P, limit, lock or calibration writes. No max_relative_target clamp. Camera: icspring, videos kept locally in
`test-output/follower-move-20261007/run*.mp4` (contact sheets here).

## Results (hold error = target - median position during hold; 11.4 ticks = 1 deg)

| Run | Joint | Move | Reached | Hold error | Drive (permille) |
|---|---|---|---|---|---|
| run1 +/-15 | wrist_roll | +/-170 | yes | 0 / 7 | - |
| run1 | gripper | +/-170 | yes | 12 / 11 | - |
| run1 | shoulder_pan | +/-170 | yes | 11 / 12 | 55 / -58 |
| run1 | wrist_flex | +/-170 | yes | 37 / 20 | 146 / -86 |
| run1 | elbow_flex | + (down, into table) / - (up) | 88 / 125 | 82 / 45 | 305 / -174 |
| run1 | shoulder_lift | + (down, into table) / - (up) | 67 / 113 | 103 / 57 | 379 / -217 |
| run2 +/-30 | wrist_flex | +/-341 | yes | 56 / 22 | - |
| run2 | elbow, shoulder | + (down) | stall guard: gripper pressed into table (run2-compare.jpg) | 192 / 234 | ~690 / ~835 |
| run3 up 30 | elbow / shoulder | -341 | 290 / 292 | 51 / 49 | - |
| run3 up 45 | elbow / shoulder | -511 | 455 (40 deg) / 468 (41 deg) | 56 / 43 | - |

All returns within 2-19 ticks. No servo status faults, no comm errors.

## Conclusion

- Every follower servo moves in both directions and returns. Elbow lifts 40 deg, shoulder 41 deg: no dead or weak servo.
- Hold drive matches 16 + 3.5 x error on four different servos (pan, wrist_flex, elbow, shoulder) within ~3 units:
  the same P16 control law everywhere. The elbow behaves like the pan servo; it just carries more gravity load.
- The residual 43-56 tick (4-5 deg) shortfall in the lift direction is the P16 gravity offset (Hiwonder 1006.pdf calls it a
  "slight positional offset"). The old 22-of-56 test was dominated by this fixed offset because the move was only 5 deg.
- Earlier "stalls" in the + direction were the resting gripper pressing into the table, not weakness.
