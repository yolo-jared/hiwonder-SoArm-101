# SO-ARM101 follower: lessons from the elbow-undertravel investigation (2026-10-07)

Read before operating, testing or debugging the Hiwonder SO-ARM101 (HX-30HM servos) with this repo.
Evidence: `audit/software-first-20261007/FINDINGS.md` (offline audit), `audit/software-first-20261007/hardware-20261007/RESULT.md`
(powered sweeps). Fork commit `1ddcfb5`, branch `worktree-fix-macos-sw-conversion-bug-investigation-claude`. Raw data,
photos and videos are local only, in the main checkout's gitignored `test-output/` (`hiwonder-evidence-20261007/`,
`follower-move-20261007/`, `p32-20261007/`). Never commit run data: `audit/.gitignore` blocks it; it can show the room,
people or home-folder paths.

## Verdict

- No follower servo fault. All 6 joints move both ways and return within 2-19 ticks. Elbow lifted 40 deg, shoulder 41 deg
  (45 requested).
- "Elbow undertravel" = position controller at P16 leaves a fixed steady-state gravity offset of 43-56 ticks (4-5 deg) on the
  loaded joints (elbow_flex, shoulder_lift), lift direction only. Hiwonder `1006.pdf` calls P16/I0/D32 "reasonable" and the
  offset "slight".
- Teleop made it worse: README `--robot.max_relative_target=2.0` re-anchors to Present every cycle, so commanded error never
  exceeds 22 ticks and drive never reaches what holding the elbow needs.
- Ruled out offline: our driver change (identical 111-write ledgers vs official), import/venv mismatch, packet encoding
  (10 golden packets + 5 negative controls), any macOS-specific motor path.

## Operating

| Rule | Why |
|---|---|
| `SOFollower.connect()` calls `configure()`, which writes P (HX-30HM: 32 on shoulder_lift/elbow_flex, else 16), I0, D32 with Lock 0 every connect | Settings persist in servo NVS, but any normal LeRobot connect rewrites them |
| Preload Goal_Position = Present_Position before Torque_Enable=1 | Stale goal registers otherwise jump the arm on enable |
| shoulder_lift / elbow_flex "+" = downward from the resting pose | "+" moves press the gripper into the table; looks like a stall |
| Per-joint `max_relative_target` dict must contain all 6 motor keys | Elbow-only dict raises ValueError (`robots/utils.py:91-92`) |
| No P/I/D, limit, lock, calibration or firmware writes without explicit owner approval | Lock 0 makes them persistent |
| Never use Hiwonder SDK register constants for PID (`POS_P=0x16` is D gain in the workbook map) | Use `bus.write("P_Coefficient", ...)` = workbook addr 21 |
| Do not flash firmware | Servos run 3.15; kit "New" file is 3.14 (downgrade, unknown effect) |
| macOS ports: `/dev/cu.usbmodem*`; follower reads P=16 | Identify arm by reading P before moving |

## Testing tools (all in `audit/software-first-20261007/`)

| Tool | Use |
|---|---|
| `run_audit.py` + `fake_hx_bus.py` + `golden.json` | Offline: packet bytes, write ledger, clamp behavior. `python -I run_audit.py --src <src> --label X --out DIR` |
| `probe_readonly.py <port>` | Read-only identity/settings probe of IDs 1-6; write paths raise |
| `follower_sweep.py --port <port> --out DIR [--amp 15] [--joints a,b] [--direction both\|plus\|minus]` | Powered sweep; writes only addr 40/42; preflight refuses P!=16, calibration mismatch, status!=0, bad voltage; `--dry-run` uses fake bus |

## Debugging lessons

1. Size the test move against the fixed offset. A 5 deg request vs a ~4 deg gravity offset reads as "22 of 56 ticks" failure; use >=30 deg.
2. Compare the control law across joints before blaming one servo. Hold drive (Present_Load, per mille) = 16 + 3.5 x error ticks on
   pan, wrist_flex, elbow and shoulder alike. The 3.5/tick slope is the P term; the constant 16 matches the min-startup-torque
   register (24), not P.
3. Check video before calling a stall. Run2 "+" stall-guard stops were table contact.
4. `sync_read("Present_Load")` already sign-decodes (bit 10). Do not decode again.
5. Bundled `hiwonder_sdk` has no reply LEN check and `clearPort` does not clear input: a late reply was read as a wrong register
   value in emulation (FINDINGS F3).
6. Gripper `Protection_Current 250` = 250 mA in HX units, not "50%" as the code comment says (FINDINGS F4).
7. A clamp or guard must be shown failing (negative control) before its pass counts as evidence.

8. Judge torque-off ONLY by reading Torque_Enable back. After a ~2 s stall (gripper jaw on jaw/object) the servo flags
   Overload in every reply; `bus.write()` raises even when a write applied, and Torque_Enable=0 may NOT apply while
   flagged. LeRobot code: `HiwonderMotorsBus.disconnect()` / `disable_torque()` now does this (writes, flushed
   readback, confirm with a second 0 read >= 0.5 s later, Goal=Present on a non-zero read, raise within ~2 s naming
   each motor). Hardware 2026-10-08: 9/9 overload shutdowns clean, while the inherited Feetech method raised 9/9
   (`hardware-20261007/p32/RESULT.md`). Standalone scripts: `software-first-20261007/torque_safety.py`. Never speak
   "torque off" before readback 0.
9. Gripper: closing = decreasing ticks; jaws meet at ~1490 but calibration range_min is 1416, so a fully closed leader
   always stalls the follower gripper and trips overload in ~2 s.

10. macOS camera indices shuffle between ffmpeg calls (two identical "icspring camera" devices plus FaceTime/iPhone).
    Grab one frame per index and identify by content (stand camera = whole arm; wrist camera = jaw at top right)
    immediately before recording.

11. A Torque_Enable=0 write can reply OK (no error) and still not take effect (run Wa: first readback 1, second
    attempt 0). Torque was also found ON 83 s after a verified-off (run T1, no other port user). Never trust one
    write or one readback; re-check torque read-only right before hands go near the arm.

12. A Goal_Position write RE-ENABLES torque on HX-30HM (2026-10-08 goal probe: TE 0, Goal=Present, TE read 1,1,1).
    Goal writers in this repo: `SOFollower.send_action` (`robots/so_follower/so_follower.py:228`) and the verified
    torque-off itself, only after a non-zero Torque_Enable read and always followed by Torque_Enable=0
    (`motors/hiwonder/hiwonder.py:383`). A teleop/record frame sent after torque-off powers the motor again.

13. Overload (Status 0x20) LATCHES with torque off when torque is cut mid-stall: it persisted >5 min with TE 0 and
    every reply carried the flag. Both the old and the new shutdown leave it set. Code trace (not yet run end to end):
    `SOFollower.connect()` -> `bus.connect()` -> `_assert_motors_exist()` pings each ID; `ping()` returns None on an
    error byte (`hiwonder.py:238-241`), so the reconnect fails with "Missing motor IDs: 6" (misleading: the gripper is
    there); `is_calibrated` -> `read_calibration()` -> `read()` raises RuntimeError on the same byte
    (`hiwonder.py:274-275`; observed for these three registers in AC7 attempt 1). Recovery: write Goal=Present to the
    gripper (turns torque on, see 12; cleared it within 2 s in 18/18 resets), then verified torque-off; or power-cycle
    the 12 V supply. `lerobot_disconnect_trials.py` does the former before every cycle (`clear_overload_latch`).
    LeRobot fix (2026-10-08, owner chose "clear on connect"): `HiwonderMotorsBus._handshake()` runs
    `_clear_overload_latches()` before the motor check: Overload-only Status -> Goal=Present, wait <= 2.5 s, verified
    torque-off (also on error/interrupt), one WARNING per motor; another fault bit or an untrustworthy Present ->
    no write, RuntimeError saying power-cycle. Unit-tested only (`test_hiwonder_torque_off.py`, LA-01..LA-14; the
    "Missing motor IDs: 6" trace reproduced on the fake bus). Live check 2026-10-08, 1 run (`lerobot_disconnect_trials.py
    --latch-check`): stall -> old shutdown raised -> Status 0x20 latched, all TE 0 -> `bus.connect()` returned in
    0.67 s with the WARNING (Goal=1491) -> Status 0, 20 s read-only watch all TE 0. PASS, n=1. The watch itself was
    shown to catch torque on (`--watch-check`, 2026-10-09: gripper torque on deliberately -> watch FAIL on id 6 at the
    first sample -> verified off), so its all-off results above are not vacuous.

14. A frozen USB adapter can block `clearPort()`/`tcdrain` with no timeout and hang shutdown for both arms; the ~2 s
    torque-off bound holds only while the adapter accepts and drains writes (spec R6). Recovery: unplug the USB cable
    or cut the 12 V servo supply, then re-check Torque_Enable read-only before hands go near the arm.

15. Shutdown paths that turn torque off (2026-10-09; unit-tested, then live on lerobot-teleoperate as below):
    Live 2026-10-09 (leader + follower, P32 in effect, `max_relative_target=2.0`, fps 10; after each stop a 75 s
    read-only watch of all six follower motors: every sample Torque_Enable 0, Status 0): SIGTERM holding (exit 143,
    1.5 s), SIGHUP holding (129), SIGTERM while the leader moved (143), 120 s time limit (0), Ctrl+C (0, both arms
    logged disconnected). A first Ctrl+C run piped through plain `tee` lost its shutdown log and exited 120: Ctrl+C
    also kills `tee`, and Python then fails to flush (torque still read 0); use `tee -i`. An unrelated crash (a second program on the follower port) also ran the shutdown: elbow torque-off
    confirmed in the log. One program per port: a watch sharing the port breaks both.
    | How LeRobot stops | Arms off? |
    |---|---|
    | Ctrl+C (teleoperate, record) | yes, follower first |
    | `kill` / IDE stop (SIGTERM), closing the terminal (SIGHUP), in teleoperate, record, replay | yes: `exit_on_termination_signals()` raises SystemExit, exit 143 / 129 |
    | A SIGTERM/SIGHUP while the arms are turning off (e.g. Ctrl+C, then the terminal is closed) | yes: held, raised once the arms are off; repeats ignored until then |
    | Ctrl+C mid-episode in record | yes, before video encoding (previously torque stayed on for the whole encode) |
    | Follower connect fails partway (calibration, camera, configure) | yes, `SOFollower.connect()` disconnects what it opened |
    | Bus handshake fails (latched fault besides Overload) | port closed, no torque-off write (it could write Goal to the faulted motor) |
    | `lerobot-find-joint-limits` stopped by a signal; `kill -9` (SIGKILL); power loss | NO: cut 12 V or unplug USB, then check Torque_Enable read-only |
    | Frozen USB adapter during shutdown (item 14) | NO, and a second `kill` is now ignored there: use Ctrl+C, `kill -9`, or unplug USB / cut 12 V |
    | A second Ctrl+C during shutdown | ends the step in progress (`disconnect_all`, FA-13), which can be a torque-off midway |

## Open

- Teleop with the leader gripper held fully closed puts the follower gripper in overload. MEASURED 2026-10-08: the
  inherited `disable_torque()` raised under overload 9/9, which stops LeRobot's shutdown with torque unverified; the
  verified `HiwonderMotorsBus.disconnect()` completed 9/9 with torque read back off. The overload latch that then
  blocked the next connect is cleared in `connect()` (item 13); a live stall -> old shutdown -> connect run passed once (n=1).

- P32 tested 2026-10-07 on elbow/shoulder (`software-first-20261007/hardware-20261007/p32/RESULT.md`): slope doubled 3.5 -> 7.0,
  offset halved (elbow 55 -> 27, shoulder 45 -> 19 ticks), no jitter in a 45 deg lift-and-hold. All six joints tested P16 vs
  P32: every joint halves hold error and peak motion lag, none jittered. `configure()` writes P32 on every HX-30HM
  joint except the gripper (`HX30HM_P32_MOTORS`, test `tests/robots/test_so_follower_configure.py`); gripper stays P16
  pending an object squeeze test. Feetech unchanged. Teleop-speed jitter and gripper grasp at P32 untested.
  Upstream lowered P to 16 "to avoid shakiness".
- The main checkout `.venv` editable install points at the MAIN checkout `src/`, not this branch: run with
  `PYTHONPATH=<worktree>/src` (verify `lerobot.__file__`) or the P32 change is not in effect.
- `follower_sweep.py --set-p N --set-p-joints a,b` writes P (addr 21) only on named joints, torque off, and restores 16 after.
- Teleop without the 2 deg cap not yet run.
