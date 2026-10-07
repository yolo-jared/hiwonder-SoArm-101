# SO-ARM101 follower: lessons from the elbow-undertravel investigation (2026-10-07)

Read before operating, testing or debugging the Hiwonder SO-ARM101 (HX-30HM servos) with this repo.
Evidence: `audit/software-first-20261007/FINDINGS.md` (offline audit), `audit/software-first-20261007/hardware-20261007/RESULT.md`
(powered sweeps). Fork commit `1ddcfb5`, branch `worktree-fix-macos-sw-conversion-bug-investigation-claude`. Sweep videos are
local only (`test-output/follower-move-20261007/`, gitignored).

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

## Open

- P32 tested 2026-10-07 on elbow/shoulder (`software-first-20261007/hardware-20261007/p32/RESULT.md`): slope doubled 3.5 -> 7.0,
  offset halved (elbow 55 -> 27, shoulder 45 -> 19 ticks), no jitter in a 45 deg lift-and-hold. All six joints tested P16 vs
  P32: every joint halves hold error and peak motion lag, none jittered. `configure()` writes P32 on HX-30HM
  shoulder_lift/elbow_flex/wrist_flex (`HX30HM_P32_MOTORS`, test `tests/robots/test_so_follower_configure.py`); pan, roll,
  gripper stay P16 pending owner decision. Feetech unchanged. Teleop-speed jitter and gripper grasp at P32 untested.
  Upstream lowered P to 16 "to avoid shakiness".
- The main checkout `.venv` editable install points at the MAIN checkout `src/`, not this branch: run with
  `PYTHONPATH=<worktree>/src` (verify `lerobot.__file__`) or the P32 change is not in effect.
- `follower_sweep.py --set-p N --set-p-joints a,b` writes P (addr 21) only on named joints, torque off, and restores 16 after.
- Teleop without the 2 deg cap not yet run.
