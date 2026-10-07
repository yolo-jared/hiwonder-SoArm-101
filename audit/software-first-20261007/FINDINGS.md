# SO-ARM101 software-first offline audit (2026-10-07)

Scope: offline only. No serial port opened (FakeSerial refuses non `fake://` ports; `real_ports_opened: []`),
no robot motion, no settings writes, calibration file hash unchanged (`calibration_file_unchanged: true`).
Hardware state: UNKNOWN (not checked this session).

## Source versions compared

| Version | Identity | Hiwonder driver? | Notes |
|---|---|---|---|
| Official GitHub baseline | `a24998f` (2026-07-09, Hiwonder-official, single squashed commit), LeRobot 0.5.1 | yes: `motors/hiwonder` + bundled `hiwonder_sdk` | this worktree |
| Manufacturer ZIP | `lerobot.zip` sha256 `9e881f28fce5a9f20f80c69bbffa19202fd6db695bb4ad71fb3a6fdd33c13d2b`, LeRobot 0.3.4 | **no**: zero Hiwonder/HX references; `so101_follower` uses `sts3215` + `feetech-servo-sdk` | stock upstream snapshot, not a pristine baseline for the HX-30HM driver |
| Local main | `8249858` | yes | only `src/` change since baseline: `8c55955` torque-off readback in `hiwonder.py` |
| Diagnostic branch | `93bde93` | yes | `git diff main fix/elbow-tracking-evidence -- src/` is empty; only one `src/` commit (`8c55955`) has ever existed on either branch |

## Change ledger and timeline (2026-10-05, +0900)

| Time | Change | Layer | Motor bytes affected? |
|---|---|---|---|
| 07-09 | Official configure writes P16/I0/D32, gripper 500/250/25 (inherited from upstream STS3215 code) | config | yes, every follower connect |
| 10:09 | follower calibration file written | calibration | Homing/limits |
| 10:20 | `3dc471e` README teleop command adds `--robot.max_relative_target=2.0 --fps=10` | our docs | yes, per-cycle clamp |
| 11:14 | `8c55955` torque-off readback + port close | our driver | **no writes changed** (proven below) |
| 11:45 → 16:26 | elbow/follower diagnostics: 56 ticks requested, 16 / 20 / 22 / 23 ticks reached | our diagnostics | raw `sync_write`, no clamp, no `configure()` |

## Verified results (offline harness, real driver code)

Harness: `run_audit.py` + `fake_hx_bus.py` (emulator written from the HX-30HM workbook, imports nothing from lerobot).
Raw output: `results/audit-official-a24998f.json`, `results/audit-main-8249858.json`, `results/SHA256SUMS`.

| Check | Result | Proof (in results JSON) |
|---|---|---|
| 10 hand-derived packets (write/read/ping/sync, sign bits 15 and 11, checksum edge) vs real serializer | 10/10 match, both versions | `golden_green` |
| 2 hand-derived status replies decoded by real SDK (2543; -5 via bit 15) | 2/2 | `golden_replies` |
| Negative controls: big-endian, wrong sign bit, elbow ID 3→4, goal addr 42→44, checksum+1 | all 5 go red, each only on the cases it should | `negative_controls` |
| Official vs main write ledger, connect → 5 teleop cycles → disconnect | **identical 111 writes**; main adds 6+6 reads only | `compare-official-vs-main.log` |
| Import provenance | each run imported lerobot from the requested `src` (`imports_from_requested_src: true`) | `provenance` |
| `max_relative_target=2.0`, elbow stationary, leader 10° away | every cycle commands goal = present + **22 ticks** (2.0° × 4095/360 = 22.75, truncated) | `clamp_loop_stationary_elbow` |
| Clamp negative controls (same +10° request) | None → 113, 2.0 → 22, 5.0 → 56, six-key dict (elbow 5.0, others 2.0) → 56 ticks | `clamp_variants_commanded_error_ticks` |
| Elbow-only dict | `ValueError: max_relative_target keys must match` (`src/lerobot/robots/utils.py:91-92`); a per-joint cap needs all 6 keys | `elbow_only_dict` |
| Real CLI parse (draccus, `TeleoperateConfig`) | `=2.0` → float 2.0; six-key JSON dict → dict | `cli_parse` |

## Ranked findings

### F1. Our README's `max_relative_target=2.0` caps elbow drive below what the elbow needs to hold (teleop only)

- Mechanism: `ensure_safe_goal_position` (`src/lerobot/robots/utils.py:83-110`) re-anchors to the measured present
  position every cycle, so the servo never sees more than 22 ticks of position error.
- Prior fitted logs (both adapters, P16): drive ≈ 16 + 3.5 × error. Holding at the diagnostic pose needed drive
  132-136 at 33-34 ticks error. Under the clamp the fit caps drive at 16 + 3.5 × 22 = **93**, about 70 % of that.
- Prediction: in teleop the follower elbow stalls or sags wherever gravity needs > ~93 raw drive, lagging the leader by a
  persistent ~2°+ gap instead of the ~3° P16 droop.
- Also caps slew: 2° per cycle × 10 fps = 20°/s for every joint, inside a 10 s run (`--teleop_time_s=10`).
- Supports: introduced by us (`3dc471e`). `~/.zsh_history` (order only, no timestamps): calibrate → the manual's Windows
  `python -m lerobot.teleoperate ... COM24` line → the ONLY two `lerobot-teleoperate` runs, both
  `--robot.max_relative_target=2.0 --fps=10 --teleop_time_s=10` → `elbow_lift_check.py` diagnostics. So every observed
  teleop ran under this clamp.
- Against / limits: does NOT explain the raw-tick diagnostics (they bypass the clamp), so F1 is an aggravating factor on
  top of F2, not the sole cause. The fit was measured near one pose; teleop poses differ. No teleop position log exists.
- Next discriminating check (proposed, not run): one bounded teleop comparison, same pose, P unchanged, logging leader
  target and follower present each cycle. Run A: `max_relative_target=2.0`. Run B: six-key dict, elbow 5.0, others 2.0
  (the commanded-error difference itself is guaranteed by code, so it is not the acceptance signal). Predeclared: F1
  supported if, at matched loaded elbow poses, A's follower-vs-leader gap exceeds 2° and keeps growing (sag) while B's gap
  settles near 33-34 ticks (~3°, the P16 droop). F1 rejected if A and B settle at the same gap.

### F2. Official configure halves the manufacturer P gain and persists it (teleop AND diagnostics)

- `src/lerobot/robots/so_follower/so_follower.py:154-168` writes P_Coefficient 16 (workbook default 32), inside `torque_disabled()`, which writes Lock 0
  first; per workbook row 55 NVS writes then persist across power cycles. Ledger confirms P16/I0/D32 to all 6 servos with
  lock 0 on every connect. Leader servos still read P32 (never configured).
- Consistent with the diagnostics: 56 requested − 33/34 error ≈ 22/23 reached. The drive fit came from those same logs,
  so this is consistency, not independent confirmation.
- Source: inherited from upstream LeRobot STS3215 code ("avoid shakiness"), present unchanged in the ZIP (0.3.4) and the
  official Hiwonder baseline. Not introduced by us.
- Limits: linear scaling with P is unverified; manufacturer guidance on P for firmware 3.15 still outstanding.

### F3. Bundled SDK accepts a mismatched late reply

Lives in the official baseline's bundled `hiwonder_sdk`; whether it was copied from the Feetech or Dynamixel SDK is
unverified (no other SDK was compared).


- `hiwonder_sdk/packet_handler.py:266-280` (`readData`) and `:239-245` (`txRxPacket`, matches on ID only) never check reply
  LEN vs requested length; `clearPort()`
  (`port_handler.py:41-42`) calls `ser.flush()`, which drains output and leaves stale input.
- Injection: a position reply that arrives after its 50 ms timeout is returned as the next `Torque_Enable` read
  (value 165 instead of 1). Clearing input before transmit fixes the in-gap case; a reply landing after the next transmit
  needs a length check. `stale_reply_injection`.
- Relevance: real robustness defect, but would cause sporadic wrong reads, not a steady 34-tick shortfall. Historical logs
  show no read errors. Low rank for the elbow.

### F4. Other config issues (not elbow)

- Gripper `Protection_Current=250`: HX workbook unit is 1 mA (250 mA; no-load current 150 mA); STS3215 unit differs.
  Gripper overcurrent likely far tighter than intended. Workbook row also lists max 511 vs default 6000 (internal conflict).
- Undocumented writes every connect: addr 7 (`Return_Delay_Time`, NVS, lock 0) and addr 85 (`Maximum_Acceleration`).
- `configure()` re-enables torque with no `Goal_Position` write anywhere in the connect ledger, so the servo enables
  against whatever goal register it already holds (the harness seeds a stale goal to show this). Whether HX firmware
  jumps to a stale goal on enable is unverified.
- `HiwonderMotorsBus.ping` (`hiwonder.py:192-213`) returns the model number from the config table, not the servo, so the
  model check cannot fail.

### Ruled out by this audit

- Our driver change (`8c55955`): byte-identical writes to official.
- Branch/venv import mismatch: only one driver version ever existed on either branch; harness ran each source explicitly.
- Packet encoding (ID, address, width, byte order, sign bits, checksum): matches hand-derived workbook bytes, with
  negative controls.

## Mac vs Windows vs Linux compatibility (source-level)

| Layer | macOS | Windows | Linux | Status |
|---|---|---|---|---|
| OS branches in `motors/` (hiwonder, feetech, motors_bus) | none | none | none | inspected (grep) |
| pyserial open, `timeout=0` polling | posix backend | win32 backend | posix backend | inspected; macOS run via fake only |
| `clearPort` = output flush, input kept | same | same | same | inspected + tested (fake) |
| Packet timeout 50 ms + bytes, `time.time()` wall clock | same | same | same | inspected |
| Loop pacing `precise_sleep` (`utils/robot_utils.py:34`) | sleep + spin | sleep + spin | plain sleep | inspected |
| USB-serial latency (`/dev/cu.usbmodem*` CDC) | unknown | unknown | unknown | NOT tested |
| Port naming / CLI entry points (README) | `/dev/cu.*`, `lerobot-teleoperate` | `COMx` | `/dev/ttyACM*` | docs only |
| Duplicate `libavdevice` in `cv2` and `av` wheels (objc "may cause mysterious crashes" warning on import) | seen in this venv | n/a | n/a | observed at import; camera path, not motor bytes |

No macOS-specific path changes motor bytes. A simulated branch is not a Windows or ServoStudio run.

## Proposed remedies (NOT applied)

1. README (ours, reversible): replace `--robot.max_relative_target=2.0` with a six-key per-joint JSON dict that leaves
   the elbow enough error (elbow-only dicts raise `ValueError`), or document that 2.0 limits holding torque and slew.
   Needs the user's safety call.
2. P gain: hx30hm-specific P_Coefficient 32 (workbook default). Persists to NVS; needs manufacturer confirmation and the
   bounded comparison first. Do not apply autonomously.
3. SDK: validate reply LEN in `readData`; `reset_input_buffer()` before transmit. Offline-testable with this harness.
4. Gripper current unit: hx30hm-specific Protection_Current in mA.

## Unverified / open

Physical cause not established. Power, mechanics, firmware 3.15 semantics and the historical torque-on recurrence remain
open. The voltage plan is deferred, not disproven.
