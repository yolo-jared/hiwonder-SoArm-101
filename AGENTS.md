Push only to Jared's fork, never Hiwonder-official:
`git -C /Users/jaredsisk/project-code/hiwonder-SoArm-101 push https://github.com/yolo-jared/hiwonder-SoArm-101.git main:main`

# Repository agent guidance

This is Hiwonder's LeRobot fork for SO-ARM101 hardware. Read the macOS `uv` setup and safety procedure in `README.md` before giving calibration or teleoperation commands. Keep this environment separate from other robotics repositories.

## Commands and hardware boundaries

- Use the installed entry points via `uv run --no-sync lerobot-calibrate` and `uv run --no-sync lerobot-teleoperate` after `uv sync --extra hiwonder`. Do not copy the manual's Windows `COM` ports or `python -m lerobot.teleoperate` command into macOS instructions; that module is absent from this checkout.
- Discover leader and follower serial ports from the currently connected hardware. Never infer their identity from an old device name alone. Keep each arm's ID unchanged between calibration and teleoperation because it selects the saved calibration file.
- Before any operation that can move the follower, establish the physical gates with the operator: follower base secured, motion area clear, ports mapped to the right arms, servo power present, both arms in comparable poses, and an accessible Ctrl+C/power cutoff. Do not run motion as a software-only verification step.
- Treat `Calibration saved` as a file-write result, not proof of correct physical correspondence. If calibration fails after resetting or writing homing offsets, do not blindly reuse an older calibration file; diagnose the motor state first.
- During range calibration, do not force joints or cables. Wrist roll is intentionally omitted from the measured range sweep and receives a preset encoder range; the operator must still check physical wrist alignment before teleoperation.

For exact command templates and the manufacturer's manual link, use `README.md`. Confirm current CLI flags from this checkout before changing examples.
