# LeRobot

English | [中文](./README_cn.md)

<p align="center">
  <img src="./sources/images/lerobot.png" alt="LeRobot Logo" width="600"/>
</p>

## Product Overview

LeRobot is a state-of-the-art AI robotics library developed by Hugging Face, adapted by Hiwonder for educational and research purposes. It provides models, datasets, and tools for real-world robotics in PyTorch. The goal is to lower the barrier to entry to robotics so that everyone can contribute and benefit from sharing datasets and pretrained models.

LeRobot contains state-of-the-art approaches that have been shown to transfer to the real-world with a focus on imitation learning and reinforcement learning. It supports various robot platforms including SO-101, HopeJR, LeKiwi, and more.

## Hiwonder SO-ARM101

Using an end-to-end imitation learning pipeline, you demonstrate tasks with a *Leader Arm*. The recorded trajectories are converted into a trained model, enabling the *Follower Arm* to reproduce the task autonomously.

<p align="center">
  <img src="./sources/images/模仿学习.gif" alt="Imitation Learning" width="600"/>
</p>

To support stable and repeatable experiments, Hiwonder SO-ARM101 features:

1. **Full access to the open-source hardware and software stack**: It can be trained using techniques like imitation learning and reinforcement learning to perform tasks such as object manipulation and motion replication. The entire system—from hardware to software and algorithms—is fully open-source.

<p align="center">
  <img src="./sources/images/so-arm101-opensource.png" alt="SO-ARM101 Open Source" width="600"/>
</p>

2. **30kg magnetic-encoder servos for high torque, low jitter, and precise control**: Combined with rotating base and flexible joint movements, the arm achieves smooth and fluid motion, eliminating the power limitations and vibrations found in the original design.

<p align="center">
  <img src="./sources/images/舵机.gif" alt="SO-ARM101 Servo" width="600"/>
</p>

3. **Dual-camera vision system for real-world perception and vision-based learning**: Precise close-range manipulation + global environment awareness.

<p align="center">
  <img src="./sources/images/双摄系统.gif" alt="SO-ARM101 Dual Camera" width="600"/>
</p>

SO-ARM101 provides a practical platform for studying learning-from-demonstration, manipulation, and embodied intelligence.

## Official Resources

### Hiwonder
- **Official Website**: [https://www.hiwonder.com/](https://www.hiwonder.com/)
- **Product Page**: [https://www.hiwonder.com/products/lerobot-so-101](https://www.hiwonder.com/products/lerobot-so-101)
- **Official Documentation**: [https://www.hiwonder.com.cn/store/learn/185.html](https://www.hiwonder.com.cn/store/learn/185.html)
- **Video Tutorial**: [https://www.youtube.com/watch?v=oitT8geMat0](https://www.youtube.com/watch?v=oitT8geMat0)
- **Technical Support**: support@hiwonder.com

### Original LeRobot
- **Original Repository**: [https://github.com/huggingface/lerobot](https://github.com/huggingface/lerobot)
- **Documentation**: [https://huggingface.co/docs/lerobot](https://huggingface.co/docs/lerobot)
- **Hugging Face Hub**: [https://huggingface.co/lerobot](https://huggingface.co/lerobot)

## Key Features

### Imitation Learning Policies
- **ACT** - Action Chunking with Transformers
- **Diffusion Policy** - Diffusion-based action generation
- **PI0 / PI0-Fast** - Flow matching policies
- **SmolVLA** - Small vision-language-action model

### Robot Platforms
- **SO-101** - Affordable 6-DOF robotic arm
- **HopeJR** - Humanoid robot arm and hand
- **LeKiwi** - Mobile robot platform
- **SO-ARM101** - Hiwonder high-torque robotic arm

### Motor Support
- **Hiwonder HX-30HM** - 30kg magnetic-encoder servo

### Programming Interface
- **Python SDK** - Complete Python programming interface
- **PyTorch** - Deep learning framework
- **Hugging Face Hub** - Dataset and model sharing
- **WandB** - Experiment tracking and visualization

## Project Structure

```
lerobot/
├── src/lerobot/
│   ├── cameras/          # Camera drivers
│   ├── configs/          # Configuration files
│   ├── datasets/         # Dataset handling
│   ├── envs/             # Simulation environments
│   ├── model/            # Neural network models
│   ├── motors/           # Motor control drivers
│   │   └── hiwonder/     # Hiwonder HX-30HM servos
│   ├── policies/         # Policy implementations
│   ├── robots/           # Robot configurations
│   ├── scripts/          # Utility scripts
│   └── teleoperators/    # Teleoperation systems
├── tests/                # Unit tests
└── sources/              # Media resources
```

## Installation

### Requirements
- Python 3.12+
- PyTorch 2.7+

### Install with uv (recommended)

```bash
git clone https://github.com/Hiwonder-official/hiwonder-SoArm-101.git
cd hiwonder-SoArm-101
uv sync
```

### Install with pip

```bash
git clone https://github.com/Hiwonder-official/hiwonder-SoArm-101.git
cd hiwonder-SoArm-101
pip install -e .
```

### Install with Hiwonder servo support

```bash
pip install -e ".[hiwonder]"
```

For a `uv` installation with Hiwonder servo support, use `uv sync --extra hiwonder` instead.

## SO-ARM101 setup on macOS with `uv`

The [Hiwonder manual](https://docs.hiwonder.com/projects/LeRobot/en/latest/docs/SO_ARM101_Open_Source_6_Axis_Robotic_Arm_User_Manual.html) shows Windows `COM` ports and `python -m lerobot.teleoperate`. In this checkout, use macOS `/dev/cu.usbmodem...` ports and the installed `lerobot-teleoperate` command. `uv run --no-sync` runs that command in this repository's existing `.venv` without requiring a `python` executable on your shell's `PATH`. The old `lerobot.teleoperate` module is not present here; its entry point is `lerobot.scripts.lerobot_teleoperate`.

Run `uv sync --extra hiwonder` once after cloning. For later commands, work from the repository root. Discover the two ports interactively with `uv run --no-sync lerobot-find-port`, disconnecting/reconnecting one arm at a time to identify it. Port names can change; do not assume a saved mapping is still correct. Replace the example port values below with the ports you identified. Keep each `--teleop.id` and `--robot.id` consistent between calibration and teleoperation: those IDs select the corresponding saved calibration files. If already calibrated, replace `my_leader` and `my_follower` with the IDs used for those calibrations; changing IDs will not load the old files.

Before calibration, connect both USB and the required servo power, clear obstacles, and support the arm while motor torque is disabled. Position each arm as shown in the manual, then run these commands **one at a time in an interactive Terminal**:

```bash
uv run --no-sync lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/cu.usbmodemLEADER \
  --teleop.id=my_leader

uv run --no-sync lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/cu.usbmodemFOLLOWER \
  --robot.id=my_follower
```

If an existing calibration-file prompt appears, press Enter only to reuse a calibration you trust; type `c` and Enter to recalibrate. During calibration, move the prompted joints gently through their usable ranges. Wrist roll is excluded from the range sweep by the code; it receives a homing offset and a preset full encoder range. Do not force any joint past physical or cable limits. A `Calibration saved` line confirms the file was written, not that paired-arm motion has been checked.

Before the first **camera-free** teleoperation test, secure the follower base, clear its entire motion area, verify which port belongs to each arm, and place leader and follower in similar poses (including wrist and gripper). The follower can move as soon as the command connects. Start with small leader movements, keep clear of the follower, and press Ctrl+C immediately for unexpected motion:

```bash
uv run --no-sync lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=/dev/cu.usbmodemFOLLOWER \
  --robot.id=my_follower \
  --robot.max_relative_target=2.0 \
  --teleop.type=so101_leader \
  --teleop.port=/dev/cu.usbmodemLEADER \
  --teleop.id=my_leader \
  --fps=10 \
  --teleop_time_s=10
```

The 10-second duration and per-command position-change cap make this a short first check; they do **not** guarantee a safe motion speed or replace the physical checks. If direction, starting pose, or movement size is wrong, stop and inspect calibration before trying again. Add cameras only after camera-free teleoperation behaves correctly.

### Hands-free follower elbow diagnostic (macOS)

If the arms need to begin resting on the table, the normal teleoperation command can try to align
different starting joint coordinates. This separate, small diagnostic instead preloads the follower's
own current positions before enabling torque. It does not change calibration or motor tuning.

```bash
uv run --no-sync python examples/hiwonder/elbow_lift_check.py \
  --leader-port=/dev/cu.usbmodemLEADER --leader-id=my_leader \
  --follower-port=/dev/cu.usbmodemFOLLOWER --follower-id=my_follower
```

Use your discovered ports and existing calibration IDs. Both bases must be clamped, the lift area
clear, servo power connected, and the power cutoff reachable. The arms must initially be torque-off.
Do not support the powered follower with your hand.

1. Press Enter for a **read-only rehearsal**. At the spoken cue, raise the leader's gripper a little
   by bending only its elbow; keep its shoulder and wrist steady. Hold for the 15-second measurement.
2. After the spoken instruction, put the leader down, clear your hands, and press Enter again.
3. Following a spoken countdown, the follower attempts less than five degrees of elbow movement,
   holds briefly, returns to its starting position, and disables torque with read-back verification.
   All other follower joints are commanded to hold their starting positions.

The taught encoder direction is **not proof of a collision-free upward path** on the follower.
Observe the attempt and cut power if it presses down, binds, or moves unexpectedly. On an error or
Ctrl+C, torque is released immediately rather than attempting a return; the arm can settle abruptly.
The script refuses mismatched calibration, excessive teaching motion, and motor faults, and stops
on excessive drift/lag. Logs go to `test-logs/elbow-lift-*.log`. This is a diagnostic, not a replacement
for calibration or a guarantee of safe motion.

Completion and torque-off do not prove a lift. New versioned logs record the same-run raw controller
configuration, calibration hash, adapter identity, separate read/write times, guard events, verified
cleanup and a separate operator observation. No tuning or protection limits are changed.

The encoder gate uses the median directed travel in the last 0.5 seconds of the actual endpoint hold:
at least four complete samples spanning 0.3 seconds, no gap over 0.2 seconds, and at least 80% of the
unchanged 56-tick request. Missing coverage cannot pass. An encoder pass is not physical acceptance.
After verified torque-off, the terminal asks what you observed; Enter/EOF leaves it unobserved.
Voice and terminal report incomplete movement rather than calling every finished run successful.

Inspect both arms without changing torque, even when initially enabled:

```bash
uv run --no-sync python examples/hiwonder/elbow_lift_check.py --inspect \
  --leader-port=/dev/cu.usbmodemLEADER --leader-id=my_leader \
  --follower-port=/dev/cu.usbmodemFOLLOWER --follower-id=my_follower
```

Configuration is in the printed diagnostic log path. Unsupported registers and failed reads are
explicitly different from zero. Model/firmware applicability and raw telemetry units remain unverified;
this output does not authorize gain changes or prove the power supply is adequate.

Analyze a saved log offline (no serial or plotting dependency is imported):

```bash
uv run --no-sync python examples/hiwonder/elbow_evidence.py test-logs/YOUR_RUN.log \
  --output test-output/elbow-summary.json
```

Create the output directory first. Legacy logs retain their peak under-travel finding, but cannot prove
sustained travel or verified shutdown. Offline analysis therefore exits nonzero for those logs.

| Exit | Meaning |
|---|---|
| 0 | Encoder gate and cleanup passed; **not** physical acceptance. Inspect mode: capture finished without reporting errors. |
| 1 | Rejected/aborted run, invalid evidence, or reporting failure. |
| 2 | Completed but insufficient travel/coverage, or negative physical observation. |
| 3 | Shutdown unverified or recovery required; takes precedence. Follow the power-cutoff instruction. |

The diagnostic uses cooperative adapter locks across worktrees and Unix main-thread deadlines around
SDK operations (including blocking flush/write), installed before handshake. Close unrelated serial tools;
these locks cannot prevent them accessing the arms. Feedback plus logging older than 0.5 seconds aborts
before another trajectory write. Cleanup attempts torque-off/readback at most three times per motor;
its recorded budget is `3 × motor_count × 2 × 0.25 + 0.5` seconds (9.5 seconds for six motors).
These are software deadlines, not hard-real-time or manufacturer-certified safety guarantees. A stuck
OS, power loss, or process kill can prevent cleanup: the physical cutoff remains necessary.
No test automatically retries or increases travel. Controller tuning and larger movements remain deferred
until model-specific evidence and separately supervised qualification support them.

## Version Information
- **Current Version**: v0.5.1 (based on upstream LeRobot v0.5.1)
- **Python Version**: 3.12+
- **PyTorch Version**: 2.7+

---

**Note**: This repository is adapted from the original [Hugging Face LeRobot](https://github.com/huggingface/lerobot) for use with Hiwonder hardware. For detailed tutorials and documentation, please refer to the [Official LeRobot Documentation](https://huggingface.co/docs/lerobot).

### Follower-only supply-comparison diagnostic

The isolated follower diagnostic (`examples/hiwonder/follower_lift_check.py`) does not connect
or move the leader. It uses the same fixed 56-tick, nine-second elbow trajectory and bounded
serial/cleanup helpers as the leader-rehearsal diagnostic. It changes no tuning, protection,
or calibration settings. This is an operator-supervised investigation tool, not teleoperation.

Before a trial, prepare a reviewed passive-direction JSON record with: `version: 1`, follower
adapter `serial`, `calibration_hash`, `motor_identity` (IDs, configured models, model numbers,
and firmware fields from the snapshot), integer `direction`, six-joint raw `rest_pose`,
`manual_before`, `manual_lift`, `manual_return`, `reviewed_upward: true`,
`camera_name: "icspring camera"`, and nonempty `sources` containing actual evidence paths and
SHA-256 hashes. The review must establish an upward manual elbow lift on camera. The program
checks identity, hashes and pose tolerances; it cannot establish collision clearance itself.
Do not fabricate a record from a leader movement or an old, changed setup.

A separately created session JSON must contain the current operator authorization:
`authorized: true`, `started_at` (Unix seconds), follower `serial`, `certificate_sha256`
(SHA-256 of the exact direction-record bytes), and `trials: []`. Each attempted trial is
reserved durably before motor writes. One session permits at most three distinct-purpose
trials within ten minutes. These limits are engineering bounds, not a safety certification.
No automatic retries, unrestricted amplitude settings, or controller-tuning options exist.

```bash
uv run --no-sync python examples/hiwonder/follower_lift_check.py \
  --follower-id=hiwonder_follower \
  --direction-record=/absolute/path/to/reviewed-direction.json \
  --session-file=/absolute/path/to/authorized-session.json \
  --purpose="Adapter B: initial fixed-travel measurement" \
  --camera-dir=/absolute/path/to/new/camera-output-directory
```

This macOS tool requires `/opt/homebrew/bin/ffmpeg`, `/usr/bin/say`, and the named
`icspring camera`. It records silent JPEG sequences in the supplied new directory. Review
the printed fresh frame and the physical setup before Enter; then spoken preparation and
countdown precede motion. The process checks camera-frame freshness, all-joint goal/torque
readbacks, faults, drift, lag, timing, and return. Camera liveness does not prove scene
visibility or prevent collisions. Keep the secured follower clear and the power cutoff
reachable. On any error, torque-off cleanup is attempted; an unverified shutdown requires
cutting servo power. Logs are written under `test-logs/follower-lift-*.jsonl`.

The result never assigns visual acceptance automatically. Review retained frames separately;
encoder success alone is not a physical fix. Supply comparisons require matching trajectory,
pose and settings, verified compatible adapters, and power-off handling between swaps.
