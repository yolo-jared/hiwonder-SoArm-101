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

Before teleoperation:

1. Secure the follower base and clear its motion area.
2. Verify which port belongs to each arm.
3. Keep the leader gripper partly open. Holding it fully closed stalls the follower gripper.

#### Check the arms (new arm or after recalibration)

Run the three modes in order. Each prints PASS, FAIL or INCOMPLETE per joint and saves results to `outputs/arm_check/`.

```bash
ARGS="--leader-port=/dev/cu.usbmodemLEADER --leader-id=my_leader --follower-port=/dev/cu.usbmodemFOLLOWER --follower-id=my_follower"
PYTHONPATH=src uv run --no-sync python examples/hiwonder/arm_check.py calibration $ARGS  # no motion
PYTHONPATH=src uv run --no-sync python examples/hiwonder/arm_check.py self $ARGS         # follower moves on its own; hands clear
PYTHONPATH=src uv run --no-sync python examples/hiwonder/arm_check.py teleop $ARGS       # spoken steps; have a soft object ready
```

- `calibration`: FAIL means leader and follower calibrations differ by more than 5 degrees on a joint. Recalibrate both arms.
- `self`: each follower joint moves 15 degrees out and back; the gripper only opens.
- `teleop`: match the arms, then move each leader joint far both ways and hold still when asked. Ends with a gripper grasp on a soft object laid in the follower's open jaws.

#### Teleoperate

Guided start (recommended): nothing moves until the leader matches the follower. Spoken cues name one joint at a time ("Elbow: 12 degrees off", "closer", "wrong way, go back"). Trust the numbers over your eye.

```bash
PYTHONPATH=src uv run --no-sync python examples/hiwonder/hiwonder_teleop.py $ARGS
```

Stock command: pose the leader to match the follower joint by joint first. On connect, the follower moves at full speed to the leader's pose.

```bash
uv run --no-sync lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=/dev/cu.usbmodemFOLLOWER \
  --robot.id=my_follower \
  --teleop.type=so101_leader \
  --teleop.port=/dev/cu.usbmodemLEADER \
  --teleop.id=my_leader \
  --fps=30 \
  --teleop_time_s=10
```

- Press Ctrl+C to stop. Torque turns off on Ctrl+C or when the time limit ends.
- Remove `--teleop_time_s=10` after the first run behaves correctly.
- Do not add `--robot.max_relative_target=2.0`. It holds the follower far behind the leader on large moves.
- If direction, starting pose, or movement size is wrong, stop and check calibration.
- Add cameras only after camera-free teleoperation behaves correctly.

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

Completion and torque-off do not prove a lift. The final report compares measured encoder travel with
requested travel and labels less than 80% as `INCOMPLETE`. Even an encoder-travel pass needs visual
confirmation. Each sample also logs raw elbow load, current, voltage, temperature, torque-enable,
and the previous command's goal read-back; these are diagnostic readings, not additional force limits.
Voice and terminal use identical result text. Rehearsal rejection reports too little/too much movement,
the measured angle, and the allowed range. Incomplete follower travel reports requested versus measured
movement and exits with code 2 after verified shutdown; it is not a successful lift result.

## Version Information
- **Current Version**: v0.5.1 (based on upstream LeRobot v0.5.1)
- **Python Version**: 3.12+
- **PyTorch Version**: 2.7+

---

**Note**: This repository is adapted from the original [Hugging Face LeRobot](https://github.com/huggingface/lerobot) for use with Hiwonder hardware. For detailed tutorials and documentation, please refer to the [Official LeRobot Documentation](https://huggingface.co/docs/lerobot).
