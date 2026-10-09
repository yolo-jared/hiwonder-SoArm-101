"""Teleoperate the Hiwonder SO-ARM101 with a guided start: nothing moves until the leader matches the follower.

Same loop and shutdown as `lerobot-teleoperate`, without `max_relative_target`, at fps 30. Before the first move it
cues one joint at a time ("Elbow: 12 degrees off", "closer", "wrong way, go back") until every joint matches, says
"Go", checks the match again, then starts. Ctrl+C, the time limit, SIGTERM and SIGHUP all turn torque off.

    PYTHONPATH=src uv run --no-sync python examples/hiwonder/hiwonder_teleop.py \
        --leader-port=/dev/cu.usbmodemLEADER --leader-id=my_leader \
        --follower-port=/dev/cu.usbmodemFOLLOWER --follower-id=my_follower
"""

import argparse
import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from examples.hiwonder.arm_tools import calibration_offsets  # noqa: E402
from examples.hiwonder.session import (  # noqa: E402
    Speaker,
    guided_match,
    make_pair,
    require_calibrated,
    require_this_checkout,
)

OFFSET_WARN_DEG = 5.0


def warn_calibration_offsets(teleop, robot):
    as_dict = lambda c: {j: {"range_min": m.range_min, "range_max": m.range_max} for j, m in c.items()}  # noqa: E731
    for joint, off in calibration_offsets(as_dict(teleop.calibration), as_dict(robot.calibration)).items():
        if off is not None and abs(off) > OFFSET_WARN_DEG:
            print(
                f"WARNING {joint}: leader and follower calibrations are {off:+.1f} deg apart; equal numbers point "
                "the arms that far apart. Recalibrate both arms to fix.",
                flush=True,
            )


def run(args):
    require_this_checkout()
    from lerobot.processor import make_default_processors
    from lerobot.scripts.lerobot_teleoperate import teleop_loop
    from lerobot.utils.robot_utils import disconnect_all, exit_on_termination_signals

    speaker = Speaker(args.quiet)
    teleop, robot = make_pair(args)
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()
    robot_connected = teleop_connected = False
    with exit_on_termination_signals() as signals:
        try:
            teleop.connect(calibrate=False)
            teleop_connected = True
            require_calibrated(teleop, "leader")
            robot.connect(calibrate=False)
            robot_connected = True
            require_calibrated(robot, "follower")
            warn_calibration_offsets(teleop, robot)
            guided_match(teleop, robot, speaker, timeout_s=args.match_timeout_s)
            teleop_loop(
                teleop=teleop,
                robot=robot,
                fps=args.fps,
                duration=args.time_s,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
            )
        except KeyboardInterrupt:
            pass
        finally:
            signals.hold()  # a SIGTERM/SIGHUP from here on takes effect once the arms are off
            disconnect_all(
                robot.disconnect if robot_connected else None,
                teleop.disconnect if teleop_connected else None,
                signals.release,
            )
            speaker.say("Teleop stopped. Torque off.", wait=True)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--leader-id", required=True)
    parser.add_argument("--follower-id", required=True)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--time-s", type=float, default=None, help="stop after this many seconds (default: Ctrl+C)"
    )
    parser.add_argument("--match-timeout-s", type=float, default=180.0)
    parser.add_argument("--quiet", action="store_true", help="no speech; cues are still printed")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(asctime)s %(message)s")
    run(args)


if __name__ == "__main__":
    main()
