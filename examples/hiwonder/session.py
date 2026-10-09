"""Hardware helpers shared by hiwonder_teleop.py and arm_check.py: checkout check, speech, connect, guided match."""

import logging
import subprocess
import time
from pathlib import Path

from examples.hiwonder.arm_tools import JOINTS, MATCH_TOL, MatchGuide

CHECKOUT_SRC = Path(__file__).resolve().parents[2] / "src"


def require_this_checkout():
    """Refuse to run against another checkout's LeRobot (its configure() may not write P32)."""
    import lerobot

    if not Path(lerobot.__file__).resolve().is_relative_to(CHECKOUT_SRC):
        raise RuntimeError(
            f"lerobot is imported from {Path(lerobot.__file__).parent}, not {CHECKOUT_SRC}. "
            f"Run with PYTHONPATH={CHECKOUT_SRC}."
        )


class Speaker:
    """Non-blocking macOS speech; a new cue is skipped while one is still playing."""

    def __init__(self, quiet=False):
        self.quiet, self.proc = quiet, None

    def busy(self):
        return self.proc is not None and self.proc.poll() is None

    def say(self, text, wait=False):
        print(f"[say] {text}", flush=True)
        if self.quiet:
            return
        if self.busy():
            self.proc.wait()
        self.proc = subprocess.Popen(["/usr/bin/say", text])
        if wait:
            self.proc.wait()


def make_pair(args):
    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
    from lerobot.robots.utils import make_robot_from_config
    from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderTeleopConfig
    from lerobot.teleoperators.utils import make_teleoperator_from_config

    teleop = make_teleoperator_from_config(SOLeaderTeleopConfig(port=args.leader_port, id=args.leader_id))
    robot = make_robot_from_config(
        SOFollowerRobotConfig(port=args.follower_port, id=args.follower_id, max_relative_target=None)
    )
    return teleop, robot


def require_calibrated(device, name):
    if not device.is_calibrated:
        raise RuntimeError(
            f"{name}: motor calibration differs from the saved file (wrong port, or the arm was recalibrated). "
            "Nothing was moved on purpose; check the ports and calibration."
        )


def gaps(teleop, robot):
    obs = robot.get_observation()
    act = teleop.get_action()
    return {j: act[f"{j}.pos"] - obs[f"{j}.pos"] for j in JOINTS}


def guided_match(teleop, robot, speaker, timeout_s=180.0):
    """Hold the follower still until the leader matches it on every joint; cue one joint at a time.

    Returns only when matched twice: before and right after a short "Go" cue (the pose can drift during a cue).
    """
    guide = MatchGuide(JOINTS, MATCH_TOL)
    speaker.say("Match the leader to the follower. Trust the spoken numbers over your eye.", wait=True)
    t0 = last_print = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        d = gaps(teleop, robot)
        if time.monotonic() - last_print > 1.0:
            print("gaps:", {j: round(v, 1) for j, v in d.items()}, flush=True)
            last_print = time.monotonic()
        if speaker.busy():
            time.sleep(0.1)
            continue
        done, msg = guide.update(d)
        if done:
            speaker.say("Go.", wait=True)
            done_again, _ = MatchGuide(JOINTS, MATCH_TOL).update(gaps(teleop, robot))
            if done_again:
                logging.info("Matched: %s", {j: round(v, 1) for j, v in d.items()})
                return d
            guide = MatchGuide(JOINTS, MATCH_TOL)
            continue
        if msg:
            speaker.say(msg)
        time.sleep(0.1)
    raise TimeoutError("The arms were not matched in time. Nothing was moved.")
