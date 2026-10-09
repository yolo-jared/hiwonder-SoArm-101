"""Software-only checks for examples/hiwonder/arm_tools.py: these tests never open a serial port.

Failure paths covered (Step 0):
- Leader and follower calibrated over different ranges: "matched" numbers are a physical offset (calibration_offsets).
- Operator cannot tell which way to move a joint: guidance must say closer / wrong way per joint, in plain names.
- Reading jitter (~0.1 deg) must not trigger "wrong way" cues (min step).
- A joint drifts after it was matched: guidance must return to it before declaring done.
- Gripper is in 0-100 units, not degrees.
- A joint the operator never moved must not PASS (INCOMPLETE), and a follower that did not move must FAIL
  (negative control: frozen follower).
- Self-test targets must stay inside the calibrated range, never close the gripper past its jaw-contact margin,
  and refuse a joint with too little room.
- A probe move that did not move the follower as commanded must stop the self-test of that joint.
"""

import pytest

from examples.hiwonder.arm_tools import (
    FAIL,
    GRIPPER,
    INCOMPLETE,
    PASS,
    MatchGuide,
    calibration_offsets,
    evaluate_grasp,
    evaluate_joint,
    plan_self_move,
    probe_ok,
    ramp,
    spoken_name,
)

TICKS_PER_DEG = 4095 / 360


def cal(range_min, range_max):
    return {"range_min": range_min, "range_max": range_max}


# ---------- calibration offsets ----------


def test_calibration_offset_is_the_midpoint_gap_in_degrees():
    leader = {"wrist_flex": cal(876, 3185), "elbow_flex": cal(860, 3049), GRIPPER: cal(1968, 3265)}
    follower = {"wrist_flex": cal(1000, 3230), "elbow_flex": cal(869, 3046), GRIPPER: cal(1416, 2959)}
    off = calibration_offsets(leader, follower)
    assert off["wrist_flex"] == pytest.approx((2030.5 - 2115) / TICKS_PER_DEG, abs=0.01)
    assert abs(off["elbow_flex"]) < 0.5
    assert off[GRIPPER] is None  # 0-100 units map range to range; a midpoint gap has no meaning


def test_calibration_offset_requires_the_same_joints():
    with pytest.raises(ValueError, match="wrist_roll"):
        calibration_offsets({"wrist_roll": cal(0, 4095)}, {})


# ---------- match guidance ----------

ORDER = ["shoulder_pan", "elbow_flex", GRIPPER]
TOL = {"shoulder_pan": 5.0, "elbow_flex": 5.0, GRIPPER: 10.0}


def test_spoken_names_are_plain_words():
    assert spoken_name("shoulder_pan") == "base rotation"
    assert spoken_name("wrist_flex") == "wrist bend"
    assert spoken_name(GRIPPER) == "gripper"


def test_guide_names_the_first_unmatched_joint_and_its_gap():
    g = MatchGuide(ORDER, TOL)
    done, msg = g.update({"shoulder_pan": 2.0, "elbow_flex": 12.4, GRIPPER: 0.0})
    assert not done
    assert msg == "Elbow: 12 degrees off. Move the leader elbow."


def test_guide_says_closer_then_wrong_way():
    g = MatchGuide(ORDER, TOL)
    g.update({"shoulder_pan": 20.0, "elbow_flex": 0, GRIPPER: 0})
    assert g.update({"shoulder_pan": 17.5, "elbow_flex": 0, GRIPPER: 0})[1] == "Base rotation closer, 18."
    assert (
        g.update({"shoulder_pan": -19.0, "elbow_flex": 0, GRIPPER: 0})[1]
        == "Base rotation wrong way, go back. 19."
    )


def test_guide_ignores_jitter_below_one_degree():
    g = MatchGuide(ORDER, TOL)
    g.update({"shoulder_pan": 20.0, "elbow_flex": 0, GRIPPER: 0})
    assert g.update({"shoulder_pan": 20.6, "elbow_flex": 0, GRIPPER: 0}) == (False, None)
    assert g.update({"shoulder_pan": 19.4, "elbow_flex": 0, GRIPPER: 0}) == (False, None)


def test_guide_moves_on_when_a_joint_matches_and_returns_if_it_drifts():
    g = MatchGuide(ORDER, TOL)
    g.update({"shoulder_pan": 20.0, "elbow_flex": 9.0, GRIPPER: 0})
    done, msg = g.update({"shoulder_pan": 3.0, "elbow_flex": 9.0, GRIPPER: 0})
    assert not done and msg == "Base rotation matched. Elbow: 9 degrees off. Move the leader elbow."
    done, msg = g.update({"shoulder_pan": 8.0, "elbow_flex": 9.0, GRIPPER: 0})
    assert not done and msg == "Base rotation: 8 degrees off. Move the leader base rotation."


def test_guide_is_done_only_when_every_joint_is_within_tolerance():
    g = MatchGuide(ORDER, TOL)
    assert g.update({"shoulder_pan": 4.9, "elbow_flex": -4.9, GRIPPER: 9.9}) == (True, "All joints matched.")


def test_gripper_gap_is_spoken_in_percent():
    g = MatchGuide(ORDER, TOL)
    assert g.update({"shoulder_pan": 0, "elbow_flex": 0, GRIPPER: 30.2})[1] == (
        "Gripper: 30 percent off. Move the leader gripper."
    )


def test_guide_rejects_a_missing_joint():
    with pytest.raises(KeyError):
        MatchGuide(ORDER, TOL).update({"shoulder_pan": 0})


# ---------- joint evaluation (guided teleop) ----------


def trace(leader, follower=None, dt=0.1):
    follower = leader if follower is None else follower
    return [i * dt for i in range(len(leader))], leader, follower


def sweep(amp=40.0, hold=15):
    up = [amp * i / 20 for i in range(21)]
    return [0.0] * hold + up + [amp] * hold + up[::-1] + [0.0] * hold


def test_tracking_follower_passes():
    t, lead, fol = trace(sweep())
    r = evaluate_joint("shoulder_pan", t, lead, fol, [0] * len(t))
    assert r["verdict"] == PASS, r


def test_frozen_follower_fails():  # negative control: the evaluator must notice a follower that never moved
    t, lead, _ = trace(sweep())
    r = evaluate_joint("shoulder_pan", t, lead, [0.0] * len(t), [0] * len(t))
    assert r["verdict"] == FAIL
    assert any("follower moved" in s for s in r["reasons"])


def test_unmoved_leader_is_incomplete_not_pass():
    t, lead, fol = trace([0.0] * 60)
    r = evaluate_joint("shoulder_pan", t, lead, fol, [0] * 60)
    assert r["verdict"] == INCOMPLETE


def test_large_settle_error_fails():
    lead = sweep()
    fol = [x - 6.0 if x == 40.0 else x for x in lead]
    t, _, _ = trace(lead)
    r = evaluate_joint("shoulder_pan", t, lead, fol, [0] * len(t))
    assert r["verdict"] == FAIL
    assert any("settle" in s for s in r["reasons"])


def test_shoulder_allows_gravity_droop_up_to_four_degrees():
    lead = sweep()
    fol = [x - 3.5 if x == 40.0 else x for x in lead]
    t, _, _ = trace(lead)
    assert evaluate_joint("shoulder_lift", t, lead, fol, [0] * len(t))["verdict"] == PASS
    assert evaluate_joint("shoulder_pan", t, lead, fol, [0] * len(t))["verdict"] == FAIL


def test_fault_status_fails():
    t, lead, fol = trace(sweep())
    status = [0] * len(t)
    status[30] = 0x20
    r = evaluate_joint("shoulder_pan", t, lead, fol, status)
    assert r["verdict"] == FAIL
    assert any("status" in s for s in r["reasons"])


def test_wrist_roll_needs_ninety_degrees_of_travel():
    t, lead, fol = trace(sweep(amp=60.0))
    assert evaluate_joint("wrist_roll", t, lead, fol, [0] * len(t))["verdict"] == INCOMPLETE
    t, lead, fol = trace(sweep(amp=100.0))
    assert evaluate_joint("wrist_roll", t, lead, fol, [0] * len(t))["verdict"] == PASS


def test_never_still_is_incomplete():
    lead = [40.0 * (i % 2) for i in range(60)]
    t, _, _ = trace(lead)
    assert evaluate_joint("shoulder_pan", t, lead, lead, [0] * 60)["verdict"] == INCOMPLETE


# ---------- gripper grasp ----------


def grasp_trace(reopen=True):
    # leader: open 80 -> closed 5 (object blocks follower at 30) -> open 80
    lead = [80.0] * 15 + [5.0] * 30 + [80.0] * 15
    fol = [80.0] * 15 + [30.0] * 30 + ([80.0] * 15 if reopen else [30.0] * 15)
    return [i * 0.1 for i in range(60)], lead, fol


def test_grasp_passes_when_the_follower_reopens_and_reports_the_blocked_gap():
    t, lead, fol = grasp_trace()
    status = [0] * 15 + [0x20] * 30 + [0] * 15
    r = evaluate_grasp(t, lead, fol, status)
    assert r["verdict"] == PASS
    assert r["metrics"]["blocked_gap"] == pytest.approx(25.0)
    assert r["metrics"]["status_seen"] == [0x20]


def test_grasp_fails_when_the_follower_stays_closed():
    t, lead, fol = grasp_trace(reopen=False)
    assert evaluate_grasp(t, lead, fol, [0] * 60)["verdict"] == FAIL


def test_grasp_without_a_close_is_incomplete():
    t = [i * 0.1 for i in range(60)]
    assert evaluate_grasp(t, [80.0] * 60, [80.0] * 60, [0] * 60)["verdict"] == INCOMPLETE


# ---------- self-test planner ----------


def test_targets_stay_inside_the_calibrated_range():
    plan = plan_self_move("wrist_flex", start=2100, calibration=cal(1000, 3230), amp_deg=15)
    for target in plan:
        assert 1000 < target < 3230
    assert plan == [2100 + round(15 * TICKS_PER_DEG), 2100, 2100 - round(15 * TICKS_PER_DEG), 2100]


def test_joint_at_its_range_end_moves_only_inward():
    plan = plan_self_move("elbow_flex", start=3045, calibration=cal(869, 3046), amp_deg=15)
    assert plan == [3045 - round(15 * TICKS_PER_DEG), 3045]


def test_amplitude_shrinks_to_the_room_left():
    # 10 deg of room on the minus side (less the 2 deg margin), none on the plus side
    start = 3046 - round(1 * TICKS_PER_DEG)
    plan = plan_self_move(
        "elbow_flex", start=start, calibration=cal(start - round(10 * TICKS_PER_DEG), 3046), amp_deg=15
    )
    assert len(plan) == 2
    assert start - plan[0] == pytest.approx(8 * TICKS_PER_DEG, abs=2)


def test_joint_with_no_room_is_refused():
    with pytest.raises(ValueError, match="room"):
        plan_self_move("wrist_flex", start=1010, calibration=cal(1000, 1050), amp_deg=15)


def test_gripper_never_closes_past_the_contact_margin():
    c = cal(1416, 2959)
    plan = plan_self_move(GRIPPER, start=1500, calibration=c, amp_deg=15)
    closed_limit = 1416 + 0.10 * (2959 - 1416)
    # Returning to the start pose is allowed: the follower gripper rests below the margin (1521 on this arm).
    assert all(t >= closed_limit for t in plan if t != 1500)
    assert max(plan) > 1500  # it still opens


def test_probe_ok_requires_half_the_move_in_the_commanded_direction():
    assert probe_ok(start=2000, target=2040, after=2025)
    assert not probe_ok(start=2000, target=2040, after=2010)  # stalled
    assert not probe_ok(start=2000, target=2040, after=1980)  # moved the wrong way


def test_ramp_reaches_the_target_in_bounded_steps():
    steps = ramp(2000, 2200, max_step=30)
    assert steps[-1] == 2200
    assert all(abs(b - a) <= 30 for a, b in zip([2000] + steps, steps, strict=False))
    assert ramp(2000, 2000, max_step=30) == [2000]
