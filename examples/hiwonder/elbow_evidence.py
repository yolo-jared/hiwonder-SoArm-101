"""Pure offline elbow/shoulder-lift evidence evaluation. Importing this module never opens hardware."""

import argparse
import json
import math
import os
import statistics
import uuid
from pathlib import Path

ELBOW = "elbow_flex"
REQUESTED = 56
OBSERVATIONS = (
    "upward/no binding/no oscillation",
    "no lift",
    "downward/binding",
    "oscillation",
    "unobserved",
)


class Recorder:
    """Exclusive append-only JSONL record, flushed before returning from every emit."""

    def __init__(self, path):
        self.path = Path(path)
        self.run_id = uuid.uuid4().hex
        self.seq = 0
        self.stream = None

    def __enter__(self):
        self.stream = self.path.open("x")
        return self

    def emit(self, event, **fields):
        record = dict(fields, version=1, run_id=self.run_id, seq=self.seq, event=event)
        line = json.dumps(record, allow_nan=False) + "\n"
        self.stream.write(line)
        self.stream.flush()
        os.fsync(self.stream.fileno())
        self.seq += 1
        return record

    def __exit__(self, *exc):
        self.stream.close()


def initial_result():
    return {
        "execution": "interrupted/unknown",
        "encoder": "not attempted",
        "shutdown": "unverified",
        "restoration": "not applicable",
        "observation": "unobserved",
        "physical_accepted": False,
        "requested_ticks": REQUESTED,
        "peak_ticks": None,
        "hold_median_ticks": None,
        "return_error_ticks": None,
        "tracking_error_ticks": None,
        "guard_events": [],
        "errors": [],
    }


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("Nonfinite or nonnumeric measurement")
    return value


def analyze(records):
    result = initial_result()
    try:
        if any(not isinstance(e, dict) for e in records):
            raise ValueError("Record must be an object")
        if not records:
            raise ValueError("Empty log")
        legacy = "version" not in records[0]
        if legacy:
            baseline = finite(records[0]["baseline"][ELBOW])
            direction = records[0]["direction"]
            if direction not in (-1, 1):
                raise ValueError("Invalid direction")
            samples = records[1:]
            previous = -math.inf
            for event in samples:
                t = finite(event["t"])
                if t <= previous:
                    raise ValueError("Invalid legacy sample timing")
                previous = t
                for value in event["actual"].values():
                    finite(value)
            travels = [direction * (finite(e["actual"][ELBOW]) - baseline) for e in samples]
            targets = [abs(finite(e["target"][ELBOW]) - baseline) for e in samples]
            result["requested_ticks"] = max(targets, default=REQUESTED)
            result["peak_ticks"] = max(travels, default=0)
            result["encoder"] = (
                "insufficient travel" if result["peak_ticks"] < 0.8 * REQUESTED else "insufficient evidence"
            )
            result["errors"].append("Legacy log lacks complete read timing and verified finalization")
            return result
        run_id = records[0]["run_id"]
        for i, event in enumerate(records):
            if event.get("version") != 1 or event.get("run_id") != run_id or event.get("seq") != i:
                raise ValueError("Invalid version, sequence or run identity")
        result["run_id"] = run_id
        starts = [e for e in records if e["event"] == "start"]
        if len(starts) != 1:
            raise ValueError("Exactly one start record required")
        joint = starts[0].get("joint", ELBOW)
        if joint not in (ELBOW, "shoulder_lift"):
            raise ValueError("Unsupported evidence joint")
        result["joint"] = joint
        for event in records:
            if (
                event["event"] in ("baseline", "readback", "sample", "command")
                and event.get("joint", ELBOW) != joint
            ):
                raise ValueError("Mixed or missing joint identity in evidence")
        finals = [e for e in records if e["event"] == "final"]
        if len(finals) > 1:
            raise ValueError("Duplicate final record")
        if finals:
            final = finals[0]
            states = {
                "execution": ("completed", "rejected", "aborted", "interrupted/unknown"),
                "shutdown": ("verified off", "unverified", "not attempted"),
                "restoration": ("verified original", "recovery required", "not applicable"),
            }
            for field, allowed in states.items():
                if final.get(field) not in allowed:
                    raise ValueError(f"Unknown {field} state")
            result["errors"].extend(final.get("errors", []))
            for field in ("execution", "shutdown", "restoration", "return_error_ticks", "guard_events"):
                if field in final:
                    result[field] = final[field]
            for event in records[final["seq"] + 1 :]:
                if event["event"] == "observation" and result["shutdown"] == "verified off":
                    if event["value"] not in OBSERVATIONS:
                        raise ValueError("Unknown observation")
                    result["observation"] = event["value"]
                elif event["event"] == "communication_error":
                    result["errors"].append(event.get("message", "Communication failed"))
                else:
                    raise ValueError("Unexpected event after finalization")
        baselines = [e for e in records if e["event"] == "baseline"]
        if len(baselines) > 1:
            raise ValueError("Duplicate baseline")
        start = baselines[0] if baselines else starts[0]
        if "baseline" not in start:
            return result
        baseline = finite(start["baseline"][joint])
        direction = start["direction"]
        if direction not in (-1, 1):
            raise ValueError("Invalid direction")
        endpoint = baseline + direction * REQUESTED
        last_readback = -math.inf
        for event in records:
            if event["event"] == "readback":
                stamp = finite(event["t"])
                if stamp <= last_readback:
                    raise ValueError("Invalid readback time order")
                last_readback = stamp
                if finite(event["goal"]) != finite(event["expected"]):
                    raise ValueError("Goal readback conflicts with sent command")
        samples = [e for e in records if e["event"] == "sample"]
        prior_start = -math.inf
        for e in samples:
            a, b = finite(e["read_start"]), finite(e["read_end"])
            if a <= prior_start or b < a:
                raise ValueError("Invalid position-read interval order")
            prior_start = a
            for value in e["actual"].values():
                finite(value)
        travels = [direction * (e["actual"][joint] - baseline) for e in samples]
        result["peak_ticks"] = max(travels, default=0)
        result["encoder"] = "insufficient evidence"
        commands = [e for e in records if e["event"] == "command"]
        prior_time = -math.inf
        for e in commands:
            t = finite(e["t"])
            finite(e["goal"])
            if t <= prior_time:
                raise ValueError("Invalid command time order")
            prior_time = t
        endpoint_index = next((i for i, e in enumerate(commands) if e["goal"] == endpoint), None)
        if endpoint_index is not None:
            first = commands[endpoint_index]
            returned = next((e for e in commands[endpoint_index + 1 :] if e["goal"] != endpoint), None)
            if returned:
                stop = returned["t"]
                lower = stop - 0.5
                accepted = any(
                    e["event"] == "readback"
                    and finite(e["t"]) <= lower
                    and e["t"] >= first["t"]
                    and e["goal"] == e["expected"] == endpoint
                    for e in records
                )
                window = [e for e in samples if lower <= e["read_start"] and e["read_end"] <= stop]
                times = [e["read_start"] for e in window]
                coverage = (
                    len(times) >= 4
                    and times[-1] - times[0] >= 0.3 - 1e-9
                    and all(b - a <= 0.2 + 1e-9 for a, b in zip(times, times[1:], strict=False))
                )
                if first["t"] <= lower and accepted and coverage:
                    median = statistics.median(direction * (e["actual"][joint] - baseline) for e in window)
                    result["hold_median_ticks"] = median
                    result["tracking_error_ticks"] = REQUESTED - median
                    result["encoder"] = "passed" if median >= 0.8 * REQUESTED else "insufficient travel"
        # Preserve independently measurable undertravel even without adequate hold coverage.
        if (
            samples
            and result["encoder"] == "insufficient evidence"
            and result["peak_ticks"] < 0.8 * REQUESTED
        ):
            result["encoder"] = "insufficient travel"
        result["physical_accepted"] = (
            exit_code(result) == 0
            and result["observation"] == OBSERVATIONS[0]
            and result["return_error_ticks"] is not None
            and finite(result["return_error_ticks"]) <= 2 * 4095 / 360
            and not result["guard_events"]
        )
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        result["encoder"] = "invalid data"
        result["physical_accepted"] = False
        result["errors"].append(str(exc))
    return result


def analyze_file(path):
    records = []
    try:
        for line in Path(path).read_text().splitlines():
            records.append(json.loads(line))
    except (OSError, ValueError) as exc:
        result = analyze(records)
        result.update(encoder="invalid data", physical_accepted=False)
        result["errors"].append(str(exc))
        return result
    return analyze(records)


def exit_code(result):
    if result["shutdown"] == "unverified" or result["restoration"] == "recovery required":
        return 3
    if (
        result["execution"] != "completed"
        or result["encoder"] in ("invalid data", "not attempted")
        or result.get("errors")
        or result.get("guard_events")
    ):
        return 1
    if result["shutdown"] != "verified off":
        return 1
    if result["encoder"] != "passed" or result["observation"] in OBSERVATIONS[1:4]:
        return 2
    return 0


def format_result(result):
    return (
        f"Execution: {result['execution']}. Encoder check: {result['encoder']}. "
        f"Joint: {result.get('joint', ELBOW).replace('_', ' ')}. "
        f"Requested {result['requested_ticks']} ticks; peak {result['peak_ticks']}; "
        f"hold median {result['hold_median_ticks']}. Shutdown: {result['shutdown']}. "
        f"Observation: {result['observation']}. "
        + (
            "Physical acceptance recorded for this small trial."
            if result["physical_accepted"]
            else "This is not physical acceptance."
        )
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = analyze_file(args.log)
    text = json.dumps(result, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.write_text(text)
    print(text, end="")
    raise SystemExit(exit_code(result))


if __name__ == "__main__":
    main()
