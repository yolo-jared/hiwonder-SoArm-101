"""Capture from both "icspring camera" devices in ONE ffmpeg process, with an index-consistency check.

macOS AVFoundation indices shuffle (two identically named UVC cameras, plus FaceTime/iPhone Continuity cameras
appearing and disappearing), and this ffmpeg cannot select by unique ID ("0x..." parses as index 0). So: list devices,
capture every icspring index in one process, list again, and refuse the result if the icspring indices moved.
Files are named by index; label them stand/wrist by looking at the first frame (stand = whole arm,
wrist = jaw tip at top right, close-up of the table).

    python -I cams.py snap OUTDIR
    python -I cams.py record OUTDIR SECONDS
"""

import json
import re
import subprocess
import sys
from pathlib import Path

NAME = "icspring camera"
FMT = ["-f", "avfoundation", "-pixel_format", "uyvy422", "-framerate", "30", "-video_size", "640x480"]


def video_devices():
    r = subprocess.run(["ffmpeg", "-hide_banner", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
                       capture_output=True, text=True)
    out, devs, in_video = r.stderr, {}, False
    for line in out.splitlines():
        if "video devices" in line:
            in_video = True
        elif "audio devices" in line:
            in_video = False
        elif in_video and (m := re.search(r"\] \[(\d+)\] (.+)$", line)):
            devs[int(m.group(1))] = m.group(2).strip()
    return devs


def main():
    mode, out = sys.argv[1], Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    before = video_devices()
    idx = sorted(i for i, n in before.items() if n == NAME)
    if len(idx) != 2:
        sys.exit(f"expected 2 '{NAME}' devices, found {idx} in {before}")
    cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error"]
    for i in idx:
        cmd += FMT + ["-i", str(i)]
    for k, i in enumerate(idx):
        if mode == "snap":
            cmd += ["-map", f"{k}:v", "-frames:v", "1", "-y", str(out / f"cam-idx{i}.jpg")]
        else:
            cmd += ["-map", f"{k}:v", "-t", sys.argv[3], "-y", str(out / f"cam-idx{i}.mp4")]
    subprocess.run(cmd, check=True)
    after = video_devices()
    consistent = [i for i, n in after.items() if n == NAME] == idx and before == after
    if mode == "record":
        for i in idx:
            subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-i", str(out / f"cam-idx{i}.mp4"),
                            "-frames:v", "1", "-y", str(out / f"cam-idx{i}-first.jpg")], check=True)
    info = {"icspring_indices": idx, "devices_before": before, "devices_after": after, "consistent": consistent}
    (out / "cams.json").write_text(json.dumps(info, indent=1))
    print(json.dumps(info))
    if not consistent:
        sys.exit("INCONSISTENT: device list changed during capture; re-run")


if __name__ == "__main__":
    main()
