#!/usr/bin/env python
"""Drive the Pi-attached SO-101 from the laptop over the ZMQ bridge, one command per call.

  uv run python scripts/arm_ctl.py obs  [--out DIR]            print joint state, save camera frames as PNG
  uv run python scripts/arm_ctl.py move pan lift elbow wflex wroll grip [--sec 3]   absolute joint targets (deg; grip 0-100)
  uv run python scripts/arm_ctl.py delta dpan dlift delbow dwflex dwroll dgrip [--sec 2]   relative move
Joints: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper.
"""

import argparse
import time

import cv2
import numpy as np

from lerobot.robots.so101_zmq import SO101Zmq, SO101ZmqConfig
from lerobot.robots.so101_zmq.config_so101_zmq import JOINTS

# Hard safety limits for commands (degrees, gripper 0-100).
LIMITS = {
    "shoulder_pan": (-110, 110),
    "shoulder_lift": (-100, 100),
    "elbow_flex": (-100, 100),
    "wrist_flex": (-100, 100),
    "wrist_roll": (-170, 170),
    "gripper": (0, 100),
}


def state(robot):
    obs = robot.get_observation()
    return np.array([obs[f"{j}.pos"] for j in JOINTS]), obs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["obs", "move", "delta"])
    ap.add_argument("vals", nargs="*", type=float)
    ap.add_argument("--sec", type=float, default=3.0)
    ap.add_argument("--ip", default="10.42.0.194")
    ap.add_argument("--out", default=".")
    a = ap.parse_args()

    robot = SO101Zmq(SO101ZmqConfig(remote_ip=a.ip))
    robot.connect()
    try:
        cur, obs = state(robot)
        if a.cmd in ("move", "delta"):
            assert len(a.vals) == 6, "need 6 values"
            tgt = np.array(a.vals) + (cur if a.cmd == "delta" else 0)
            tgt = np.array([np.clip(t, *LIMITS[j]) for t, j in zip(tgt, JOINTS, strict=True)])
            n = max(2, int(a.sec * 30))
            ease = (1 - np.cos(np.linspace(0, np.pi, n))) / 2  # smooth start/stop
            robot.set_mode("policy")
            robot.send_chunk(cur[None] + (tgt - cur)[None] * ease[:, None])
            time.sleep(a.sec + 0.6)
            cur, obs = state(robot)
        print("state:", {j: round(float(v), 1) for j, v in zip(JOINTS, cur, strict=True)})
        for name in robot.config.cameras:
            cv2.imwrite(f"{a.out}/{name}.png", cv2.cvtColor(obs[name], cv2.COLOR_RGB2BGR))
        print("frames ->", a.out, "mode:", robot.last_header.get("mode"))
    finally:
        robot.hold()
        robot.disconnect()


if __name__ == "__main__":
    main()
