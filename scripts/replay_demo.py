#!/usr/bin/env python
"""Replay a recorded episode on the Pi-attached follower over the ZMQ bridge (smooth ramp in/out, snapshots).

uv run python scripts/replay_demo.py --episode 0 [--root ~/datasets/so101_abc] [--ramp 4] [--out DIR]
"""

import argparse
import glob
import time

import cv2
import numpy as np
import pandas as pd

from lerobot.robots.so101_zmq import SO101Zmq, SO101ZmqConfig
from lerobot.robots.so101_zmq.config_so101_zmq import JOINTS


def ease(n):
    return ((1 - np.cos(np.linspace(0, np.pi, n))) / 2)[:, None]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--root", default="/home/joelsgc/datasets/so101_abc")
    ap.add_argument("--ramp", type=float, default=4.0, help="seconds to reach the first pose / go home")
    ap.add_argument("--snap_every", type=float, default=2.5)
    ap.add_argument("--out", default=".")
    ap.add_argument("--ip", default="10.42.0.194")
    a = ap.parse_args()

    df = pd.concat(pd.read_parquet(f) for f in sorted(glob.glob(f"{a.root}/data/*/*.parquet")))
    acts = np.stack(df[df.episode_index == a.episode]["action"].values).astype(float)

    robot = SO101Zmq(SO101ZmqConfig(remote_ip=a.ip))
    robot.connect()
    try:
        obs = robot.get_observation()
        cur = np.array([obs[f"{j}.pos"] for j in JOINTS])
        home = cur.copy()
        n = int(a.ramp * 30)
        robot.set_mode("policy")
        robot.send_chunk(cur + (acts[0] - cur) * ease(n))
        time.sleep(a.ramp + 0.5)
        robot.send_chunk(acts)
        t0, k = time.monotonic(), 0
        while time.monotonic() - t0 < len(acts) / 30 + 0.5:
            time.sleep(0.1)
            if time.monotonic() - t0 >= k * a.snap_every:
                o = robot.get_observation()
                for name in ("front", "wrist"):
                    cv2.imwrite(
                        f"{a.out}/ep{a.episode}_{name}_{k}.png", cv2.cvtColor(o[name], cv2.COLOR_RGB2BGR)
                    )
                k += 1
        obs = robot.get_observation()
        end = np.array([obs[f"{j}.pos"] for j in JOINTS])
        robot.send_chunk(end + (home - end) * ease(n))
        time.sleep(a.ramp + 0.5)
        print("done; snapshots:", k)
    finally:
        robot.hold()
        robot.disconnect()


if __name__ == "__main__":
    main()
