"""Measure link quality to the Pi host: observation rate, ping RTT and image freshness. Moves nothing.

uv run python -m lerobot.robots.so101_zmq.latency_test --remote_ip thanos --seconds 20
"""

import argparse
import time

import numpy as np

from .config_so101_zmq import SO101ZmqConfig
from .so101_zmq_client import SO101Zmq


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--remote_ip", default="thanos")
    ap.add_argument("--seconds", type=float, default=20)
    args = ap.parse_args()

    robot = SO101Zmq(SO101ZmqConfig(remote_ip=args.remote_ip))
    robot.connect()
    robot.set_mode("idle")
    rtts, gaps, n_obs, _last_t, last_rx = [], [], 0, None, None
    end = time.monotonic() + args.seconds
    next_ping = 0.0
    while time.monotonic() < end:
        if time.monotonic() >= next_ping:
            robot.ping()
            next_ping = time.monotonic() + 0.25
        before = robot.last_rtt_s
        if robot._drain(50):
            n_obs += 1
            now = time.monotonic()
            if last_rx is not None:
                gaps.append((now - last_rx) * 1e3)
            last_rx = now
            if robot.last_rtt_s is not None and robot.last_rtt_s != before:
                rtts.append(robot.last_rtt_s * 1e3)
    robot.disconnect()

    def stats(name, v):
        if v:
            print(
                f"{name}: n={len(v)} mean={np.mean(v):.0f} p50={np.percentile(v, 50):.0f} "
                f"p95={np.percentile(v, 95):.0f} max={np.max(v):.0f} ms"
            )
        else:
            print(f"{name}: no samples")

    print(f"observation batches/s: {n_obs / args.seconds:.1f} (host publishes at its fps; fewer = dropped)")
    stats("ping RTT", rtts)
    stats("gap between received obs", gaps)


if __name__ == "__main__":
    main()
