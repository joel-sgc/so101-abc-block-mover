#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Raspberry Pi side of the SO-101 ZMQ bridge.

Run on the Pi (from the repo root):

    uv run python -m lerobot.robots.so101_zmq.so101_zmq_host

Wire protocol
-------------
Observations (host -> laptop), ZMQ PUB, multipart ``[json_header, jpeg_0, jpeg_1, ...]``. Sent every tick.
The header holds the follower state (``<joint>.pos``), the leader state (``leader.<joint>.pos``, teleop mode
only), the host mode, the id of the last chunk received, the number of actions still queued, any ``pong``
and ``_cams`` (names of the JPEG frames that follow; empty on ticks without a camera frame).

Commands (laptop -> host), ZMQ PUSH/PULL, one JSON object each:
    {"type": "chunk", "id": int, "actions": [[6 floats], ...]}   queue of goal positions, played at ``fps``
    {"type": "mode", "mode": "idle" | "teleop" | "policy"}
    {"type": "hold"}                                              drop the queue, hold the current goal
    {"type": "ping", "id": int}                                   echoed back as ``pong`` for RTT measurement

The arm is always driven by the local loop, so Wi-Fi stalls never freeze or jerk the motors mid-step: when the
queue runs dry the arm holds its last goal, and the watchdog drops back to idle if the laptop goes silent.
"""

import contextlib
import json
import logging
import signal
import time
from collections import deque

import cv2
import draccus

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
from lerobot.utils.import_utils import _zmq_available, require_package

from .config_so101_zmq import JOINTS, SO101ZmqHostConfig

if _zmq_available:
    import zmq
else:
    zmq = None

MODES = ("idle", "teleop", "policy")


class SO101ZmqHost:
    def __init__(self, cfg: SO101ZmqHostConfig):
        require_package("pyzmq", extra="pyzmq-dep", import_name="zmq")
        self.cfg = cfg
        cams = {
            "front": OpenCVCameraConfig(
                index_or_path=cfg.front_cam, fps=30, width=640, height=480, fourcc="MJPG"
            ),
            "wrist": OpenCVCameraConfig(
                index_or_path=cfg.wrist_cam, fps=30, width=640, height=480, fourcc="MJPG"
            ),
        }
        self.follower = SO101Follower(
            SO101FollowerConfig(
                port=cfg.follower_port,
                id=cfg.follower_id,
                cameras=cams,
                max_relative_target=cfg.max_relative_target,
            )
        )
        self.leader = (
            SO101Leader(SO101LeaderConfig(port=cfg.leader_port, id=cfg.leader_id)) if cfg.use_leader else None
        )

        self.ctx = zmq.Context()
        self.cmd_sock = self.ctx.socket(zmq.PULL)
        self.cmd_sock.bind(f"tcp://*:{cfg.port_cmd}")
        self.obs_sock = self.ctx.socket(zmq.PUB)
        # Shed stale observations during Wi-Fi stalls instead of queueing them.
        self.obs_sock.setsockopt(zmq.SNDHWM, 2)
        self.obs_sock.bind(f"tcp://*:{cfg.port_obs}")

        self.mode = cfg.start_mode
        self.queue: deque[dict[str, float]] = deque()
        self.goal: dict[str, float] | None = None  # last goal sent to the motors
        self.last_chunk_id = -1
        self.pong: int | None = None
        self.last_cmd_t = time.monotonic()
        self.stop = False

    def connect(self):
        self.follower.connect()
        if self.leader is not None:
            self.leader.connect()
        obs = self.follower.get_observation()
        self.goal = {f"{j}.pos": float(obs[f"{j}.pos"]) for j in JOINTS}

    def disconnect(self):
        for dev in (self.follower, self.leader):
            try:
                if dev is not None and dev.is_connected:
                    dev.disconnect()
            except Exception as e:  # keep closing the rest
                logging.error(f"disconnect error: {e}")
        self.cmd_sock.close(0)
        self.obs_sock.close(0)
        self.ctx.term()

    # -- commands ---------------------------------------------------------------------------------------
    def _handle(self, msg: dict) -> None:
        kind = msg.get("type")
        if kind == "chunk":
            self.queue = deque(
                dict(zip((f"{j}.pos" for j in JOINTS), map(float, a), strict=True)) for a in msg["actions"]
            )
            self.last_chunk_id = int(msg["id"])
        elif kind == "mode":
            if msg["mode"] in MODES:
                self.mode = msg["mode"]
                self.queue.clear()
                logging.info(f"mode -> {self.mode}")
        elif kind == "hold":
            self.queue.clear()
        elif kind == "ping":
            self.pong = int(msg["id"])

    def _poll_commands(self) -> None:
        while True:
            try:
                raw = self.cmd_sock.recv(zmq.NOBLOCK)
            except zmq.Again:
                return
            try:
                self._handle(json.loads(raw))
            except (json.JSONDecodeError, KeyError, ValueError, TypeError) as e:
                logging.warning(f"bad command ignored: {e}")
                continue
            self.last_cmd_t = time.monotonic()

    # -- control ----------------------------------------------------------------------------------------
    def _drive(self, leader_pos: dict[str, float] | None) -> None:
        if self.mode == "teleop" and leader_pos is not None:
            self.goal = {f"{j}.pos": leader_pos[f"{j}.pos"] for j in JOINTS}
        elif self.mode == "policy":
            if time.monotonic() - self.last_cmd_t > self.cfg.watchdog_ms / 1000:
                logging.warning("watchdog: no command from laptop, holding position (mode -> idle)")
                self.mode = "idle"
                self.queue.clear()
            elif self.queue:
                self.goal = self.queue.popleft()
        if self.mode != "idle" and self.goal is not None:
            self.follower.send_action(self.goal)

    def _encode(self, obs: dict) -> list[bytes]:
        frames = []
        for name in ("wrist", "front"):
            img = cv2.resize(
                obs[name], (self.cfg.stream_width, self.cfg.stream_height), interpolation=cv2.INTER_AREA
            )
            ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), self.cfg.jpeg_quality])
            if ok:
                frames.append(buf.tobytes())
        return frames

    def run(self) -> None:
        period = 1.0 / self.cfg.fps
        img_every = max(1, round(self.cfg.fps / self.cfg.stream_fps))
        tick = 0
        logging.info(f"host up: mode={self.mode} fps={self.cfg.fps} stream_fps={self.cfg.stream_fps}")
        while not self.stop:
            t0 = time.perf_counter()
            self._poll_commands()
            leader_pos = self.leader.get_action() if (self.leader and self.mode == "teleop") else None
            self._drive(leader_pos)

            obs = self.follower.get_observation()
            header = {j: float(obs[j]) for j in obs if j.endswith(".pos")}
            if leader_pos is not None:
                header.update({f"leader.{k}": float(v) for k, v in leader_pos.items()})
            header.update(t=time.time(), mode=self.mode, chunk_id=self.last_chunk_id, queued=len(self.queue))
            if self.pong is not None:
                header["pong"], self.pong = self.pong, None
            frames = self._encode(obs) if tick % img_every == 0 else []
            header["_cams"] = ["wrist", "front"] if frames else []
            with contextlib.suppress(zmq.Again):
                self.obs_sock.send_multipart([json.dumps(header).encode(), *frames], zmq.NOBLOCK)
            tick += 1
            time.sleep(max(0.0, period - (time.perf_counter() - t0)))


@draccus.wrap()
def main(cfg: SO101ZmqHostConfig) -> None:
    logging.basicConfig(level=logging.INFO)
    host = SO101ZmqHost(cfg)

    def _stop(*_):
        host.stop = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    host.connect()
    try:
        host.run()
    finally:
        host.disconnect()


if __name__ == "__main__":
    main()
