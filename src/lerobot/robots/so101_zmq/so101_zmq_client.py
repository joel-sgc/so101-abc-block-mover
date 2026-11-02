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

"""Laptop side of the SO-101 ZMQ bridge. See ``so101_zmq_host`` for the protocol."""

import json
import logging
import time
from functools import cached_property

import cv2
import numpy as np

from lerobot.lerobot_types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.import_utils import _zmq_available, require_package

if _zmq_available:
    import zmq
else:
    zmq = None

from ..robot import Robot
from .config_so101_zmq import JOINTS, SO101ZmqConfig

logger = logging.getLogger(__name__)


class SO101Zmq(Robot):
    """SO-101 follower driven through a Raspberry Pi over ZMQ.

    `send_action` queues a single goal; `send_chunk` queues many (e.g. an ACT action chunk) that the Pi plays
    locally at its own rate, which is what you want on a slow link.
    """

    config_class = SO101ZmqConfig
    name = "so101_zmq"

    def __init__(self, config: SO101ZmqConfig):
        require_package("pyzmq", extra="pyzmq-dep", import_name="zmq")
        super().__init__(config)
        self.config = config
        self.ctx = None
        self.cmd_sock = None
        self.obs_sock = None
        self._connected = False
        self._chunk_id = 0
        self._ping_id = 0
        self.last_header: dict = {}
        self.last_frames: dict[str, np.ndarray] = {}
        self.last_rtt_s: float | None = None
        self._ping_sent: dict[int, float] = {}

    # -- features ---------------------------------------------------------------------------------------
    @cached_property
    def _state_ft(self) -> dict[str, type]:
        return dict.fromkeys((f"{j}.pos" for j in JOINTS), float)

    @cached_property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {name: (cfg.height, cfg.width, 3) for name, cfg in self.config.cameras.items()}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._state_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._state_ft

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True  # calibration lives on the Pi

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    # -- connection -------------------------------------------------------------------------------------
    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.ctx = zmq.Context()
        self.cmd_sock = self.ctx.socket(zmq.PUSH)
        self.cmd_sock.setsockopt(zmq.SNDHWM, 4)
        self.cmd_sock.setsockopt(zmq.LINGER, 0)
        self.cmd_sock.connect(f"tcp://{self.config.remote_ip}:{self.config.port_cmd}")
        self.obs_sock = self.ctx.socket(zmq.SUB)
        # Large on purpose: with a tiny HWM zmq leaves a stale backlog in the TCP buffers and each read only gets
        # the oldest couple of messages. _drain() empties the queue and keeps only the newest.
        self.obs_sock.setsockopt(zmq.RCVHWM, 1000)
        self.obs_sock.setsockopt(zmq.SUBSCRIBE, b"")
        self.obs_sock.connect(f"tcp://{self.config.remote_ip}:{self.config.port_obs}")

        deadline = time.monotonic() + self.config.connect_timeout_s
        while time.monotonic() < deadline:
            self._drain(timeout_ms=200)
            if self.last_header and len(self.last_frames) == len(self.config.cameras):
                self._connected = True
                return
        self._close()
        raise DeviceNotConnectedError(
            f"No observations (state + camera frames) from the Pi at {self.config.remote_ip} within "
            f"{self.config.connect_timeout_s}s. Is so101_zmq_host running?"
        )

    def _close(self) -> None:
        for s in (self.cmd_sock, self.obs_sock):
            if s is not None:
                s.close(0)
        if self.ctx is not None:
            self.ctx.term()
        self.ctx = self.cmd_sock = self.obs_sock = None

    @check_if_not_connected
    def disconnect(self) -> None:
        try:
            self._send({"type": "mode", "mode": "idle"})
        finally:
            self._connected = False
            self._close()

    # -- io ---------------------------------------------------------------------------------------------
    def _send(self, msg: dict) -> None:
        try:
            self.cmd_sock.send(json.dumps(msg).encode(), zmq.NOBLOCK)
        except zmq.Again:
            logger.warning("command queue full (link stalled?); dropped %s", msg.get("type"))

    def _drain(self, timeout_ms: int) -> bool:
        """Read every pending observation, keep the newest header and newest frame per camera."""
        got = False
        newest_jpegs: dict[str, bytes] = {}
        if not self.obs_sock.poll(timeout_ms):
            return False
        while True:
            try:
                parts = self.obs_sock.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                break
            try:
                header = json.loads(parts[0])
                names = header.pop("_cams", [])
                newest_jpegs.update(zip(names, parts[1:], strict=True))
            except (json.JSONDecodeError, ValueError):
                continue
            if "pong" in header:
                sent = self._ping_sent.pop(header["pong"], None)
                if sent is not None:
                    self.last_rtt_s = time.monotonic() - sent
            self.last_header = header
            got = True
        # Frames are RGB on the Pi and are encoded as-is, so decode needs no colour conversion.
        for name, jpeg in newest_jpegs.items():
            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                self.last_frames[name] = img
        return got

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        self._drain(self.config.polling_timeout_ms if not self.last_header else 0)
        obs: RobotObservation = {k: self.last_header[k] for k in self._state_ft}
        for name, shape in self._cameras_ft.items():
            obs[name] = self.last_frames.get(name, np.zeros(shape, np.uint8))
        return obs

    @property
    def leader_action(self) -> RobotAction | None:
        """Leader arm positions (only streamed while the host is in teleop mode)."""
        pre = "leader."
        lead = {k[len(pre) :]: v for k, v in self.last_header.items() if k.startswith(pre)}
        return lead or None

    # -- commands ---------------------------------------------------------------------------------------
    def set_mode(self, mode: str) -> None:
        self._send({"type": "mode", "mode": mode})

    def hold(self) -> None:
        self._send({"type": "hold"})

    def ping(self) -> None:
        self._ping_id += 1
        self._ping_sent[self._ping_id] = time.monotonic()
        self._send({"type": "ping", "id": self._ping_id})

    @check_if_not_connected
    def send_chunk(self, actions: np.ndarray) -> int:
        """Queue [N, 6] joint goals (order = JOINTS). Replaces whatever the Pi had left. Returns chunk id."""
        actions = np.asarray(actions, dtype=np.float64).reshape(-1, len(JOINTS))
        self._chunk_id += 1
        self._send({"type": "chunk", "id": self._chunk_id, "actions": actions.tolist()})
        return self._chunk_id

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        vec = np.array([float(action[f"{j}.pos"]) for j in JOINTS])
        self.send_chunk(vec[None])
        return {f"{j}.pos": float(v) for j, v in zip(JOINTS, vec, strict=True)}
