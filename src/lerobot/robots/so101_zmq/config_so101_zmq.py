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

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig

from ..config import RobotConfig

# Stable device paths on the Pi (survive reboots / re-enumeration).
FOLLOWER_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7A001171-if00"  # ttyACM0
LEADER_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79083336-if00"  # ttyACM1
FRONT_CAM = "/dev/v4l/by-id/usb-046d_Brio_101_2627APAAGJX8-video-index0"
WRIST_CAM = "/dev/v4l/by-id/usb-Generic_USB_HD_Camera_200910120001-video-index0"

# Size of the frames streamed to the laptop (the link is slow, so keep them small).
STREAM_WIDTH = 320
STREAM_HEIGHT = 240

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def stream_cameras_config() -> dict[str, CameraConfig]:
    """Camera features as seen by the laptop (client side). Shapes match the streamed frames."""
    return {
        "wrist": OpenCVCameraConfig(index_or_path=0, fps=30, width=STREAM_WIDTH, height=STREAM_HEIGHT),
        "front": OpenCVCameraConfig(index_or_path=0, fps=30, width=STREAM_WIDTH, height=STREAM_HEIGHT),
    }


@dataclass
class SO101ZmqHostConfig:
    """Runs on the Raspberry Pi."""

    follower_port: str = FOLLOWER_PORT
    leader_port: str = LEADER_PORT
    follower_id: str = "trevor_follower"
    leader_id: str = "trevor_leader"
    front_cam: str = FRONT_CAM
    wrist_cam: str = WRIST_CAM

    port_cmd: int = 5555
    port_obs: int = 5556

    fps: int = 30  # local control + state publish rate
    stream_fps: int = 10  # camera frames sent to the laptop (state goes out every tick)
    jpeg_quality: int = 65
    stream_width: int = STREAM_WIDTH
    stream_height: int = STREAM_HEIGHT

    # Per-tick clamp on how far a goal can be from the current position (degrees).
    max_relative_target: float = 15.0
    # If no command arrives for this long in policy mode, the host holds position.
    watchdog_ms: int = 3000
    # Start with the leader mirrored to the follower (manual teleop) instead of idle.
    start_mode: str = "idle"  # idle | teleop | policy
    use_leader: bool = True


@RobotConfig.register_subclass("so101_zmq")
@dataclass
class SO101ZmqConfig(RobotConfig):
    """Runs on the laptop; talks to `so101_zmq_host` on the Pi."""

    remote_ip: str = "10.42.0.194"  # Pi over the direct ethernet cable
    port_cmd: int = 5555
    port_obs: int = 5556
    cameras: dict[str, CameraConfig] = field(default_factory=stream_cameras_config)
    connect_timeout_s: float = 15.0
    polling_timeout_ms: int = 500
