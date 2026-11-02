#!/usr/bin/env bash
# Record manual demos of one route on the Pi.  Usage: scripts/record_route.sh A B [num_episodes] [episode_s] [reset_s]
# Example: scripts/record_route.sh A B 10
# Resolution: WIDTH=1280 HEIGHT=720 scripts/record_route.sh A B 10  (default 640x480; keep it constant across a dataset)
set -euo pipefail
FROM=${1:?from spot (A|B|C)}; TO=${2:?to spot (A|B|C)}
N=${3:-10}; EP=${4:-20}; RESET=${5:-10}
WIDTH=${WIDTH:-640}; HEIGHT=${HEIGHT:-480}  # e.g. WIDTH=1280 HEIGHT=720 for 720p
ROOT=${DATASET_ROOT:-$HOME/datasets/so101_abc}
RESUME=false; [ -d "$ROOT" ] && RESUME=true
FRONT=/dev/v4l/by-id/usb-046d_Brio_101_2627APAAGJX8-video-index0
WRIST=/dev/v4l/by-id/usb-Generic_USB_HD_Camera_200910120001-video-index0
CAM="{front: {type: opencv, index_or_path: $FRONT, width: $WIDTH, height: $HEIGHT, fps: 30, fourcc: MJPG}, wrist: {type: opencv, index_or_path: $WRIST, width: $WIDTH, height: $HEIGHT, fps: 30, fourcc: MJPG}}"
exec uv run lerobot-record \
  --robot.type=so101_follower --robot.id=trevor_follower \
  --robot.port=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7A001171-if00 \
  --robot.cameras="$CAM" \
  --teleop.type=so101_leader --teleop.id=trevor_leader \
  --teleop.port=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79083336-if00 \
  --dataset.repo_id=local/so101_abc --dataset.root="$ROOT" \
  --dataset.single_task="Move the block from spot $FROM to spot $TO" \
  --dataset.num_episodes="$N" --dataset.episode_time_s="$EP" --dataset.reset_time_s="$RESET" \
  --dataset.push_to_hub=false --play_sounds=false --resume=$RESUME
