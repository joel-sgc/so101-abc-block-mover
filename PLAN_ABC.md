# SO-101 A/B/C Block Mover — Plan (draft)

Goal: a policy that moves a small block between any two of three spots (A, B, C) — 6 ordered tasks:
A→B, A→C, B→A, B→C, C→A, C→B.

## Architecture

```
 Raspberry Pi (hardware only)              Desktop (GPU: train + infer)
 ┌──────────────────────────┐   ZMQ       ┌───────────────────────────┐
 │ so101_host.py            │ ──────────► │ SO101ZmqRobot (client)    │
 │  - follower motors       │  obs: state │  - lerobot Robot subclass │
 │  - wrist + ext cameras   │  + JPEG     │  - lerobot-record/train/  │
 │  - leader motors (teleop)│ ◄────────── │    eval work unchanged    │
 │  - watchdog / safe stop  │  action     │                           │
 └──────────────────────────┘             └───────────────────────────┘
```

Model after `robots/lekiwi/lekiwi_host.py` + `lekiwi_client.py` (PUSH/PULL for actions, PUB/SUB with `CONFLATE` for observations). Cameras are encoded to JPEG on the Pi and decoded on the desktop.

- **Host (Pi):** reads follower state + both cameras at fixed rate (30 fps), publishes observation; receives action and writes goal positions; **watchdog** holds position if no action arrives for ~500 ms; clamps actions to calibrated limits.
- **Client (desktop):** a `Robot` subclass (`so101_zmq`) exposing the same `observation_features` / `action_features` as `SOFollower` plus `wrist` and `external` cameras, so all stock lerobot scripts work.
- **Teleop for demos:** leader arm also lives on the Pi; the host also publishes leader positions so the desktop can record `(obs, leader_action)` pairs. Implement as a `Teleoperator` client (`so101_zmq_leader`).

## Phases

1. **Repo/infra** (done: fork published). Pi pulls, installs `lerobot[feetech]` + `pyzmq` + `opencv`; calibrate follower + leader (`lerobot-calibrate`).
2. **ZMQ bridge:** host + client + config; measure latency/jitter and dropped frames (target < 50 ms round trip on LAN, wired Ethernet preferred). Test with `lerobot-teleoperate` through the bridge.
3. **Physical setup:** fix spots A/B/C with tape/markers at reachable positions, fixed external-camera mount, consistent lighting. Verify block is visible from the external cam at all three spots.
4. **Manual demos (you, 2 rounds):**
   - Round 1: ~10 episodes per task × 6 tasks = 60 episodes, with small variation in block pose/orientation at the start spot.
   - Record with a task string per episode (`"move block from A to B"` etc.) in one dataset; ~15–20 s episodes.
   - Round 2: after we eval round 1 and see failure modes, record targeted extra demos on the weak tasks.
5. **Train (Framework 13, Ryzen 7 7640U, no NVIDIA GPU):** training is CPU-bound or experimental ROCm on the 780M iGPU, so budget for it: use **ACT** only (small, ~50M params), low-res images (e.g. 224x224 or smaller), small batch, and expect hours per run. SmolVLA is out of reach locally; if we need it, rent a cloud GPU for a few hours. Original notes follow. Start with **ACT** (fast, reliable for single-task pick/place) — but ACT is not language-conditioned, so choose one of:
   - (a) six separate ACT policies (simplest, most reliable), or
   - (b) one task-conditioned policy: **SmolVLA** (fine-tune, language prompt selects the route). Preferred if GPU has ≥ 16 GB; try after ACT baseline.
6. **Autonomous loop (me):** evaluate each route N=10 via the bridge with a human reset (or A↔B↔C chained runs so each success sets up the next), log success rate per route, then collect policy-rollout data / DAgger-style corrections where possible, retrain, repeat until ≥ 80% per route.
7. **Safety:** e-stop key on desktop, joint-limit clamp, torque limit on gripper, watchdog on Pi, workspace limits.

## Open questions

- Pi is on a separate Wi-Fi network the laptop joins: expect 10-50 ms jitter, so use JPEG q~70 at 640x480 or lower and CONFLATE sockets; give the Pi a static IP/hostname.
- Optional: cloud GPU for the heavier training runs.
- Can the block be reset by hand between eval trials (yes assumed), or should eval chain routes?
- Camera resolution / fps the Pi can sustain with two cameras.

## Next steps

1. You pull the repo on the Pi and confirm hardware + calibration.
2. Reuse `src/lerobot/utils/zmq_utils.py` (your helper, moved off the shadowing `utils/zmq.py` name; `np.fromstring` -> `np.frombuffer`) for pub/sub of images and state. I build `so101_zmq` host/client under `src/lerobot/robots/so101_zmq/` and a latency test.
3. Then we record the first 60 episodes.
