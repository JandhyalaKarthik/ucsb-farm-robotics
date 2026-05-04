# ROS 2 interface contract

This document is the **single place** to agree how subsystems talk over the ROS graph. When you add a publisher or subscriber, update this file in the same PR so merges stay compatible.

**ROS distro:** Humble (per root `Dockerfile`). **Workspace root:** repository `src/` is overlaid into `/farm_robot_ws/src` in Docker and should be the same layout for native Pi builds.

---

## Repository layout (where code lives)

| Area | Suggested package path | Owner |
|------|------------------------|--------|
| Systems (state machine, watchdog) | `src/systems` | Systems |
| Computer vision / Hailo | `src/farm_cv` (or name agreed with lead) | CV |
| Navigation / Nav2 / sensors | `src/farm_navigation` (or agreed name) | Navigation |
| Pico firmware (optional in this repo) | `firmware/pico` or separate repo | Internal |

Add new ROS packages as siblings under `src/<package_name>/` with `package.xml` and `setup.py` / `CMakeLists.txt` as appropriate. Avoid dumping unrelated nodes into `systems` without discussion.

---

## Topics: who publishes what

Message types use standard ROS 2 packages (`std_msgs`, `geometry_msgs`, `sensor_msgs`, etc.) unless noted.

### Motion and safety (shared)

| Topic | Type | Publisher | Subscriber(s) | Notes |
|-------|------|-----------|-----------------|-------|
| `/cmd_vel` | `geometry_msgs/Twist` | Navigation (primary); Watchdog when latched | Systems state machine; Pico drivetrain (per internal spec) | Linear `x`, angular `z` are the usual fields. Watchdog publishes zeros while e-stop is active. |
| `/e_stop` | `std_msgs/Bool` | Watchdog on fault | Systems state machine | `data: true` means enter fault handling. |
| `/imu/data` | `sensor_msgs/Imu` | Navigation (BNO055 / fused pipeline) | Watchdog | Watchdog expects messages **at least every 200 ms**; silence triggers e-stop. |
| `/pico/heartbeat` | `std_msgs/Bool` | Pico (micro-ROS) | Watchdog | Any message resets the Pico timer; **200 ms** silence triggers e-stop. |

### Row / field logic (systems ↔ navigation)

| Topic | Type | Publisher | Subscriber | Notes |
|-------|------|-----------|------------|-------|
| `/end_of_row` | `std_msgs/Bool` | Navigation | Systems | `true` when end of row detected. |
| `/row_detected` | `std_msgs/Bool` | Navigation | Systems | Used after turn to find next row. |
| `/start_system` | `std_msgs/Bool` | Operator / launch UI | Systems | `true` to allow startup sequence. |

### Vision and spray (CV ↔ systems ↔ Pico)

| Topic | Type | Publisher | Subscriber | Notes |
|-------|------|-----------|------------|-------|
| `/pesticide_detected` | `std_msgs/Bool` | CV (or fusion node) | Systems | `true` when model / policy says treat (e.g. stop row, pesticide spray state). **Implemented today** in `systems`. |
| `/spray_control` | `std_msgs/String` | Systems | Pico (recommended) | Values in code: `start`, `stop` for **session-style** spray (row run, turns, pesticide burst). |
| `/spray_type` | `std_msgs/String` | Systems | Pico (recommended) | Values: `regular`, `pesticide` (see `SprayType` in state machine node). |
| `/spray_cmd` | `std_msgs/Bool` | CV (spatial line cross) | Pico (recommended) | `true` when bbox crosses calibration line → **short hardware pulse** (geometry / timing in frame). CV does **not** need to know SMACH state. |

#### Recommendation: keep **both** `/spray_cmd` and `/spray_control`

Use **different topics for different jobs** so a single `Bool` is not overloaded.

| Topic | Role |
|-------|------|
| **`/spray_cmd`** | **Event / pulse path (CV → hardware).** “This frame says we are at the nozzle line; fire the valve once (or for a fixed ms).” Matches Emma’s calibration story and Aylin’s brief. Systems should **not** publish here unless you later add a deliberate test pulser. |
| **`/spray_control`** + **`/spray_type`** | **Orchestration path (systems → hardware).** “Start/stop continuous spray for this row” and “which chemistry.” That is string state, not a single bool; folding it into `/spray_cmd` would be awkward (no `stop`, no `regular` vs `pesticide` without a new message type). |

**Pico (Aylin):** subscribe to **both** `/spray_cmd` and `/spray_control` (and `/spray_type`). Firmware merges them with clear priority, for example: respect `spray_control` == `stop` and e-stop immediately; while latched “on” from systems, optional CV pulses can be ignored or OR’d depending on hardware (document that rule in firmware comments).

**Why not rename everything to one name (e.g. only `/spray_cmd`)?** You would still need **two message shapes** (bool pulse vs string session) or a custom `msg`, which is more churn for every package than keeping two global topics with obvious meanings.

**Relays:** Only add a remap node if an upstream library is frozen and cannot subscribe to the chosen names; prefer remaps in **launch** files over a permanent relay executable.

### Navigation outputs (planned)

| Topic | Type | Publisher | Subscriber | Notes |
|-------|------|-----------|------------|-------|
| `/scan` | `sensor_msgs/LaserScan` | LIDAR driver / Navigation | Nav2, controllers | Standard Nav2 input. |

Encoder and custom topics from Pico (e.g. wheel ticks) should be added to this table when message types and names are fixed.

---

## Nodes in package `systems`

| Executable | Role |
|------------|------|
| `state_machine` | SMACH high-level states: idle, navigating with spray, pesticide treatment, turning, end of field, fault. |
| `watchdog` | Monitors `/imu/data` and `/pico/heartbeat`; on timeout publishes `/e_stop` and zeros `/cmd_vel`. |

---

## State machine (behavioral summary)

High-level states in code align with the lead doc as follows; names in logs may differ slightly from SMACH state IDs.

| Lead concept | Code / behavior |
|--------------|-----------------|
| IDLE | `IDLE` (timer then start farming) |
| NAVIGATING_ROW | `NAVIGATING_ROW` (includes continuous regular spray in current implementation) |
| SPRAYING | Not a separate SMACH state; spraying is combined with navigating / pesticide treatment |
| TURNING | `TURNING` |
| FAULT / E-STOP | `FAULT`; triggered by `/e_stop` |

---

## Docker and build

- Root **`Dockerfile`**: ROS Humble base, Nav2, SMACH, expects **`installers/hailort_latest_arm64.deb`** beside the Dockerfile for HailoRT on ARM64.
- Workspace is built with **`colcon build`** after `src/` is copied in.

If your package needs extra apt or pip dependencies, add them to the Dockerfile (or a documented layer) so Pi images stay reproducible.

---

## Change process

1. Propose new topic names in review **before** merging.
2. Update this file and the owning `package.xml` / `CMakeLists.txt` together.
3. Prefer **remap** in launch files for experiments; change defaults here only when the team adopts the remap.
