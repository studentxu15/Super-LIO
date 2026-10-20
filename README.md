<div align="center">
  <h1>⚡Super-LIO</h1>
  <h2>Super-LIO: A Robust and Efficient LiDAR-Inertial Odometry System with a Compact Mapping Strategy</h2>
  <p><strong>This work has been accepted to <i> IEEE Robotics and Automation Letters (RA-L 2026)</i>.</strong></p>
  <br>

  [![Code](https://img.shields.io/badge/GitHub-181717?style=flat-square&logo=github&logoWidth=16)](https://github.com/Liansheng-Wang/Super-LIO.git) [![arXiv](https://img.shields.io/badge/arXiv-blue?logo=arxiv&color=%23B31B1B)](https://arxiv.org/abs/2509.05723) [![IEEE](https://img.shields.io/badge/RAL2026-004088.svg)](https://ieeexplore.ieee.org/document/11347459) [![Bilibili](https://img.shields.io/badge/Bilibili-00A1D6?style=flat-square&logo=bilibili&logoColor=white&logoWidth=16)](https://www.bilibili.com/video/BV11wBeBYEp6) [![YouTube](https://img.shields.io/badge/YouTube-FF0000?style=flat-square&logo=youtube&logoColor=white&logoWidth=16)](https://youtu.be/m9-hl8s5DDw)
</div>


<div align="center">
  <p>
    <a href="https://github.com/Liansheng-Wang/Super-LIO/tree/ros1" style="text-decoration: none;">
      <img src="https://img.shields.io/badge/🔄 SWITCH - ROS1 Noetic-3b82f6?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22" alt="Switch to ROS1"
      onmouseover="this.src='https://img.shields.io/badge/🔄 SWITCH - ROS1 Noetic-1e40af?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"
      onmouseout="this.src='https://img.shields.io/badge/🔄 SWITCH - ROS1 Noetic-3b82f6?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"/>
    </a>&nbsp;&nbsp;
    <a href="https://github.com/Liansheng-Wang/Super-LIO/tree/ros2" style="text-decoration: none;">
      <img src="https://img.shields.io/badge/✅ ACTIVE - ROS2 Humble/Iron/Jazzy-22c55e?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22" alt="ROS2 Active"
      onmouseover="this.src='https://img.shields.io/badge/✅ ACTIVE - ROS2 Humble/Iron/Jazzy-166534?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"
      onmouseout="this.src='https://img.shields.io/badge/✅ ACTIVE - ROS2 Humble/Iron/Jazzy-22c55e?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"/>
    </a>&nbsp;&nbsp;
    <a href="#" style="text-decoration: none; cursor: default;">
      <img src="https://img.shields.io/badge/🖥️ PLATFORM - X86 + ARM64-8b5cf6?style=for-the-badge&logo=linux&logoColor=white&logoWidth=22" alt="X86 and ARM Support"
      onmouseover="this.src='https://img.shields.io/badge/🖥️ PLATFORM - X86_64 + ARM64-4f46e5?style=for-the-badge&logo=linux&logoColor=white&logoWidth=22'"
      onmouseout="this.src='https://img.shields.io/badge/🖥️ PLATFORM - X86_64 + ARM64-8b5cf6?style=for-the-badge&logo=linux&logoColor=white&logoWidth=22'"/>
    </a>
  </p>
</div>

## Overview

<p align="center">
  <img src="docs/system_overview.png" width="95%">
</p>

**Key Features:  Efficient · Robust · Cross-Platform Compatible · Supports Both ROS1/ROS2 Versions**

Super-LIO is a robust and efficient LiDAR–Inertial Odometry (LIO) system designed for real-time and large-scale autonomous navigation. It introduces a compact and structured mapping strategy that enables predictable correspondence search and stable state estimation. The system is validated through extensive real-world experiments and comparisons with state-of-the-art methods, which demonstrates that Super-LIO not only achieves **excellent accuracy** but also maintains **lower resource consumption** and realizes a nearly **1.2–4× higher real-time processing speed**⚡.


**Contributors**: [Liansheng Wang](https://github.com/Liansheng-Wang), [Xinke Zhang](https://github.com/PSQzzzxk), [Chenhui Li](https://github.com/kermitLHH), [Dongjiao He](https://github.com/Joanna-HE), [Yihan pan](https://github.com/pyh3552), Jianjun Yi.


## Quickly Run

**For ROS1 Users**: Please switch to the **ros1** branch and follow the instructions at [ros1 branch](https://github.com/Liansheng-Wang/Super-LIO/tree/ros1)

### Requirements

Ubuntu 24(22).04 · C++20 · ROS Jazzy(Humble) · Eigen · PCL 

### Dependencies

glog · TBB · GTSAM >= 4.2

```bash
sudo apt install libgoogle-glog-dev libtbb-dev
# ROS 2 Humble:
sudo apt install ros-humble-gtsam
```

### Build & Run
```bash
git clone https://github.com/Liansheng-Wang/Super-LIO.git
cd Super-LIO
colcon build

source install/setup.bash
ros2 launch super_lio Livox_mid360.py

```

#### 🔁 Relocalization Mode
Super-LIO supports relocalization using a pre-built map, allowing the system to resume localization from a saved map without restarting the mapping process.
This mode is useful for long-term deployment, repeated missions, or recovery after tracking loss.

Before running relocalization, please make sure that:
- A map has been previously saved to disk.

```bash
cd PATH_2_Super-LIO
source install/setup.bash
ros2 launch super_lio relocation.py
```


## Datasets
<p align="center">
  <img src="docs/datasets_compressed.png" width="95%">
</p>

Super-LIO is evaluated on multiple real-world datasets covering diverse environments,
including indoor, outdoor, and large-scale scenes.

> **TODO**: Dataset download links and detailed descriptions will be provided in the future.


---

## Publications

If your like our projects, please cite us and support us with a star 🌟.
We kindly recommend to cite [our paper](https://ieeexplore.ieee.org/document/11347459) if you find this library useful:

```latex
@article{wang2026superlio,
  title   = {Super-LIO: A Robust and Efficient LiDAR-Inertial Odometry System with a Compact Mapping Strategy},
  author  = {Wang, Liansheng and Zhang, Xinke and Li, Chenhui and He, Dongjiao and Pan, Yihan and Yi, Jianjun},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2026},
  volume  = {11},
  number  = {3},
  pages   = {2666--2673},
  doi     = {10.1109/LRA.2026.3653372}
}
```


## Update Logs

<details>
<summary>Click to expand <b>Update Logs</b> (click to collapse)</summary>

<br>

- 2026-01-04  
  - Separate ROS interface and algorithm.
  - Refactor SuperLIOReLoc to inherit from SuperLIO.
  - Code style aligned with ROS2 version.

- 2026-01-04
  - The main branch is renamed to ros1
  - add ros2 branch

- 2026-01-04 21:51
  - release ROS2 version

- 2026-06-07
 - [Important revisions]: Fixed some known errors and improved algorithm accuracy!
 
</details>
## Optional loop closure (ROS 2)

Loop closure is **disabled by default**. The mapping node supports a background
backend using PCL GICP registration and a GTSAM iSAM2 incremental SE(3) pose graph.
Local ESKF propagation, voxel mapping, and existing `/lio/odom`, `/lio/path`, and
`/lio/cloud_world` outputs retain their original coordinate frame and behavior.
The relocalization node does not instantiate this backend. IMU input is still
required by the LIO frontend.

### Dependencies and build

Requires GTSAM 4.2 or newer in addition to the existing dependencies. GICP is
provided by PCL. For ROS 2 Humble:

```bash
sudo apt install ros-humble-gtsam
colcon build --packages-up-to super_lio
source install/setup.bash
```

### Run

```bash
# Local LIO, loop closure disabled:
ros2 launch super_lio slam.launch.py loop_closure:=false

# Loop closure enabled with the optimized RViz view:
ros2 launch super_lio slam.launch.py loop_closure:=true

# Select another sensor configuration:
ros2 launch super_lio slam.launch.py loop_closure:=true config:=/absolute/path/to/hesai.yaml

# Recorded data, in separate terminals:
ros2 launch super_lio slam.launch.py loop_closure:=true use_sim_time:=true
ros2 bag play /path/to/rosbag --clock --delay 2
```

`config/loop_closure.yaml` is loaded after the sensor configuration. Select a
custom overlay with `loop_config:=/absolute/path/to/loop_closure.yaml`.
`loop_closure` overrides `lio.loop.enabled`. These are read-only startup
parameters: restart the node to change them. Restart SLAM before replaying a bag
again; timestamp resets are not a new mapping session.

The default Livox configuration subscribes to `/livox/lidar` (`CustomMsg`) and
`/imu/data`. Pacecat point clouds can use an external `pacecat_repub` converter
before entering this interface. Preserve per-point acquisition times and set
LiDAR-to-IMU extrinsics to match the actual installation.

### Backend behavior

Distance, rotation and elapsed time select keyframes. Spatial radius and time
separation select historical candidates, which are ranked using a yaw-invariant
polar occupancy descriptor. This descriptor is a local revisit filter, not the
original Scan Context algorithm or a global place-recognition system.

GICP aligns each candidate current scan to a historical keyframe submap. The
result must pass Euclidean inlier MSE, forward/reverse overlap, correction bounds,
and point-to-plane information checks. The acceptance MSE is measured in m²; it
is not GICP's covariance-weighted internal objective.

Only accepted loops submit pending odometry factors and new keyframe poses to
iSAM2. Prior incremental structure is retained rather than rebuilding a batch
solver. The first pose has a tight prior; odometry and robust Huber loop factors
constrain relative poses. GTSAM Pose3 noise parameters use rotation-first,
translation-second ordering. A trial solver copy provides rollback if an update
fails validation. The frontend and its local map are not rewritten by the graph.

A bounded queue and one backend thread keep registration, optimization and map
reconstruction off the sensor callback. Keyframe clouds are voxel filtered.
At `max_keyframes`, the backend stops retaining additional keyframes and warns;
local LIO continues. Memory still grows with retained keyframes, and a large
loop can require updating a substantial part of the graph.

### Outputs and RViz

| Output | Meaning |
| --- | --- |
| `/lio/odom`, `/lio/imu/odom`, `/lio/robo/odom` | Original local estimates |
| `/lio/loop/odom` | Current pose with global correction applied |
| `/lio/loop/path` | Entire optimized keyframe trajectory |
| `/lio/loop/map` | Entire map rebuilt using optimized keyframe poses |
| `/lio/loop/constraints` | Accepted loop edges, displayed as red lines |
| TF `loop_map -> <lio.ros.odom_frame_id>` | Global correction of local odometry |

The enabled launch loads `rviz/loop.rviz`. Map, path and markers use reliable
transient-local QoS. Each map message replaces the complete map; keep map decay
at zero. A verified loop immediately triggers optimized path/map publication.
If `map_frame` is changed, change RViz Fixed Frame to match.

When `lio.map.save_map` is true, shutdown drains pending backend keyframes and
saves an additional downsampled `loop_map.pcd` in the configured map directory.
Existing local-map saving keeps its original filename and behavior.

`lio.map.save_map_dir` is resolved relative to the current user's `HOME`.
`/road_map/`, `road_map`, and `~/road_map/` all save under `~/road_map/`.
The default `map` saves under `~/map/`. A leading slash does not select the
filesystem root for this parameter.
The output directory is created automatically when map saving is enabled.

### Parameters

All names below have the prefix `lio.loop.`; see `config/loop_closure.yaml`.

| Parameter | Purpose |
| --- | --- |
| `enabled` | Enable the backend at startup; default false |
| `keyframe_distance`, `keyframe_angle_deg`, `keyframe_max_time` | Keyframe spacing in m, degrees, seconds |
| `min_time_separation`, `search_radius` | Historical candidate separation in seconds and search radius in m |
| `voxel_size`, `submap_frames`, `max_candidates` | Registration cloud size, historical submap span, candidate budget |
| `gicp_iterations` | Maximum GICP outer iterations |
| `gicp_neighbors` | Neighbor count for local covariance estimation; default 20 |
| `gicp_optimizer_iterations` | Maximum inner GICP optimizer iterations; default 20 |
| `max_fitness`, `min_overlap` | Euclidean inlier MSE limit in m² and minimum forward overlap |
| `max_correction`, `min_eigen_ratio` | Maximum proposed translation correction and geometric degeneracy threshold |
| `descriptor_threshold` | Candidate occupancy descriptor distance threshold |
| `map_publish_period` | Normal full-map publication interval in sensor seconds; loops bypass this interval |
| `queue_limit`, `max_keyframes` | Bounds on pending and retained keyframes |
| `min_points`, `map_frame` | Minimum cloud size and corrected map frame |

`icp_iterations` is retained as a deprecated fallback only when
`gicp_iterations` is not supplied. GICP uses the odometry pose as its initial
estimate and rejects proposed rotations above 45 degrees. It does not provide
global registration. Repetitive geometry, low overlap and weakly constrained
surfaces can still cause missed or incorrect closures; quality checks and a
robust loss reduce this risk but cannot eliminate it.
