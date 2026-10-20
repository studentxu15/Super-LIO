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

#### Relocalization mode

Use the same sensor configuration and calibrated extrinsics as during mapping:

```bash
ros2 launch super_lio slam.launch.py localization:=true map_path:=/home/USER/road_map/loop_map.pcd
```

Automatic localization uses the matching `loop_map.pcd.index` described below.
No initial position is required. The legacy `relocation_node` is a separate
initial-guess-based implementation; the new mode uses `super_lio_node`.


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

## Global map localization (ROS 2)

`localization:=true` enables global place retrieval and continuous scan-to-PCD
localization. It is mutually exclusive with `loop_closure:=true`. Disabled mode
does not load a map or index. The IMU/LiDAR frontend continues to build its local
odometry map; an independent worker uses its deskewed scans and motion prediction
to match the immutable reference PCD. Accepted matches update `map -> odom`.
This is a loosely coupled prediction/correction design, not a second EKF that
assumes the correlated odometry and scan matches are independent.

### Map and index

When loop mapping saves `loop_map.pcd`, it also saves `loop_map.pcd.index` by
default (`lio.loop.save_place_index: true`). The index stores compact polar
occupancy/height descriptors and verified keyframe positions in the final map
coordinates. It follows the polar place-retrieval approach; it is an in-project
implementation, not the published Scan Context library or a learned model.
Keep the PCD and index together. An order-independent point fingerprint rejects
an index paired with a different PCD; do not edit the PCD without rebuilding its
index.

For an existing PCD without an index, replay its mapping bag once in index-build
mode. Keyframe scans are registered to that PCD before indexing. This mode writes
only the index and leaves the reference PCD unchanged:

```bash
ros2 launch super_lio slam.launch.py loop_closure:=true rviz:=false \
  index_reference_map:=/home/USER/road_map/loop_map.pcd
# In another terminal, run the required point-cloud converter and play the mapping bag.
# After playback and processing finish, Ctrl+C the SLAM node and wait for index saving.
```

`index_output:=/absolute/path/custom.index` changes the index destination.
Index generation assumes the mapping trajectory is already in approximately the
reference map's coordinates; it is not a converter for an unrelated map/bag.

### Start and configure

```bash
# Default: automatic global search, no manual pose:
ros2 launch super_lio slam.launch.py localization:=true \
  map_path:=/home/USER/road_map/loop_map.pcd

# Original LIO with both global backends disabled:
ros2 launch super_lio slam.launch.py localization:=false loop_closure:=false
```

`localization_config:=/absolute/path/localization.yaml` selects an overlay; the
sensor configuration remains selected by `config:=...`. Relevant YAML options:

```yaml
lio.localization.enabled: false
lio.localization.map_path: "~/road_map/loop_map.pcd"
lio.localization.index_path: ""  # Empty selects map_path + ".index".
lio.localization.use_initial_pose: false
lio.localization.initial_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

Map/index input paths support `~/...`, absolute paths, and paths relative to the
process working directory. These input paths do not use the special HOME-prefix
rule of `lio.map.save_map_dir`. `initial_pose` is optional `[x,y,z,roll,pitch,yaw]`
in meters/degrees for the frontend body in the map frame. RViz `/initialpose`
messages in the configured map frame can reset localization at runtime; z is
preserved. A manual prior can also be used without an index, but automatic global
recovery then requires an index. All configuration parameters are startup-only.

### Quality and failure behavior

The worker retrieves descriptor candidates across the entire indexed trajectory,
estimates yaw, and refines candidates with GICP against a cropped map. It checks
Euclidean inlier MSE, overlap, geometric observability, gravity consistency and
correction size, rejects competing well-separated hypotheses with similar scores,
and requires consecutive consistent matches before confirming localization.
Only the latest pending scan is retained, so global search does not create an
unbounded sensor backlog. Local LIO is not reset or corrected by this worker.

After acquisition, map matching continues at `period` seconds (default 0.5).
Odometry provides the motion prediction between matches. Short failures report
`DEGRADED`; `max_failures` consecutive rejects or `max_age` without a valid map
constraint invalidate global localization. Global odometry and new TF publication
stop while invalid, local odometry continues, and global search retries. Consumers
must check `/lio/localization/valid` and odometry freshness; a cached TF is not
evidence of valid tracking.
A stationary sensor or paused bag can also trigger the wall-time watchdog.

Important options in `config/localization.yaml` include `voxel_size`,
`crop_radius`, `period`, `max_mse` (m²), `min_overlap`, `min_eigen_ratio`,
`max_correction` (m), `max_rotation_deg`, `confirmations`, `max_failures`,
`max_age`, `global_retry_period`, `global_candidates`, `ambiguity_distance`,
and `ambiguity_margin`. Startup global verification is stricter than
routine tracking. Scores and covariance values are not calibrated probabilities;
odometry covariance is a heuristic indicator and grows with constraint age; it
is not a statistically calibrated uncertainty estimate.
Repeated geometry, changed environments and insufficient overlap can prevent
acquisition; automatic localization does not guarantee a unique place everywhere.

### Outputs and RViz

| Topic / TF | Meaning |
| --- | --- |
| `/lio/localization/map` | Immutable reference PCD, PointCloud2, transient-local |
| `/lio/localization/path` | Accepted map-matching poses, Path, transient-local |
| `/lio/localization/odom` | Current globally corrected pose while valid |
| `/lio/localization/status` | SEARCHING / CONFIRMING / TRACKING / DEGRADED / LOST / ERROR and quality details |
| `/lio/localization/valid` | Machine-readable validity flag, Bool, transient-local |
| TF `map -> odom` | Last valid global correction, using configured frame names |
| `/lio/odom`, `/lio/imu/odom` | Existing local frontend outputs |

RViz is configured automatically with Fixed Frame `map`, the reference map,
localization path and live `/lio/cloud_world`. A missing map-to-odom TF before
acquisition is expected. Localization launch disables map saving to protect the
reference map; direct node startup must also set `lio.map.save_map: false`.
