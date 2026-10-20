# Super-LIO

**English** | [简体中文](README.zh-CN.md)

Efficient LiDAR–inertial odometry with optional loop closure and global map localization. **LiDAR and IMU input are required.**

Based on [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO), published in [IEEE RA-L 2026](https://ieeexplore.ieee.org/document/11347459). This version adds GICP + GTSAM loop closure and descriptor-based map localization.

## Build

ROS 2 Humble / Ubuntu 22.04, C++20, Eigen, PCL, glog, TBB and GTSAM ≥ 4.2. Install `livox_ros_driver2` in the workspace or source its installation first.

```bash
source /opt/ros/humble/setup.bash
sudo apt install libgoogle-glog-dev libtbb-dev ros-humble-gtsam
# From the workspace root:
colcon build --packages-up-to super_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Run

```bash
# Local odometry (default)
ros2 launch super_lio slam.launch.py

# Mapping with loop closure
ros2 launch super_lio slam.launch.py loop_closure:=true

# Automatic localization in an existing map
ros2 launch super_lio slam.launch.py localization:=true \
  map_path:=/home/USER/road_map/loop_map.pcd
```

Loop closure and localization are mutually exclusive. RViz opens with the matching configuration; add `rviz:=false` to disable it.

| Configuration | Purpose |
| --- | --- |
| [livox_360.yaml](src/super_lio/config/livox_360.yaml) | Default sensor topics, extrinsics, frontend and map saving |
| [hesai.yaml](src/super_lio/config/hesai.yaml) | Alternative sensor configuration |
| [loop_closure.yaml](src/super_lio/config/loop_closure.yaml) | Keyframes, candidate search, GICP and pose graph |
| [localization.yaml](src/super_lio/config/localization.yaml) | Map/index paths, search and matching thresholds |

Select files with `config:=/path/sensor.yaml`, `loop_config:=/path/loop.yaml` or `localization_config:=/path/localization.yaml`. Launch flags override the corresponding YAML enable switches. Restart to apply parameter changes.

The default input is `/livox/lidar` (`livox_ros_driver2/CustomMsg`) and `/imu/data`. Use calibrated extrinsics and preserve per-point timestamps. For Pacecat data, run the external converter in another terminal:

```bash
ros2 launch livox_repub_ros2 pacecat_repub.launch.py
```

For recorded data, add `use_sim_time:=true` to the SLAM command, then run:

```bash
ros2 bag play /path/to/bag --clock --delay 2
```

## Save a map

Set these in the sensor YAML **before starting mapping**:

```yaml
lio.map.save_map: true
lio.map.save_map_dir: "/road_map/"
```

After processing finishes, press **Ctrl+C and wait for a clean exit**.

| Mode | Saved files |
| --- | --- |
| Local odometry | `map.pcd` |
| Loop mapping | `map.pcd`, optimized `loop_map.pcd`, matching `loop_map.pcd.index` |
| Localization | Saving disabled; reference map unchanged |

The save directory always starts at HOME: `/road_map/`, `road_map` and `~/road_map/` all mean `~/road_map/`. This special rule applies only to `lio.map.save_map_dir`; localization input paths follow normal absolute/relative/`~/` semantics.

## Localization

Keep `loop_map.pcd` and its matching `.index` together. The index enables automatic global search without an initial pose, using an in-project polar occupancy/height descriptor followed by GICP verification. It is not the official Scan Context implementation.

After acquisition, matching continues against local portions of the PCD (default every 0.5 s). Accepted matches update **`map → odom`**; **`odom → lidar_footprint`** remains the original LIO estimate. Neither the frontend state nor the reference PCD is rewritten.

- Use the same sensor configuration and extrinsics as mapping.
- Repeated geometry can delay or prevent acquisition. Check `/lio/localization/valid`, status and odometry freshness; cached TF alone does not indicate valid localization.
- Repeated failures or stale constraints stop global odometry/TF publication and trigger global search; local LIO continues.
- Optional `use_initial_pose` and `initial_pose: [x,y,z,roll,pitch,yaw]` use meters/degrees. RViz `/initialpose` can reset localization in the configured map frame. A manual prior can work without an index; automatic recovery requires one.

For an existing PCD without an index, replay its original mapping bag once:

```bash
ros2 launch super_lio slam.launch.py loop_closure:=true rviz:=false \
  index_reference_map:=/home/USER/road_map/loop_map.pcd
# Run the converter if needed, then play the mapping bag in another terminal.
# After processing finishes, Ctrl+C and wait for index saving.
```

This writes only the index. It assumes the replay trajectory is approximately aligned with the reference map. `index_output:=/path/custom.index` changes the output; configure `lio.localization.index_path` to use a custom index. Rebuild the index if the PCD changes.

## Topics and RViz

| Output | Meaning |
| --- | --- |
| `/lio/odom`, `/lio/imu/odom` | Original local odometry |
| `/lio/path`, `/lio/cloud_world` | Local path and live point cloud |
| `/lio/loop/odom` | Globally corrected current pose |
| `/lio/loop/path`, `/lio/loop/map` | Optimized keyframe path and rebuilt global map |
| `/lio/loop/constraints` | Accepted loop edges |
| `/lio/localization/odom` | Globally corrected pose while localization is valid |
| `/lio/localization/path` | Accepted map-matching poses; history is not reoptimized |
| `/lio/localization/map` | Reference PCD |
| `/lio/localization/status`, `/lio/localization/valid` | Tracking state and validity |

RViz Fixed Frame defaults to `odom` for local LIO, `loop_map` for loop mapping and `map` for localization. Global corrections are published as `loop_map → odom` or `map → odom`; local LIO publishes `odom → lidar_footprint`. Map displays use **Reliable / Transient Local**, with decay set to zero.

## Citation

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
