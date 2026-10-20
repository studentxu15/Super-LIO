# Super-LIO

[English](README.md) | **简体中文**

高效激光惯性里程计，支持可选回环建图和全局地图重定位。**需要同时输入激光雷达和 IMU 数据。**

基于发表于 [IEEE RA-L 2026](https://ieeexplore.ieee.org/document/11347459) 的 [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO)，本版本增加了 GICP + GTSAM 回环，以及基于描述子的地图重定位。

## 编译

环境：ROS 2 Humble / Ubuntu 22.04、C++20、Eigen、PCL、glog、TBB、GTSAM ≥ 4.2。请先将 `livox_ros_driver2` 放入工作空间，或加载其安装环境。

```bash
source /opt/ros/humble/setup.bash
sudo apt install libgoogle-glog-dev libtbb-dev ros-humble-gtsam
# 在工作空间根目录执行：
colcon build --packages-up-to super_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 运行

```bash
# 原始里程计（默认模式）
ros2 launch super_lio slam.launch.py

# 开启回环建图
ros2 launch super_lio slam.launch.py loop_closure:=true

# 使用已有地图自动重定位
ros2 launch super_lio slam.launch.py localization:=true \
  map_path:=/home/USER/road_map/loop_map.pcd
```

回环与重定位不能同时开启。启动时自动加载对应的 RViz 配置，添加 `rviz:=false` 可关闭 RViz。

| 配置文件 | 用途 |
| --- | --- |
| [livox_360.yaml](src/super_lio/config/livox_360.yaml) | 默认传感器话题、外参、前端及地图保存 |
| [hesai.yaml](src/super_lio/config/hesai.yaml) | 其他雷达配置 |
| [loop_closure.yaml](src/super_lio/config/loop_closure.yaml) | 关键帧、候选搜索、GICP 和位姿图 |
| [localization.yaml](src/super_lio/config/localization.yaml) | 地图与索引路径、搜索及匹配阈值 |

通过 `config:=/path/sensor.yaml`、`loop_config:=/path/loop.yaml`、`localization_config:=/path/localization.yaml` 指定配置文件。启动参数优先于 YAML 中对应的模式开关，修改参数后需重启。

默认输入为 `/livox/lidar`（`livox_ros_driver2/CustomMsg`）和 `/imu/data`。请使用实际标定外参，并保留每个点的采集时间。Pacecat 数据可在另一终端启动外部转换节点：

```bash
ros2 launch livox_repub_ros2 pacecat_repub.launch.py
```

播放录制数据时，在 SLAM 启动命令中添加 `use_sim_time:=true`，再执行：

```bash
ros2 bag play /path/to/bag --clock --delay 2
```

## 保存地图

**启动建图前**，在传感器 YAML 中设置：

```yaml
lio.map.save_map: true
lio.map.save_map_dir: "/road_map/"
```

数据处理完成后，按 **Ctrl+C，并等待程序正常退出**。

| 模式 | 保存文件 |
| --- | --- |
| 原始里程计 | `map.pcd` |
| 回环建图 | `map.pcd`、优化后的 `loop_map.pcd` 及配套 `loop_map.pcd.index` |
| 重定位 | 自动关闭保存，不修改参考地图 |

保存目录统一从 HOME 开始：`/road_map/`、`road_map` 和 `~/road_map/` 都表示 `~/road_map/`。此规则仅适用于 `lio.map.save_map_dir`；重定位地图输入路径遵循正常的绝对路径、相对路径及 `~/` 规则。

## 重定位

将 `loop_map.pcd` 与对应的 `.index` 放在一起。系统通过自实现的极坐标占据/高度描述子进行全局检索，再用 GICP 验证，无需提供初始位置。该描述子并非官方 Scan Context 实现。

定位成功后，持续与 PCD 的局部区域匹配（默认每 0.5 秒一次）。有效匹配更新 **`map → odom`**，**`odom → lidar_footprint`** 仍使用原始 LIO 输出；不修改前端状态和参考 PCD。

- 使用与建图一致的传感器配置和外参。
- 重复结构可能导致定位延迟或无法定位。使用时检查 `/lio/localization/valid`、状态及里程计时间，不能仅凭缓存 TF 判断定位有效。
- 连续匹配失败或约束超时后，停止发布全局里程计和新 TF，并重新全局搜索；原始 LIO 继续运行。
- 可通过 `use_initial_pose` 和 `initial_pose: [x,y,z,roll,pitch,yaw]` 提供初值，单位为米/度。也可通过 RViz `/initialpose` 在地图坐标系下重置定位。手动初值模式可以不使用索引，自动全局恢复需要索引。

如果已有 PCD 没有索引，可回放一次原始建图 bag 生成：

```bash
ros2 launch super_lio slam.launch.py loop_closure:=true rviz:=false \
  index_reference_map:=/home/USER/road_map/loop_map.pcd
# 在另一终端运行所需转换节点，再播放原始建图 bag。
# 处理完成后 Ctrl+C，等待索引保存完成。
```

此模式只写索引，要求回放轨迹与参考地图坐标大致对齐。可通过 `index_output:=/path/custom.index` 指定输出，并在 `lio.localization.index_path` 中配置使用。PCD 修改后需重新生成索引。

## 话题与 RViz

| 输出 | 含义 |
| --- | --- |
| `/lio/odom`、`/lio/imu/odom` | 原始局部里程计 |
| `/lio/path`、`/lio/cloud_world` | 局部路径、实时点云 |
| `/lio/loop/odom` | 回环修正后的当前位姿 |
| `/lio/loop/path`、`/lio/loop/map` | 优化后的关键帧路径、重建的全局地图 |
| `/lio/loop/constraints` | 已接受的回环连线 |
| `/lio/localization/odom` | 定位有效时的全局修正位姿 |
| `/lio/localization/path` | 已接受的地图匹配位姿，不回溯优化历史路径 |
| `/lio/localization/map` | 参考 PCD 地图 |
| `/lio/localization/status`、`/lio/localization/valid` | 定位状态、有效标志 |

RViz 的 Fixed Frame：原始里程计为 `odom`，回环建图为 `loop_map`，重定位为 `map`。全局修正分别通过 `loop_map → odom` 或 `map → odom` 发布；局部里程计发布 `odom → lidar_footprint`。地图显示使用 **Reliable / Transient Local**，Decay Time 设为 0。

## 论文引用

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
