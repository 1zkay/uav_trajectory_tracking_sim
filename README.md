# UAV Trajectory Tracking Simulation

PX4 SITL + Gazebo Harmonic + ROS 2 Jazzy 的本地 NED 参数化轨迹跟踪仿真工作区。当前视觉链路采用 **YOLO + BoT-SORT 目标跟踪**，主机 `x500_0` 使用官方 **x500_mono_cam 前向固定单目相机**。视觉链路通过相机内参、固定安装姿态和 PX4 机体姿态计算 LOS，保留现有 PNG 外层导引。

云台版本已保存为 Git 标签 `archive/gimbal-20260907`；固定相机修改位于 `feat/fixed-camera` 分支。迁移依据、接口与验证见 [固定相机迁移说明](docs/fixed_camera_migration.md)。

## 环境

- PX4: `/home/zk/PX4-Autopilot`
- Python venv: `/home/zk/px4-venv`
- ROS 2: Jazzy
- Gazebo: Harmonic
- DDS Agent: `MicroXRCEAgent udp4 -p 8888`

## 构建

```bash
cd /home/zk/uav_trajectory_tracking_sim
./scripts/build.sh
```

## 启动

终端 1:

```bash
cd /home/zk/uav_trajectory_tracking_sim
./scripts/start_agent.sh
```

终端 2:

```bash
cd /home/zk/uav_trajectory_tracking_sim
./scripts/start_px4_gazebo.sh
```

如果只做主机轨迹跟踪，终端 3 启动主机轨迹跟踪、相机桥接、YOLO + BoT-SORT 跟踪、日志记录和 RViz 可视化发布节点：

```bash
cd /home/zk/uav_trajectory_tracking_sim
./scripts/start_trajectory_tracking.sh
```

如果做视觉拦截，不要同时启动 `start_trajectory_tracking.sh` 控制主机，因为它和 `visual_pursuit_interceptor` 都会向 `/fmu/in/trajectory_setpoint` 发布 setpoint。视觉拦截流程从目标机开始：

终端 3，启动第二架目标无人机的 PX4 实例：

```bash
cd /home/zk/uav_trajectory_tracking_sim
./scripts/start_target_px4_gazebo.sh
```

终端 4，让目标无人机按目标轨迹飞行：

```bash
cd /home/zk/uav_trajectory_tracking_sim
./scripts/start_target_trajectory_tracking.sh
```

终端 5，启动主机视觉拦截链路。该入口会启动相机桥接、YOLO + BoT-SORT、固定相机目标观测、主机 Gazebo truth 日志桥接和 `visual_pursuit_interceptor`：

```bash
cd /home/zk/uav_trajectory_tracking_sim
./scripts/start_visual_interception.sh
```

终端 6 可选，用于查看轨迹、目标点、规划路径和实际飞行尾迹：

```bash
cd /home/zk/uav_trajectory_tracking_sim
./scripts/start_rviz.sh
```

终端 7 可选，用于通过 Foxglove 查看 ROS 2 话题、曲线、图像和运动状态：

```bash
cd /home/zk/uav_trajectory_tracking_sim
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=127.0.0.1 port:=8765
```

打开 Foxglove 后选择 `Foxglove WebSocket`，连接地址为：

```text
ws://localhost:8765
```

当前 PX4 main 在这台机器上发布的是 `/fmu/out/vehicle_status_v4` 和
`/fmu/out/vehicle_local_position_v1`，启动文件默认已经使用这两个话题。
`start_trajectory_tracking.sh` 会同时启动控制节点、RViz 可视化发布节点、相机桥接和 YOLO + BoT-SORT 跟踪节点。

## 多机与坐标系

第二架目标无人机按 PX4 Gazebo 官方多机方式运行：每架机一个独立 PX4 SITL 实例。
主机使用默认实例 `px4_instance=0`、Gazebo 模型 `x500_0`、PX4 官方
`gz_x500_mono_cam` airframe（4010）、ROS 2 话题 `/fmu/...`、`MAV_SYS_ID=1`。目标机使用
`scripts/start_target_px4_gazebo.sh` 启动为
`px4_instance=1`，连接 Gazebo 中预加载的 `x500_1`；PX4 会自动设置
`MAV_SYS_ID=2`、`UXRCE_DDS_KEY=2`，ROS 2 话题带 `/px4_1/...` namespace。
因此目标机的 Offboard 控制必须向 `/px4_1/fmu/in/...` 发布，并把
`VehicleCommand.target_system` 设为 `2`。`scripts/start_target_trajectory_tracking.sh`
已经按这个规则配置。`px4_instance > 0` 时不要复用主机的 `/fmu/in/*` 话题。

主机启动脚本默认轨迹在 `src/uav_trajectory_tracking/config/trajectory_hold.yaml`，目标机默认轨迹在
`src/uav_trajectory_tracking/config/target_trajectory_linear.yaml`。两者坐标系都是 PX4 本地 NED：

- `x`: 北/前
- `y`: 东/右
- `z`: 下，起飞点上方 5 m 写作 `-5.0`

只改主机默认轨迹时修改 `trajectory_hold.yaml` 或用 `TRAJECTORY_FILE=...` 覆盖；只改目标机轨迹时修改 `target_trajectory_linear.yaml` 或用 `TRAJECTORY_FILE=...` 覆盖。
`scripts/start_px4_gazebo.sh` 会在启动前自动根据主机 YAML
生成 Gazebo world，并同步到 PX4 的 worlds 目录；`scripts/start_trajectory_tracking.sh`
也会默认把同一个主机 YAML 传给控制节点和 RViz 可视化节点。目标机启动脚本默认使用
`target_trajectory_linear.yaml`，用于给拦截链路提供持续横向运动目标。若需要定点目标，可把
`TRAJECTORY_FILE` 指向 `src/uav_trajectory_tracking/config/target_trajectory_hold.yaml`。
轨迹控制采用 `entry -> trajectory -> return -> finished` 阶段：无人机先飞到曲线起点，
满足统一的 `acceptance_radius_m` 到达判定并短暂稳定后，才开始参数化时间 `t`。默认主机
`trajectory_hold.yaml` 的悬停点为 `(0, 0, -5)`；如果改用 `trajectory_figure8.yaml`，
8 字轨迹的交叉点、起点和终点均为 `(0, 0, -5)`，QGC 的水平轨迹不会包含额外的长距离进场线。

使用自定义轨迹文件时，PX4/Gazebo 终端和 ROS 终端都传入同一个变量：

```bash
TRAJECTORY_FILE=/home/zk/my_trajectory.yaml ./scripts/start_px4_gazebo.sh
TRAJECTORY_FILE=/home/zk/my_trajectory.yaml ./scripts/start_trajectory_tracking.sh
TRAJECTORY_FILE=/home/zk/my_target_trajectory.yaml ./scripts/start_target_trajectory_tracking.sh
```

Gazebo 如果暂停，点击左下角播放按钮。轨迹仿真默认使用
`trajectory_tracking` 世界，预加载主机 `x500_0` 和目标机 `x500_1`。`x500_0` 使用本仓库的
`x500_mono_cam_trajectory_wind` 包装模型，内部直接包含官方 `x500_mono_cam`；
`x500_1` 仍使用普通
`x500_trajectory_wind`。两个机体都由 world 预加载，PX4 启动后分别通过
`PX4_GZ_MODEL_NAME=x500_0` 和 `PX4_GZ_MODEL_NAME=x500_1` 连接，因此可以只在本仓库内启用
受风模型和真值 odometry，不修改 PX4 原始 `x500_base` / `x500_mono_cam`。固定相机使用官方渲染设置，不再应用旧云台的自机消隐。
`default1` 保留给 `/home/zk/gimbal_track` 使用，其中仍包含 `x500_target_moving`。
Gazebo 世界默认不插入轨迹线、边界、参照方块或风向箭头，以减少视觉识别干扰；RViz 使用 ROS ENU
`map` 坐标显示轨迹，其中 PX4 NED 会自动转换为 `x=east, y=north, z=up`。如果需要临时查看 Gazebo 静态轨迹标记，可用 `SHOW_TRAJECTORY_VISUALS=true ./scripts/start_px4_gazebo.sh` 启动。
Gazebo 世界坐标同样按 ENU 显示：`Gazebo x=east`、`Gazebo y=north`、`Gazebo z=up`。

## 固定相机与视觉观测

主机采用 PX4 官方 `x500_mono_cam`：`CameraJoint` 为 `fixed`，相机相对机体
位姿为 `(0.12, 0.03, 0.242, 0, 0, 0)`，图像为 1280×960、30 Hz。
Gazebo 图像及 CameraInfo 话题中的传感器仍叫 `camera`，ROS 话题保持：

- `/x500_0/camera/image_raw`
- `/x500_0/camera/camera_info`
- `/x500_0/yolo/tracks`
- `/x500_0/yolo/tracks_image`

视觉入口增加 `fixed_camera_target_tracker`，从实际 `CameraInfo.K` 计算目标角偏差。
`/x500_0/fixed_camera_target_tracker/error` 的 x/y 单位为 **弧度**，正方向为图像右/下，z 为置信度。
`tracking_active` 表示新鲜有效观测；`lock_active` 表示同一目标已连续观测达到确认时长，
不要求目标先居中。未收到内参、观测过期或 frame 不匹配时不输出可用观测。

控制器在固定模式下不订阅云台关节或搜索反馈，也不启动云台管理、伺服或性能监测节点。
无目标时在悬停点以机体偏航搜索，观测到目标后通过 `yaw_mode: face_los` 转向目标；
固定相机的画面会随无人机俯仰和横滚变化，不具备云台的独立稳像能力。

配置文件：

- `config/yolo_tracking.yaml`：检测、跟踪和推理参数。
- `config/fixed_camera_tracking.yaml`：目标 ID/类别、置信度、超时及确认时长。
- `config/visual_interception.yaml`：固定外参、图像观测、偏航搜索及现有导引参数。

以上路径相对 `src/uav_trajectory_tracking/`。可使用 `FIXED_CAMERA_CONFIG_FILE` 覆盖观测参数文件。
普通轨迹入口保留旧云台选项供旧模型使用，但默认关闭；当前固定机型不要启用它们。

## 风场

默认风场在 `src/uav_trajectory_tracking/config/wind.yaml`，坐标系是 Gazebo ENU：

- `linear_velocity_mps[0]`: 向东风速
- `linear_velocity_mps[1]`: 向北风速
- `linear_velocity_mps[2]`: 向上风速

默认值 `[3.0, 0.0, 0.0]` 是 3 m/s 向东水平风。风场使用 Gazebo 标准的
`<wind>` + `WindEffects` 结构，`wind_effects` 下的字段和 Gazebo SDF 标签保持一致；
其中 `time_for_rise` / `period` 单位是秒，方向扰动 `amplitude` 单位是弧度。
启动前改 YAML 会自动渲染到 Gazebo world；仿真运行中也可以临时改基础风速：

```bash
cd /home/zk/uav_trajectory_tracking_sim
./scripts/set_wind.sh 3.0 0.0 0.0   # 3 m/s east wind
```

常用可复现实验风况放在 `src/uav_trajectory_tracking/config/wind_profiles/`：

- `calm.yaml`: 无风基线。
- `steady_east_3ms.yaml`: 3 m/s 向东恒定风。
- `gust_east_3ms.yaml`: 3 m/s 向东基础风，带轻微阵风，和默认配置一致。
- `gust_east_5ms.yaml`: 5 m/s 向东基础风，带更强阵风。

使用指定风况启动：

```bash
WIND_FILE=/home/zk/uav_trajectory_tracking_sim/src/uav_trajectory_tracking/config/wind_profiles/gust_east_5ms.yaml \
  ./scripts/start_px4_gazebo.sh
```

监听 Gazebo 运行时风速控制消息：

```bash
./scripts/watch_wind.sh
```

## 轨迹日志

每次启动 `start_trajectory_tracking.sh` 或 `start_visual_interception.sh` 时默认会同步记录主机
`x500_0` 的两份 CSV 轨迹日志，目录为 `log/trajectory_runs/<启动时间>/` 或
`log/trajectory_runs/host_<启动时间>/`。`start_target_trajectory_tracking.sh` 会记录目标机
`x500_1`，目录名前缀为 `target_`：

- `px4_estimate.csv`: PX4 EKF/控制侧看到的估计状态，坐标系为 PX4 local NED，机体系为 FRD。
- `gazebo_truth.csv`: Gazebo 物理真值 odometry，位置原始坐标系为 Gazebo ENU，twist 原始坐标系为
  `child_frame_id` 对应的 body FLU，同时写入 PX4 可比的 NED/FRD 等效列。

真值日志由 `x500_mono_cam_trajectory_wind` 内的 Gazebo `OdometryPublisher` 发布
`/model/x500_0/odometry_with_covariance`，再通过 `ros_gz_bridge` 桥接到 ROS 2。PX4 估计日志订阅
`/fmu/out/vehicle_local_position_v1`、`/fmu/out/vehicle_attitude` 和
`/fmu/out/vehicle_odometry`。Gazebo 真值 CSV 中的速度会先由 body FLU 旋转到 NED，
姿态会由 ENU/FLU 转为 NED/FRD；真值加速度由转换后的 NED 速度差分得到；
PX4 估计 CSV 中的加速度直接来自 `VehicleLocalPosition.ax/ay/az`。
logger 同时会发布位置、速度、加速度、姿态和角速度的在线对比话题，主机默认前缀为
`/x500_0/state_compare`，目标机脚本默认前缀为 `/x500_1/state_compare`，可直接在
Foxglove `Plot` 面板中画 `vector.x/y/z`。
两份 CSV 都保留 `ros_time_s` / `ros_elapsed_s`；其中 `ros_time_s` 是写入时的系统时间，
`ros_elapsed_s` 是 logger 启动后的相对时间。PX4 表额外写入 `px4_time_s` 和
`px4_elapsed_s`，Gazebo 表额外写入 `gazebo_time_s` 和 `gazebo_elapsed_s`。
跨表误差分析优先用 `ros_time_s` 或 `ros_elapsed_s` 做最近邻/插值对齐；`px4_elapsed_s`
和 `gazebo_elapsed_s` 只适合同一来源内部分析，不应直接互相对齐。

可以指定日志根目录或本次运行名。视觉拦截链路同样支持这些变量：

```bash
LOG_ROOT=/home/zk/uav_logs RUN_ID=wind_3ms_figure8 ./scripts/start_trajectory_tracking.sh
LOG_ROOT=/home/zk/uav_logs RUN_ID=host_intercept_01 ./scripts/start_visual_interception.sh
```

不需要 CSV 记录时：

```bash
./scripts/start_trajectory_tracking.sh enable_csv_logging:=false
ENABLE_CSV_LOGGING=false ./scripts/start_visual_interception.sh
```

`enable_csv_logging:=false` 会关闭 `trajectory_logger`，因此也不会发布在线对比话题。
如果只是不需要在线对比、仍要保留 CSV，使用下面的开关。

不需要在线对比话题时：

```bash
PUBLISH_STATE_COMPARE_TOPICS=false ./scripts/start_trajectory_tracking.sh
PUBLISH_STATE_COMPARE_TOPICS=false ./scripts/start_visual_interception.sh
```

## 可视化话题

- `/fmu/out/vehicle_status_v4`: 主机 PX4 状态，包括导航状态、解锁状态和 failsafe 相关状态。
- `/fmu/out/vehicle_local_position_v1`: 主机 PX4 EKF 本地位置/速度/加速度估计，坐标系为 PX4 local NED，常用字段为 `x/y/z`、`vx/vy/vz`、`ax/ay/az`、`heading` 和有效标志。
- `/fmu/out/vehicle_attitude`: 主机 PX4 姿态估计，`q` 为 body FRD 到 local NED 的四元数，顺序为 `w,x,y,z`。
- `/fmu/out/vehicle_odometry`: 主机 PX4 里程计估计，包含位置、姿态、速度、body FRD 角速度和方差。
- `/model/x500_0/odometry_with_covariance`: 主机 Gazebo truth odometry。位置原始坐标系为 Gazebo ENU；twist 按 `child_frame_id` 在 body FLU 下表达，`trajectory_logger` 会转换出 NED/FRD 等效列。
- `/x500_0/state_compare/px4_position_ned`: 主机 PX4 估计位置，`vector.x/y/z = N/E/D`。
- `/x500_0/state_compare/truth_position_ned`: 主机 Gazebo truth 转换后的 NED 位置。
- `/x500_0/state_compare/position_error_ned`: 主机位置误差，`PX4 - truth`，`vector.x/y/z = N/E/D`。
- `/x500_0/state_compare/px4_velocity_ned`: 主机 PX4 估计速度，`vector.x/y/z = N/E/D`。
- `/x500_0/state_compare/truth_velocity_ned`: 主机 Gazebo truth 转换后的 NED 速度。
- `/x500_0/state_compare/velocity_error_ned`: 主机速度误差，`PX4 - truth`。
- `/x500_0/state_compare/px4_acceleration_ned`: 主机 PX4 估计加速度，`vector.x/y/z = N/E/D`，单位为 `m/s^2`。
- `/x500_0/state_compare/truth_acceleration_ned`: 主机 Gazebo truth NED 速度差分得到的加速度，单位为 `m/s^2`。
- `/x500_0/state_compare/acceleration_error_ned`: 主机加速度误差，`PX4 - truth`，单位为 `m/s^2`。
- `/x500_0/state_compare/px4_rpy_ned_frd`: 主机 PX4 姿态估计，`vector.x/y/z = roll/pitch/yaw`。
- `/x500_0/state_compare/truth_rpy_ned_frd`: 主机 Gazebo truth 转换后的 NED/FRD 姿态。
- `/x500_0/state_compare/rpy_error_ned_frd`: 主机姿态误差，`PX4 - truth`，yaw 误差会 wrap 到 `[-pi, pi]`。
- `/x500_0/state_compare/px4_angular_velocity_body_frd`: 主机 PX4 body FRD 角速度。
- `/x500_0/state_compare/truth_angular_velocity_body_frd`: 主机 Gazebo truth 转换后的 body FRD 角速度。
- `/x500_0/state_compare/angular_velocity_error_body_frd`: 主机角速度误差，`PX4 - truth`。
- `/fmu/in/offboard_control_mode`: 主机 PX4 Offboard 控制模式输入。
- `/fmu/in/trajectory_setpoint`: 主机 PX4 轨迹/速度 setpoint 输入，轨迹跟踪或视觉拦截控制器会发布到这里。
- `/fmu/in/vehicle_command`: 主机 PX4 MAVLink 命令输入，例如切模式、解锁、降落。
- `/fmu/out/vehicle_command_ack_v1`: 主机 PX4 命令 ACK 输出。
- `/px4_1/fmu/out/vehicle_status_v4`: 目标机 PX4 状态。
- `/px4_1/fmu/out/vehicle_local_position_v1`: 目标机 PX4 EKF 本地位置/速度/加速度估计。
- `/px4_1/fmu/out/vehicle_attitude`: 目标机 PX4 姿态估计。
- `/px4_1/fmu/out/vehicle_odometry`: 目标机 PX4 里程计估计。
- `/trajectory_markers`: 轨迹起点、终点和当前飞行位置。
- `/trajectory_path`: YAML 参数化曲线采样得到的规划轨迹。
- `/vehicle_path`: 飞行过程中累积的实际轨迹。
- `/trajectory_tracker/current_stage`: 当前轨迹阶段，`0=entry`、`1=trajectory`、`2=return`、`3=finished`。
- `/x500_0/camera/image_raw`: 主机固定相机原始图像。
- `/x500_0/camera/camera_info`: 主机固定相机内参。
- `/x500_0/yolo/tracks`: YOLO + BoT-SORT 跟踪框，类型为 `vision_msgs/Detection2DArray`，其中 `Detection2D.id` 是跨帧 track id。
- `/x500_0/yolo/tracks_image`: YOLO + BoT-SORT 标注后的图像。
- `/x500_0/fixed_camera_target_tracker/error`：图像右/下角误差（rad）及置信度，保留观测时间戳。
- `/x500_0/fixed_camera_target_tracker/tracking_active`：有新鲜有效目标观测。
- `/x500_0/fixed_camera_target_tracker/lock_active`：同一目标连续确认完成。
- `/x500_0/visual_pursuit_interceptor/diagnostics`: 视觉拦截诊断，包含 `state`、`pursuing`、`velocity_control_active`、`visual_error_fresh`、`dkf_*`、`closing_speed_mps`、`visual_los_ned_*`、`los_rate_*`、相机安装模式和输出速度。
- `/target/trajectory_markers`: 目标无人机轨迹可视化。
- `/target/trajectory_path`: 目标无人机规划路径。
- `/target/vehicle_path`: 目标无人机实际轨迹。
- `/target/trajectory_tracker/current_stage`: 目标无人机当前轨迹阶段，编号含义同主机。

## 常见检查

如果 `x500_0` 不起飞，先确认：

```bash
source /opt/ros/jazzy/setup.bash
source /home/zk/uav_trajectory_tracking_sim/install/setup.bash
ros2 topic info /fmu/out/vehicle_status_v4 --verbose
ros2 topic echo /fmu/out/vehicle_local_position_v1 --qos-reliability best_effort --qos-durability transient_local --once
```

如果没有跟踪结果，检查：

```bash
ros2 topic echo /x500_0/camera/image_raw --once
ros2 topic echo /x500_0/camera/camera_info --once
ros2 topic echo /x500_0/yolo/tracks --once
```

并确认：

- `yolov8s.pt` 是否存在于仓库根目录，或通过 `YOLO_WEIGHTS_PATH` 指定正确路径。
- `src/uav_trajectory_tracking/config/yolo_tracking.yaml` 中的 `classes` 是否过滤掉目标类别。
- `confidence_threshold` 是否过高。
- 相机图像桥接是否开启：`ENABLE_CAMERA_BRIDGE=true`。

如果视觉链路不进入 `pursuit`，检查：

```bash
ros2 topic echo /x500_0/fixed_camera_target_tracker/lock_active --once
ros2 topic echo /x500_0/fixed_camera_target_tracker/error --once
ros2 topic echo /x500_0/visual_pursuit_interceptor/diagnostics --once
```

确认 CameraInfo 为 1280×960、fx/fy 有效，观测与内参 frame 一致，
`camera_mount=fixed`、`visual_error_fresh=true`。检测需连续满足
`fixed_camera_tracking.yaml` 的 `lock_confirm_s`，且推理延迟小于 `observation_timeout_s`。
短暂掉锁保留原 `coast_on_lock_loss` 衰减行为，超时后悬停并转向搜索。

## World 文件说明

`/home/zk/PX4-Autopilot/Tools/simulation/gz/worlds/trajectory_tracking.sdf`
需要只包含基础世界、预加载的 `x500_0` / `x500_1` 和可选风场，
并加载 `AirSpeed`、`NavSat`、`Magnetometer`、`WindEffects` 等系统插件。该世界还必须包含
`spherical_coordinates`，否则 Gazebo 的 NavSat/GNSS 和磁场基准会异常，PX4 可能出现
安全检查失败或起飞后高度估计发散。修改世界文件后必须重启 PX4/Gazebo。

本仓库的 `px4_overlays/worlds/trajectory_tracking.sdf` 是基础世界，只放 Gazebo/PX4
必须的物理、传感器、地面、固定相机主机 `x500_0`、普通目标机 `x500_1` 和地理基准；
轨迹线、起降垫、边界和初始风向箭头只在传入 `SHOW_TRAJECTORY_VISUALS=true` 时由
`scripts/render_trajectory_world.py` 渲染到
`build/generated/worlds/trajectory_tracking.sdf`，再由 `scripts/start_px4_gazebo.sh`
同步到 PX4 的 Gazebo worlds 目录。
