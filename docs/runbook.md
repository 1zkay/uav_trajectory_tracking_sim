# Runbook

## 快速检查

保持 QGroundControl 运行；若 PX4 拒绝解锁并报告缺少地面站连接，先检查其 SITL 连接。

```bash
source /opt/ros/jazzy/setup.bash
source /home/zk/uav_trajectory_tracking_sim/install/setup.bash
ros2 topic list | rg '/fmu/'
ros2 topic echo /fmu/out/vehicle_status_v4 --qos-reliability best_effort --qos-durability volatile --once
ros2 topic echo /fmu/out/vehicle_local_position_v1 --qos-reliability best_effort --qos-durability volatile --once
```

## 可视化检查

```bash
ros2 topic list | rg 'trajectory|vehicle_path'
ros2 topic echo /trajectory_tracker/current_stage --once
ros2 topic echo /trajectory_path --once
```

`/trajectory_tracker/current_stage` 的阶段编号为：`0=entry`、`1=trajectory`、`2=return`、`3=finished`。
正常轨迹应先进入 `entry`，到达曲线起点并稳定后再进入 `trajectory`。当前默认主机轨迹是
`trajectory_hold.yaml` 的 `(0, 0, -5)` 悬停点；切换到 `trajectory_figure8.yaml` 后可用同一阶段逻辑检查 8 字轨迹。

主机固定相机和 YOLO 检测启动后：

```bash
ros2 topic list | rg 'x500_0/(camera|yolo)'
ros2 topic echo /x500_0/yolo/tracks --once
```

目标无人机启动后，按官方多机规则会出现在 `/px4_1/...` namespace：

```bash
ros2 topic list | rg '/px4_1/fmu/'
ros2 topic echo /px4_1/fmu/out/vehicle_status_v4 --qos-reliability best_effort --qos-durability volatile --once
ros2 topic echo /target/trajectory_tracker/current_stage --once
```

Foxglove 3D 面板使用 `map` 作为固定坐标系和显示坐标系。`trajectory_visualizer` 根据各机
`ref_lat/ref_lon/ref_alt` 将 PX4 本地 NED 转为共同世界 ENU，原点来自 `trajectory_logging.yaml`。
因此显示坐标为 `x=east, y=north, z=up`，并保留两机起飞原点之间的实际偏移。

`x500_0` 是带前向固定相机的主机，底层使用官方 `x500_mono_cam`。
PX4 0 由 `scripts/start_px4_gazebo.sh` 以 `gz_x500_mono_cam` / 4010 启动；
目标机 `x500_1` 继续使用普通 `gz_x500` / `PX4_SYS_AUTOSTART=4001`。

Gazebo 静态标记不需要手工改 world。主机启动脚本默认轨迹改
`src/uav_trajectory_tracking/config/trajectory_hold.yaml`，目标机轨迹改
`src/uav_trajectory_tracking/config/target_trajectory_hold.yaml`，然后重启 PX4/Gazebo：

```bash
./scripts/start_px4_gazebo.sh
./scripts/start_trajectory_tracking.sh
./scripts/start_target_px4_gazebo.sh
./scripts/start_target_trajectory_tracking.sh
```

`start_px4_gazebo.sh` 会调用 `scripts/render_trajectory_world.py`，把基础 world 和风场配置转换成
`build/generated/worlds/trajectory_tracking.sdf`，再同步到
`/home/zk/PX4-Autopilot/Tools/simulation/gz/worlds/trajectory_tracking.sdf`。Gazebo 静态轨迹/方块标记默认关闭；需要调试空间位置时用 `SHOW_TRAJECTORY_VISUALS=true ./scripts/start_px4_gazebo.sh` 打开。
如果使用自定义 YAML，PX4/Gazebo 终端和 ROS 终端都传入同一个变量：

```bash
TRAJECTORY_FILE=/home/zk/my_trajectory.yaml ./scripts/start_px4_gazebo.sh
TRAJECTORY_FILE=/home/zk/my_trajectory.yaml ./scripts/start_trajectory_tracking.sh
TRAJECTORY_FILE=/home/zk/my_target_trajectory.yaml ./scripts/start_target_trajectory_tracking.sh
```

## 视觉拦截检查

视觉拦截使用独立入口：

```bash
./scripts/start_visual_interception.sh
```

它会启动相机桥接、YOLO + BoT-SORT、固定相机目标观测、主机 truth 日志桥接和 `visual_pursuit_interceptor`。视觉拦截时不要同时运行主机 `start_trajectory_tracking.sh`，否则两个节点会同时向 `/fmu/in/trajectory_setpoint` 发布 setpoint。启动后检查：

```bash
ros2 topic list | rg 'fixed_camera_target_tracker|visual_pursuit'
ros2 topic echo /x500_0/fixed_camera_target_tracker/tracking_active --once
ros2 topic echo /x500_0/fixed_camera_target_tracker/lock_active --once
ros2 topic echo /x500_0/fixed_camera_target_tracker/bearing --once
ros2 topic echo /x500_0/visual_pursuit_interceptor/diagnostics --once
```

正常进入追击时，诊断应包含：

```text
state: pursuit
pursuing: true
velocity_control_active: true
lock_active: true
bearing_fresh: true
```

短暂掉锁时应进入 `coast_on_lock_loss` 并继续 velocity control；如果直接 `target_lost` 或 position hold，优先检查 `lock_loss_grace_s`、`/x500_0/fixed_camera_target_tracker/bearing` 是否新鲜，以及固定相机观测是否连续满足确认时长。

## 常见问题

1. 看不到 `/fmu/out/*`

确认 `MicroXRCEAgent udp4 -p 8888` 已启动，并且 PX4 SITL 日志里 `uxrce_dds_client` 正常连接。
当前 PX4 main 的版本化输出话题中，tracker 默认订阅 `/fmu/out/vehicle_status_v4`
和 `/fmu/out/vehicle_local_position_v1`。

2. 节点切不进 Offboard

Offboard 模式要求先持续发送 setpoint。该节点默认先发送 1.5 秒 setpoint，再请求 Offboard 和解锁。

3. 无人机不按预期方向飞

轨迹使用 PX4 本地 NED，不是 ROS ENU。`z=-5` 表示离起飞点高度 5 m。
Foxglove 显示使用公共世界 ENU `map`；不要把显示的高度值直接写回本地 NED YAML。
Gazebo 的 3D 标记由脚本自动完成 NED 到 Gazebo 坐标的转换，不需要手工同步 SDF。
如果 Gazebo 里的轨迹标记相对实际飞行轨迹旋转 90 度，优先检查生成脚本是否使用了
`x=east=NED y, y=north=NED x` 的转换。

4. `x500_0` 不起飞

确认轨迹仿真启动脚本使用的是 `trajectory_tracking` 世界。`trajectory_tracking.sdf` 需要加载 `gz-sim-navsat-system`
和 `gz-sim-magnetometer-system`，并且不包含 `x500_target_moving`。`x500_0` 应包含
`x500_mono_cam_trajectory_wind`，`x500_1` 应包含 `x500_trajectory_wind`。同时必须包含
`spherical_coordinates`，这是 NavSat/GNSS 和 Gazebo 磁场模型生成有效量测的基准。
如果 `/fmu/out/vehicle_local_position_v1` 里 `xy_valid: false`，PX4 没有可用水平位置估计，
Offboard 轨迹仿真不会起飞。若 `estimator_status_flags` 里 `cs_mag_field_disturbed: true`，
先检查 Gazebo 磁罗盘话题是否是约 `0.2-0.5` Gauss 的量级，不要再在 PX4 GZBridge 中额外
乘 `10000`。修改世界或模型文件后需要完整重启 PX4/Gazebo。

5. 起飞后一直向上飞

先确认轨迹没有写错：PX4 本地 NED 里 `z=-5` 表示上升到 5 m，而不是下降。若轨迹正确但
Gazebo 中高度持续增大，通常是状态估计输入和 Gazebo 真值脱节。重点检查：

```bash
gz topic -e -n 1 -t /world/trajectory_tracking/model/x500_0/link/base_link/sensor/navsat_sensor/navsat
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
ros2 topic echo /fmu/out/estimator_status_flags --once
```

正常状态下 NavSat 应输出有效经纬高，`xy_valid/z_valid` 应为 `true`，
`cs_yaw_align/cs_gnss_pos` 应为 `true`，`cs_mag_field_disturbed` 应为 `false`。

6. Foxglove 3D 面板看不到轨迹

主机视觉拦截使用 `start_visual_interception.sh`，目标机使用 `start_target_trajectory_tracking.sh`；
这两个入口都会启动轨迹发布节点。确认公共仿真时钟和 Foxglove Bridge 已运行，再检查发布者：

```bash
ros2 topic info /vehicle_path --verbose
ros2 topic info /target/vehicle_path --verbose
ros2 topic echo /vehicle_path --qos-reliability reliable --qos-durability transient_local --once
```

在 Foxglove 连接 `ws://localhost:8765`，3D 面板的固定/显示坐标系均选择 `map`，
启用 `/vehicle_path` 和 `/target/vehicle_path`；目标规划路径可启用 `/target/trajectory_path`。
当前位置及规划起终点可启用 `/trajectory_markers` 和 `/target/trajectory_markers`，标记话题不重复发布规划线。
视觉拦截模式不发布主机 `/trajectory_path`。若 `poses` 一直为空，检查公共 `/clock`、
`use_sim_time=true`、PX4 位置有效标志和全局参考原点；位置样本的 `timestamp_sample` 不应超前 `/clock` 超过 20 ms。
不要另外启动主机轨迹控制器来补可视化。

## 日志对齐状态

主机检查 `/x500_0/state_compare/status`，目标机检查 `/x500_1/state_compare/status`。
`waiting_for_sim_clock` 时检查 `/clock` 桥接和 `use_sim_time=true`；
`waiting_for_global_reference` 时检查 `VehicleLocalPosition.xy_global/z_global` 和 `ref_*`。
不要用接收时间代替采样时间强行匹配。误差分析使用 `state_comparison.csv`；
详细配置和状态含义见 [Logger 对齐说明](logger_alignment.md)。

## 固定相机时间与坐标检查

重新构建后使用 `bearing_topic`，不要继续传入旧 `visual_error_topic`。
Image、CameraInfo 和 bearing 的 frame 应为 `x500_0/camera_optical_frame`；采集时间均为 Gazebo 时间。
`./scripts/start_simulation_clock.sh` 独立提供 `/world/trajectory_tracking/clock → /clock`，
在启动业务节点前启动一次，并保持到仿真结束。相关 ROS 节点统一使用仿真时钟；双机 `UXRCE_DDS_SYNCT=0`。
两个业务 launch 都不管理时钟桥接，目标机可独立运行，重启视觉链路不会切断其时钟。
外部图像必须提供正确的 optical frame 和仿真采集时间，不要用接收时间给检测结果重新打戳。

```bash
ros2 topic echo /clock --once
ros2 param get /visual_pursuit_interceptor use_sim_time
ros2 run tf2_ros tf2_echo x500_0/base_link x500_0/camera_optical_frame
```

控制器诊断中的 `los_filter_horizontal_deg` / `los_filter_vertical_deg` 是 NED 方位角/俯角，
不再是图像残差；图像偏差仍由 `image_yaw_error_deg` / `image_pitch_error_deg` 表示。
`initializing` 且 `vehicle_state_fresh=false` 时先恢复 PX4 状态输入；此时控制器主动停止 Offboard 输出，
而非依靠旧状态继续飞行。视觉丢失时的 COAST 仅在 PX4 状态仍有效时使用。

视觉 PURSUIT 和 COAST 均使用 `TrajectorySetpoint` 速度控制：`velocity=true`，其他 Offboard 层级为 false。
速度模式的位置、加速度、jerk、yawspeed 字段为 NaN；偏航角沿 LOS 或由 `yaw_mode` 指定。
起飞和搜索使用位置控制。项目不再发布角速度或推力，PX4 内部完成速度、姿态和推力控制。
检查时同时查看 Offboard 模式、速度参考及实际速度，控制历史见 `visual_control.csv`。
接口边界与验证记录见 [视觉制导说明](visual_guidance.md)。
