# Logger 状态比较的时间与坐标契约

当前实现用于本项目的 PX4 Gazebo SITL：Gazebo 世界为 EARTH_WGS84 / ENU、heading=0；
PX4 GZBridge 跟随 Gazebo 仿真时钟；双机 `UXRCE_DDS_SYNCT=0`，ROS 节点统一 `use_sim_time=true`。
配置在 `src/uav_trajectory_tracking/config/trajectory_logging.yaml`。

## 时间配对

- PX4 `timestamp_sample` 已是仿真微秒，直接乘 `1e-6`，与 Gazebo header 配对。
  不订阅 TimesyncStatus，不估计或叠加偏移，也不重复检查 PX4 同步器的 RTT/创新量。
- 不使用消息接收时刻替代采样时间，不把两个流的首帧强制对齐。缺少时间戳、
  时间域错误、重复/倒序或过期的消息不会更新比较状态。
- 位置/速度/加速度使用 `VehicleLocalPosition.timestamp_sample`；姿态使用
  `VehicleAttitude.timestamp_sample`；角速度使用 `VehicleOdometry.timestamp_sample`。
  一条消息只验证一次采样时间，再供各量入队。
- 每个 PX4 样本仅消费一次，在包围其时刻的两帧真值之间插值，不外推。向量线性插值，
  姿态先做四元数最短弧 SLERP，再输出 RPY 和 wrap 到 `[-pi,pi]` 的分量差。
- 默认真值插值跨度不超过 0.1 s；采样年龄和单调墙钟队列等待上限均为 0.5 s。
  独立 `/clock` 与状态回调的到达顺序允许最多 20 ms 仿真时间偏差；不改变原始采样时间。
  状态定时器使用单调墙钟，因此暂停仿真后仍会使旧配对失效。
- 原点变化或 EKF 重置会清空配对缓存并递增 `alignment_epoch`。
  重置估计不会通过重新拟合误差零点来隐藏状态跳变。

配对后的 PX4/truth/error 三个话题拥有相同的 **Gazebo 仿真采样时间**；诊断 header
和 `ros_publish_time_s` 使用发布时的仿真时间。接收等待年龄使用独立的单调墙钟。
控制器直接按曝光仿真时间做姿态 SLERP，见 `fixed_camera_migration.md`。

该契约用于本项目 SITL；连接真实飞控时不能直接沿用关闭 DDS 时钟转换的仿真启动配置。

## 机体参考点

Gazebo `OdometryPublisher` 发布模型原点位姿及该点的 body FLU 速度。
本机 PX4 EKF2 发布 body origin 的位置、速度及加速度；当前默认 IMU 位于 `base_link`，
`EKF2_IMU_POS_*` 为零。展开两种 x500 包装模型后，`base_link` 相对模型原点为 `(0,0,0.24)` m。

logger 先把真值搬移到这个参考点：

```text
p_body_world = p_model_world + R_model_world × r_model_to_body
v_body_world = R_model_world × (v_model_flu + omega_model_flu × r_model_to_body)
```

随后再转换 ENU/NED 和 FLU/FRD。角速度不因刚体上的参考点变化而变化。
真值加速度对修正后的机体原点速度做差分，采样时刻标为差分区间中点；超过允许跨度时不生成加速度。
Gazebo odometry 的速度本身来自有限差分和滤波，因此加速度误差仍会包含滤波及数值微分效应。

更换机型、修改模型 pose、改变 IMU 安装或 PX4 机体参考点时，需重新核实 `truth_body_offset_flu_m`。
该配置假设模型坐标轴与 `base_link` 坐标轴平行；不支持任意安装旋转的机型。

## 本地原点

位置真值先从 Gazebo ENU 经 WGS84 ECEF 转为经纬高，再使用每个 PX4 样本自己的
`ref_lat/ref_lon/ref_alt` 投影为本地 NED。XY 使用与 PX4 `MapProjection` 相同的球面等距方位投影
（半径 6371000 m），Z 为 `ref_alt - truth_alt`。

只在 `xy_global/z_global` 有效、`ref_timestamp` 非零且参考值有效时比较位置。
不依赖无人机出生点，不把首帧或均值误差归零：目标机在世界 `(0,5,0)` 出生造成的坐标原点差会消除，
真实定位偏差仍然保留。估计状态还需满足位置/速度有效性标志和有限数检查。

当前 Gazebo GZBridge 把 NavSat altitude 同时用作 MSL 和椭球高，因此本仿真不另加 geoid 修正。
这不代表真实飞行中 MSL 与椭球高相等。配置的世界经纬高必须与正在运行的 SDF 一致。
局部 ENU/NED 轴转换适用于当前小范围仿真，不把它作为跨大地距离的完整导航坐标变换。

每个比较话题的 `frame_id` 带实例前缀，例如
`x500_1/state_compare/px4_local_ned`，避免把多架无人机的不同本地原点称为同一 frame。
这些是数值比较话题，位置和 RPY 均复用 `Vector3Stamped` 容器，不应把它们作为普通向量直接套用 TF。

## 日志与状态

`px4_estimate.csv`、`gazebo_truth.csv` 保留原始接收日志用途。PX4 表中的附带姿态/角速度仍是
接收时缓存快照，其独立采样时间已记录；原始两表不应直接逐行相减。Gazebo 原始 `*_ned_equiv_*`
字段仍表示模型原点的坐标轴转换结果。

每次运行还保存 `alignment_config.yaml`，记录实际地理参考、机体偏移、输入话题和配对阈值。

定量分析使用新增 `state_comparison.csv`：一行是一种量在共同采样时刻的比较，包含：

- `sample_sim_time_s`、`quantity`、`px4_x/y/z`、`truth_x/y/z`、`error_x/y/z` 分量列；
- `truth_left_sim_time_s` / `truth_right_sim_time_s`、`truth_bracket_s` 和 `px4_queue_age_s`；
- `ref_lat/ref_lon/ref_alt`、`alignment_epoch`、`frame_id`；
- `ros_publish_time_s`，仅用于追踪实际输出时间。

误差符号为 `PX4 - truth`。RPY 单位 rad，角速度 rad/s，位置 m，速度 m/s，加速度 m/s²。
`publish_state_compare_topics=false` 只关闭三联数值话题，配对 CSV 和对齐状态诊断仍保留。

状态话题为 `/x500_0/state_compare/status` 或 `/x500_1/state_compare/status`：

- `waiting_for_sim_clock`：尚未收到仿真时钟；
- `waiting_for_global_reference`：不能进行位置比较，其他量可继续配对；
- `waiting_for_paired_samples`：没有新鲜可比较样本；
- `partially_paired`：只有部分量近期成功配对，查看 `fresh_quantities`；
- `paired`：五种量近期均成功配对。

## 使用与核验

```bash
./scripts/build.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=83 python3 -m pytest -q src/uav_trajectory_tracking/test

# 每个命令在对应仿真启动后使用
ros2 topic echo /x500_0/state_compare/status --once
ros2 topic echo /x500_1/state_compare/status --once
```

两个 launch 均提供 `logger_config_file`。配对 CSV 的 schema_version 为 2，删除旧偏移列；历史日志保持不变。
自定义 logger 配置可直接传给现有启动脚本：

```bash
./scripts/start_visual_interception.sh logger_config_file:=/absolute/path/trajectory_logging.yaml
```

依据：[PX4 ROS 2 时间同步](https://docs.px4.io/main/en/ros2/user_guide)、
[Gazebo NavSat 的 LOCAL2 转换](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/Util.cc)、
[Gazebo OdometryPublisher](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/odometry_publisher/OdometryPublisher.cc)。
本机 PX4 的 `EKF2::PublishLocalPosition`、`GZBridge::clockCallback` 和 DDS 生成序列化代码已逐项核对。
