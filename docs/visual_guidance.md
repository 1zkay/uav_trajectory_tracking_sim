# 固定相机视觉制导与 PX4 速度控制接口

当前版本保留《Precise Interception Flight Targets by Image-based Visual Servoing of Multicopter》
的 LOS 与 PNG 速度方向方法，并将 NED 速度和偏航角交给 PX4 速度控制器。
项目不再实现论文的图像偏航 PD、推力方向控制及总推力计算；姿态、角速度和推力由 PX4 内环负责。
因此属于工程适配，不是论文完整复现，也不继承其完整稳定性或命中率结论。
估计层复用 PX4 状态及 LOS 角度 Kalman 滤波，不另建论文的 18 维 DKF。

代码：`src/uav_trajectory_tracking/uav_trajectory_tracking/visual_pursuit_interceptor.py`。
配置：`src/uav_trajectory_tracking/config/visual_interception.yaml`。

## 观测和统一状态样本

场景限定为单目标，观测节点忽略 track ID（包括空 ID），按既有类别、置信度及几何条件选择有效观测。
ID 变化不重置锁定确认或中断 bearing；真正的观测超时仍会清空确认历史。
`target_track_id` 参数已删除，上游跟踪 ID 仅作为跟踪输出信息。

`observation_timeout_s` 只在 `fixed_camera_tracking.yaml` 的 `/**` 参数段定义，
视觉 launch 将该参数文件同时传给观测节点和制导节点；默认 0.2 s。
`visual_interception.yaml` 中的 `tracking_status_timeout_s` 统一控制两个 Bool 状态话题的接收超时，
默认仍为 0.2 s，各话题独立记录接收时间。它替代原来的 `tracking_active_timeout_s` / `lock_active_timeout_s`。
LOS 预测上限仍为 0.12 s；本次配置去重不改变有效观测期限的取最小值规则。
单独运行制导节点时，共享期限通过 ROS 参数文件或 `-p observation_timeout_s:=...` 传入，
不再从制导业务 YAML 读取。

1. 相机 optical bearing 使用曝光时间，经固定安装旋转和曝光姿态 SLERP 转为 NED LOS。
   姿态历史和当前速度都来自 `/fmu/out/vehicle_odometry`；不拼接独立话题的最新姿态、速度。
2. LOS 角度/角速度滤波在采集时刻更新，在新里程计样本到达后预测至该样本的 `timestamp_sample`。
   固定安装旋转及其逆矩阵在初始化时计算。
3. 用同一样本的姿态将预测 LOS 转回 optical：要求有限射线、正深度，且依据实际 `CameraInfo.K`
   投影至 `[0,width) × [0,height)`；无效时进入既有丢失流程，不截断分母继续追踪。
4. 只有新状态样本推进制导历史和速度参考。其他发布周期保持上一速度及偏航角，更新消息时间戳。
   观测过期和状态失效检查仍按每个发布周期执行，不能无限保持失效指令。

相机平移描述光心相对机体的位置；未知目标距离时不重新锚定射线。
Gazebo 真值只用于评估和日志，不进入视觉制导。公共时钟和 logger 对齐流程保持原实现。

## 速度方向与速度大小

垂直角为 `atan2(z, hypot(x,y))`，水平角为 `atan2(y,x)`；对应 NED 方向为
`[cos(vertical)*cos(horizontal), cos(vertical)*sin(horizontal), sin(vertical)]`。
为配合 PX4 的有限速度响应，PNG 按角参考积分实现：

```
desired_angle[k] = desired_angle[k-1] + K * delta_los_angle
```

参考从 LOS 初始化，之后保留累计转向要求，不被尚未跟上的实际速度方向覆盖。
这保留了比例导引的角增量关系，但不是论文式（9）中以上一实际速度角为基准的逐式复现。
每个新样本存储实际速度角用于有效性判定与诊断，跨 ±pi 使用最短角差：

- 首次或重获目标时，以 LOS 为期望速度方向初始化。
- 实际速度非零且达到 `min_velocity_direction_mps`，当前方向与上一样本方向、LOS 历史均有效后运行 PNG。
- 低速时沿 LOS 给指令，同时清空不可用的实际速度角；速度恢复的首个样本重新初始化，下一有效样本运行 PNG。
- 不再要求速度方向与 LOS 夹角小于指定阈值，也不保留 10°/20° 滞回或 `png_active` 切换状态。
- 既有的 PNG 候选方向与 LOS 点积非正时，本样本仍采用 LOS，避免直接给出远离目标的速度指令。

默认 PNG 两轴增益均为 3。没有额外对准等待阶段；首个样本初始化，后续新样本开始提高速度参考。

速度参考独立于后续实际速度爬升：

```
首次或重新捕获：speed_ref = min(|actual_velocity|, speed_limit)
后续新样本：    speed_ref = min(speed_ref + speed_accel_mps2 * dt, speed_limit)
velocity_sp = speed_ref * desired_direction_ned
```

`dt` 为相邻制导状态样本时间差。默认速度上限 3 m/s，参考速度增长率 1 m/s²。
首次样本 `dt=0`，后续样本开始增长；不再每周期从实际低速重新计算速度参考。
增长率约束的是速度大小，不是转向时三维速度向量的变化率。实际速度响应、加速度及倾斜限制由 PX4 决定。

## PX4 输出和丢失处理

| 阶段 | Offboard 层级 | TrajectorySetpoint |
| --- | --- | --- |
| 起飞、进场、等待、偏航/高度搜索 | position | 位置和偏航角 |
| PURSUIT（LOS 初始化或 PNG） | velocity | NED 速度和偏航角 |
| COAST | velocity | 衰减的 NED 速度和最后 LOS 偏航角 |

速度模式下 `position`、`acceleration`、`jerk` 和 `yawspeed` 均为 NaN，避免混入位置控制或加速度前馈。
`yaw_mode: face_los` 使用 `atan2(los_E, los_N)`，由 PX4 控制机头朝向；`fixed_north` 使用零偏航角。
`body_rate`、`attitude`、`thrust_and_torque`、`direct_actuator` 标志均为 false。
不发布 `VehicleRatesSetpoint`，不保留旧角速度话题参数或自定义推力计算。
接口依据：[PX4 Offboard 官方文档](https://docs.px4.io/v1.17/en/flight_modes/offboard#copter)。

COAST 从最后有效追踪的状态样本时刻计算宽限期，视觉节点持续发送 `lock=true` 不会延长失效追踪。
进入 COAST 时清空方向和速度参考历史，以实际速度初始化指数减速；重获目标后重新初始化速度参考及方向判定。
COAST 结束后捕获当前位置并搜索。EKF 重置统一由 `VehicleOdometry.reset_counter` 清空观测和制导历史。

## 配置、诊断与验证

自定义配置需移除 `max_guidance_accel_mps2`、`max_body_rate_rad_s`、`yaw_error_kp`、`yaw_error_kd`；
启动参数 `vehicle_rates_setpoint_topic` 及环境变量 `VEHICLE_RATES_SETPOINT_TOPIC` 已删除。
`los_filter_*`、`max_pursuit_speed_mps`、低速方向阈值和搜索配置继续使用。
`pursuit_alignment_angle_deg` 配置及 `png_active` 诊断已删除。

`visual_control.csv` 记录实际/期望 NED 速度、LOS、`guidance_sample_timestamp_us`、
`guidance_projection_valid` 和状态；旧角速度、推力和自定义加速度诊断列已删除。历史 CSV 保留原表头。

删除角度门槛并改为方向参考积分后，113 项 pytest 通过；覆盖低速初始化/恢复、
不依赖夹角的 PNG 更新、参考在实际速度滞后时保持、新样本与指令保持、重获目标及出画处理。
简化速度响应闭环覆盖静止、向前、侧向、反向和垂直初速度，以及 10/20 ms 更新周期，
在有恒定横向/垂直扰动时保持原来的 20 s 内距离小于 1 m 判据，全部通过。
单独删除门槛而保留上一实际速度角作为基准时，原有 6 个闭环案例均失败；
因此不能把门槛视为此前完全无效的代码。修正参考积分后才消除了对它的依赖。
这些是制导层简化模型测试，本次未运行完整 PX4/Gazebo SITL，不代表实际飞行性能。
此前 `ibvs_*`、`host_20260907_173124`、`host_20260907_174358` 及
`speed_calm_20260907_175543_*` 日志来自旧角速度/推力版本，不能作为当前速度接口的性能结果。

此前保留角度门槛的无风双机 SITL 验证：首次 PURSUIT 后约 12.83 s 内，双机模型原点距离从 6.30 m 降至 0.64 m，
实际追踪速度最高 2.74 m/s；仍有视觉掉锁及 COAST。测试进程已关闭，
详见 [速度接口实测记录](../log/trajectory_runs/velocity_calm_20260907_180908/analysis.md)。
该单次无风近距结果不作为命中率、风场或运动目标性能保证。


单目标 ID 筛选删除后，115 项 pytest 和包构建通过。同配置无风双机 SITL 中，
追踪期间 13 次 track ID 变化未触发 COAST；3.86 s 内模型原点距离从 6.36 m 降至 0.99 m。
详见 [单目标 ID 连续性验证](../log/trajectory_runs/single_target_id_20260907_195715/analysis.md)。
本轮验证 ID 更换不再导致间歇刹车，不作为全部机动条件下的稳定性或碰撞成功率保证。
