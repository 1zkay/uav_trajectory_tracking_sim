# Visual Pursuit Interception

本文档说明当前 `visual_pursuit_interceptor` 的实际实现。云台视觉伺服提供导引头锁定和图像角误差，拦截器用云台关节角加残余图像角构造视觉 LOS，并按
`articles/Precise Interception Flight Targets by Image-based Visual Servoing of Multicopter.pdf`
中的比例导引速度角更新方法生成 PX4 NED velocity/acceleration setpoint。

## 控制分层

```text
YOLO + BoT-SORT
  -> gimbal_target_tracker: 图像误差视觉伺服，把目标拉到相机中心
  -> seeker lock: tracking_active / lock_active / error
  -> visual_pursuit_interceptor: gimbal-camera LOS / paper-style PNG velocity angle guidance
  -> PX4 Offboard velocity/acceleration 或 position setpoint
  -> PX4 速度、姿态和电机内环
```

`tracking_active` 表示有新鲜目标检测/跟踪；`lock_active` 表示云台导引头已经把目标居中并且残差稳定。拦截器只在 `lock_active`、云台 joint feedback 和 `/x500_0/gimbal_target_tracker/error` 都新鲜时进入 `pursuit`。

## 启动入口

常规顺序：

```bash
./scripts/start_agent.sh
./scripts/start_px4_gazebo.sh
./scripts/start_target_px4_gazebo.sh
./scripts/start_target_trajectory_tracking.sh
./scripts/start_visual_interception.sh
```

视觉拦截时不要同时运行主机 `start_trajectory_tracking.sh`，否则 `trajectory_tracker` 和 `visual_pursuit_interceptor` 会同时向 `/fmu/in/trajectory_setpoint` 发布 setpoint。

## 状态机

```text
initializing
takeoff
transit_to_hover
hold
gimbal_search
acquiring_target
pursuit
coast_on_lock_loss
target_lost
```

主要转移逻辑：

```text
vehicle/gimbal 未准备好
  -> initializing / hold / target_lost

初始高度和悬停点到达
  -> hold

tracking_active 或 lock_active 新鲜，但未满足追击条件
  -> acquiring_target

lock_active 新鲜，云台反馈新鲜，视觉误差新鲜
  -> pursuit

pursuit 后短暂掉锁，且仍在 lock_loss_grace_s 内
  -> coast_on_lock_loss

长时间掉锁或检测丢失
  -> target_lost，捕获当前位置并等待云台搜索

云台状态为 local_search/global_search 且垂直搜索开启
  -> gimbal_search，固定 XY 并按配置上下扫描高度
```

`pursuit` 和 `coast_on_lock_loss` 使用 PX4 velocity control：

```text
OffboardControlMode.position = false
OffboardControlMode.velocity = true
TrajectorySetpoint.position = [NaN, NaN, NaN]
TrajectorySetpoint.velocity = [vx, vy, vz]
```

`hold`、`gimbal_search`、`acquiring_target` 和 `target_lost` 使用 PX4 position control。

## 视觉 LOS

云台节点发布 `/x500_0/gimbal_target_tracker/error`，其中：

```text
vector.x = yaw image angular error, deg
vector.y = pitch image angular error, deg
vector.z = target score
```

拦截器先把图像残余角按 `visual_error_yaw_sign` 和
`visual_error_pitch_sign` 转成 Gazebo/PX4 使用的相机射线符号，再变成相机传感器坐标下的目标射线：

```text
signed_yaw_error   = visual_error_yaw_sign * yaw_error
signed_pitch_error = visual_error_pitch_sign * pitch_error
ray_sensor = normalize([1, tan(signed_yaw_error), tan(signed_pitch_error)])
```

再使用与 SDF 匹配的云台运动学常量把它转成 PX4 body FRD 和 NED：

```text
R_base_sensor =
  R_mount_rpy
  * R_axis(yaw_axis, yaw)
  * R_axis(roll_axis, roll)
  * R_axis(pitch_axis, pitch)
  * R_sensor_rpy

los_flu = R_base_sensor * ray_sensor
los_body_frd = [los_flu.x, -los_flu.y, -los_flu.z]
visual_los_ned = R_body_to_ned(vehicle_attitude.q) * los_body_frd
```

`gimbal_los_ned` 仍表示云台光轴方向；`visual_los_ned` 是加入图像残余角后的目标 LOS，也是默认导引用的 LOS。

## 论文式 PNG

当前实现把 NED 方向拆成两个角：

```text
q_vertical   = atan2(los_z, hypot(los_x, los_y))
q_horizontal = atan2(los_y, los_x)
```

速度方向角同样由当前速度或上一帧期望方向给出。按论文 Eq. 6 和 Eq. 9 的离散形式更新期望速度角：

```text
sigma_vertical_d =
  sigma_vertical_previous
  + png_vertical_gain * wrap(q_vertical(k) - q_vertical(k-1))

sigma_horizontal_d =
  sigma_horizontal_previous
  + png_horizontal_gain * wrap(q_horizontal(k) - q_horizontal(k-1))
```

再转换成期望速度方向和速度 setpoint：

```text
nvd = [
  cos(sigma_vertical_d) * cos(sigma_horizontal_d),
  cos(sigma_vertical_d) * sin(sigma_horizontal_d),
  sin(sigma_vertical_d)
]

velocity_cmd = pursuit_speed_mps * nvd
```

速度变化率由 `max_pursuit_accel_mps2` 限制。PX4 acceleration setpoint 按论文 Eq. 17 的形式由速度误差生成：

```text
accel_cmd = (velocity_cmd - current_velocity_ned) / dt
accel_cmd = limit_norm(accel_cmd, max_guidance_accel_mps2)
```

本仓库仍通过 PX4 `TrajectorySetpoint` 接口输出速度/加速度，不替换 PX4 内部姿态/推力控制器。论文中 strapdown 相机的机体 FOV holding 部分在这里由云台视觉伺服承担。

## 关键配置

`src/uav_trajectory_tracking/config/visual_interception.yaml`：

```yaml
pursuit_speed_mps: 3.0
max_pursuit_accel_mps2: 2.0
png_vertical_gain: 3.5
png_horizontal_gain: 3.5
max_guidance_accel_mps2: 2.0
visual_error_timeout_s: 0.2

lock_loss_grace_s: 0.4
coast_velocity_decay_s: 0.5
search_vertical_motion_enabled: true
search_vertical_amplitude_m: 2.0
search_vertical_period_s: 12.0
search_vertical_min_z_ned: -8.0
search_vertical_max_z_ned: -2.0
yaw_mode: fixed_north
```

## 诊断话题

```bash
ros2 topic echo /x500_0/visual_pursuit_interceptor/diagnostics --once
```

常看字段：

- `state`
- `pursuing`
- `velocity_control_active`
- `detection_active`
- `lock_active`
- `visual_error_fresh`
- `image_yaw_error_deg`
- `image_pitch_error_deg`
- `visual_los_ned_*`
- `png_los_vertical_angle_deg`
- `png_los_horizontal_angle_deg`
- `png_desired_vertical_angle_deg`
- `png_desired_horizontal_angle_deg`
- `velocity_setpoint_ned_*`
- `guidance_accel_ned_*`
- `gimbal_los_ned_*`

典型正常追击：

```text
state: pursuit
pursuing: true
velocity_control_active: true
lock_active: true
visual_error_fresh: true
```

短暂掉锁但不回拉：

```text
state: coast_on_lock_loss
pursuing: false
velocity_control_active: true
```

长时间丢失：

```text
state: target_lost
pursuing: false
velocity_control_active: false
hold_x_m/hold_y_m/hold_z_m: 丢失时当前位置
```

## 常见问题

如果一直不进入 `pursuit`：

```bash
ros2 topic echo /x500_0/gimbal_target_tracker/lock_active --once
ros2 topic echo /x500_0/gimbal_target_tracker/error --once
ros2 topic echo /x500_0/gimbal_target_tracker/state --once
ros2 topic echo /x500_0/visual_pursuit_interceptor/diagnostics --once
```

重点确认：

- `/x500_0/gimbal_target_tracker/lock_active` 为 `true`。
- `/x500_0/gimbal_target_tracker/error` 持续新鲜，诊断中的 `visual_error_fresh=true`。
- `GIMBAL_ERROR_TOPIC`、launch 的 `gimbal_error_topic` 和云台节点 `error_topic` 一致。
- `lock_active=false` 时先看云台 state 中的 `lock_centered` 和 `lock_residual_rate_ok`。

如果接近目标后反复后退：

1. 诊断应确认短暂掉锁进入 `coast_on_lock_loss`，而不是直接 `target_lost`。
2. `lock_loss_grace_s` 太短会让 position hold 更早介入。
3. `lock_active` 抖动时先看云台端 unlock 阈值和 residual error rate。

如果视觉 LOS 方向明显错误：

1. 检查 `/x500_0/gimbal/joint_states` 是否来自当前 `x500_0`。
2. 检查 `visual_error_yaw_sign`、`visual_error_pitch_sign` 是否需要反号。
3. 检查 `gimbal_mount_rpy_rad`、各关节轴和 `camera_sensor_rpy_rad` 是否与 SDF 一致。
