# 固定相机迁移与项目分析

核对日期：2026-09-07。原版本：`ea0a2c8a735649395e4a15cabd31ec6b8c7f0603`。
Git 存档标签：`archive/gimbal-20260907`；迁移分支：`feat/fixed-camera`。

## 官方依据与机型选择

PX4 [Gazebo Vehicles](https://docs.px4.io/main/en/sim_gazebo_gz/vehicles) 列出了固定单目、下视单目及前向深度相机机型。
本项目使用单目图像做 YOLO/BoT-SORT 跟踪，因此选择前向 **x500_mono_cam**。
`x500_mono_cam_down` 面向下视任务；`x500_depth` 是 OAK-D 类前向深度相机，当前算法没有使用其深度数据。

固定连接的证据是官方 [x500_mono_cam/model.sdf](https://github.com/PX4/PX4-gazebo-models/blob/3eb05f716a81bca316ab5771f53e509a07dce3a6/models/x500_mono_cam/model.sdf)：
`CameraJoint type="fixed"`，父 link 为 `base_link`，子 link 为 `camera_link`。
传感器参数见同版本 [mono_cam/model.sdf](https://github.com/PX4/PX4-gazebo-models/blob/3eb05f716a81bca316ab5771f53e509a07dce3a6/models/mono_cam/model.sdf)。
以上固定提交是本机 PX4 模型库版本；也已在线核对官方 main 中的连接和相机参数。

| 项目 | 迁移后 |
| --- | --- |
| PX4 make 目标 | `make px4_sitl gz_x500_mono_cam` |
| 本机官方 airframe | `4010_gz_x500_mono_cam` |
| 仿真主机实例名 | `x500_0`，保持不变 |
| 项目包装模型 | `x500_mono_cam_trajectory_wind`，直接 include 官方模型 |
| 固定安装平移，Gazebo 机体 FLU | `(0.12, 0.03, 0.242)` m |
| 固定安装 roll/pitch/yaw | `(0, 0, 0)` rad，前向 |
| 图像尺寸 / 更新率 | 1280×960 / 30 Hz |
| 水平视场 | 1.74 rad，约 99.69° |
| 相机质量 | 0.050 kg |
| 展开 SDF 后总质量 | 2.1143076923 kg，含机体、四旋翼和相机 |

模型库源于 `/home/zk/PX4-Autopilot/Tools/simulation/gz/models`；本机 PX4 为
`v1.17.0-alpha1-1665-gf101bf4adb-dirty`。本次没有升级或重置 PX4 源码。

## 原项目的实际结构

1. `scripts/start_px4_gazebo.sh` 渲染轨迹世界、加载风场、把生成世界同步到 PX4，并启动主机 SITL。
   原默认目标为 `gz_x500_gimbal`，并把本仓库的 4019 airframe 覆盖到 PX4 ROMFS/build。
2. 原主机模型链为 `x500_gimbal_trajectory_wind → x500_gimbal_self_filtered → x500_self_hidden + gimbal_self_hidden`。
   云台自身有 yaw/roll/pitch 运动关节、伺服插件、关节反馈和自机视觉消隐。
3. 普通轨迹入口 `start_trajectory_tracking.sh → trajectory_tracking.launch.py → trajectory_tracker.py`，
   使用 PX4 状态，按参数化轨迹发送 Offboard setpoint；`trajectory_visualizer` 负责 ENU 可视化。
4. 原视觉入口 `start_visual_interception.sh → visual_interception.launch.py` 启动图像桥接、YOLO/BoT-SORT、
   `gimbal_target_tracker` 和 `visual_pursuit_interceptor`。云台节点兼做目标选择、图像角误差、居中锁定及云台命令。
5. 原控制器使用图像误差、云台关节和 PX4 姿态构造 LOS，再运行当前已有的 PNG 速度角更新。
   **此 Git 版本的视觉控制器不订阅目标真值进行导引**；Gazebo host truth 桥接用于 `trajectory_logger`，
   与更早项目记录中的真值导引实现不同。
6. 目标机由 `start_target_px4_gazebo.sh` 连接 world 中的 `x500_1`，ROS namespace 为 `/px4_1`；
   其轨迹控制和主机视觉控制相互独立。

`SAM6_250217`、独立硬件实验文件和目标机轨迹算法不属于本次相机安装方式迁移的修改范围。

## 修改后的数据流和契约

```text
官方固定单目相机 → Gazebo Image / CameraInfo → ROS 图像桥接
    → yolo_tracker（Detection2DArray）
    → fixed_camera_target_tracker（角误差 / tracking_active / lock_active）
    → visual_pursuit_interceptor + PX4 VehicleAttitude / VehicleLocalPosition
    → PX4 OffboardControlMode / TrajectorySetpoint / VehicleCommand
```

相机 Gazebo 话题仍使用 `/world/trajectory_tracking/model/x500_0/link/camera_link/sensor/camera/` 前缀。
ROS 图像、CameraInfo、YOLO 和主机/目标机 PX4 话题不变。新观测输出前缀为
`/x500_0/fixed_camera_target_tracker/`。

- `error: geometry_msgs/Vector3Stamped`：x 是 `atan((u-cx)/fx)`，y 是 `atan((v-cy)/fy)`，单位 **rad**；
  正方向为图像右/下；z 是置信度。此消息是标量观测容器，不能作为几何向量直接调用 TF 旋转。
- 相机内参只读实际 `CameraInfo.K`，不再使用旧云台 1280×720 的 fallback。
- 原样保留观测 header。相机图像与 CameraInfo 保留官方 `camera_link` frame 名；投影显式处理光学坐标轴，
  不把 Gazebo 相机 link 的 +X 前向误当作 ROS 光学系 +Z 前向。当前项目没有发布完整机器人 TF 树。
- YOLO 沿用原来的时间策略：图像抵达 ROS 时以 ROS 控制时钟记录测量时间；观测节点不再次改时间戳。
  这不是 Gazebo 曝光时间同步。过期、未来、重复/倒序、无有效内参、frame 不匹配的观测不刷新可用状态。
- `tracking_active`：仍在观测有效窗口内。`lock_active`：同一个跟踪 ID 已由不同时间的观测连续确认。
  固定相机没有独立居中执行器，因此不沿用“云台先居中再允许机体运动”的门控。
- 固定模式下控制器不订阅 `JointState` 或云台搜索状态；视觉 launch 不启动云台节点、关节桥或性能监测。
  旧模型及旧节点保留以便历史对照，普通轨迹 launch 的旧云台选项默认关闭。

## 坐标变换与行为边界

按照 [ROS REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst) 的光学/机体系约定，
以及 [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide) 的 FRD/NED 约定：

```text
光学射线：[(u-cx)/fx, (v-cy)/fy, 1]          # 右、下、前
Gazebo 相机 FLU：[1, -(u-cx)/fx, -(v-cy)/fy] # 前、左、上
机体 FLU：R_mount × 射线
PX4 机体 FRD：[x_FLU, -y_FLU, -z_FLU]
PX4 NED：R_vehicle_attitude × normalize(机体 FRD)
```

固定安装姿态来自官方 SDF，默认单位旋转。安装平移改变射线原点，不改变方向；
当前算法没有单目距离估计，也没有借助未知距离补偿近距离视差。

`yaw_mode` 改为 `face_los`。达到初始悬停点后，目标不可见时保持位置并以 20°/s 扫描机体 yaw；
目标可见时朝向视觉 LOS。原有加速度约束、PNG 更新和短时掉锁速度衰减保留。
旧云台的俯仰搜索/机体升降搜索在固定模式下禁用。
固定相机随机体俯仰/横滚运动，目标仍可能移出垂直视场；本次不宣称具备云台稳像或所有轨迹下的持续跟踪能力。

## Git 存档与切换

原工作区无已跟踪文件改动，因此直接对已有提交创建 annotated tag，无需额外的空提交。

```bash
# 查看原版本及当前迁移差异
git show --no-patch archive/gimbal-20260907
git diff archive/gimbal-20260907

# 创建独立旧版本工作区，不影响当前修改
git worktree add ../uav_sim_gimbal archive/gimbal-20260907

# 当前开发分支
git switch feat/fixed-camera
```

`main` 仍指向原提交。标签和分支目前只在本地，未推送到远端。
Git 标签不包含 `.gitignore` 排除的权重、wind.yaml、日志、构建产物、独立硬件实验及外部 PX4 工作树。
之前额外生成的压缩包位于项目外 `../uav_trajectory_tracking_sim_archives/gimbal_20260907_085100/`；
它不是本次 Git 存档机制的一部分。

## 启动与验证

从当前分支执行 `./scripts/build.sh`，然后按 README 的终端顺序启动 Agent、主机 PX4/Gazebo、目标机 PX4、
目标机轨迹和 `./scripts/start_visual_interception.sh`。同一主机不要同时运行普通轨迹控制器与视觉控制器。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=83 python3 -m pytest -q src/uav_trajectory_tracking/test/test_fixed_camera.py
ros2 launch uav_trajectory_tracking visual_interception.launch.py --show-args
```

测试覆盖方向、固定外参、PX4 姿态旋转、内参要求、观测超时/重复、跟踪 ID、固定模式无关节反馈和丢失状态。

本次验证结果：

- 13 项 pytest 测试通过；Python 编译检查、Shell 语法及 `git diff --check` 通过。
- `./scripts/build.sh` 成功构建现有工作区；实际视觉 launch 能创建新观测节点和控制器。
- PX4 SITL 实际启动识别 `SYS_AUTOSTART=4010`，启动脚本正常完成。
- 展开 SDF 后仅有四个旋翼转动关节与固定 `CameraJoint`，没有云台运动关节。
- Gazebo 和 ROS 均实际收到 1280×960 的 Image/CameraInfo，frame 同为 `camera_link`；
  实测 `fx≈539.93633`、`fy≈539.93637`、`cx=640`、`cy=480`，畸变系数全零。
- 隔离 ROS domain/Gazebo partition 的启动检查中，节点为两个相机桥、
  `fixed_camera_target_tracker` 和 `visual_pursuit_interceptor`，没有云台节点；检查后已停止测试进程。
- 本次未做完整双机飞行和 YOLO 目标捕获成功率评估，启动检查关闭了 YOLO 推理和自动飞行数据通路。
  因此不把模型迁移和接口测试通过等同于闭环拦截性能验收。
