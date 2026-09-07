# 固定相机迁移与项目分析

核对日期：2026-09-07。原版本：`ea0a2c8a735649395e4a15cabd31ec6b8c7f0603`。
Git 存档标签：`archive/gimbal-20260907`，已上传 GitHub；当前固定相机版本位于 `main`。

## 官方依据与机型选择

PX4 [Gazebo Vehicles](https://docs.px4.io/main/en/sim_gazebo_gz/vehicles) 列出了固定单目、下视单目及前向深度相机机型。
本项目使用单目图像做 YOLO/BoT-SORT 跟踪，因此选择前向 **x500_mono_cam**。
`x500_mono_cam_down` 面向下视任务；`x500_depth` 是 OAK-D 类前向深度相机，当前算法没有使用其深度数据。

固定连接的证据是官方 [x500_mono_cam/model.sdf](https://github.com/PX4/PX4-gazebo-models/blob/3eb05f716a81bca316ab5771f53e509a07dce3a6/models/x500_mono_cam/model.sdf)：
`CameraJoint type="fixed"`，父 link 为 `base_link`，子 link 为 `camera_link`。
传感器参数见同版本 [mono_cam/model.sdf](https://github.com/PX4/PX4-gazebo-models/blob/3eb05f716a81bca316ab5771f53e509a07dce3a6/models/mono_cam/model.sdf)。

项目通过包装模型的 `include/experimental:params` 仅覆盖传感器 image width/height，
采用 [SDFormat 参数覆盖机制](https://sdformat.org/tutorials/specification/param_passing_tutorial/)。
当前输出为 1920×1080，水平视场角仍为 1.74 rad、帧率仍为 30 Hz；CameraInfo 自动匹配新尺寸。
以下历史验证中的 1280×960 为修改分辨率前的记录。
以上固定提交是本机 PX4 模型库版本；也已在线核对官方 main 中的连接和相机参数。

| 项目 | 迁移后 |
| --- | --- |
| PX4 make 目标 | `make px4_sitl gz_x500_mono_cam` |
| 本机官方 airframe | `4010_gz_x500_mono_cam` |
| 仿真主机实例名 | `x500_0`，保持不变 |
| 项目包装模型 | `x500_mono_cam_trajectory_wind`，直接 include 官方模型 |
| 固定安装平移，相对 `base_link` 的 FLU | `(0.12, 0.03, 0.002)` m |
| 固定安装 roll/pitch/yaw | `(0, 0, 0)` rad，前向 |
| 图像尺寸 / 更新率 | 1920×1080 / 30 Hz（项目覆盖尺寸） |
| 水平视场 | 1.74 rad，约 99.69° |
| 相机质量 | 0.050 kg |
| 展开 SDF 后总质量 | 2.1143076923 kg，含机体、四旋翼和相机 |

相机相对模型原点的平移为 `(0.12, 0.03, 0.242)` m；展开 include 后 `base_link` 位于模型原点上方 0.24 m，
因此上表使用两者的相对平移。`CameraJoint` 自身的 pose 不等同于子 link 相对父 link 的 pose。

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
    → fixed_camera_target_tracker（单位 bearing / tracking_active / lock_active）
    → visual_pursuit_interceptor + PX4 VehicleAttitude / VehicleLocalPosition
    → PX4 OffboardControlMode / TrajectorySetpoint / VehicleCommand
```

相机 Gazebo 话题仍使用 `/world/trajectory_tracking/model/x500_0/link/camera_link/sensor/camera/` 前缀。
ROS 图像、CameraInfo、YOLO 和主机/目标机 PX4 话题不变。新观测输出前缀为
`/x500_0/fixed_camera_target_tracker/`。

- `bearing: geometry_msgs/Vector3Stamped`：光学系单位视线向量，x 右、y 下、z 前；无量纲。
  置信度保留在 YOLO Detection2DArray 中，只用于观测筛选，不再占用向量的 z 分量。
- 相机内参只读实际 `CameraInfo.K`。当前支持无畸变、全幅、无 binning 的相机；不支持的标定明确拒绝。
- 两个主机入口共用 `camera.launch.py`：一个官方 `ros_gz_bridge/parameter_bridge` 直接发布
  Image/CameraInfo，通过 `override_frame_id` 设置 `x500_0/camera_optical_frame`，保留采集时间和像素。
  两条桥接均为 GZ_TO_ROS、RELIABLE/VOLATILE、队列深度 1，没有中间图像转发节点。
- 官方 `tf2_ros/static_transform_publisher` 发布静态 TF：
  `x500_0/base_link → x500_0/camera_link → x500_0/camera_optical_frame`。
  安装外参读取现有 `visual_interception.yaml`；视觉入口使用传入的同一份控制器配置。
- YOLO 直接传递图像的 Gazebo 采集时间。观测节点使用 `use_sim_time=true`，由独立的 `start_simulation_clock.sh` 桥接 `/clock`。
- 双机 `UXRCE_DDS_SYNCT=0`，所有 ROS 处理节点使用 Gazebo `/clock`；控制器直接按采集仿真时间，
  对包围曝光时刻的 PX4 姿态做 SLERP，不外推；等待右侧姿态时保留有界队列。
  先计算 NED 视线，再对 NED 方位角/俯角进行 DKF 预测，方位角跨 ±pi 时展开。
- 只有通过时间、姿态与几何检查的观测才能刷新导引有效期；DKF 预测超过上限直接失效。
  缺少时钟、过期、超出时钟回调容差的未来帧、重复/倒序、无有效内参或 frame 不匹配均不刷新可用状态。
- `tracking_active`：仍在观测有效窗口内。`lock_active`：单目标已由不同时间的有效观测连续确认；track ID 变化不重置确认，也不阻断 bearing。
  固定相机没有独立居中执行器，因此不沿用“云台先居中再允许机体运动”的门控。
- 控制器只支持固定相机，不订阅 `JointState` 或云台搜索状态。
  旧云台节点、配置、模型、自机消隐模型、4019 airframe 和专用历史文档已删除；
  普通轨迹 launch 的云台选项、节点注册，以及控制器中的云台运动学和升降搜索兼容代码也已移除。
  历史实现请查看存档标签。

## 坐标变换与行为边界

按照 [ROS REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst) 的光学/机体系约定，
以及 [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide) 的 FRD/NED 约定：

```text
光学射线：[(u-cx)/fx, (v-cy)/fy, 1]          # 右、下、前
Gazebo 相机 FLU：[1, -(u-cx)/fx, -(v-cy)/fy] # 前、左、上
机体 FLU：R_mount × 射线
PX4 机体 FRD：[x_FLU, -y_FLU, -z_FLU]
PX4 NED：R_odometry_q(曝光时刻) × normalize(机体 FRD)
```

固定安装姿态来自官方 SDF，默认单位旋转。安装平移改变射线原点，不改变方向；
当前算法没有单目距离估计，也没有借助未知距离补偿近距离视差。

`yaw_mode` 改为 `face_los`。达到初始悬停点后，目标不可见时保持 XY，以 20°/s 扫描机体 yaw，
并绕搜索开始时的实际高度做正弦升降：幅度 2 m、周期 12 s，Z 设定值限于本地 NED 的 −8～−2 m。
收到新鲜观测且当前里程计样本的预测投影仍在视场内后停止升降并朝向视觉 LOS；等待锁定时保持当前高度，锁定后恢复追击。
起飞、前往初始悬停点和 COAST 阶段不升降搜索；`search_vertical_amplitude_m: 0.0` 可关闭升降。
恢复的高度搜索复用位置设定值和现有观测状态，不恢复云台俯仰、搜索状态话题或额外超时。
PNG 从 LOS 初始化并积分 LOS 角增量，形成连续速度方向参考；追踪输出为 NED 速度和 LOS 偏航角，由 PX4 内环计算姿态和推力。短时掉锁交给 PX4 速度控制减速，详见 [视觉制导说明](visual_guidance.md)。
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
git switch main
```

远端 `main` 为固定相机版本，原云台提交由远端标签保存；旧分支 `feature/gimbal-target-centering` 已删除。
Git 标签不包含 `.gitignore` 排除的权重、wind.yaml、日志、构建产物、独立硬件实验及外部 PX4 工作树。
之前额外生成的压缩包位于项目外 `../uav_trajectory_tracking_sim_archives/gimbal_20260907_085100/`；
它不是本次 Git 存档机制的一部分。

## 启动与验证

从当前分支执行 `./scripts/build.sh`，然后按 README 的终端顺序启动 Agent、主机 PX4/Gazebo、公共仿真时钟、目标机 PX4、
目标机轨迹和 `./scripts/start_visual_interception.sh`。同一主机不要同时运行普通轨迹控制器与视觉控制器。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=83 python3 -m pytest -q src/uav_trajectory_tracking/test/test_fixed_camera.py
ros2 launch uav_trajectory_tracking visual_interception.launch.py --show-args
```

测试覆盖方向、固定外参、PX4 姿态旋转、内参要求、观测超时/重复、单目标 ID 切换连续性、固定模式无关节反馈和丢失状态。

首次机型迁移的历史验证结果（早于后续时间/接口修正）：

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

## 旧云台文件清理验证

2026-09-07 清理旧云台实现后，删除本包的 `build/uav_trajectory_tracking` 和
`install/uav_trajectory_tracking` 并重新运行 `./scripts/build.sh`，构建成功。
已有工作区更新到此版本时也应清理这两个生成目录后重建，避免 colcon 保留旧节点入口或配置。

- 14 项 pytest 测试通过，覆盖固定相机投影、观测确认、追击、掉锁减速、位置保持和偏航搜索；
  同时验证控制器不再声明旧关节/搜索参数，诊断中不再出现旧云台字段。
- 普通轨迹和视觉拦截 launch 的 `--show-args` 均通过，三个启动脚本传入的参数均有对应声明。
- `ros2 pkg executables uav_trajectory_tracking` 只列出当前七个节点，无旧云台节点。
- 剩余 SDF 的模型引用、YAML、Python/Shell 语法及 `git diff --check` 检查通过。
- 本次清理未重新进行完整双机飞行或拦截性能验证。

## 后续规范修正

当前接口为 `bearing_topic` 和 `/x500_0/fixed_camera_target_tracker/bearing`，旧 `error` 话题不再提供。
控制器配置删除 `camera_mount`、图像正负号开关及无时间戳固定延迟回退。
`camera_mount_translation_flu_m` 仅定义静态 TF 外参，不表示已经得到目标距离或完成光心到机体原点的视差补偿。
PNG 使用 LOS 角增量；重复的诊断 LOS-rate 滤波和 `los_rate_filter_alpha` 参数已删除。

PX4 姿态、位置、速度、状态必须有效且新鲜；失效时停止发布 Offboard 心跳/设定值/自动解锁请求，
由 PX4 已配置的 Offboard-loss 策略接管，恢复后重新积累预发送周期。仅视觉丢失而 PX4 状态正常时，
仍按 COAST → 悬停搜索处理。姿态/本地估计重置会清空观测历史。

所有输入共用仿真时钟，省去 DDS 偏移转换；采样配对与时效检查仍保留。本实现面向当前 PX4 Gazebo SITL。
无目标距离时，NED bearing 仍来自相机光心，近距离视差是明确保留的单目算法边界。

统一仿真时钟改动前的验证（2026-09-07）：52 项 pytest 全部通过，包含真实 ROS DDS 消息链路、
相机采集 header 保留、光学静态 TF、仿真时钟、曝光姿态插值、延迟拒绝、状态超时、
非零 DDS 偏移、估计重置和跨 ±pi 处理。工作区构建、两种 launch 参数解析、
Shell 语法与 `git diff --check` 通过。隔离 ROS domain/Gazebo partition 的视觉 launch
已验证启动及干净退出；关闭 YOLO 推理、日志和真值桥接，未进行完整双机飞行性能验收。

## 统一仿真时钟验证（2026-09-07）

双机启动参数实际为 `UXRCE_DDS_SYNCT=0`；主机控制器、目标轨迹节点和双机 logger
实际 `use_sim_time=true`，`/clock` 只有一个发布者。56 项 pytest 通过，工作区构建及
两种 launch 参数解析通过。双机实际起飞，主机进入过 PURSUIT，两套 logger 均完成配对。
暂停时 logger 通过单调墙钟使旧配对失效，恢复后重新配对；原点和参考点算法保持不变。

构建、探针、暂停恢复和测试证据位于 `log/bringup/20260907_135648/`。
本次修复验证时钟与配对契约，视觉仍有掉锁，未完成持续拦截性能验收。

## 官方相机桥接替换验证（2026-09-07）

共享 `camera.launch.py` 替代自定义图像转发节点；两个主机入口保持原公共话题不变。
54 项 pytest 通过，其中隔离集成测试实际运行官方桥接及静态 TF 发布器，验证
1280×960 RGB 像素不变、采集时间不变、光学 frame 一致、RELIABLE 发布、队列配置为 1，
以及下游曝光姿态配对。工作区构建、两个主机 launch 参数解析和 Shell 语法检查通过。
此次使用合成 Gazebo 消息验证接口，没有进行新的飞行性能测试。

桥接依据：[ros_gz 1.0.22 官方相机 frame 配置](https://github.com/gazebosim/ros_gz/blob/1.0.22/ros_gz_bridge/README.md#gz-to-ros-frame_id-override)。
