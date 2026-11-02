# 果树拍摄仿真小车

可移动差速底盘，带升降、水平转动和俯仰的 RGB 相机，配套现有荔枝树场景。尺寸和质量是功能原型设置，尚未标定到真实车辆。

## 启动

运行以下命令打开场景和自带的控制面板：

```bash
bash sim/orchard-rover/start.sh
```

启动后右侧会加载 **Rover Control（Teleop）** 面板，速度话题已设为 `/orchard/cmd_vel`。

- **Buttons**：点击面板的前后左右按钮，点击 Stop 停车。
- **Keyboard**：切换到该标签，焦点放在 Gazebo 窗口，按 W/S/A/D 前后左右控制，松开按键停止。
- 最大 Forward 建议设为 0.25 m/s，最大 Yaw 建议设为 0.60 rad/s；面板可以直接修改。底盘插件另有限速约束。
- 自带插件的 Q/E 是竖直方向键，对本地面底盘不承担升降功能，不能按终端脚本的 Q 退出逻辑理解。

不需要另开控制终端。下面脚本保留为可选的终端控制入口：

```bash
bash sim/orchard-rover/control.sh
```

在第二个终端按键控制：W 前进、S 后退、A 左转、D 右转、空格停止、Q 退出。持续运动需按住按键或重复按键；停止收到按键约 0.4 秒后发送零速度。退出时也会发送停车指令。

查看实时相机画面，在另一个终端运行：

```bash
bash sim/orchard-rover/view_camera.sh
```

在 Image Display 窗口选择 `/orchard/camera/image`。也可以在 Gazebo 插件菜单中打开 Image Display。辅助俯视画面话题为 `/orchard/overview`。

## 调整相机

更新后需关闭旧场景，再运行上面的 `start.sh`。界面会增加 **Camera Lift and Gimbal** 面板，自动选中并锁定 `camera_rig`。展开关节条目，修改目标位置并结束编辑；仿真处于运行状态时，机构会移动到目标。它与底盘的 Rover Control 面板分别控制。

| 面板关节 | 输入单位与范围 | 使用示例 |
|---|---|---|
| `lift_joint` | 米，0–1.1；名义安装高度 = 0.7 + 输入值 | 输入 0.8，相机升到约 1.5 米 |
| `pan_joint` | 弧度，约 −2.967–2.967；正数向车体左侧转 | 输入 0.524，向左转约 30° |
| `tilt_joint` | 弧度，约 −1.047–1.047；负数向上仰 | 输入 −0.349，向上仰约 20° |

面板的 Reset 会将关节目标设为零，即最低高度、朝正前方、水平姿态。默认拍摄姿态是 `lift_joint=0.3`、`pan_joint=0`、`tilt_joint=-0.139626`，与 Reset 不同。Teleop 的 Vertical 不用于控制升降机构。

也可以用米和角度直接设置目标，例如升至 1.5 米、向左转 30°、向上仰 20°：

```bash
bash sim/orchard-rover/camera.sh --height 1.5 --pan 30 --tilt 20
```

只提供一个参数就只修改对应轴。回到默认的 1 米、正前方、向上仰 8°：

```bash
bash sim/orchard-rover/camera.sh --home
```

脚本发布目标后退出，仿真控制器继续执行和保持目标；输出“已发送”不代表机构已经到位。

## 三点自动拍摄

先按前面的步骤运行场景，再在另一个终端运行：

```bash
bash sim/orchard-rover/run_mission.sh
```

小车会依次到达树东侧的三个拍摄点，车头朝树，分别将相机调整到约 1.0、1.3、1.6 米高，停车稳定后保存照片。每次运行自动新建文件夹，终端会显示保存位置：

```text
sim/orchard-rover/outputs/missions/<本次运行文件夹>/
  01_east_low.png
  02_northeast_mid.png
  03_southeast_high.png
  mission.json          拍摄结果、实际位姿、角度、时间与误差
  trajectory.json       实际行驶轨迹
  plan.json             本次任务的目标点
  camera_config.json    本次相机配置
```

任务期间由脚本控制车辆和相机，请勿同时使用手动控制。按 **Ctrl+C** 发送停车指令并结束任务，已有照片会保留。反馈中断、仿真时间停滞或执行超时也会停车并记录失败；场景应保持运行状态。

拍摄点在 `missions/three_points.json`。坐标单位为米，位于 Gazebo 的 world 坐标系，树干中心为 (0,0)；车头偏航角 `yaw_deg` 从世界 +X 轴逆时针计算，180° 朝 −X。`height_m` 为名义安装高度，`pan_deg` 向左为正，`tilt_deg` 向上为正。修改后直接重新运行任务脚本，不需要重建模型。

```bash
bash sim/orchard-rover/run_mission.sh --mission /绝对路径/你的任务.json
```

当前使用仿真真实位姿反馈控制到点，在已知空地中转向、直线逼近目标。示例会检查点间直线不穿入树干中心半径 2.5 米的演示禁入区；如果车辆当前位置到首点的直线不满足条件，任务会拒绝执行。该圆形约束不是完整树冠或其他障碍物碰撞检测。A*、RPP、SLAM 和自动视点优化尚未接入。

验收条件：位置误差 ≤0.06 米、车头角度误差 ≤3°，相机高度误差 ≤0.01 米、角度误差 ≤1°，低速并满足上述条件至少 1 秒仿真时间后取新帧。实际位姿、关节状态和图像的时间戳均写入结果，取图时图像与两种状态的时间差不超过 0.2 秒。

独立运行完整任务检查（会自行启动和关闭测试场景，不操作手动场景）：

```bash
python3 sim/orchard-rover/tools/validate_mission.py
```

## 初始设置

| 项目 | 配置 |
|---|---|
| 底盘 | 两驱差速，前后低摩擦支撑球 |
| 车体盒子 | 长 0.55 米，宽 0.40 米，高 0.16 米 |
| 驱动轮 | 半径 0.12 米，中心轮距 0.46 米 |
| 相机初始姿态 | 名义安装高度 1.0 米，向上仰 8 度 |
| 升降范围 | 名义安装高度 0.7–1.8 米，关节限速 0.25 m/s |
| 云台范围 | 水平 ±170°、俯仰 ±60°；关节限速分别为 0.8、0.6 rad/s |
| RGB 相机 | 640×480，水平视场 60 度，10 Hz 仿真更新率 |
| 初始位置 | 树干东侧 4 米，车头朝向树 |
| 手动速度 | 前进 0.25 m/s，后退 0.20 m/s，转向 0.60 rad/s |

相机的默认姿态、运动范围、分辨率和视场参数在 `config.json`。修改配置后运行：

```bash
python3 sim/orchard-rover/tools/build_model.py
```

重新启动场景使配置修改生效。运行时调整姿态直接使用上面的面板或脚本。安装高度指云台旋转中心在水平地面上的名义高度；镜头位于旋转中心前方 4.5 cm，俯仰、车体倾斜和轮地接触会影响实际光心高度。相机从最低位启动，随后自动移动到默认姿态。

## 接口与验证

- Gazebo 速度指令：`/orchard/cmd_vel`，`gz.msgs.Twist`。
- 轮式里程计：`/orchard/odom`，`gz.msgs.Odometry`；以出生位置为里程计原点。
- RGB 图像：`/orchard/camera/image`，`gz.msgs.Image`。
- 相机关节状态：`/orchard/camera/joint_state`，`gz.msgs.Model`。
- 相机目标：`/model/camera_rig/joint/{lift_joint,pan_joint,tilt_joint}/0/cmd_pos`，`gz.msgs.Double`，单位分别为米、弧度、弧度。
- 当前使用 Gazebo 接口；已有指定点自动到达与拍摄任务，尚未接入 ROS2 导航、A*、RPP 或自动视点规划。
- `verification.json` 记录物理位姿与里程计独立测量。功能检查使用仿真时间，不能将脚本运行的墙钟时间当作车辆执行速度基准。
- `outputs/initial_overview.png` 是小车与树的实际场景画面，`initial_rover.png` 是实际车载相机画面；还保存了前进和转向后的图片。
- `camera_verification.json` 记录升高、水平转动、俯仰和回位的关节状态与图像变化，截图为 `outputs/camera_*.png`。
- `gui/camera_verification.json` 记录原生面板加载与选中相机机构的无显示检查；未进行真实鼠标滑块操作检查。

重新运行检查：

```bash
python3 sim/orchard-rover/tools/validate.py
python3 sim/orchard-rover/tools/validate_camera.py
python3 sim/orchard-rover/tools/validate_camera_gui.py
```

该检查在独立 Gazebo partition 中启动并关闭测试实例，不控制正在手动使用的场景。

## 模型与依赖

小车由基本几何体及关节构成，在 `tools/build_model.py` 生成，使用 Gazebo 自带的 DiffDrive 系统。树模型从相邻的 `$LYCHEE_TREE_MODELS_PATH` 引用，没有另行复制素材。

相机使用 Gazebo 自带 JointPositionController 进行 PID 力/力矩控制，升降含按原型质量设置的重力补偿。机构的速度、质量和控制增益用于仿真原型，尚未模拟或标定具体电机、传动、间隙和编码器。该嵌套机构固定名为 `camera_rig`，当前话题及面板配置面向单车场景；扩展多车前需分别命名和设置独立话题。

树的尺度、替代果实材质和简化树干碰撞限制见相邻树模型的 README；当前场景不能用于证明完整树冠碰撞安全。车辆惯量、轮地接触及相机参数仍需按实机校准。

依赖本机已安装的 Gazebo Harmonic、Python Gazebo Transport/Messages 绑定及 Pillow；未安装新系统包。

参考：[Gazebo Harmonic 差速车控制说明](https://gazebosim.org/docs/harmonic/moving_robot/)、[关节位置控制器](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointPositionController.html)。
