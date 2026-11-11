# JetArmMCP

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
![Rust](https://img.shields.io/badge/Rust-2024-orange.svg)
![MCP](https://img.shields.io/badge/MCP-Streamable%20HTTP-blue.svg)

让 AI 直接控制 Hiwonder JetArm 机械臂的 MCP 服务器，用纯 Rust 实现，常驻在 Jetson 上。

Claude 等 MCP 客户端可以通过它读取机械臂状态、按关节角或空间坐标运动、开合夹爪、拍照，还能录制并回放动作。整个程序不依赖 ROS、Python，也不依赖厂商的二进制库：串口协议用 Rust 重新实现，编译出来是单个可执行文件。

## 特性

- **纯 Rust 协议栈**：`AA 55` 帧和 CRC8 用 Rust 实现，发出的字节与官方 SDK 逐字节一致
- **运动学**：正逆解；不指定 pitch 时自动搜索可行的末端朝向
- **安全机制**：
  - 关节限位：超限直接拒绝，臂不动
  - 限速：动作太快会自动放慢
  - 最低高度：末端不能低于桌面高度
  - 运动排队：同一时间只执行一个动作
  - 急停：不用排队，能立刻打断正在进行的运动和动作回放
- **容错**：读舵机带超时重试；USB 断开后每 2 秒自动重连，服务本身不退出
- **相机**：通过 V4L2 拍照，优先用相机自带的 MJPG，不支持时用 YUYV 自行编码，结果以 MCP 图片内容返回
- **拖动示教**：松力后用手摆姿势，逐帧录制，然后回放
- **测试**：用模拟舵机总线做单元测试，不需要硬件

## 架构

```
MCP 客户端 ──HTTP──▶ jetarm-mcp（Jetson）
                      ├─ server      MCP 工具定义
                      ├─ arm         安全规则、运动锁、急停
                      │   ├─ kinematics  正逆运动学
                      │   └─ board       串口收发、超时、重连
                      │       └─ protocol  帧编解码、CRC8
                      ├─ camera      V4L2 → JPEG
                      ├─ actions     动作组（JSON 文件）
                      └─ config      config.toml
                              │
                     /dev/ttyUSB0 (1 Mbps)
                              ▼
                     STM32 控制板 ──▶ 总线舵机 ×6
```

详细设计见 [`docs/spec.md`](docs/spec.md)。

## 工具一览

| 分组 | 工具 | 作用 |
|---|---|---|
| 状态与基础 | `get_state` | 各舵机的脉冲、关节角、电压、温度，以及末端坐标 |
| | `move_joints` | 按关节角运动（度），只给要动的关节 |
| | `move_servos` | 直接给舵机原始脉冲（0–1000），用于调试和标定 |
| | `gripper` | 夹爪张开、闭合，或给指定脉冲 |
| | `go_home` | 回到初始姿态 |
| | `stop` | 急停 |
| | `set_torque` | 上力 / 松力 |
| | `beep` | 蜂鸣器响一声 |
| 坐标运动 | `move_to` | 末端运动到 (x, y, z)，单位米 |
| | `compute_ik` | 只计算逆解，不运动 |
| 相机 | `capture_image` | 拍照 |
| 动作 | `record_keyframe` | 把当前姿态追加为动作的一帧 |
| | `list_actions` | 列出所有动作 |
| | `play_action` | 回放动作 |
| | `delete_action` | 删除动作 |

坐标系：原点在底座底面中心，x 向前，z 向上，单位米；pitch 为末端与竖直方向的夹角，0° 朝天、90° 水平向前、180° 垂直朝下。

## 快速开始

### 1. 硬件

- Hiwonder JetArm（控制板通过 USB 接到 Jetson，出现为 `/dev/ttyUSB0`）
- Jetson（已在 Orin NX 8GB、Ubuntu 22.04 上开发）
- 机械臂上的 USB 相机（`/dev/video0`）

### 2. 部署到 Jetson

```bash
# 在 Jetson 上装 libclang（编译 V4L2 绑定需要）
sudo apt install -y libclang-dev

# 在开发机上：同步源码、在 Jetson 上编译、装成 systemd 服务
JETSON=jetson@<jetson-ip> deploy/deploy.sh --install-service
```

之后更新代码只需运行 `deploy/deploy.sh`（同步、编译、重启服务）。查看日志：`journalctl -u jetarm-mcp -f`。

也可以直接在 Jetson 上手动编译：

```bash
cargo build --release
./target/release/jetarm-mcp [--config config.toml]
```

### 3. 接入 MCP 客户端

```bash
claude mcp add --transport http jetarm http://<jetson-ip>:8848/mcp
```

## 配置

所有参数都有缺省值（即实测值）。需要修改时，把 [`deploy/config.example.toml`](deploy/config.example.toml) 复制到 `~/.config/jetarm-mcp/config.toml`，只写要改的项即可。常用项：

| 项 | 缺省值 | 说明 |
|---|---|---|
| `server.port` | `8848` | 监听端口 |
| `serial.device` | `/dev/ttyUSB0` | 串口设备 |
| `servos.limits` | 全部 `[0, 1000]` | 各舵机允许的脉冲范围 |
| `motion.max_pulse_per_sec` | `250` | 限速，约 60°/s |
| `motion.min_z` | `0.01` | `move_to` 允许的最低末端高度（米） |
| `motion.home` | 实测开机姿态 | `go_home` 的目标 |
| `gripper.open` / `close` | `200` / `500` | 夹爪开合脉冲 |

## 开发

```bash
cargo test     # 单元测试：协议、运动学、安全规则、急停、重连，不需要硬件
cargo clippy
```

`tests/sdk_vectors.py` 用官方 Python SDK 生成协议测试向量，需要先下载 `ros_robot_controller_sdk.py`。上游仓库未声明许可证，所以本仓库不附带：

```bash
mkdir -p reference/hiwonder_jetarm_ros_orin_nx_ros2
gh api 'repos/Hiwonder/JetArm-ROS/contents/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py?ref=Jetson_Orin_NX_ros2' \
  --jq .content | base64 -d > reference/hiwonder_jetarm_ros_orin_nx_ros2/ros_robot_controller_sdk.py
python3 tests/sdk_vectors.py
```

## 安全提示

> [!WARNING]
> 服务对局域网开放、**没有认证**，能访问端口的任何客户端都能控制机械臂。请只在可信网络中运行。

- 第一次运行前清空机械臂周围的障碍物，手放在急停或电源开关附近。
- 松力（`set_torque` 设为 `false`）或断电前先托住机械臂，否则它会因重力下垂。
- 舵机长时间保持上力会发热。

## 项目状态

已完成单元测试，以及在开发机上的端到端 HTTP 测试。以下几项待实机确认：

- [ ] 夹爪张开 / 闭合对应的脉冲值
- [ ] 力矩命令 `0x0B` / `0x0C` 哪个是上力
- [ ] 关节 2–4 的方向
- [ ] 全部工具在 Jetson 上的实机验证

## 致谢

协议细节参考了 Hiwonder 官方的 [JetArm-ROS](https://github.com/Hiwonder/JetArm-ROS)。本项目与 Hiwonder 无官方关联。

## 许可证

[MIT](LICENSE) © 2026 Yao0454
