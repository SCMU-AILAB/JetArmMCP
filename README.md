# jetarm-mcp

Hiwonder JetArm 机械臂的 MCP 服务器（Rust），运行在 Jetson 上，通过 Streamable HTTP 提供 15 个工具：
状态读取、关节/坐标运动、逆解计算、夹爪、急停、力矩、蜂鸣、拍照、动作录制与回放。设计见 `docs/spec.md`。

## 开发（Mac）

```bash
cargo test            # 协议、运动学、安全规则、急停等单元测试，不需要硬件
python3 tests/sdk_vectors.py   # 用官方 SDK 重新生成协议测试向量（需要先下载 SDK，见下）
```

`tests/sdk_vectors.py` 需要官方的 `ros_robot_controller_sdk.py`。上游仓库未声明许可证，所以本仓库不附带，请自行下载到
`reference/hiwonder_jetarm_ros_orin_nx_ros2/`：

```bash
mkdir -p reference/hiwonder_jetarm_ros_orin_nx_ros2
gh api 'repos/Hiwonder/JetArm-ROS/contents/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py?ref=Jetson_Orin_NX_ros2' \
  --jq .content | base64 -d > reference/hiwonder_jetarm_ros_orin_nx_ros2/ros_robot_controller_sdk.py
```

## 部署（Jetson）

```bash
deploy/deploy.sh --install-service   # 首次：同步、编译、装成 systemd 服务
deploy/deploy.sh                     # 之后：同步、编译、重启服务
```

首次需要 Jetson 上有 `libclang-dev`（`sudo apt install -y libclang-dev`），脚本会自动装 Rust。
配置：复制 `deploy/config.example.toml` 到 Jetson 的 `~/.config/jetarm-mcp/config.toml` 后修改。

## 接入 Claude Code

```bash
claude mcp add --transport http jetarm http://192.168.0.11:8848/mcp
```

⚠️ 服务对局域网开放、无认证。

## 待实机确认

夹爪开合脉冲（`[gripper]`）、`torque_on_cmd`（0x0B/0x0C）、关节 2–4 方向。
