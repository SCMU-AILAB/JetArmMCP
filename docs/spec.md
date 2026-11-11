# jetarm-mcp 规格

Hiwonder JetArm 机械臂的 MCP 服务器，纯 Rust，常驻 Jetson Orin NX（`jetson@192.168.0.11`）。

## 1. 硬件与协议（实测）

- 串口 `/dev/ttyUSB0`（CH340），1 Mbps。
- 帧：`AA 55 | func | len | data… | crc8`，CRC8（Maxim 表）覆盖 func+len+data。
- 总线舵机 func=5，子命令：`01` 设位置（`01 dur_lo dur_hi n [id pos_lo pos_hi]…`，时长 ms）、`03` 停止（`03 n id…`）、
  `05` 读位置（回包 `<BBbh`）、`07` 读电压 mV（`<BBbH`）、`09` 读温度（`<BBbB`）、`0B/0C` 力矩（方向待实测）、`0D` 读力矩状态。
- 蜂鸣器 func=2：`<HHHH` freq, on_ms, off_ms, repeat。
- 舵机：关节 1–5（1 为底座、5 为腕旋），夹爪 10。脉冲 0–1000 = 0–240°，500 为零位。
- 相机 `/dev/video0`，640×480。

## 2. 架构

单个可执行文件 `jetarm-mcp`，rmcp Streamable HTTP，监听 `0.0.0.0:8848/mcp`。

| 模块 | 职责 |
|---|---|
| protocol | 帧编码/解析、CRC8，纯函数 |
| board | 串口收发；后台读线程分拣回包；读请求带超时；`Transport` trait 可换模拟串口；断线自动重连 |
| kinematics | 正逆运动学（移植自 `~/jetarm/jetarm.py`），参数来自配置 |
| arm | 安全规则（限位、限速、工作空间）、角度↔脉冲、运动锁、急停 |
| camera | 按需打开 V4L2，丢弃前几帧，返回 JPEG；仅 Linux |
| actions | 动作组 JSON，存于 `~/.local/share/jetarm-mcp/actions/` |
| config | `config.toml`，缺省值即实测参数 |
| server | MCP 工具定义 |

并发：运动锁（同一时间一个动作）+ 串口锁（只在收发期间持有）。`stop` 只拿串口锁，并通过取消信号打断正在等待的运动和回放。

## 3. 工具（15 个）

状态与基础：`get_state`、`move_joints`、`move_servos`、`gripper`、`go_home`、`stop`、`set_torque`、`beep`
坐标：`move_to`、`compute_ik`
相机：`capture_image`
动作：`record_keyframe`、`list_actions`、`play_action`、`delete_action`

运动类工具返回运动后的实际状态。

## 4. 安全

- 每个舵机有脉冲限位（默认 0–1000；初始姿态里 3 号舵机≈0，所以默认不收紧）。超限直接报错不动，不截断。
- 限速 250 脉冲/秒，最短 0.5 s；客户端给的时长过短时拉长并在结果里注明。
- `move_to` 拒绝 z < `min_z`（默认 0.01 m）。
- 启动只读状态不运动；`go_home` 须显式调用。初始姿态默认 `{1:512, 2:761, 3:0, 4:320, 5:522}`。
- 无认证（用户选择），启动日志明确提示。

## 5. 错误处理

- 读舵机：每次超时 0.5 s，重试 3 次后报"舵机 X 无应答"。
- 串口断开：工具报错，后台每 2 s 重连。
- 相机失败只影响 `capture_image`。
- 错误信息用中文，写明原因和建议。

## 6. 测试

- Mac 单元测试：协议字节与官方 Python SDK 对照；FK/IK 往返并与 `jetarm.py` 对照；模拟串口测限位、限速、急停打断、读超时。
- Jetson 实机：读状态 → 单舵机小动 → 试夹爪开合值 → 验证 0x0B/0x0C → move_to → 拍照 → 录制回放。
- 从 Mac 用 `claude mcp add --transport http jetarm http://192.168.0.11:8848/mcp` 接入并逐个调用。

## 7. 待实机确认

夹爪开/合脉冲；`0x0B`/`0x0C` 哪个是松力；关节 2–4 方向。
