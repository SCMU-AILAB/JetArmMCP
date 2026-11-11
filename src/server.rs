//! MCP 工具定义。业务错误一律以 `CallToolResult::error` 返回，让 AI 能看到原因。

use std::collections::BTreeMap;
use std::sync::Arc;

use base64::Engine;
use rmcp::{
    ErrorData, ServerHandler,
    handler::server::{router::tool::ToolRouter, wrapper::Parameters},
    model::{CallToolResult, ContentBlock, Implementation, ServerCapabilities, ServerConfig},
    schemars, tool, tool_handler, tool_router,
};
use serde::{Deserialize, Serialize};

use crate::actions::{ActionStore, Keyframe};
use crate::arm::{Arm, GripperAction};
use crate::camera;

type ToolResult = Result<CallToolResult, ErrorData>;

fn ok_json<T: Serialize>(v: &T) -> ToolResult {
    let text = serde_json::to_string_pretty(v).unwrap_or_else(|e| format!("序列化失败：{e}"));
    Ok(CallToolResult::success(vec![ContentBlock::text(text)]))
}

fn fail(e: impl std::fmt::Display) -> ToolResult {
    Ok(CallToolResult::error(vec![ContentBlock::text(format!("{e:#}"))]))
}

fn respond<T: Serialize>(r: anyhow::Result<T>) -> ToolResult {
    match r {
        Ok(v) => ok_json(&v),
        Err(e) => fail(format!("{e:#}")),
    }
}

#[derive(Clone)]
pub struct JetArmServer {
    arm: Arc<Arm>,
    actions: Arc<ActionStore>,
    tool_router: ToolRouter<Self>,
}

// ---------- 参数 ----------

#[derive(Debug, Deserialize, schemars::JsonSchema)]
pub struct StateParams {
    /// 是否同时读取每个舵机的电压和温度（慢一些），默认 true
    pub full: Option<bool>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
pub struct MoveJointsParams {
    /// 关节 1 底座旋转（度），不给则不动
    pub j1: Option<f64>,
    /// 关节 2 肩部俯仰（度），0 为竖直，正值向前
    pub j2: Option<f64>,
    /// 关节 3 肘部俯仰（度），正值向前
    pub j3: Option<f64>,
    /// 关节 4 腕部俯仰（度），正值向前
    pub j4: Option<f64>,
    /// 关节 5 腕部旋转（度）
    pub j5: Option<f64>,
    /// 运动时长（秒）；不给则按限速自动计算，给得太短会被拉长
    pub duration: Option<f64>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
pub struct MoveServosParams {
    /// 舵机 ID → 目标脉冲（0–1000，500 为中位），例如 {"1": 500, "10": 300}
    pub targets: BTreeMap<u8, i32>,
    /// 运动时长（秒）；不给则按限速自动计算
    pub duration: Option<f64>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
pub struct GripperParams {
    /// open 张开 / close 闭合
    pub action: Option<GripperAction>,
    /// 直接给夹爪脉冲（0–1000），优先于 action
    pub pulse: Option<i32>,
    /// 运动时长（秒）
    pub duration: Option<f64>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
pub struct DurationParams {
    /// 运动时长（秒）；不给则按限速自动计算
    pub duration: Option<f64>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
pub struct TorqueParams {
    /// 舵机 ID 列表；空或不给表示全部（关节 + 夹爪）
    pub ids: Option<Vec<u8>>,
    /// true 上力（锁住），false 松力（可以用手掰动；臂会因重力下垂，先托住）
    pub enable: bool,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
pub struct BeepParams {
    /// 频率 Hz，默认 1900
    pub freq: Option<u16>,
    /// 时长毫秒，默认 100，最长 2000
    pub ms: Option<u16>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
pub struct TargetParams {
    /// 末端 x（米），底座正前方为正
    pub x: f64,
    /// 末端 y（米），从臂后往前看左侧为正
    pub y: f64,
    /// 末端 z（米），底座底面为 0，向上为正
    pub z: f64,
    /// 末端与竖直方向夹角（度）：0 朝天，90 水平向前，180 垂直朝下；不给则自动选（优先朝下）
    pub pitch: Option<f64>,
    /// 腕部旋转角（度）；不给则腕部旋转不动
    pub roll: Option<f64>,
    /// 运动时长（秒），仅 move_to 使用
    pub duration: Option<f64>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
pub struct CaptureParams {
    /// 输出图片最大宽度（像素），不给则用相机原始分辨率
    pub width: Option<u32>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
pub struct RecordParams {
    /// 动作名（字母、数字、汉字、下划线、连字符）
    pub name: String,
    /// 回放时从上一帧运动到这一帧的时长（秒），默认 1.0
    pub duration: Option<f64>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
pub struct PlayParams {
    pub name: String,
    /// 速度倍率，默认 1.0；2.0 表示快一倍（仍受限速约束）
    pub speed: Option<f64>,
}

#[derive(Debug, Deserialize, schemars::JsonSchema)]
pub struct NameParams {
    pub name: String,
}

// ---------- 工具 ----------

#[tool_router]
impl JetArmServer {
    pub fn new(arm: Arc<Arm>, actions: Arc<ActionStore>) -> Self {
        Self { arm, actions, tool_router: Self::tool_router() }
    }

    #[tool(description = "读取机械臂状态：每个舵机的脉冲、关节角、电压、温度，以及由正解算出的末端坐标（米）和 pitch（度）。不会运动。")]
    async fn get_state(&self, Parameters(p): Parameters<StateParams>) -> ToolResult {
        ok_json(&self.arm.state(p.full.unwrap_or(true)).await)
    }

    #[tool(description = "按关节角运动（度），只给要动的关节。超出限位会报错且不动。返回运动后的实际状态。")]
    async fn move_joints(&self, Parameters(p): Parameters<MoveJointsParams>) -> ToolResult {
        respond(self.arm.move_joints([p.j1, p.j2, p.j3, p.j4, p.j5], p.duration).await)
    }

    #[tool(description = "直接给舵机原始脉冲（0–1000，500 为中位，1 脉冲 = 0.24°），用于调试和标定。关节为舵机 1–5，夹爪为 10。")]
    async fn move_servos(&self, Parameters(p): Parameters<MoveServosParams>) -> ToolResult {
        respond(self.arm.move_pulses(p.targets, p.duration).await)
    }

    #[tool(description = "夹爪张开或闭合，或直接给夹爪脉冲。")]
    async fn gripper(&self, Parameters(p): Parameters<GripperParams>) -> ToolResult {
        respond(self.arm.gripper(p.action, p.pulse, p.duration).await)
    }

    #[tool(description = "回到配置里的初始姿态（折叠静止姿态）。")]
    async fn go_home(&self, Parameters(p): Parameters<DurationParams>) -> ToolResult {
        respond(self.arm.go_home(p.duration).await)
    }

    #[tool(description = "急停：所有舵机立即停在当前位置，并中断正在进行的运动和动作回放。不会排队等待。")]
    async fn stop(&self) -> ToolResult {
        respond(self.arm.stop().await)
    }

    #[tool(description = "舵机上力或松力。松力后可以用手掰动机械臂（拖动示教），但臂会在重力下垂，松力前请先托住。")]
    async fn set_torque(&self, Parameters(p): Parameters<TorqueParams>) -> ToolResult {
        respond(self.arm.set_torque(p.ids.unwrap_or_default(), p.enable).await)
    }

    #[tool(description = "控制板蜂鸣器响一声。")]
    async fn beep(&self, Parameters(p): Parameters<BeepParams>) -> ToolResult {
        let ms = p.ms.unwrap_or(100).min(2000);
        respond(self.arm.beep(p.freq.unwrap_or(1900), ms).await.map(|_| "已蜂鸣"))
    }

    #[tool(description = "末端运动到坐标 (x, y, z)（米，原点在底座底面中心，x 向前，z 向上）。够不到或低于最低高度时报错且不动。返回逆解结果和运动后状态。")]
    async fn move_to(&self, Parameters(p): Parameters<TargetParams>) -> ToolResult {
        #[derive(Serialize)]
        struct Out {
            ik: crate::arm::IkSolution,
            report: crate::arm::MoveReport,
        }
        respond(
            self.arm
                .move_to(p.x, p.y, p.z, p.pitch, p.roll, p.duration)
                .await
                .map(|(ik, report)| Out { ik, report }),
        )
    }

    #[tool(description = "只计算逆运动学，不运动：返回关节角、舵机脉冲和正解验算的末端位姿，用于先规划再执行。")]
    async fn compute_ik(&self, Parameters(p): Parameters<TargetParams>) -> ToolResult {
        respond(self.arm.compute_ik(p.x, p.y, p.z, p.pitch, p.roll))
    }

    #[tool(description = "用机械臂上的相机拍一张照片，以图片返回。")]
    async fn capture_image(&self, Parameters(p): Parameters<CaptureParams>) -> ToolResult {
        let cfg = self.arm.config().camera.clone();
        let r = tokio::task::spawn_blocking(move || camera::capture(&cfg, p.width)).await;
        match r {
            Ok(Ok(j)) => Ok(CallToolResult::success(vec![
                ContentBlock::image(base64::engine::general_purpose::STANDARD.encode(&j.bytes), "image/jpeg"),
                ContentBlock::text(format!("{}×{}", j.width, j.height)),
            ])),
            Ok(Err(e)) => fail(format!("{e:#}")),
            Err(e) => fail(format!("相机任务崩溃：{e}")),
        }
    }

    #[tool(description = "读取当前姿态（关节 + 夹爪），追加为动作的一帧；动作不存在则新建。拖动示教：set_torque 松力 → 手动摆姿势 → 逐帧 record_keyframe → 上力 → play_action。")]
    async fn record_keyframe(&self, Parameters(p): Parameters<RecordParams>) -> ToolResult {
        let duration = p.duration.unwrap_or(1.0);
        if !(duration.is_finite() && duration > 0.0) {
            return fail("duration 必须是正数");
        }
        let pulses = match self.arm.read_pulses(self.arm.config().all_ids()).await {
            Ok(v) => v,
            Err(e) => return fail(format!("{e:#}")),
        };
        #[derive(Serialize)]
        struct Out {
            name: String,
            frames: usize,
            recorded: BTreeMap<u8, i32>,
        }
        let r = self.actions.append(&p.name, Keyframe { duration, pulses: pulses.clone() });
        respond(r.map(|frames| Out { name: p.name, frames, recorded: pulses }))
    }

    #[tool(description = "列出所有已录制的动作及其帧数、总时长。")]
    async fn list_actions(&self) -> ToolResult {
        respond(self.actions.list())
    }

    #[tool(description = "回放动作。可被 stop 中断。每帧仍受限位和限速约束。")]
    async fn play_action(&self, Parameters(p): Parameters<PlayParams>) -> ToolResult {
        let speed = p.speed.unwrap_or(1.0);
        if !(speed.is_finite() && speed > 0.0) {
            return fail("speed 必须是正数");
        }
        let action = match self.actions.load(&p.name) {
            Ok(a) => a,
            Err(e) => return fail(format!("{e:#}")),
        };
        if action.frames.is_empty() {
            return fail("动作没有任何帧");
        }
        let frames = action.frames.into_iter().map(|f| (f.pulses, f.duration / speed)).collect();
        #[derive(Serialize)]
        struct Out {
            played_frames: usize,
            #[serde(skip_serializing_if = "Vec::is_empty")]
            notes: Vec<String>,
            state: crate::arm::ArmState,
        }
        respond(self.arm.play(frames).await.map(|(played_frames, notes, state)| Out { played_frames, notes, state }))
    }

    #[tool(description = "删除一个动作。")]
    async fn delete_action(&self, Parameters(p): Parameters<NameParams>) -> ToolResult {
        respond(self.actions.delete(&p.name).map(|_| format!("已删除 {}", p.name)))
    }
}

#[tool_handler(router = self.tool_router)]
impl ServerHandler for JetArmServer {
    fn get_info(&self) -> ServerConfig {
        ServerConfig::new(ServerCapabilities::builder().enable_tools().build())
            .with_server_info(Implementation::new("jetarm-mcp", env!("CARGO_PKG_VERSION")))
            .with_instructions(
                "Hiwonder JetArm 机械臂控制。坐标单位米，原点在底座底面中心，x 向前、z 向上；角度单位度。\
                 运动前建议先 get_state；不确定能否到达时先 compute_ik。有危险时立刻调用 stop。",
            )
    }
}
