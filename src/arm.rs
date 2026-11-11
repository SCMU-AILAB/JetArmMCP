//! 机械臂高层逻辑。所有安全规则集中在这里：限位、限速、工作空间、运动锁、急停。

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Context, anyhow, bail, ensure};
use serde::{Deserialize, Serialize};
use tokio::sync::{Mutex, watch};

use crate::board::{self, Board};
use crate::config::Config;
use crate::kinematics::Pose;
use crate::protocol;

#[derive(Debug, Clone, Serialize)]
pub struct ServoState {
    pub id: u8,
    pub pulse: Option<i32>,
    /// 关节角（度）；夹爪没有
    pub angle_deg: Option<f64>,
    pub voltage_v: Option<f64>,
    pub temp_c: Option<u8>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ArmState {
    pub serial_connected: bool,
    pub servos: Vec<ServoState>,
    /// 关节 1–5 的角度（度），任一读失败则为空
    pub joint_angles_deg: Option<[f64; 5]>,
    /// 末端位姿（米 / 度），由正解算出
    pub end_effector: Option<Pose>,
}

#[derive(Debug, Clone, Serialize)]
pub struct MoveReport {
    pub duration_s: f64,
    pub targets: BTreeMap<u8, i32>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub notes: Vec<String>,
    pub state: ArmState,
}

#[derive(Debug, Clone, Serialize)]
pub struct IkSolution {
    /// 关节 1–4 的角度（度）
    pub joint_angles_deg: [f64; 4],
    /// 舵机 ID → 脉冲
    pub pulses: BTreeMap<u8, i32>,
    /// 用正解验算的末端位姿
    pub check: Pose,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, schemars::JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum GripperAction {
    Open,
    Close,
}

/// 急停打断正在等待的运动时返回的错误
#[derive(Debug, thiserror::Error)]
#[error("运动被急停（stop）中断")]
pub struct Cancelled;

pub struct Arm {
    board: Arc<Board>,
    cfg: Arc<Config>,
    /// 同一时间只执行一个动作
    motion: Mutex<()>,
    /// 每次急停加一，正在等待的运动据此提前结束
    stop_epoch: watch::Sender<u64>,
}

impl Arm {
    pub fn new(board: Arc<Board>, cfg: Arc<Config>) -> Self {
        Self { board, cfg, motion: Mutex::new(()), stop_epoch: watch::channel(0).0 }
    }

    pub fn config(&self) -> &Config {
        &self.cfg
    }

    async fn io<T, F>(&self, f: F) -> anyhow::Result<T>
    where
        T: Send + 'static,
        F: FnOnce(&Board) -> board::Result<T> + Send + 'static,
    {
        let b = self.board.clone();
        Ok(tokio::task::spawn_blocking(move || f(&b)).await.context("串口任务崩溃")??)
    }

    // ---------- 读状态 ----------

    pub async fn read_pulses(&self, ids: Vec<u8>) -> anyhow::Result<BTreeMap<u8, i32>> {
        self.io(move |b| ids.iter().map(|&id| Ok((id, b.read_position(id)?))).collect()).await
    }

    /// `full = true` 时额外读电压和温度
    pub async fn state(&self, full: bool) -> ArmState {
        let ids = self.cfg.all_ids();
        let connected = self.board.is_connected();
        let raw = self
            .io(move |b| {
                Ok(ids
                    .iter()
                    .map(|&id| {
                        let p = b.read_position(id);
                        let (v, t) = if full && p.is_ok() { (b.read_vin(id).ok(), b.read_temp(id).ok()) } else { (None, None) };
                        (id, p, v, t)
                    })
                    .collect::<Vec<_>>())
            })
            .await
            .unwrap_or_default();
        let joints = &self.cfg.servos.joints;
        let servos: Vec<ServoState> = raw
            .into_iter()
            .map(|(id, p, v, t)| {
                let joint = joints.iter().position(|&j| j == id);
                ServoState {
                    id,
                    pulse: p.as_ref().ok().copied(),
                    angle_deg: match (joint, &p) {
                        (Some(j), Ok(p)) => Some(round2(self.cfg.pulse_to_angle(j, *p))),
                        _ => None,
                    },
                    voltage_v: v.map(|mv| mv as f64 / 1000.0),
                    temp_c: t,
                    error: p.err().map(|e| e.to_string()),
                }
            })
            .collect();
        let angles: Option<Vec<f64>> =
            joints.iter().map(|id| servos.iter().find(|s| s.id == *id).and_then(|s| s.angle_deg)).collect();
        let joint_angles_deg = angles.map(|a| [a[0], a[1], a[2], a[3], a[4]]);
        let end_effector = joint_angles_deg.map(|a| round_pose(self.cfg.geometry.fk([a[0], a[1], a[2], a[3]])));
        ArmState { serial_connected: connected, servos, joint_angles_deg, end_effector }
    }

    // ---------- 运动 ----------

    /// 直接给脉冲。所有运动最终都走这里。
    pub async fn move_pulses(&self, targets: BTreeMap<u8, i32>, duration: Option<f64>) -> anyhow::Result<MoveReport> {
        let _g = self.motion.lock().await;
        let mut rx = self.stop_epoch.subscribe();
        rx.borrow_and_update();
        self.move_locked(targets, duration, &mut rx).await
    }

    async fn move_locked(
        &self,
        targets: BTreeMap<u8, i32>,
        duration: Option<f64>,
        stop: &mut watch::Receiver<u64>,
    ) -> anyhow::Result<MoveReport> {
        ensure!(!targets.is_empty(), "没有给任何目标");
        let known = self.cfg.all_ids();
        for (&id, &p) in &targets {
            ensure!(known.contains(&id), "舵机 {id} 不在配置里（已知：{known:?}）");
            let [lo, hi] = self.cfg.limit(id);
            ensure!((lo..=hi).contains(&p), "舵机 {id} 的目标脉冲 {p} 超出限位 [{lo}, {hi}]，未运动");
        }
        if let Some(d) = duration {
            ensure!(d.is_finite() && d > 0.0, "时长必须是正数");
        }
        let m = &self.cfg.motion;
        let now = self.read_pulses(targets.keys().copied().collect()).await?;
        let max_delta = targets.iter().map(|(id, p)| (p - now[id]).abs()).max().unwrap_or(0);
        let needed = (max_delta as f64 / m.max_pulse_per_sec).max(m.min_duration);
        let mut notes = Vec::new();
        let dur = match duration {
            Some(d) if d < needed => {
                notes.push(format!("要求 {d:.2} s 超过限速，已拉长到 {needed:.2} s"));
                needed
            }
            Some(d) => d,
            None => needed,
        };
        ensure!(dur <= m.max_duration, "运动时长 {dur:.1} s 超过上限 {} s", m.max_duration);
        let cmd: Vec<(u8, u16)> = targets.iter().map(|(&id, &p)| (id, p as u16)).collect();
        let ms = (dur * 1000.0).round() as u16;
        self.io(move |b| b.set_position(ms, &cmd)).await?;
        tokio::select! {
            _ = tokio::time::sleep(Duration::from_secs_f64(dur + 0.1)) => {}
            _ = stop.changed() => return Err(Cancelled.into()),
        }
        Ok(MoveReport { duration_s: round2(dur), targets, notes, state: self.state(false).await })
    }

    /// 按关节角运动，`None` 的关节不动。
    pub async fn move_joints(&self, angles: [Option<f64>; 5], duration: Option<f64>) -> anyhow::Result<MoveReport> {
        let targets: BTreeMap<u8, i32> = angles
            .iter()
            .enumerate()
            .filter_map(|(j, a)| a.map(|a| (self.cfg.servos.joints[j], self.cfg.angle_to_pulse(j, a))))
            .collect();
        ensure!(!targets.is_empty(), "至少给一个关节角");
        self.move_pulses(targets, duration).await
    }

    pub fn compute_ik(&self, x: f64, y: f64, z: f64, pitch: Option<f64>, roll: Option<f64>) -> anyhow::Result<IkSolution> {
        ensure!([x, y, z].iter().all(|v| v.is_finite()), "坐标必须是有限数");
        let m = &self.cfg.motion;
        ensure!(z >= m.min_z, "z = {z:.3} m 低于允许的最低高度 {:.3} m（防止撞桌面），未运动", m.min_z);
        let cfg = &self.cfg;
        let feasible = |a: &[f64; 4]| {
            (0..4).all(|j| {
                let [lo, hi] = cfg.limit(cfg.servos.joints[j]);
                (lo..=hi).contains(&cfg.angle_to_pulse(j, a[j]))
            })
        };
        let g = &cfg.geometry;
        let a = g.ik(x, y, z, pitch, &feasible).ok_or_else(|| {
            let d = (x.hypot(y)).hypot(z - g.base_height);
            let p = pitch.map_or("自动搜索".to_string(), |p| format!("{p}°"));
            anyhow!(
                "够不到 ({x:.3}, {y:.3}, {z:.3})，pitch {p}：离肩关节 {d:.3} m，最大臂展约 {:.3} m；也可能是关节限位挡住了，试试换个 pitch 或不指定 pitch",
                g.reach()
            )
        })?;
        let mut pulses: BTreeMap<u8, i32> = (0..4).map(|j| (cfg.servos.joints[j], cfg.angle_to_pulse(j, a[j]))).collect();
        if let Some(r) = roll {
            pulses.insert(cfg.servos.joints[4], cfg.angle_to_pulse(4, r));
        }
        Ok(IkSolution { joint_angles_deg: a.map(round2), pulses, check: round_pose(g.fk(a)) })
    }

    pub async fn move_to(
        &self,
        x: f64,
        y: f64,
        z: f64,
        pitch: Option<f64>,
        roll: Option<f64>,
        duration: Option<f64>,
    ) -> anyhow::Result<(IkSolution, MoveReport)> {
        let sol = self.compute_ik(x, y, z, pitch, roll)?;
        let report = self.move_pulses(sol.pulses.clone(), duration).await?;
        Ok((sol, report))
    }

    pub async fn gripper(&self, action: Option<GripperAction>, pulse: Option<i32>, duration: Option<f64>) -> anyhow::Result<MoveReport> {
        let p = match (action, pulse) {
            (_, Some(p)) => p,
            (Some(GripperAction::Open), None) => self.cfg.gripper.open,
            (Some(GripperAction::Close), None) => self.cfg.gripper.close,
            (None, None) => bail!("给 action（open/close）或 pulse 其中之一"),
        };
        self.move_pulses(BTreeMap::from([(self.cfg.servos.gripper, p)]), duration).await
    }

    pub async fn go_home(&self, duration: Option<f64>) -> anyhow::Result<MoveReport> {
        self.move_pulses(self.cfg.home(), duration).await
    }

    /// 急停：不排队，立即发停止指令，并打断正在等待的运动和回放。
    pub async fn stop(&self) -> anyhow::Result<ArmState> {
        let ids = self.cfg.all_ids();
        let r = self.io(move |b| b.stop(&ids)).await;
        self.stop_epoch.send_modify(|e| *e += 1);
        r?;
        Ok(self.state(false).await)
    }

    pub async fn set_torque(&self, ids: Vec<u8>, enable: bool) -> anyhow::Result<ArmState> {
        let known = self.cfg.all_ids();
        let ids = if ids.is_empty() { known.clone() } else { ids };
        for id in &ids {
            ensure!(known.contains(id), "舵机 {id} 不在配置里（已知：{known:?}）");
        }
        let on = self.cfg.servos.torque_on_cmd;
        let cmd = if enable { on } else if on == protocol::SERVO_TORQUE_A { protocol::SERVO_TORQUE_B } else { protocol::SERVO_TORQUE_A };
        let _g = self.motion.lock().await;
        self.io(move |b| ids.iter().try_for_each(|&id| b.torque(cmd, id))).await?;
        Ok(self.state(false).await)
    }

    pub async fn beep(&self, freq: u16, ms: u16) -> anyhow::Result<()> {
        self.io(move |b| b.buzzer(freq, ms, 10, 1)).await
    }

    /// 依次执行关键帧。整段持有运动锁，可被急停打断。
    pub async fn play(&self, frames: Vec<(BTreeMap<u8, i32>, f64)>) -> anyhow::Result<(usize, Vec<String>, ArmState)> {
        let _g = self.motion.lock().await;
        let mut rx = self.stop_epoch.subscribe();
        rx.borrow_and_update();
        let mut notes = Vec::new();
        for (i, (targets, dur)) in frames.iter().enumerate() {
            let r = self
                .move_locked(targets.clone(), Some(*dur), &mut rx)
                .await
                .with_context(|| format!("第 {} 帧", i + 1))?;
            notes.extend(r.notes.into_iter().map(|n| format!("第 {} 帧：{n}", i + 1)));
        }
        Ok((frames.len(), notes, self.state(false).await))
    }
}

fn round2(v: f64) -> f64 {
    (v * 100.0).round() / 100.0
}

fn round_pose(p: Pose) -> Pose {
    let r4 = |v: f64| (v * 10000.0).round() / 10000.0;
    Pose { x: r4(p.x), y: r4(p.y), z: r4(p.z), pitch: round2(p.pitch) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mock::MockBus;
    use std::time::Instant;

    fn setup(servos: &[(u8, i32)]) -> (MockBus, Arc<Arm>) {
        let bus = MockBus::new(servos);
        let board = Board::new(bus.opener(), Duration::from_millis(50), 2);
        assert!(board.wait_connected(Duration::from_secs(1)));
        let mut cfg = Config::default();
        cfg.motion.min_duration = 0.05;
        cfg.motion.max_pulse_per_sec = 2000.0;
        cfg.servos.limits.insert("2".into(), [100, 900]);
        (bus, Arc::new(Arm::new(Arc::new(board), Arc::new(cfg))))
    }

    const ALL: [(u8, i32); 6] = [(1, 500), (2, 500), (3, 500), (4, 500), (5, 500), (10, 300)];

    #[tokio::test]
    async fn rejects_out_of_limit_without_moving() {
        let (bus, arm) = setup(&ALL);
        let e = arm.move_pulses(BTreeMap::from([(2, 950)]), None).await.unwrap_err();
        assert!(e.to_string().contains("超出限位"), "{e}");
        assert_eq!(bus.pulse(2), Some(500));
        assert!(bus.servo_frames().iter().all(|d| d[0] != protocol::SERVO_SET_POSITION));
    }

    #[tokio::test]
    async fn unknown_servo_rejected() {
        let (_bus, arm) = setup(&ALL);
        assert!(arm.move_pulses(BTreeMap::from([(7, 500)]), None).await.is_err());
    }

    #[tokio::test]
    async fn too_fast_is_lengthened() {
        let (bus, arm) = setup(&ALL);
        // 400 脉冲 / 2000 每秒 = 0.2 s
        let r = arm.move_pulses(BTreeMap::from([(1, 900)]), Some(0.01)).await.unwrap();
        assert_eq!(r.duration_s, 0.2);
        assert_eq!(r.notes.len(), 1);
        assert_eq!(bus.pulse(1), Some(900));
        let set = bus.servo_frames().into_iter().find(|d| d[0] == protocol::SERVO_SET_POSITION).unwrap();
        assert_eq!(u16::from_le_bytes([set[1], set[2]]), 200);
    }

    #[tokio::test]
    async fn auto_duration_uses_min() {
        let (_bus, arm) = setup(&ALL);
        let r = arm.move_pulses(BTreeMap::from([(1, 510)]), None).await.unwrap();
        assert_eq!(r.duration_s, 0.05);
        assert!(r.notes.is_empty());
    }

    #[tokio::test]
    async fn stop_interrupts_waiting_motion() {
        let (bus, arm) = setup(&ALL);
        let a = arm.clone();
        let t = Instant::now();
        let mv = tokio::spawn(async move { a.move_pulses(BTreeMap::from([(1, 600)]), Some(5.0)).await });
        tokio::time::sleep(Duration::from_millis(150)).await;
        arm.stop().await.unwrap();
        let e = mv.await.unwrap().unwrap_err();
        assert!(e.downcast_ref::<Cancelled>().is_some(), "{e}");
        assert!(t.elapsed() < Duration::from_secs(1));
        assert!(bus.servo_frames().iter().any(|d| d[0] == protocol::SERVO_STOP));
    }

    #[tokio::test]
    async fn stop_interrupts_playback() {
        let (_bus, arm) = setup(&ALL);
        let a = arm.clone();
        let frames = vec![(BTreeMap::from([(1, 520)]), 2.0), (BTreeMap::from([(1, 540)]), 2.0)];
        let t = Instant::now();
        let play = tokio::spawn(async move { a.play(frames).await });
        tokio::time::sleep(Duration::from_millis(150)).await;
        arm.stop().await.unwrap();
        assert!(play.await.unwrap().is_err());
        assert!(t.elapsed() < Duration::from_secs(1));
    }

    #[tokio::test]
    async fn move_to_refuses_below_table() {
        let (bus, arm) = setup(&ALL);
        let e = arm.move_to(0.2, 0.0, 0.0, None, None, None).await.unwrap_err();
        assert!(e.to_string().contains("最低高度"), "{e}");
        assert!(bus.servo_frames().iter().all(|d| d[0] != protocol::SERVO_SET_POSITION));
    }

    #[tokio::test]
    async fn move_to_reaches_and_reports() {
        let (_bus, arm) = setup(&ALL);
        let (sol, r) = arm.move_to(0.2, 0.0, 0.1, None, None, None).await.unwrap();
        // 与 jetarm.py 的解一致：[500, 366, 367, 38]，但 2 号限位 [100, 900] 内仍然可行
        assert_eq!(sol.pulses, BTreeMap::from([(1, 500), (2, 366), (3, 367), (4, 38)]));
        let p = r.state.end_effector.unwrap();
        assert!((p.x - 0.2).abs() < 0.002 && (p.z - 0.1).abs() < 0.002, "{p:?}");
    }

    #[tokio::test]
    async fn out_of_reach_message() {
        let (_bus, arm) = setup(&ALL);
        let e = arm.compute_ik(0.6, 0.0, 0.1, None, None).unwrap_err();
        assert!(e.to_string().contains("够不到"), "{e}");
    }

    #[tokio::test]
    async fn state_reports_missing_servo() {
        let (_bus, arm) = setup(&[(1, 500), (2, 500), (3, 500), (4, 500), (5, 500)]);
        let s = arm.state(true).await;
        let g = s.servos.iter().find(|s| s.id == 10).unwrap();
        assert!(g.error.as_deref().unwrap().contains("无应答"));
        assert!(s.end_effector.is_some());
        assert_eq!(s.servos[0].voltage_v, Some(12.0));
    }

    #[tokio::test]
    async fn torque_off_uses_opposite_command() {
        let (bus, arm) = setup(&ALL);
        arm.set_torque(vec![2], false).await.unwrap();
        assert_eq!(bus.torque_cmd(2), Some(protocol::SERVO_TORQUE_B));
        arm.set_torque(vec![2], true).await.unwrap();
        assert_eq!(bus.torque_cmd(2), Some(protocol::SERVO_TORQUE_A));
    }

    #[tokio::test]
    async fn gripper_open_close() {
        let (bus, arm) = setup(&ALL);
        arm.gripper(Some(GripperAction::Close), None, None).await.unwrap();
        assert_eq!(bus.pulse(10), Some(arm.config().gripper.close));
        arm.gripper(None, Some(250), None).await.unwrap();
        assert_eq!(bus.pulse(10), Some(250));
    }
}
