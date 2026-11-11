//! `config.toml`。所有字段都有缺省值（即实测参数），配置文件只需写要改的项。

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use anyhow::Context;
use serde::{Deserialize, Serialize};

use crate::kinematics::Geometry;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
#[derive(Default)]
pub struct Config {
    pub server: Server,
    pub serial: Serial,
    pub servos: Servos,
    pub geometry: Geometry,
    pub motion: Motion,
    pub gripper: Gripper,
    pub camera: Camera,
    /// 动作组目录；缺省为 `~/.local/share/jetarm-mcp/actions`
    pub actions_dir: Option<PathBuf>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct Server {
    pub bind: String,
    pub port: u16,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct Serial {
    pub device: String,
    pub baud: u32,
    /// 每次读舵机等待回包的超时（秒）
    pub read_timeout: f64,
    pub read_retries: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct Servos {
    /// 关节 1..5 对应的舵机 ID（底座、肩、肘、腕俯仰、腕旋转）
    pub joints: [u8; 5],
    pub gripper: u8,
    /// 关节角正方向 → 脉冲方向（+1 / -1），与 `joints` 一一对应
    pub direction: [i8; 5],
    /// 1 脉冲对应的角度
    pub deg_per_pulse: f64,
    /// 每个舵机允许的脉冲范围；未列出的用 [0, 1000]
    pub limits: BTreeMap<String, [i32; 2]>,
    /// 力矩"上力"子命令。官方 SDK 用 0x0B，Wiki 说法相反，待实机确认。
    pub torque_on_cmd: u8,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct Motion {
    /// 限速：每秒最多多少脉冲（250 ≈ 60°/s）
    pub max_pulse_per_sec: f64,
    /// 最短运动时长（秒）
    pub min_duration: f64,
    /// 单个动作的最长时长（秒）
    pub max_duration: f64,
    /// `move_to` 允许的最低末端高度（米）
    pub min_z: f64,
    /// 初始姿态：舵机 ID → 脉冲
    pub home: BTreeMap<String, i32>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct Gripper {
    /// 张开时的脉冲（待实机确认）
    pub open: i32,
    /// 闭合时的脉冲（待实机确认）
    pub close: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct Camera {
    pub device: String,
    pub width: u32,
    pub height: u32,
    /// 开相机后丢掉的帧数，等曝光稳定
    pub warmup_frames: u32,
    pub jpeg_quality: u8,
}


impl Default for Server {
    fn default() -> Self {
        Self { bind: "0.0.0.0".into(), port: 8848 }
    }
}

impl Default for Serial {
    fn default() -> Self {
        Self { device: "/dev/ttyUSB0".into(), baud: 1_000_000, read_timeout: 0.5, read_retries: 3 }
    }
}

impl Default for Servos {
    fn default() -> Self {
        Self {
            joints: [1, 2, 3, 4, 5],
            gripper: 10,
            direction: [1, -1, -1, -1, 1],
            deg_per_pulse: 0.24,
            limits: BTreeMap::new(),
            torque_on_cmd: 0x0B,
        }
    }
}

impl Default for Motion {
    fn default() -> Self {
        let home = [(1, 512), (2, 761), (3, 0), (4, 320), (5, 522)];
        Self {
            max_pulse_per_sec: 250.0,
            min_duration: 0.5,
            max_duration: 30.0,
            min_z: 0.01,
            home: home.into_iter().map(|(k, v)| (k.to_string(), v)).collect(),
        }
    }
}

impl Default for Gripper {
    fn default() -> Self {
        Self { open: 200, close: 500 }
    }
}

impl Default for Camera {
    fn default() -> Self {
        Self { device: "/dev/video0".into(), width: 640, height: 480, warmup_frames: 5, jpeg_quality: 80 }
    }
}

impl Config {
    pub fn load(path: &Path) -> anyhow::Result<Self> {
        let text = std::fs::read_to_string(path).with_context(|| format!("读取 {}", path.display()))?;
        let cfg: Self = toml::from_str(&text).with_context(|| format!("解析 {}", path.display()))?;
        cfg.validate()?;
        Ok(cfg)
    }

    pub fn validate(&self) -> anyhow::Result<()> {
        for (k, [lo, hi]) in &self.servos.limits {
            k.parse::<u8>().with_context(|| format!("servos.limits 的键 {k:?} 不是舵机 ID"))?;
            anyhow::ensure!(0 <= *lo && lo < hi && *hi <= 1000, "舵机 {k} 的限位 [{lo}, {hi}] 无效");
        }
        for k in self.motion.home.keys() {
            k.parse::<u8>().with_context(|| format!("motion.home 的键 {k:?} 不是舵机 ID"))?;
        }
        anyhow::ensure!(self.servos.direction.iter().all(|d| d.abs() == 1), "servos.direction 只能是 1 或 -1");
        anyhow::ensure!(self.motion.max_pulse_per_sec > 0.0, "motion.max_pulse_per_sec 必须大于 0");
        anyhow::ensure!(self.servos.torque_on_cmd == 0x0B || self.servos.torque_on_cmd == 0x0C, "servos.torque_on_cmd 只能是 0x0B 或 0x0C");
        Ok(())
    }

    pub fn limit(&self, id: u8) -> [i32; 2] {
        self.servos.limits.get(&id.to_string()).copied().unwrap_or([0, 1000])
    }

    pub fn home(&self) -> BTreeMap<u8, i32> {
        self.motion.home.iter().map(|(k, v)| (k.parse().expect("已校验"), *v)).collect()
    }

    pub fn actions_dir(&self) -> PathBuf {
        self.actions_dir.clone().unwrap_or_else(|| {
            dirs::data_local_dir().unwrap_or_else(|| PathBuf::from(".")).join("jetarm-mcp/actions")
        })
    }

    /// 全部舵机：关节 + 夹爪
    pub fn all_ids(&self) -> Vec<u8> {
        let mut v = self.servos.joints.to_vec();
        v.push(self.servos.gripper);
        v
    }

    pub fn angle_to_pulse(&self, joint: usize, deg: f64) -> i32 {
        (500.0 + self.servos.direction[joint] as f64 * deg / self.servos.deg_per_pulse).round() as i32
    }

    pub fn pulse_to_angle(&self, joint: usize, pulse: i32) -> f64 {
        self.servos.direction[joint] as f64 * (pulse - 500) as f64 * self.servos.deg_per_pulse
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn partial_file_keeps_defaults() {
        let cfg: Config = toml::from_str("[server]\nport = 9000\n[servos.limits]\n2 = [100, 900]\n").unwrap();
        cfg.validate().unwrap();
        assert_eq!(cfg.server.port, 9000);
        assert_eq!(cfg.server.bind, "0.0.0.0");
        assert_eq!(cfg.serial.baud, 1_000_000);
        assert_eq!(cfg.limit(2), [100, 900]);
        assert_eq!(cfg.limit(3), [0, 1000]);
        assert_eq!(cfg.home()[&2], 761);
    }

    #[test]
    fn rejects_bad_limits() {
        let cfg: Config = toml::from_str("[servos.limits]\n2 = [900, 100]\n").unwrap();
        assert!(cfg.validate().is_err());
    }
}
