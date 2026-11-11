//! 动作组：每个动作一个 JSON 文件，内容是关键帧列表。

use std::collections::BTreeMap;
use std::path::PathBuf;

use anyhow::{Context, ensure};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Keyframe {
    /// 从上一帧运动到这一帧的时长（秒）
    pub duration: f64,
    /// 舵机 ID → 脉冲
    pub pulses: BTreeMap<u8, i32>,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default, PartialEq)]
pub struct Action {
    pub frames: Vec<Keyframe>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ActionSummary {
    pub name: String,
    pub frames: usize,
    pub total_duration_s: f64,
}

pub struct ActionStore {
    dir: PathBuf,
}

impl ActionStore {
    pub fn new(dir: PathBuf) -> Self {
        Self { dir }
    }

    fn path(&self, name: &str) -> anyhow::Result<PathBuf> {
        ensure!(
            !name.is_empty()
                && name.chars().count() <= 64
                && name.chars().all(|c| c.is_alphanumeric() || c == '_' || c == '-'),
            "动作名只能用字母、数字、汉字、下划线和连字符，最长 64 个字符"
        );
        Ok(self.dir.join(format!("{name}.json")))
    }

    pub fn load(&self, name: &str) -> anyhow::Result<Action> {
        let p = self.path(name)?;
        let text = std::fs::read_to_string(&p).with_context(|| format!("没有名为 {name} 的动作"))?;
        serde_json::from_str(&text).with_context(|| format!("动作文件 {} 损坏", p.display()))
    }

    fn save(&self, name: &str, a: &Action) -> anyhow::Result<()> {
        let p = self.path(name)?;
        std::fs::create_dir_all(&self.dir).with_context(|| format!("创建 {}", self.dir.display()))?;
        let tmp = p.with_extension("json.tmp");
        std::fs::write(&tmp, serde_json::to_string_pretty(a)?)?;
        std::fs::rename(&tmp, &p)?;
        Ok(())
    }

    /// 追加一帧；动作不存在则新建。返回追加后的帧数。
    pub fn append(&self, name: &str, frame: Keyframe) -> anyhow::Result<usize> {
        let mut a = match self.load(name) {
            Ok(a) => a,
            Err(_) if !self.path(name)?.exists() => Action::default(),
            Err(e) => return Err(e),
        };
        a.frames.push(frame);
        self.save(name, &a)?;
        Ok(a.frames.len())
    }

    pub fn list(&self) -> anyhow::Result<Vec<ActionSummary>> {
        let Ok(rd) = std::fs::read_dir(&self.dir) else { return Ok(vec![]) };
        let mut out = Vec::new();
        for e in rd.flatten() {
            let p = e.path();
            if p.extension().and_then(|s| s.to_str()) != Some("json") {
                continue;
            }
            let Some(name) = p.file_stem().and_then(|s| s.to_str()) else { continue };
            if let Ok(a) = self.load(name) {
                out.push(ActionSummary {
                    name: name.to_string(),
                    frames: a.frames.len(),
                    total_duration_s: a.frames.iter().map(|f| f.duration).sum(),
                });
            }
        }
        out.sort_by(|a, b| a.name.cmp(&b.name));
        Ok(out)
    }

    pub fn delete(&self, name: &str) -> anyhow::Result<()> {
        let p = self.path(name)?;
        std::fs::remove_file(&p).with_context(|| format!("没有名为 {name} 的动作"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn kf(d: f64, p: i32) -> Keyframe {
        Keyframe { duration: d, pulses: BTreeMap::from([(1, p)]) }
    }

    #[test]
    fn record_list_load_delete() {
        let dir = tempfile::tempdir().unwrap();
        let s = ActionStore::new(dir.path().join("actions"));
        assert!(s.list().unwrap().is_empty());
        assert_eq!(s.append("挥手", kf(1.0, 400)).unwrap(), 1);
        assert_eq!(s.append("挥手", kf(0.5, 600)).unwrap(), 2);
        let l = s.list().unwrap();
        assert_eq!(l.len(), 1);
        assert_eq!((l[0].name.as_str(), l[0].frames, l[0].total_duration_s), ("挥手", 2, 1.5));
        assert_eq!(s.load("挥手").unwrap().frames[1], kf(0.5, 600));
        s.delete("挥手").unwrap();
        assert!(s.load("挥手").is_err());
    }

    #[test]
    fn rejects_path_tricks() {
        let dir = tempfile::tempdir().unwrap();
        let s = ActionStore::new(dir.path().to_path_buf());
        for bad in ["", "../x", "a/b", "a.b"] {
            assert!(s.append(bad, kf(1.0, 500)).is_err(), "{bad}");
        }
    }
}
