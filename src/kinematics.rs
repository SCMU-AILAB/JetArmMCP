//! 正逆运动学，移植自 `~/jetarm/jetarm.py`（已在实机上验证）。纯函数。
//!
//! 坐标系（米 / 度），原点在底座底面中心：z 向上；x 是底座舵机在 500 时臂"向前"的方向；y 按右手系。
//! pitch 是末端与竖直方向的夹角：0 = 朝天，90 = 水平向前，180 = 垂直朝下。
//! 关节角 t1..t5：t1 底座旋转，t2..t4 向前俯仰（全 0 = 竖直伸直），t5 腕部旋转。

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct Geometry {
    /// 底面 → 肩关节
    pub base_height: f64,
    /// 肩 → 肘
    pub l1: f64,
    /// 肘 → 腕
    pub l2: f64,
    /// 腕 → 夹爪末端
    pub l3: f64,
}

impl Default for Geometry {
    fn default() -> Self {
        Self { base_height: 0.10314916, l1: 0.12941764, l2: 0.12941764, l3: 0.05945313 + 0.11054687 }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub pitch: f64,
}

impl Geometry {
    pub fn reach(&self) -> f64 {
        self.l1 + self.l2 + self.l3
    }

    /// 关节角 t1..t4（度）→ 末端位姿。
    pub fn fk(&self, a: [f64; 4]) -> Pose {
        let [t1, t2, t3, t4] = a.map(f64::to_radians);
        let (p, q, r) = (t2, t2 + t3, t2 + t3 + t4);
        let rho = self.l1 * p.sin() + self.l2 * q.sin() + self.l3 * r.sin();
        let z = self.base_height + self.l1 * p.cos() + self.l2 * q.cos() + self.l3 * r.cos();
        Pose { x: rho * t1.cos(), y: rho * t1.sin(), z, pitch: r.to_degrees() }
    }

    /// 末端位置 → 关节角 t1..t4（度）。`pitch = None` 时从 180° 往下每 5° 搜索可行解（优先朝下）。
    /// `feasible` 判断一组关节角是否在舵机限位内。够不到返回 None。
    pub fn ik(&self, x: f64, y: f64, z: f64, pitch: Option<f64>, feasible: &dyn Fn(&[f64; 4]) -> bool) -> Option<[f64; 4]> {
        let Some(pitch) = pitch else {
            return (0..=36).rev().find_map(|k| self.ik(x, y, z, Some(k as f64 * 5.0), feasible));
        };
        let t = pitch.to_radians();
        let t1 = y.atan2(x);
        let rho = x.hypot(y);
        let rw = rho - self.l3 * t.sin();
        let hw = z - self.base_height - self.l3 * t.cos();
        let c3 = (rw * rw + hw * hw - self.l1 * self.l1 - self.l2 * self.l2) / (2.0 * self.l1 * self.l2);
        if c3.abs() > 1.0 {
            return None;
        }
        // 先试肘部在上的解
        for t3 in [c3.acos(), -c3.acos()] {
            let t2 = rw.atan2(hw) - (self.l2 * t3.sin()).atan2(self.l1 + self.l2 * t3.cos());
            let t4 = t - t2 - t3;
            let sol = [t1, t2, t3, t4].map(|v| wrap_pi(v).to_degrees());
            if feasible(&sol) {
                return Some(sol);
            }
        }
        None
    }
}

fn wrap_pi(v: f64) -> f64 {
    use std::f64::consts::PI;
    (v + PI).rem_euclid(2.0 * PI) - PI
}

#[cfg(test)]
mod tests {
    use super::*;

    const DEG_PER_PULSE: f64 = 0.24;
    const DIR: [f64; 4] = [1.0, -1.0, -1.0, -1.0];

    fn pulses(a: &[f64; 4]) -> [i32; 4] {
        std::array::from_fn(|i| (500.0 + DIR[i] * a[i] / DEG_PER_PULSE).round() as i32)
    }

    fn in_range(a: &[f64; 4]) -> bool {
        pulses(a).iter().all(|p| (0..=1000).contains(p))
    }

    fn close(a: &[f64], b: &[f64], eps: f64) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < eps)
    }

    #[test]
    fn fk_matches_python() {
        let p = Geometry::default().fk([10.0, 30.0, 60.0, 40.0]);
        assert!(close(&[p.x, p.y, p.z, p.pitch], &[0.319426, 0.056323, 0.105954, 130.0], 1e-5), "{p:?}");
    }

    #[test]
    fn ik_matches_python() {
        let g = Geometry::default();
        let cases: [((f64, f64, f64, Option<f64>), Option<([f64; 4], [i32; 4])>); 4] = [
            ((0.20, 0.0, 0.10, None), Some(([0.0, 32.109565, 31.964326, 110.926109], [500, 366, 367, 38]))),
            ((0.15, 0.05, 0.05, Some(180.0)), Some(([18.434949, 12.961826, 81.145353, 85.892822], [577, 446, 162, 142]))),
            ((0.25, -0.05, 0.15, Some(90.0)), None),
            ((0.18, 0.0, 0.02, None), Some(([0.0, 24.789114, 78.906719, 76.304167], [500, 397, 171, 182]))),
        ];
        for ((x, y, z, p), want) in cases {
            let got = g.ik(x, y, z, p, &in_range);
            match (got, want) {
                (None, None) => {}
                (Some(a), Some((wa, wp))) => {
                    assert!(close(&a, &wa, 1e-4), "{a:?} vs {wa:?}");
                    assert_eq!(pulses(&a), wp);
                }
                _ => panic!("({x},{y},{z},{p:?}): got {got:?}"),
            }
        }
    }

    #[test]
    fn ik_fk_roundtrip() {
        let g = Geometry::default();
        for &(x, y, z) in &[(0.15, 0.0, 0.05), (0.2, 0.1, 0.12), (0.12, -0.08, 0.2), (0.25, 0.0, 0.03)] {
            let a = g.ik(x, y, z, None, &in_range).unwrap_or_else(|| panic!("够不到 {x},{y},{z}"));
            let p = g.fk(a);
            assert!(close(&[p.x, p.y, p.z], &[x, y, z], 1e-9), "{p:?}");
        }
    }

    #[test]
    fn out_of_reach() {
        let g = Geometry::default();
        assert!(g.ik(0.6, 0.0, 0.1, None, &|_| true).is_none());
    }
}
