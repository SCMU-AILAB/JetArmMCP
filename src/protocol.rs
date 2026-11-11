//! Hiwonder ros_robot_controller 串口协议：帧编解码与 CRC8。纯函数，不碰 IO。
//!
//! 帧格式：`AA 55 | func | len | data… | crc8`，CRC8 覆盖 func + len + data。

pub const FUNC_BUZZER: u8 = 2;
pub const FUNC_BUS_SERVO: u8 = 5;
/// 官方 SDK 只接受小于此值的功能码。
const FUNC_NONE: u8 = 11;

// 总线舵机子命令
pub const SERVO_SET_POSITION: u8 = 0x01;
pub const SERVO_STOP: u8 = 0x03;
pub const SERVO_READ_POSITION: u8 = 0x05;
pub const SERVO_READ_VIN: u8 = 0x07;
pub const SERVO_READ_TEMP: u8 = 0x09;
pub const SERVO_TORQUE_A: u8 = 0x0B;
pub const SERVO_TORQUE_B: u8 = 0x0C;
pub const SERVO_READ_TORQUE: u8 = 0x0D;

/// Dallas/Maxim CRC8（反射多项式 0x8C），与官方 SDK 的查表一致。
const CRC8_TABLE: [u8; 256] = {
    let mut t = [0u8; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u8;
        let mut b = 0;
        while b < 8 {
            c = if c & 1 != 0 { (c >> 1) ^ 0x8C } else { c >> 1 };
            b += 1;
        }
        t[i] = c;
        i += 1;
    }
    t
};

pub fn crc8(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |c, &b| CRC8_TABLE[(c ^ b) as usize])
}

pub fn encode(func: u8, data: &[u8]) -> Vec<u8> {
    assert!(data.len() <= 255, "帧数据过长");
    let mut buf = Vec::with_capacity(data.len() + 5);
    buf.extend_from_slice(&[0xAA, 0x55, func, data.len() as u8]);
    buf.extend_from_slice(data);
    buf.push(crc8(&buf[2..]));
    buf
}

/// 多个舵机在 `duration_ms` 内运动到目标脉冲。
pub fn set_position(duration_ms: u16, targets: &[(u8, u16)]) -> Vec<u8> {
    let mut d = vec![SERVO_SET_POSITION];
    d.extend_from_slice(&duration_ms.to_le_bytes());
    d.push(targets.len() as u8);
    for &(id, pos) in targets {
        d.push(id);
        d.extend_from_slice(&pos.to_le_bytes());
    }
    encode(FUNC_BUS_SERVO, &d)
}

pub fn stop(ids: &[u8]) -> Vec<u8> {
    let mut d = vec![SERVO_STOP, ids.len() as u8];
    d.extend_from_slice(ids);
    encode(FUNC_BUS_SERVO, &d)
}

/// 单字节参数的舵机命令（读请求、力矩开关）。
pub fn servo_cmd(cmd: u8, id: u8) -> Vec<u8> {
    encode(FUNC_BUS_SERVO, &[cmd, id])
}

pub fn buzzer(freq: u16, on_ms: u16, off_ms: u16, repeat: u16) -> Vec<u8> {
    let mut d = Vec::with_capacity(8);
    for v in [freq, on_ms, off_ms, repeat] {
        d.extend_from_slice(&v.to_le_bytes());
    }
    encode(FUNC_BUZZER, &d)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    pub func: u8,
    pub data: Vec<u8>,
}

/// 总线舵机读命令的回包：`id, cmd, success(i8), value…`。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ServoReply {
    pub id: u8,
    pub cmd: u8,
    pub ok: bool,
    pub value: Vec<u8>,
}

impl ServoReply {
    pub fn parse(frame: &Frame) -> Option<Self> {
        if frame.func != FUNC_BUS_SERVO || frame.data.len() < 3 {
            return None;
        }
        Some(Self {
            id: frame.data[0],
            cmd: frame.data[1],
            ok: frame.data[2] as i8 == 0,
            value: frame.data[3..].to_vec(),
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Start1,
    Start2,
    Func,
    Len,
    Data,
    Crc,
}

/// 字节流 → 帧的状态机，与官方 SDK `recv_task` 相同的规则。
#[derive(Debug)]
pub struct Parser {
    state: State,
    func: u8,
    len: u8,
    data: Vec<u8>,
}

impl Default for Parser {
    fn default() -> Self {
        Self { state: State::Start1, func: 0, len: 0, data: Vec::new() }
    }
}

impl Parser {
    pub fn feed(&mut self, b: u8) -> Option<Frame> {
        match self.state {
            State::Start1 => {
                if b == 0xAA {
                    self.state = State::Start2;
                }
            }
            // 比官方 SDK 多一步：连续两个 0xAA 时留在 Start2，免得噪声吞掉紧随其后的真帧头
            State::Start2 => {
                self.state = match b {
                    0x55 => State::Func,
                    0xAA => State::Start2,
                    _ => State::Start1,
                }
            }
            State::Func => {
                if b < FUNC_NONE {
                    self.func = b;
                    self.state = State::Len;
                } else {
                    self.state = State::Start1;
                }
            }
            State::Len => {
                self.len = b;
                self.data.clear();
                self.state = if b == 0 { State::Crc } else { State::Data };
            }
            State::Data => {
                self.data.push(b);
                if self.data.len() >= self.len as usize {
                    self.state = State::Crc;
                }
            }
            State::Crc => {
                self.state = State::Start1;
                let mut covered = vec![self.func, self.len];
                covered.extend_from_slice(&self.data);
                if crc8(&covered) == b {
                    return Some(Frame { func: self.func, data: std::mem::take(&mut self.data) });
                }
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_table_matches_official_sdk() {
        // 官方 crc8_table 的开头与结尾几项
        assert_eq!(&CRC8_TABLE[..8], &[0, 94, 188, 226, 97, 63, 221, 131]);
        assert_eq!(&CRC8_TABLE[248..], &[182, 232, 10, 84, 215, 137, 107, 53]);
    }

    // 下面的期望字节由官方 Python SDK 的 buf_write 生成（见 tests/sdk_vectors.py）。
    #[test]
    fn set_position_bytes() {
        assert_eq!(
            set_position(1500, &[(1, 500), (2, 761)]),
            vec![0xAA, 0x55, 0x05, 0x0A, 0x01, 0xDC, 0x05, 0x02, 0x01, 0xF4, 0x01, 0x02, 0xF9, 0x02, 0x95]
        );
    }

    #[test]
    fn read_position_bytes() {
        assert_eq!(servo_cmd(SERVO_READ_POSITION, 3), vec![0xAA, 0x55, 0x05, 0x02, 0x05, 0x03, 0xD3]);
    }

    #[test]
    fn stop_bytes() {
        assert_eq!(stop(&[1, 2, 3]), vec![0xAA, 0x55, 0x05, 0x05, 0x03, 0x03, 0x01, 0x02, 0x03, 0x3C]);
    }

    #[test]
    fn buzzer_bytes() {
        assert_eq!(
            buzzer(1900, 100, 50, 1),
            vec![0xAA, 0x55, 0x02, 0x08, 0x6C, 0x07, 0x64, 0x00, 0x32, 0x00, 0x01, 0x00, 0x21]
        );
    }

    #[test]
    fn parser_roundtrip_with_noise() {
        let frame = encode(FUNC_BUS_SERVO, &[3, SERVO_READ_POSITION, 0, 0x2C, 0x01]);
        let mut stream = vec![0x00, 0xAA, 0x13, 0xAA];
        stream.extend_from_slice(&frame);
        let mut p = Parser::default();
        let got: Vec<_> = stream.iter().filter_map(|&b| p.feed(b)).collect();
        assert_eq!(got.len(), 1);
        let r = ServoReply::parse(&got[0]).unwrap();
        assert_eq!((r.id, r.cmd, r.ok), (3, SERVO_READ_POSITION, true));
        assert_eq!(i16::from_le_bytes([r.value[0], r.value[1]]), 300);
    }

    #[test]
    fn parser_rejects_bad_crc() {
        let mut frame = encode(FUNC_BUS_SERVO, &[1, 5, 0, 1, 0]);
        *frame.last_mut().unwrap() ^= 0xFF;
        let mut p = Parser::default();
        assert!(frame.iter().all(|&b| p.feed(b).is_none()));
    }
}
