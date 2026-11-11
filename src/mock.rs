//! 模拟舵机总线，供单元测试使用（不需要硬件）。
//! 收到"设位置"立即更新脉冲；收到读命令按真实格式回包；可模拟拔线。

use std::collections::BTreeMap;
use std::io::{self, Read, Write};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::board::{Opener, Reader, Writer};
use crate::protocol::{self, Frame, Parser};

#[derive(Default)]
struct State {
    pulses: BTreeMap<u8, i32>,
    torque: BTreeMap<u8, u8>,
    frames: Vec<Frame>,
    epoch: u64,
}

#[derive(Clone)]
pub struct MockBus(Arc<Mutex<State>>);

impl MockBus {
    pub fn new(servos: &[(u8, i32)]) -> Self {
        let s = State { pulses: servos.iter().copied().collect(), ..Default::default() };
        Self(Arc::new(Mutex::new(s)))
    }

    pub fn opener(&self) -> Opener {
        let state = self.0.clone();
        Box::new(move || {
            let epoch = state.lock().unwrap().epoch;
            let (tx, rx) = mpsc::channel();
            let w = MockWriter { state: state.clone(), epoch, tx, parser: Parser::default() };
            let r = MockReader { state: state.clone(), epoch, rx, pending: Vec::new() };
            Ok((Box::new(r) as Reader, Box::new(w) as Writer))
        })
    }

    /// 模拟拔掉 USB：当前连接立即失效，之后可以重新打开。
    pub fn unplug(&self) {
        self.0.lock().unwrap().epoch += 1;
    }

    pub fn pulse(&self, id: u8) -> Option<i32> {
        self.0.lock().unwrap().pulses.get(&id).copied()
    }

    /// 直接改舵机位置（模拟手动掰动）
    pub fn set_pulse(&self, id: u8, p: i32) {
        self.0.lock().unwrap().pulses.insert(id, p);
    }

    /// 收到的总线舵机帧（func=5）的数据部分
    pub fn servo_frames(&self) -> Vec<Vec<u8>> {
        let s = self.0.lock().unwrap();
        s.frames.iter().filter(|f| f.func == protocol::FUNC_BUS_SERVO).map(|f| f.data.clone()).collect()
    }

    pub fn frames(&self) -> Vec<Frame> {
        self.0.lock().unwrap().frames.clone()
    }

    pub fn reads_of(&self, id: u8) -> usize {
        self.servo_frames().iter().filter(|d| d.len() == 2 && d[0] == protocol::SERVO_READ_POSITION && d[1] == id).count()
    }

    pub fn torque_cmd(&self, id: u8) -> Option<u8> {
        self.0.lock().unwrap().torque.get(&id).copied()
    }
}

struct MockWriter {
    state: Arc<Mutex<State>>,
    epoch: u64,
    tx: Sender<Vec<u8>>,
    parser: Parser,
}

impl Write for MockWriter {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mut s = self.state.lock().unwrap();
        if s.epoch != self.epoch {
            return Err(io::Error::new(io::ErrorKind::BrokenPipe, "unplugged"));
        }
        for &b in buf {
            let Some(f) = self.parser.feed(b) else { continue };
            s.frames.push(f.clone());
            if f.func != protocol::FUNC_BUS_SERVO || f.data.is_empty() {
                continue;
            }
            let d = &f.data;
            match d[0] {
                protocol::SERVO_SET_POSITION => {
                    let n = d[3] as usize;
                    for k in 0..n {
                        let o = 4 + k * 3;
                        let pos = u16::from_le_bytes([d[o + 1], d[o + 2]]) as i32;
                        s.pulses.insert(d[o], pos);
                    }
                }
                protocol::SERVO_TORQUE_A | protocol::SERVO_TORQUE_B => {
                    s.torque.insert(d[1], d[0]);
                }
                cmd @ (protocol::SERVO_READ_POSITION | protocol::SERVO_READ_VIN | protocol::SERVO_READ_TEMP) => {
                    let id = d[1];
                    let Some(&p) = s.pulses.get(&id) else { continue };
                    let mut reply = vec![id, cmd, 0];
                    match cmd {
                        protocol::SERVO_READ_POSITION => reply.extend_from_slice(&(p as i16).to_le_bytes()),
                        protocol::SERVO_READ_VIN => reply.extend_from_slice(&12000u16.to_le_bytes()),
                        _ => reply.push(35),
                    }
                    let _ = self.tx.send(protocol::encode(protocol::FUNC_BUS_SERVO, &reply));
                }
                _ => {}
            }
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

struct MockReader {
    state: Arc<Mutex<State>>,
    epoch: u64,
    rx: Receiver<Vec<u8>>,
    pending: Vec<u8>,
}

impl Read for MockReader {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if self.state.lock().unwrap().epoch != self.epoch {
            return Err(io::Error::new(io::ErrorKind::BrokenPipe, "unplugged"));
        }
        if self.pending.is_empty() {
            match self.rx.recv_timeout(Duration::from_millis(20)) {
                Ok(v) => self.pending = v,
                Err(_) => return Err(io::ErrorKind::TimedOut.into()),
            }
        }
        let n = buf.len().min(self.pending.len());
        buf[..n].copy_from_slice(&self.pending[..n]);
        self.pending.drain(..n);
        Ok(n)
    }
}
