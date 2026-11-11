//! 串口收发。后台线程持续读串口、解析帧、分拣舵机回包；断线后每 2 秒重连。
//! 所有方法都是阻塞的（单次最多 read_timeout × read_retries），异步代码应放进 `spawn_blocking`。

use std::io::{self, Read, Write};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use thiserror::Error;

use crate::protocol::{self, Parser, ServoReply};

pub type Reader = Box<dyn Read + Send>;
pub type Writer = Box<dyn Write + Send>;
/// 打开（或重新打开）串口，返回读写两端。
pub type Opener = Box<dyn FnMut() -> io::Result<(Reader, Writer)> + Send>;

const RECONNECT_INTERVAL: Duration = Duration::from_secs(2);

#[derive(Debug, Error)]
pub enum BoardError {
    #[error("串口未连接（检查控制板 USB 线和电源，服务会每 2 秒自动重连）")]
    Disconnected,
    #[error("串口写入失败：{0}")]
    Write(io::Error),
    #[error("舵机 {0} 无应答（检查舵机线和 12V 电源）")]
    NoReply(u8),
    #[error("舵机 {0} 回包格式不对")]
    BadReply(u8),
}

pub type Result<T> = std::result::Result<T, BoardError>;

pub struct Board {
    writer: Arc<Mutex<Option<Writer>>>,
    /// 保证同一时间只有一个读请求在等回包
    replies: Mutex<Receiver<ServoReply>>,
    timeout: Duration,
    retries: u32,
}

impl Board {
    pub fn new(opener: Opener, timeout: Duration, retries: u32) -> Self {
        let writer: Arc<Mutex<Option<Writer>>> = Arc::default();
        let (tx, rx) = mpsc::channel();
        let w = writer.clone();
        thread::Builder::new()
            .name("serial-reader".into())
            .spawn(move || reader_loop(opener, w, tx))
            .expect("启动串口线程");
        Self { writer, replies: Mutex::new(rx), timeout, retries: retries.max(1) }
    }

    /// 打开真实串口设备。
    pub fn open_serial(device: String, baud: u32, timeout: Duration, retries: u32) -> Self {
        let opener: Opener = Box::new(move || {
            let port = serialport::new(&device, baud)
                .timeout(Duration::from_millis(50))
                .open()
                .map_err(io::Error::other)?;
            let reader = port.try_clone().map_err(io::Error::other)?;
            Ok((Box::new(reader) as Reader, Box::new(port) as Writer))
        });
        Self::new(opener, timeout, retries)
    }

    pub fn is_connected(&self) -> bool {
        self.writer.lock().unwrap().is_some()
    }

    /// 等待串口连上（启动时用）。
    pub fn wait_connected(&self, max: Duration) -> bool {
        let t = Instant::now();
        while t.elapsed() < max {
            if self.is_connected() {
                return true;
            }
            thread::sleep(Duration::from_millis(20));
        }
        self.is_connected()
    }

    /// 写一帧。只在写的几毫秒内持有串口锁，急停不会被运动等待卡住。
    pub fn send(&self, frame: &[u8]) -> Result<()> {
        let mut guard = self.writer.lock().unwrap();
        let w = guard.as_mut().ok_or(BoardError::Disconnected)?;
        let r = w.write_all(frame).and_then(|_| w.flush());
        if let Err(e) = r {
            *guard = None;
            return Err(BoardError::Write(e));
        }
        Ok(())
    }

    /// 发读请求并等待对应回包，超时重试。返回值字节。
    pub fn read_servo(&self, id: u8, cmd: u8) -> Result<Vec<u8>> {
        let rx = self.replies.lock().unwrap();
        for _ in 0..self.retries {
            while rx.try_recv().is_ok() {} // 丢掉残留回包
            self.send(&protocol::servo_cmd(cmd, id))?;
            let deadline = Instant::now() + self.timeout;
            loop {
                let left = deadline.saturating_duration_since(Instant::now());
                match rx.recv_timeout(left) {
                    Ok(r) if r.id == id && r.cmd == cmd => {
                        if r.ok {
                            return Ok(r.value);
                        }
                        break; // 舵机报告失败，重试
                    }
                    Ok(_) => continue, // 别的回包
                    Err(RecvTimeoutError::Timeout) => break,
                    Err(RecvTimeoutError::Disconnected) => return Err(BoardError::Disconnected),
                }
            }
        }
        Err(BoardError::NoReply(id))
    }

    pub fn read_position(&self, id: u8) -> Result<i32> {
        let v = self.read_servo(id, protocol::SERVO_READ_POSITION)?;
        let b: [u8; 2] = v.get(..2).and_then(|s| s.try_into().ok()).ok_or(BoardError::BadReply(id))?;
        Ok(i16::from_le_bytes(b) as i32)
    }

    /// 电压，毫伏
    pub fn read_vin(&self, id: u8) -> Result<u32> {
        let v = self.read_servo(id, protocol::SERVO_READ_VIN)?;
        let b: [u8; 2] = v.get(..2).and_then(|s| s.try_into().ok()).ok_or(BoardError::BadReply(id))?;
        Ok(u16::from_le_bytes(b) as u32)
    }

    /// 温度，摄氏度
    pub fn read_temp(&self, id: u8) -> Result<u8> {
        let v = self.read_servo(id, protocol::SERVO_READ_TEMP)?;
        v.first().copied().ok_or(BoardError::BadReply(id))
    }

    pub fn set_position(&self, duration_ms: u16, targets: &[(u8, u16)]) -> Result<()> {
        self.send(&protocol::set_position(duration_ms, targets))
    }

    pub fn stop(&self, ids: &[u8]) -> Result<()> {
        self.send(&protocol::stop(ids))
    }

    pub fn torque(&self, cmd: u8, id: u8) -> Result<()> {
        self.send(&protocol::servo_cmd(cmd, id))?;
        thread::sleep(Duration::from_millis(20)); // 与官方 SDK 一致
        Ok(())
    }

    pub fn buzzer(&self, freq: u16, on_ms: u16, off_ms: u16, repeat: u16) -> Result<()> {
        self.send(&protocol::buzzer(freq, on_ms, off_ms, repeat))
    }
}

fn reader_loop(mut opener: Opener, writer: Arc<Mutex<Option<Writer>>>, tx: Sender<ServoReply>) {
    let mut warned = false;
    loop {
        let (mut reader, w) = match opener() {
            Ok(rw) => rw,
            Err(e) => {
                if !warned {
                    tracing::warn!("打不开串口：{e}；每 2 秒重试");
                    warned = true;
                }
                thread::sleep(RECONNECT_INTERVAL);
                continue;
            }
        };
        tracing::info!("串口已连接");
        warned = false;
        *writer.lock().unwrap() = Some(w);
        let mut parser = Parser::default();
        let mut buf = [0u8; 256];
        loop {
            match reader.read(&mut buf) {
                Ok(0) => thread::sleep(Duration::from_millis(1)),
                Ok(n) => {
                    for &b in &buf[..n] {
                        if let Some(r) = parser.feed(b).as_ref().and_then(ServoReply::parse)
                            && tx.send(r).is_err() {
                                return; // Board 已销毁
                            }
                    }
                }
                Err(e) if matches!(e.kind(), io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock | io::ErrorKind::Interrupted) => {}
                Err(e) => {
                    tracing::warn!("串口断开：{e}");
                    break;
                }
            }
            // 写端在写入失败时会被清空，这里也随之重连
            if writer.lock().unwrap().is_none() {
                break;
            }
        }
        *writer.lock().unwrap() = None;
        thread::sleep(RECONNECT_INTERVAL);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mock::MockBus;

    fn board(bus: &MockBus) -> Board {
        let b = Board::new(bus.opener(), Duration::from_millis(100), 3);
        assert!(b.wait_connected(Duration::from_secs(1)));
        b
    }

    #[test]
    fn reads_position_and_status() {
        let bus = MockBus::new(&[(1, 512), (3, -4)]);
        let b = board(&bus);
        assert_eq!(b.read_position(1).unwrap(), 512);
        assert_eq!(b.read_position(3).unwrap(), -4);
        assert_eq!(b.read_vin(1).unwrap(), 12000);
        assert_eq!(b.read_temp(1).unwrap(), 35);
    }

    #[test]
    fn missing_servo_times_out_with_retries() {
        let bus = MockBus::new(&[(1, 500)]);
        let b = board(&bus);
        let t = Instant::now();
        assert!(matches!(b.read_position(7), Err(BoardError::NoReply(7))));
        let e = t.elapsed();
        assert!(e >= Duration::from_millis(300) && e < Duration::from_secs(1), "{e:?}");
        assert_eq!(bus.reads_of(7), 3);
    }

    #[test]
    fn set_position_reaches_bus() {
        let bus = MockBus::new(&[(1, 500), (2, 500)]);
        let b = board(&bus);
        b.set_position(1000, &[(1, 600), (2, 400)]).unwrap();
        assert_eq!(b.read_position(1).unwrap(), 600);
        assert_eq!(b.read_position(2).unwrap(), 400);
    }

    #[test]
    fn reconnects_after_unplug() {
        let bus = MockBus::new(&[(1, 500)]);
        let b = board(&bus);
        bus.unplug();
        thread::sleep(Duration::from_millis(100));
        assert!(matches!(b.read_position(1), Err(BoardError::Disconnected) | Err(BoardError::Write(_))));
        assert!(b.wait_connected(Duration::from_secs(3)), "没有自动重连");
        assert_eq!(b.read_position(1).unwrap(), 500);
    }
}
