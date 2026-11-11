//! 相机：每次调用时打开 V4L2 设备，丢弃前几帧等曝光稳定，取一帧返回 JPEG，然后关掉。
//! 优先用相机自带的 MJPG（直接拿到 JPEG），不支持时退回 YUYV 自行编码。

use anyhow::{Context, bail};
use image::{DynamicImage, ImageFormat, RgbImage, codecs::jpeg::JpegEncoder, imageops::FilterType};

use crate::config::Camera as CameraConfig;

pub struct Jpeg {
    pub bytes: Vec<u8>,
    pub width: u32,
    pub height: u32,
}

/// 拍一张。`max_width` 给出时按比例缩小。阻塞调用。
pub fn capture(cfg: &CameraConfig, max_width: Option<u32>) -> anyhow::Result<Jpeg> {
    let raw = grab(cfg)?;
    match raw {
        Raw::Jpeg { bytes, width, height } => match max_width {
            Some(w) if w < width => {
                let img = image::load_from_memory_with_format(&bytes, ImageFormat::Jpeg).context("解码相机 JPEG")?;
                encode(img, cfg.jpeg_quality, Some(w))
            }
            _ => Ok(Jpeg { bytes, width, height }),
        },
        Raw::Yuyv { data, width, height, stride } => {
            let rgb = yuyv_to_rgb(&data, width, height, stride)?;
            encode(DynamicImage::ImageRgb8(rgb), cfg.jpeg_quality, max_width)
        }
    }
}

#[allow(dead_code)] // 非 Linux 下 grab 只报错，不构造任何变体
enum Raw {
    Jpeg { bytes: Vec<u8>, width: u32, height: u32 },
    Yuyv { data: Vec<u8>, width: u32, height: u32, stride: u32 },
}

#[cfg(target_os = "linux")]
fn grab(cfg: &CameraConfig) -> anyhow::Result<Raw> {
    use v4l::buffer::Type;
    use v4l::io::traits::CaptureStream;
    use v4l::prelude::*;
    use v4l::video::Capture;
    use v4l::{FourCC, Format};

    let dev = Device::with_path(&cfg.device).with_context(|| format!("打不开相机 {}（是否被别的程序占用？）", cfg.device))?;
    let mjpg = FourCC::new(b"MJPG");
    let yuyv = FourCC::new(b"YUYV");
    let mut fmt = dev.set_format(&Format::new(cfg.width, cfg.height, mjpg)).context("设置相机格式")?;
    if fmt.fourcc != mjpg {
        fmt = dev.set_format(&Format::new(cfg.width, cfg.height, yuyv)).context("设置相机格式")?;
    }
    if fmt.fourcc != mjpg && fmt.fourcc != yuyv {
        bail!("相机不支持 MJPG 或 YUYV（当前 {}）", fmt.fourcc);
    }
    let mut stream = MmapStream::with_buffers(&dev, Type::VideoCapture, 4).context("启动相机取流")?;
    for _ in 0..cfg.warmup_frames {
        stream.next().context("读相机帧")?;
    }
    let (buf, meta) = stream.next().context("读相机帧")?;
    let used = (meta.bytesused as usize).min(buf.len());
    let data = if used > 0 { buf[..used].to_vec() } else { buf.to_vec() };
    Ok(if fmt.fourcc == mjpg {
        Raw::Jpeg { bytes: data, width: fmt.width, height: fmt.height }
    } else {
        Raw::Yuyv { data, width: fmt.width, height: fmt.height, stride: fmt.stride }
    })
}

#[cfg(not(target_os = "linux"))]
fn grab(_cfg: &CameraConfig) -> anyhow::Result<Raw> {
    bail!("相机只在 Linux（Jetson）上可用")
}

fn encode(img: DynamicImage, quality: u8, max_width: Option<u32>) -> anyhow::Result<Jpeg> {
    let img = match max_width {
        Some(w) if w > 0 && w < img.width() => {
            let h = (img.height() as u64 * w as u64 / img.width() as u64).max(1) as u32;
            DynamicImage::ImageRgb8(image::imageops::resize(&img.to_rgb8(), w, h, FilterType::Triangle))
        }
        _ => img,
    };
    let rgb = img.to_rgb8();
    let mut bytes = Vec::new();
    JpegEncoder::new_with_quality(&mut bytes, quality.clamp(1, 100)).encode_image(&rgb).context("JPEG 编码")?;
    Ok(Jpeg { bytes, width: rgb.width(), height: rgb.height() })
}

/// YUYV 4:2:2 → RGB（BT.601）。`stride` 为每行字节数，0 表示紧密排列。
pub fn yuyv_to_rgb(data: &[u8], width: u32, height: u32, stride: u32) -> anyhow::Result<RgbImage> {
    let stride = if stride == 0 { width * 2 } else { stride } as usize;
    let (w, h) = (width as usize, height as usize);
    if data.len() < stride * (h - 1) + w * 2 || w % 2 != 0 {
        bail!("YUYV 数据长度不对：{} 字节，{width}×{height}", data.len());
    }
    let mut out = RgbImage::new(width, height);
    let px = |y: f32, u: f32, v: f32| {
        let c = |x: f32| x.round().clamp(0.0, 255.0) as u8;
        [c(y + 1.402 * v), c(y - 0.344136 * u - 0.714136 * v), c(y + 1.772 * u)]
    };
    for row in 0..h {
        let line = &data[row * stride..row * stride + w * 2];
        for (k, q) in line.as_chunks::<4>().0.iter().enumerate() {
            let (y0, u, y1, v) = (q[0] as f32, q[1] as f32 - 128.0, q[2] as f32, q[3] as f32 - 128.0);
            out.put_pixel(2 * k as u32, row as u32, image::Rgb(px(y0, u, v)));
            out.put_pixel(2 * k as u32 + 1, row as u32, image::Rgb(px(y1, u, v)));
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn yuyv_gray_and_red() {
        // 一行 4 像素：两个灰（Y=128, U=V=128），两个红（BT.601 下约为 Y=76, U=85, V=255）
        let data = [128, 128, 128, 128, 76, 85, 76, 255];
        let img = yuyv_to_rgb(&data, 4, 1, 0).unwrap();
        assert_eq!(img.get_pixel(0, 0).0, [128, 128, 128]);
        let r = img.get_pixel(2, 0).0;
        assert!(r[0] > 240 && r[1] < 20 && r[2] < 20, "{r:?}");
    }

    #[test]
    fn yuyv_honors_stride() {
        let mut data = vec![0u8; 2 * 8];
        data[8..12].copy_from_slice(&[200, 128, 200, 128]);
        let img = yuyv_to_rgb(&data, 2, 2, 8).unwrap();
        assert_eq!(img.get_pixel(0, 1).0, [200, 200, 200]);
    }

    #[test]
    fn encode_resizes() {
        let img = DynamicImage::ImageRgb8(RgbImage::from_pixel(640, 480, image::Rgb([10, 200, 30])));
        let j = encode(img, 80, Some(320)).unwrap();
        assert_eq!((j.width, j.height), (320, 240));
        assert_eq!(&j.bytes[..2], &[0xFF, 0xD8]);
    }

    #[test]
    fn short_buffer_rejected() {
        assert!(yuyv_to_rgb(&[0; 6], 4, 1, 0).is_err());
    }
}
