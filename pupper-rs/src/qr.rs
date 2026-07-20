//! QR-based WiFi pairing: receive camera frames over ZMQ (published by the
//! hailo detection node on port 5557), decode a WiFi QR code from them, and
//! connect via the pupper-pairing helper. Also generates a QR code pointing at
//! the qifi.org generator so a phone can be aimed at Pupper's screen.

use eframe::egui::ColorImage;
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

/// WiFi credentials parsed from a `WIFI:...;;` QR payload (qifi.org format).
#[derive(Debug, Clone)]
pub struct WifiCreds {
    pub ssid: String,
    pub password: String,
    pub auth: String, // "WPA", "WEP", or "nopass"
    pub hidden: bool,
}

fn unescape(s: &str) -> String {
    let mut out = String::new();
    let mut chars = s.chars();
    while let Some(c) = chars.next() {
        if c == '\\' {
            if let Some(n) = chars.next() {
                out.push(n);
            }
        } else {
            out.push(c);
        }
    }
    out
}

/// Parse the standard WiFi QR payload, e.g. `WIFI:S:MyNet;T:WPA;P:secret;H:false;;`.
/// Fields are `;`-separated `KEY:VALUE`; values may backslash-escape `;,:\`.
pub fn parse_wifi_string(s: &str) -> Option<WifiCreds> {
    let body = s
        .strip_prefix("WIFI:")
        .or_else(|| s.strip_prefix("wifi:"))?;

    // Split into fields on unescaped ';'.
    let mut fields: Vec<String> = Vec::new();
    let mut cur = String::new();
    let mut chars = body.chars();
    while let Some(c) = chars.next() {
        if c == '\\' {
            cur.push(c);
            if let Some(n) = chars.next() {
                cur.push(n);
            }
        } else if c == ';' {
            fields.push(std::mem::take(&mut cur));
        } else {
            cur.push(c);
        }
    }
    if !cur.is_empty() {
        fields.push(cur);
    }

    let mut ssid: Option<String> = None;
    let mut password = String::new();
    let mut auth = String::from("WPA");
    let mut hidden = false;
    for f in fields {
        // Key is up to the first ':' (keys never contain escapes); the rest is
        // the (possibly escaped) value.
        let Some((k, v)) = f.split_once(':') else {
            continue;
        };
        match k {
            "S" => ssid = Some(unescape(v)),
            "P" => password = unescape(v),
            "T" => auth = if v.is_empty() { "nopass".into() } else { v.to_string() },
            "H" => hidden = v.eq_ignore_ascii_case("true"),
            _ => {}
        }
    }

    let ssid = ssid?;
    if ssid.is_empty() {
        return None;
    }
    Some(WifiCreds {
        ssid,
        password,
        auth,
        hidden,
    })
}

/// Decode a JPEG frame into an egui ColorImage for on-screen display.
pub fn jpeg_to_color_image(jpeg: &[u8]) -> Option<ColorImage> {
    let img = image::load_from_memory(jpeg).ok()?.to_rgba8();
    let (w, h) = img.dimensions();
    Some(ColorImage::from_rgba_unmultiplied(
        [w as usize, h as usize],
        img.as_raw(),
    ))
}

/// Try to decode a WiFi QR code from a JPEG frame.
pub fn decode_wifi_from_jpeg(jpeg: &[u8]) -> Option<WifiCreds> {
    let gray = image::load_from_memory(jpeg).ok()?.to_luma8();
    let mut prepared = rqrr::PreparedImage::prepare(gray);
    for grid in prepared.detect_grids() {
        if let Ok((_meta, content)) = grid.decode() {
            if let Some(creds) = parse_wifi_string(&content) {
                return Some(creds);
            }
        }
    }
    None
}

/// Render `text` as a QR code, upscaled, into an egui ColorImage (black on white).
pub fn qr_color_image(text: &str, scale: usize, quiet: usize) -> Option<ColorImage> {
    let code = qrcode::QrCode::new(text.as_bytes()).ok()?;
    let width = code.width();
    let colors = code.to_colors();
    let dim = (width + 2 * quiet) * scale;
    let mut pixels = vec![255u8; dim * dim * 4]; // white RGBA
    for y in 0..width {
        for x in 0..width {
            if colors[y * width + x] == qrcode::Color::Dark {
                for dy in 0..scale {
                    for dx in 0..scale {
                        let px = (x + quiet) * scale + dx;
                        let py = (y + quiet) * scale + dy;
                        let idx = (py * dim + px) * 4;
                        pixels[idx] = 0;
                        pixels[idx + 1] = 0;
                        pixels[idx + 2] = 0;
                        pixels[idx + 3] = 255;
                    }
                }
            }
        }
    }
    Some(ColorImage::from_rgba_unmultiplied([dim, dim], &pixels))
}

/// Progress of a QR-scan connect attempt.
#[derive(Debug, Clone)]
pub enum ScanStatus {
    Scanning,
    Connecting(String),
    Connected(String),
    Failed(String),
}

/// Subscribes to the camera JPEG stream on ZMQ 5557 while alive. Dropping it
/// stops the thread. Only created while the scan view is open, so there's no
/// frame-decoding cost during normal operation.
pub struct FrameReceiver {
    latest: Arc<Mutex<Option<Vec<u8>>>>,
    running: Arc<AtomicBool>,
    handle: Option<thread::JoinHandle<()>>,
}

impl FrameReceiver {
    pub fn start() -> Self {
        let latest = Arc::new(Mutex::new(None));
        let running = Arc::new(AtomicBool::new(true));
        let l = Arc::clone(&latest);
        let r = Arc::clone(&running);
        let handle = thread::spawn(move || {
            let ctx = zmq::Context::new();
            let sock = match ctx.socket(zmq::SUB) {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("qr: failed to make zmq socket: {e}");
                    return;
                }
            };
            let _ = sock.set_conflate(true); // keep only the newest frame
            if let Err(e) = sock.connect("tcp://127.0.0.1:5557") {
                eprintln!("qr: failed to connect to frame stream: {e}");
                return;
            }
            let _ = sock.set_subscribe(b"");
            let _ = sock.set_rcvtimeo(200);
            while r.load(Ordering::Relaxed) {
                match sock.recv_bytes(0) {
                    Ok(buf) => {
                        if let Ok(mut g) = l.lock() {
                            *g = Some(buf);
                        }
                    }
                    Err(zmq::Error::EAGAIN) => {}
                    Err(e) => {
                        eprintln!("qr: frame recv error: {e}");
                        thread::sleep(Duration::from_millis(100));
                    }
                }
            }
        });
        Self {
            latest,
            running,
            handle: Some(handle),
        }
    }

    pub fn latest(&self) -> Option<Vec<u8>> {
        self.latest.lock().ok().and_then(|g| g.clone())
    }
}

impl Drop for FrameReceiver {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_wpa() {
        let c = parse_wifi_string("WIFI:S:MyNet;T:WPA;P:s3cret;;").unwrap();
        assert_eq!(c.ssid, "MyNet");
        assert_eq!(c.password, "s3cret");
        assert_eq!(c.auth, "WPA");
        assert!(!c.hidden);
    }

    #[test]
    fn parses_open_and_hidden() {
        let c = parse_wifi_string("WIFI:S:Cafe;T:nopass;P:;H:true;;").unwrap();
        assert_eq!(c.ssid, "Cafe");
        assert_eq!(c.password, "");
        assert_eq!(c.auth, "nopass");
        assert!(c.hidden);
    }

    #[test]
    fn handles_escapes() {
        let c = parse_wifi_string(r"WIFI:S:My\;Net;T:WPA;P:a\:b\\c;;").unwrap();
        assert_eq!(c.ssid, "My;Net");
        assert_eq!(c.password, r"a:b\c");
    }

    #[test]
    fn rejects_non_wifi() {
        assert!(parse_wifi_string("https://qifi.org/").is_none());
        assert!(parse_wifi_string("WIFI:T:WPA;P:x;;").is_none()); // no SSID
    }
}

/// Connect to a network from scanned credentials in a background thread,
/// reporting progress through `status`.
pub fn connect_async(creds: WifiCreds, status: Arc<Mutex<ScanStatus>>) {
    thread::spawn(move || {
        if let Ok(mut s) = status.lock() {
            *s = ScanStatus::Connecting(creds.ssid.clone());
        }
        let mut cmd = Command::new("sudo");
        cmd.args(["-n", "/usr/local/bin/pupper-pairing", "add", &creds.ssid]);
        // Empty password => open network; the helper omits the password arg.
        let psk = if creds.auth.eq_ignore_ascii_case("nopass") {
            ""
        } else {
            creds.password.as_str()
        };
        cmd.arg(psk);
        if creds.hidden {
            cmd.arg("hidden");
        }
        let result = cmd.output();
        if let Ok(mut s) = status.lock() {
            *s = match result {
                Ok(o) if o.status.success() => ScanStatus::Connected(creds.ssid.clone()),
                Ok(o) => {
                    let msg = String::from_utf8_lossy(&o.stderr);
                    ScanStatus::Failed(msg.trim().to_string())
                }
                Err(e) => ScanStatus::Failed(e.to_string()),
            };
        }
    });
}
