use std::process::Command;
use std::time::{Duration, Instant};

/// Current WiFi pairing/hotspot state, as reported by `pupper-pairing status`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PairingMode {
    /// Connected to a normal WiFi network.
    Connected,
    /// The Pupper-Setup-<nnn> pairing hotspot is up (comitup HOTSPOT).
    Hotspot,
    /// wlan0 is down / not connected.
    Disconnected,
    Unknown,
}

/// Polls the `pupper-pairing` helper on a throttled interval so the GUI can
/// show whether Pupper is currently in pairing mode. Mirrors the other
/// system monitors (see service.rs / network.rs).
pub struct PairingMonitor {
    pub mode: PairingMode,
    last_check: Instant,
    poll_interval: Duration,
}

impl PairingMonitor {
    pub fn new() -> Self {
        Self {
            mode: PairingMode::Unknown,
            last_check: Instant::now(),
            poll_interval: Duration::from_secs(5),
        }
    }

    pub fn update(&mut self) {
        if self.last_check.elapsed() >= self.poll_interval {
            self.mode = query_mode();
            self.last_check = Instant::now();
        }
    }

    pub fn get_mode(&self) -> PairingMode {
        self.mode
    }
}

impl Default for PairingMonitor {
    fn default() -> Self {
        Self::new()
    }
}

fn query_mode() -> PairingMode {
    // `sudo -n` never prompts: a scoped NOPASSWD rule authorizes this helper.
    match Command::new("sudo")
        .args(["-n", "/usr/local/bin/pupper-pairing", "status"])
        .output()
    {
        Ok(out) => {
            let s = String::from_utf8_lossy(&out.stdout);
            if s.contains("MODE=hotspot") {
                PairingMode::Hotspot
            } else if s.contains("MODE=connected") {
                PairingMode::Connected
            } else if s.contains("MODE=disconnected") {
                PairingMode::Disconnected
            } else {
                PairingMode::Unknown
            }
        }
        Err(_) => PairingMode::Unknown,
    }
}

/// Toggle pairing mode. Runs the helper in a detached thread so dropping WiFi
/// (which can take a moment) never blocks the UI thread.
pub fn set_pairing(on: bool) {
    let sub = if on { "on" } else { "off" };
    std::thread::spawn(move || {
        match Command::new("sudo")
            .args(["-n", "/usr/local/bin/pupper-pairing", sub])
            .status()
        {
            Ok(s) if s.success() => {}
            Ok(s) => eprintln!("pupper-pairing {sub} exited with status {s}"),
            Err(e) => eprintln!("failed to run pupper-pairing {sub}: {e}"),
        }
    });
}
