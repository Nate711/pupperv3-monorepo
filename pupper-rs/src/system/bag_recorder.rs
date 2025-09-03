use crate::config::BagRecorderConfig;
use std::process::Command;
use std::time::{Duration, Instant};

#[derive(Debug, Clone)]
pub enum BagRecorderStatus {
    Recording,
    Stopped,
    Unknown,
}

pub struct BagRecorderMonitor {
    pub status: BagRecorderStatus,
    pub last_check: Instant,
}

impl BagRecorderMonitor {
    pub fn new() -> Self {
        Self {
            status: BagRecorderStatus::Unknown,
            last_check: Instant::now(),
        }
    }

    pub fn update(&mut self, config: &BagRecorderConfig) {
        if self.last_check.elapsed() >= Duration::from_secs_f32(config.poll_interval_secs) {
            self.status = query_bag_recording_status();
            self.last_check = Instant::now();
        }
    }

    pub fn get_status(&self) -> &BagRecorderStatus {
        &self.status
    }
}

fn query_bag_recording_status() -> BagRecorderStatus {
    // Check if any ros2 bag record processes are running
    match Command::new("pgrep")
        .args(["-f", "ros2 bag record"])
        .output()
    {
        Ok(output) => {
            if output.status.success() && !output.stdout.is_empty() {
                println!("Bag recording process found");
                BagRecorderStatus::Recording
            } else {
                println!("No bag recording process found");
                BagRecorderStatus::Stopped
            }
        }
        Err(_) => {
            println!("Failed to check for bag recording process");
            BagRecorderStatus::Unknown
        }
    }
}
