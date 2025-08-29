use crate::config::ServiceConfig;
use std::process::Command;
use std::time::{Duration, Instant};

#[derive(Debug, Clone)]
pub enum ServiceStatus {
    Active,
    Inactive,
    Unknown,
}

pub struct ServiceMonitor {
    pub status: ServiceStatus,
    pub last_check: Instant,
}

impl ServiceMonitor {
    pub fn new() -> Self {
        Self {
            status: ServiceStatus::Unknown,
            last_check: Instant::now(),
        }
    }

    pub fn update(&mut self, config: &ServiceConfig) {
        if self.last_check.elapsed() >= Duration::from_secs(config.poll_interval_secs) {
            self.status = query_robot_status();
            self.last_check = Instant::now();
        }
    }

    pub fn get_status(&self) -> &ServiceStatus {
        &self.status
    }
}

fn query_robot_status() -> ServiceStatus {
    match Command::new("systemctl")
        .args(["is-active", "robot"])
        .output()
    {
        Ok(output) => {
            let stdout = String::from_utf8_lossy(&output.stdout);
            println!("Service status: {}", stdout.trim());
            match stdout.trim() {
                "active" => ServiceStatus::Active,
                "inactive" | "failed" => ServiceStatus::Inactive,
                _ => ServiceStatus::Unknown,
            }
        }
        Err(_) => {
            println!("Failed to execute systemctl command");
            ServiceStatus::Unknown
        }
    }
}