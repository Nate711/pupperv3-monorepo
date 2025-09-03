use crate::config::ServiceConfig;
use std::process::Command;
use std::time::{Duration, Instant};

#[derive(Debug, Clone)]
pub enum ServiceStatus {
    Active,
    Inactive,
    Unknown,
}

#[derive(Debug, Clone)]
pub enum LlmServiceStatus {
    Active,
    Inactive,
    Unknown,
}

pub struct ServiceMonitor {
    pub status: ServiceStatus,
    pub last_check: Instant,
}

pub struct LlmServiceMonitor {
    pub status: LlmServiceStatus,
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
        if self.last_check.elapsed() >= Duration::from_secs_f32(config.poll_interval_secs) {
            self.status = query_robot_status();
            self.last_check = Instant::now();
        }
    }

    pub fn get_status(&self) -> &ServiceStatus {
        &self.status
    }
}

impl LlmServiceMonitor {
    pub fn new() -> Self {
        Self {
            status: LlmServiceStatus::Unknown,
            last_check: Instant::now(),
        }
    }

    pub fn update(&mut self, config: &ServiceConfig) {
        if self.last_check.elapsed() >= Duration::from_secs_f32(config.poll_interval_secs) {
            self.status = query_llm_status();
            self.last_check = Instant::now();
        }
    }

    pub fn get_status(&self) -> &LlmServiceStatus {
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
            println!("Robot service status: {}", stdout.trim());
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

fn query_llm_status() -> LlmServiceStatus {
    match Command::new("systemctl")
        .args(["is-active", "llm-agent"])
        .output()
    {
        Ok(output) => {
            let stdout = String::from_utf8_lossy(&output.stdout);
            println!("LLM service status: {}", stdout.trim());
            match stdout.trim() {
                "active" => LlmServiceStatus::Active,
                "inactive" | "failed" => LlmServiceStatus::Inactive,
                _ => LlmServiceStatus::Unknown,
            }
        }
        Err(_) => {
            println!("Failed to execute systemctl command for LLM service");
            LlmServiceStatus::Unknown
        }
    }
}
