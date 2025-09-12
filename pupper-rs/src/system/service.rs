use crate::config::ServiceConfig;
use std::fs;
use std::path::Path;
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
    Loading,  // Service is up but agent not yet confirmed
    Unknown,
}

pub struct ServiceMonitor {
    pub status: ServiceStatus,
    pub last_check: Instant,
}

pub struct LlmServiceMonitor {
    pub status: LlmServiceStatus,
    pub last_check: Instant,
    agent_confirmed: bool,  // Track if we've seen the agent start
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
            agent_confirmed: false,
        }
    }

    pub fn update(&mut self, config: &ServiceConfig) {
        if self.last_check.elapsed() >= Duration::from_secs_f32(config.poll_interval_secs) {
            let (new_status, agent_detected) = query_llm_status();
            
            // If agent was detected, mark it as confirmed
            if agent_detected {
                self.agent_confirmed = true;
            }
            
            // Update status based on systemctl and agent confirmation
            self.status = match new_status {
                LlmServiceStatus::Active if self.agent_confirmed => LlmServiceStatus::Active,
                LlmServiceStatus::Active => LlmServiceStatus::Loading, // Service up but agent not confirmed - show loading
                LlmServiceStatus::Inactive => {
                    // Service is down, reset agent confirmation
                    self.agent_confirmed = false;
                    LlmServiceStatus::Inactive
                }
                _ => LlmServiceStatus::Unknown,
            };
            
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

fn query_llm_status() -> (LlmServiceStatus, bool) {
    // First check systemctl status
    let systemctl_status = match Command::new("systemctl")
        .args(["is-active", "llm-agent"])
        .output()
    {
        Ok(output) => {
            let stdout = String::from_utf8_lossy(&output.stdout);
            println!("LLM service systemctl status: {}", stdout.trim());
            stdout.trim().to_string()
        }
        Err(_) => {
            println!("Failed to execute systemctl command for LLM service");
            return (LlmServiceStatus::Unknown, false);
        }
    };
    
    // Check for the pupster_agent_started file
    let marker_file = Path::new("/tmp/pupster_agent_started");
    let file_was_detected = marker_file.exists();
    
    // If file exists, always delete it (it's a one-time signal)
    if file_was_detected {
        if let Err(e) = fs::remove_file(marker_file) {
            println!("Failed to delete marker file: {}", e);
        } else {
            println!("Detected and deleted /tmp/pupster_agent_started");
        }
    }
    
    // Return systemctl status and whether file was detected
    let status = match systemctl_status.as_str() {
        "active" => LlmServiceStatus::Active,
        "inactive" | "failed" => LlmServiceStatus::Inactive,
        _ => LlmServiceStatus::Unknown,
    };
    
    (status, file_was_detected)
}
