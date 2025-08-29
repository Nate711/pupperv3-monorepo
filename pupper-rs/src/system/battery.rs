use crate::config::BatteryConfig;
use std::process::Command;
use std::time::{Duration, Instant};

pub struct BatteryMonitor {
    pub percentage: Option<u8>,
    pub last_check: Instant,
    pub flash_battery: bool,
    pub flash_timer: Instant,
}

impl BatteryMonitor {
    pub fn new() -> Self {
        Self {
            percentage: None,
            last_check: Instant::now(),
            flash_battery: false,
            flash_timer: Instant::now(),
        }
    }

    pub fn update(&mut self, config: &BatteryConfig) {
        // Poll battery status based on configured interval
        if self.last_check.elapsed() >= Duration::from_secs(config.poll_interval_secs) {
            self.percentage = query_battery_percentage();
            println!("Battery percentage: {:?}", self.percentage);
            self.last_check = Instant::now();
        }
        
        // Handle flashing for low battery
        if let Some(percentage) = self.percentage {
            if percentage < config.low_threshold {
                if self.flash_timer.elapsed() >= Duration::from_millis(500) {
                    self.flash_battery = !self.flash_battery;
                    self.flash_timer = Instant::now();
                }
            } else {
                self.flash_battery = false;
            }
        }
    }

    pub fn should_flash(&self) -> bool {
        self.flash_battery
    }
}

fn query_battery_percentage() -> Option<u8> {
    // Try to get the absolute path to the script
    let home = std::env::var("HOME").ok()?;
    let script_path = format!("{}/pupperv3-monorepo/robot/utils/check_batt_voltage.py", home);
    
    match Command::new("python3")
        .args([&script_path, "--percentage_only"])
        .output()
    {
        Ok(output) => {
            if output.status.success() {
                let stdout = String::from_utf8_lossy(&output.stdout);
                stdout.trim().parse::<u8>().ok()
            } else {
                println!("Battery script failed");
                None
            }
        }
        Err(e) => {
            println!("Failed to execute battery check: {}", e);
            None
        }
    }
}