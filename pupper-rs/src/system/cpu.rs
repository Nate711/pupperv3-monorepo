use crate::config::CpuConfig;
use std::fs;
use std::time::Instant;

pub struct CpuMonitor {
    last_update: Instant,
    pub usage: Option<f32>,
    pub temperature: Option<f32>,
    prev_idle: u64,
    prev_total: u64,
    enabled: bool,
}

impl CpuMonitor {
    pub fn new() -> Self {
        Self {
            last_update: Instant::now(),
            usage: None,
            temperature: None,
            prev_idle: 0,
            prev_total: 0,
            enabled: true,
        }
    }

    pub fn update(&mut self, config: &CpuConfig) {
        self.enabled = config.enabled;

        if !config.enabled {
            self.usage = None;
            self.temperature = None;
            return;
        }

        if self.last_update.elapsed().as_secs_f32() < config.poll_interval_secs {
            return;
        }
        self.last_update = Instant::now();

        // Update CPU usage
        if let Ok(stat) = fs::read_to_string("/proc/stat") {
            if let Some(line) = stat.lines().next() {
                let values: Vec<&str> = line.split_whitespace().collect();
                if values.len() > 4 && values[0] == "cpu" {
                    let user: u64 = values[1].parse().unwrap_or(0);
                    let nice: u64 = values[2].parse().unwrap_or(0);
                    let system: u64 = values[3].parse().unwrap_or(0);
                    let idle: u64 = values[4].parse().unwrap_or(0);
                    let iowait: u64 = values.get(5).and_then(|v| v.parse().ok()).unwrap_or(0);
                    let irq: u64 = values.get(6).and_then(|v| v.parse().ok()).unwrap_or(0);
                    let softirq: u64 = values.get(7).and_then(|v| v.parse().ok()).unwrap_or(0);

                    let idle_time = idle + iowait;
                    let non_idle = user + nice + system + irq + softirq;
                    let total = idle_time + non_idle;

                    if self.prev_total > 0 {
                        let total_delta = total - self.prev_total;
                        let idle_delta = idle_time - self.prev_idle;

                        if total_delta > 0 {
                            let usage = 100.0 * (1.0 - (idle_delta as f32 / total_delta as f32));
                            self.usage = Some(usage.clamp(0.0, 100.0));
                        }
                    }

                    self.prev_idle = idle_time;
                    self.prev_total = total;
                }
            }
        }

        // Update CPU temperature - try common thermal zone paths
        let temp_paths = vec![
            "/sys/class/thermal/thermal_zone0/temp",
            "/sys/class/thermal/thermal_zone1/temp",
            "/sys/devices/platform/coretemp.0/hwmon/hwmon0/temp1_input",
            "/sys/class/hwmon/hwmon0/temp1_input",
        ];

        for path in temp_paths {
            if let Ok(temp_str) = fs::read_to_string(path) {
                if let Ok(temp_millidegrees) = temp_str.trim().parse::<f32>() {
                    self.temperature = Some(temp_millidegrees / 1000.0);
                    break;
                }
            }
        }
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
}
