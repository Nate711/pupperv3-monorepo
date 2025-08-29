use serde::{Deserialize, Serialize};
use std::fs;
use std::path::PathBuf;

#[derive(Debug, Deserialize, Serialize)]
pub struct Config {
    pub battery: BatteryConfig,
    pub service: ServiceConfig,
    pub blink: BlinkConfig,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct BatteryConfig {
    pub low_threshold: u8,
    pub poll_interval_secs: u64,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct ServiceConfig {
    pub poll_interval_secs: u64,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct BlinkConfig {
    pub interval_secs: f64,
    pub duration_secs: f64,
    pub eye_delay_secs: f64,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            battery: BatteryConfig {
                low_threshold: 15,
                poll_interval_secs: 5,
            },
            service: ServiceConfig {
                poll_interval_secs: 1,
            },
            blink: BlinkConfig {
                interval_secs: 4.0,
                duration_secs: 0.3,
                eye_delay_secs: 0.08,
            },
        }
    }
}

pub fn load_config() -> Result<Config, String> {
    // Use CARGO_MANIFEST_DIR to find config.toml relative to the Cargo.toml file
    // This is set at compile time and always points to the crate root
    // let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let manifest_dir = ".";
    let config_path = PathBuf::from(manifest_dir).join("config.toml");
    
    let contents = fs::read_to_string(&config_path)
        .map_err(|e| format!("Failed to read config file '{}': {}", config_path.display(), e))?;
    
    let config = toml::from_str(&contents)
        .map_err(|e| format!("Failed to parse config file '{}': {}", config_path.display(), e))?;
    
    println!("Loaded config from {}", config_path.display());
    Ok(config)
}

pub fn print_config_info(config: &Config) {
    println!("Battery low threshold: {}%", config.battery.low_threshold);
    println!("Battery poll interval: {}s", config.battery.poll_interval_secs);
    println!("Service poll interval: {}s", config.service.poll_interval_secs);
    println!("Blink interval: {}s", config.blink.interval_secs);
    println!("Blink duration: {}s", config.blink.duration_secs);
    println!("Eye delay: {}s", config.blink.eye_delay_secs);
}