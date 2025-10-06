use serde::{Deserialize, Serialize};
use std::fs;
use std::path::PathBuf;

#[derive(Debug, Deserialize, Serialize)]
pub struct Config {
    pub battery: BatteryConfig,
    #[serde(default)]
    pub bag_recorder: BagRecorderConfig,
    pub cpu: CpuConfig,
    pub service: ServiceConfig,
    pub blink: BlinkConfig,
    pub eye_tracking: EyeTrackingConfig,
    #[serde(default)]
    pub ui: UiConfig,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct BatteryConfig {
    pub low_threshold: u8,
    pub poll_interval_secs: f32,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct BagRecorderConfig {
    pub poll_interval_secs: f32,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct CpuConfig {
    pub enabled: bool,
    pub poll_interval_secs: f32,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct ServiceConfig {
    pub poll_interval_secs: f32,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct BlinkConfig {
    pub interval_secs: f64,
    pub duration_secs: f64,
    pub eye_delay_secs: f64,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct EyeTrackingConfig {
    pub enabled: bool,
    pub mode: EyeTrackingMode,
    pub source: EyeTrackingSource,
    pub sensitivity: f32,
    pub max_pupil_offset: f32,
    pub max_eye_offset: f32,
    pub combined_pupil_ratio: f32,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct UiConfig {
    // When true, show a visible button to toggle the top bar; when false, use an invisible hotzone
    pub toggle_button_visible: bool,
}

#[derive(Debug, Deserialize, Serialize, Clone, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum EyeTrackingMode {
    PupilsOnly,
    WholeEye,
    Combined,
}

#[derive(Debug, Deserialize, Serialize, Clone, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum EyeTrackingSource {
    Mouse,
    Person, // Track the largest detected person
}

impl Default for Config {
    fn default() -> Self {
        Config {
            battery: BatteryConfig {
                low_threshold: 15,
                poll_interval_secs: 5.0,
            },
            bag_recorder: BagRecorderConfig::default(),
            cpu: CpuConfig {
                enabled: true,
                poll_interval_secs: 2.0,
            },
            service: ServiceConfig {
                poll_interval_secs: 1.0,
            },
            blink: BlinkConfig {
                interval_secs: 4.0,
                duration_secs: 0.3,
                eye_delay_secs: 0.08,
            },
            eye_tracking: EyeTrackingConfig {
                enabled: true,
                mode: EyeTrackingMode::Combined,
                source: EyeTrackingSource::Mouse,
                sensitivity: 0.3,
                max_pupil_offset: 25.0,
                max_eye_offset: 50.0,
                combined_pupil_ratio: 0.5,
            },
            ui: UiConfig::default(),
        }
    }
}

impl Default for BagRecorderConfig {
    fn default() -> Self {
        BagRecorderConfig {
            poll_interval_secs: 2.0,
        }
    }
}

impl Default for UiConfig {
    fn default() -> Self {
        UiConfig {
            toggle_button_visible: false,
        }
    }
}

pub fn load_config() -> Result<Config, String> {
    // Use CARGO_MANIFEST_DIR to find config.toml relative to the Cargo.toml file
    // This is set at compile time and always points to the crate root
    // let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let manifest_dir = ".";
    let config_path = PathBuf::from(manifest_dir).join("config.toml");

    let contents = fs::read_to_string(&config_path).map_err(|e| {
        format!(
            "Failed to read config file '{}': {}",
            config_path.display(),
            e
        )
    })?;

    let config = toml::from_str(&contents).map_err(|e| {
        format!(
            "Failed to parse config file '{}': {}",
            config_path.display(),
            e
        )
    })?;

    println!("Loaded config from {}", config_path.display());
    Ok(config)
}

pub fn print_config_info(config: &Config) {
    println!("Configuration: {:#?}", config);
}
