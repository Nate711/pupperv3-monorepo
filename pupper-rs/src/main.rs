use eframe::{App, egui};
use egui::{Color32, Pos2, RichText, Shape, Stroke, Vec2};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::PathBuf;
use std::process::Command;
use std::time::{Duration, Instant};

#[derive(Debug, Deserialize, Serialize)]
struct Config {
    battery: BatteryConfig,
    service: ServiceConfig,
}

#[derive(Debug, Deserialize, Serialize)]
struct BatteryConfig {
    low_threshold: u8,
    poll_interval_secs: u64,
}

#[derive(Debug, Deserialize, Serialize)]
struct ServiceConfig {
    poll_interval_secs: u64,
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
        }
    }
}

enum ServiceStatus {
    Active,
    Inactive,
    Unknown,
}

struct ImageApp {
    service_status: ServiceStatus,
    last_check: Instant,
    battery_percentage: Option<u8>,
    last_battery_check: Instant,
    flash_battery: bool,
    flash_timer: Instant,
    config: Config,
}

fn load_config() -> Result<Config, String> {
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

impl ImageApp {
    fn new(_cc: &eframe::CreationContext<'_>) -> Result<Self, String> {
        let config = load_config()?;
        println!("Battery low threshold: {}%", config.battery.low_threshold);
        println!("Battery poll interval: {}s", config.battery.poll_interval_secs);
        println!("Service poll interval: {}s", config.service.poll_interval_secs);
        
        Ok(Self {
            service_status: ServiceStatus::Unknown,
            last_check: Instant::now(),
            battery_percentage: None,
            last_battery_check: Instant::now(),
            flash_battery: false,
            flash_timer: Instant::now(),
            config,
        })
    }

    fn poll_service_status(&mut self) {
        if self.last_check.elapsed() >= Duration::from_secs(self.config.service.poll_interval_secs) {
            self.service_status = query_robot_status();
            self.last_check = Instant::now();
        }
    }

    fn poll_battery_status(&mut self) {
        if self.last_battery_check.elapsed() >= Duration::from_secs(self.config.battery.poll_interval_secs) {
            self.battery_percentage = query_battery_percentage();
            println!("Battery percentage: {:?}", self.battery_percentage);
            self.last_battery_check = Instant::now();
        }
        
        // Handle flashing for low battery
        if let Some(percentage) = self.battery_percentage {
            if percentage < self.config.battery.low_threshold {
                if self.flash_timer.elapsed() >= Duration::from_millis(500) {
                    self.flash_battery = !self.flash_battery;
                    self.flash_timer = Instant::now();
                }
            } else {
                self.flash_battery = false;
            }
        }
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

fn quadratic_bezier_points(start: Pos2, ctrl: Pos2, end: Pos2, steps: usize) -> Vec<Pos2> {
    (0..=steps)
        .map(|i| {
            let t = i as f32 / steps as f32;
            let u = 1.0 - t;
            Pos2 {
                x: u * u * start.x + 2.0 * u * t * ctrl.x + t * t * end.x,
                y: u * u * start.y + 2.0 * u * t * ctrl.y + t * t * end.y,
            }
        })
        .collect()
}

fn draw_eye(painter: &egui::Painter, center: Pos2) {
    // Palette (tuned to the reference)
    let ring_outer = Color32::from_rgb(0x2a, 0x2f, 0x36); // dark gray ring
    let ring_blue = Color32::from_rgb(075, 149, 181); // bright blue ring
    let under_highlight_blue = Color32::from_rgb(50, 102, 136); // cyan-blue
    let gloss = Color32::from_rgba_premultiplied(255, 255, 255, (0.95 * 255.0) as u8);

    // Outer dark ring
    painter.add(Shape::circle_stroke(
        center,
        140.0,
        Stroke::new(8.0, ring_outer),
    ));
    // Thick blue bezel
    painter.add(Shape::circle_stroke(
        center,
        116.0,
        Stroke::new(30.0, ring_blue),
    ));

    // Pupil / eye interior
    painter.add(Shape::circle_filled(center, 90.0, Color32::BLACK));

    // under highlight (arc)
    let radius = 80.0;
    let start_angle = 45.0_f32.to_radians();
    let end_angle = 135.0_f32.to_radians();
    let steps = 28;
    let arc_pts = (0..=steps)
        .map(|i| {
            let t = i as f32 / steps as f32;
            let angle = start_angle + t * (end_angle - start_angle);
            center + Vec2::new(radius * angle.cos(), radius * angle.sin())
        })
        .collect::<Vec<_>>();
    painter.add(Shape::line(
        arc_pts,
        Stroke::new(14.0, under_highlight_blue),
    ));
    // Little blue dot at the right end
    painter.add(Shape::circle_filled(
        center + Vec2::new(76.0, 34.0),
        8.0,
        under_highlight_blue,
    ));

    // Gloss highlights (top-left)
    painter.add(Shape::circle_filled(
        center + Vec2::new(-42.0, -54.0),
        26.0,
        gloss,
    ));
    painter.add(Shape::circle_filled(
        center + Vec2::new(-70.0, -8.0),
        12.0,
        gloss,
    ));

    // Eyebrow (slight arch)
    let brow_col = Color32::from_rgb(0x33, 0x36, 0x3c);
    let brow_start = center + Vec2::new(-88.0, -150.0);
    let brow_ctrl = center + Vec2::new(0.0, -195.0);
    let brow_end = center + Vec2::new(88.0, -150.0);
    let brow_pts = quadratic_bezier_points(brow_start, brow_ctrl, brow_end, 24);
    painter.add(Shape::line(brow_pts, Stroke::new(14.0, brow_col)));
}

impl App for ImageApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint_after(Duration::from_secs(1));
        println!("Redrawing UI");
        self.poll_service_status();
        self.poll_battery_status();

        egui::CentralPanel::default()
            .frame(egui::Frame::none().fill(Color32::BLACK))
            .show(ctx, |ui| {
                // Use the whole panel; place eyes relative to the center
                let rect = ui.max_rect();
                let painter = ui.painter();

                let center = rect.center();
                // Horizontal spacing between eyes
                let offset_x = 190.0;
                // Slight vertical offset so they sit a bit high in the frame
                let offset_y = -10.0;

                draw_eye(&painter, center + Vec2::new(-offset_x, offset_y));
                draw_eye(&painter, center + Vec2::new(offset_x, offset_y));
            });

        egui::Area::new(egui::Id::new("service_status"))
            .anchor(egui::Align2::RIGHT_TOP, [-30.0, 30.0])
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    // Draw status icon
                    let (color, symbol, text) = match self.service_status {
                        ServiceStatus::Active => (Color32::from_rgb(34, 197, 94), "✓", "Robot up"),
                        ServiceStatus::Inactive => (Color32::from_rgb(239, 68, 68), "✗", "Robot down"),
                        ServiceStatus::Unknown => (Color32::GRAY, "?", "Robot status unknown"),
                    };
                    
                    // Draw colored circle with symbol
                    let radius = 10.0;
                    let (response, painter) = ui.allocate_painter(Vec2::new(radius * 2.0, radius * 2.0), egui::Sense::hover());
                    let rect = response.rect;
                    let center = rect.center();
                    
                    // Draw filled circle
                    painter.circle_filled(center, radius, color);
                    
                    // Draw symbol in black
                    painter.text(
                        center,
                        egui::Align2::CENTER_CENTER,
                        symbol,
                        egui::FontId::proportional(14.0),
                        Color32::BLACK,
                    );
                    
                    // Add some spacing and then the text
                    ui.add_space(1.0);
                    ui.label(RichText::new(text).color(Color32::WHITE).size(14.0));
                });
            });
        
        // Battery indicator in top-left
        egui::Area::new(egui::Id::new("battery_status"))
            .anchor(egui::Align2::LEFT_TOP, [30.0, 30.0])
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    if let Some(percentage) = self.battery_percentage {
                        // Determine if we should show red (flashing or solid)
                        let is_low = percentage < self.config.battery.low_threshold;
                        let show_red = is_low && self.flash_battery;
                        
                        // Percentage text
                        let text_color = if show_red {
                            Color32::from_rgb(239, 68, 68)
                        } else {
                            Color32::WHITE
                        };
                        ui.label(RichText::new(format!("{}%", percentage))
                            .color(text_color)
                            .size(14.0));
                        
                        ui.add_space(4.0);
                        
                        // Battery icon
                        let battery_width = 30.0;
                        let battery_height = 16.0;
                        let terminal_width = 3.0;
                        let terminal_height = 8.0;
                        
                        let (response, painter) = ui.allocate_painter(
                            Vec2::new(battery_width + terminal_width, battery_height),
                            egui::Sense::hover()
                        );
                        let rect = response.rect;
                        
                        // Battery outline color
                        let outline_color = if show_red {
                            Color32::from_rgb(239, 68, 68)
                        } else {
                            Color32::GRAY
                        };
                        
                        // Draw battery body outline
                        let battery_rect = egui::Rect::from_min_size(
                            rect.min,
                            Vec2::new(battery_width, battery_height)
                        );
                        painter.rect_stroke(
                            battery_rect,
                            2.0,
                            Stroke::new(2.0, outline_color)
                        );
                        
                        // Draw battery terminal
                        let terminal_rect = egui::Rect::from_min_size(
                            rect.min + Vec2::new(battery_width, (battery_height - terminal_height) / 2.0),
                            Vec2::new(terminal_width, terminal_height)
                        );
                        painter.rect_filled(terminal_rect, 0.0, outline_color);
                        
                        // Draw battery fill
                        let padding = 2.0;
                        let fill_width = (battery_width - 2.0 * padding) * (percentage as f32 / 100.0);
                        let fill_color = if show_red {
                            Color32::from_rgb(239, 68, 68)
                        } else if percentage > 50 {
                            Color32::from_rgb(34, 197, 94)  // Green
                        } else if percentage > 20 {
                            Color32::from_rgb(251, 191, 36)  // Yellow/Orange
                        } else {
                            Color32::from_rgb(239, 68, 68)  // Red
                        };
                        
                        if fill_width > 0.0 {
                            let fill_rect = egui::Rect::from_min_size(
                                rect.min + Vec2::new(padding, padding),
                                Vec2::new(fill_width, battery_height - 2.0 * padding)
                            );
                            painter.rect_filled(fill_rect, 1.0, fill_color);
                        }
                    } else {
                        ui.label(RichText::new("Battery: Unkown")
                            .color(Color32::GRAY)
                            .size(14.0));
                    }
                });
            });
    }
}

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size(Vec2::new(720.0, 720.0))
            .with_min_inner_size(Vec2::new(720.0, 720.0))
            .with_maximize_button(true)
            .with_resizable(true),
        ..Default::default()
    };

    eframe::run_native(
        "pupper-rs",
        options,
        Box::new(|cc| {
            match ImageApp::new(cc) {
                Ok(app) => Box::new(app) as Box<dyn App>,
                Err(e) => {
                    eprintln!("Failed to initialize application: {}", e);
                    std::process::exit(1);
                }
            }
        }),
    )
}
