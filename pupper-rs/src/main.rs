use eframe::{App, egui};
use egui::{Color32, Vec2};

mod config;
mod eyes;
mod system;
mod ui;

use config::{load_config, print_config_info, Config};
use eyes::{draw_eye, draw_eyebrow, BlinkState};
use system::{BatteryMonitor, ServiceMonitor};
use ui::{draw_battery_indicator, draw_service_status};

struct ImageApp {
    config: Config,
    blink_state: BlinkState,
    battery_monitor: BatteryMonitor,
    service_monitor: ServiceMonitor,
}

impl ImageApp {
    fn new(_cc: &eframe::CreationContext<'_>) -> Result<Self, String> {
        let config = load_config()?;
        print_config_info(&config);
        
        Ok(Self {
            config,
            blink_state: BlinkState::new(),
            battery_monitor: BatteryMonitor::new(),
            service_monitor: ServiceMonitor::new(),
        })
    }

    fn draw_main_ui(&mut self, ctx: &egui::Context) {
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

                let left_eye_center = center + Vec2::new(-offset_x, offset_y);
                let right_eye_center = center + Vec2::new(offset_x, offset_y);
                
                // Draw eyes (without eyebrows)
                draw_eye(&painter, left_eye_center);
                draw_eye(&painter, right_eye_center);
                
                // Draw blinking animation (black boxes coming down)
                self.blink_state.draw_blink_boxes(
                    &painter,
                    left_eye_center,
                    right_eye_center,
                    &self.config.blink,
                );
                
                // Draw eyebrows on top layer so they're never covered by blinks
                draw_eyebrow(&painter, left_eye_center);
                draw_eyebrow(&painter, right_eye_center);
            });
    }

    fn draw_status_ui(&mut self, ctx: &egui::Context) {
        // Service status indicator in top-right
        egui::Area::new(egui::Id::new("service_status"))
            .anchor(egui::Align2::RIGHT_TOP, [-30.0, 30.0])
            .show(ctx, |ui| {
                draw_service_status(ui, self.service_monitor.get_status());
            });
        
        // Battery indicator in top-left
        egui::Area::new(egui::Id::new("battery_status"))
            .anchor(egui::Align2::LEFT_TOP, [30.0, 30.0])
            .show(ctx, |ui| {
                draw_battery_indicator(
                    ui,
                    self.battery_monitor.percentage,
                    self.battery_monitor.should_flash(),
                    &self.config.battery,
                );
            });
    }
}

impl App for ImageApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint();

        // Update all subsystems
        self.battery_monitor.update(&self.config.battery);
        self.service_monitor.update(&self.config.service);
        self.blink_state.update(&self.config.blink);
        
        // Draw UI
        self.draw_main_ui(ctx);
        self.draw_status_ui(ctx);
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