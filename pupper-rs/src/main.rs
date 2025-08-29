use eframe::{App, egui};
use egui::{Color32, Vec2};

mod config;
mod eyes;
mod system;
mod ui;

use config::{Config, load_config, print_config_info};
use eyes::{BlinkState, EyeTracker, draw_eye, draw_eyebrow};
use system::{BatteryMonitor, ServiceMonitor};
use ui::{draw_battery_indicator, draw_service_status};

struct ImageApp {
    config: Config,
    blink_state: BlinkState,
    battery_monitor: BatteryMonitor,
    service_monitor: ServiceMonitor,
    eye_tracker: EyeTracker,
    is_fullscreen: bool,
}

impl ImageApp {
    fn new(_cc: &eframe::CreationContext<'_>) -> Result<Self, String> {
        egui_extras::install_image_loaders(&_cc.egui_ctx);
        let config = load_config()?;
        print_config_info(&config);

        Ok(Self {
            config,
            blink_state: BlinkState::new(),
            battery_monitor: BatteryMonitor::new(),
            service_monitor: ServiceMonitor::new(),
            eye_tracker: EyeTracker::new(),
            is_fullscreen: true,
        })
    }

    fn draw_main_ui(&mut self, ctx: &egui::Context) {
        egui::CentralPanel::default()
            .frame(egui::Frame::none().fill(Color32::BLACK))
            .show(ctx, |ui| {
                // Use the whole panel; place eyes relative to the center
                let rect = ui.max_rect();
                let painter = ui.painter();

                // Update eye tracker
                self.eye_tracker.update(ctx, rect);

                let center = rect.center();
                // Horizontal spacing between eyes
                let offset_x = 190.0;
                // Slight vertical offset so they sit a bit high in the frame
                let offset_y = -10.0;

                // Calculate eye positions (with potential whole-eye movement)
                let eye_offset = self
                    .eye_tracker
                    .get_whole_eye_offset(&self.config.eye_tracking);
                let left_eye_center = center + Vec2::new(-offset_x, offset_y) + eye_offset;
                let right_eye_center = center + Vec2::new(offset_x, offset_y) + eye_offset;

                // Calculate pupil offset (for pupil-only movement)
                let pupil_offset = self.eye_tracker.get_pupil_offset(&self.config.eye_tracking);

                // Draw eyes (with pupil tracking)
                draw_eye(&painter, left_eye_center, pupil_offset);
                draw_eye(&painter, right_eye_center, pupil_offset);

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
                if let Some(_) = draw_service_status(ui, self.service_monitor.get_status()) {
                    self.is_fullscreen = !self.is_fullscreen;
                    if self.is_fullscreen {
                        ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(true));
                    } else {
                        ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(false));
                        ctx.send_viewport_cmd(egui::ViewportCommand::Maximized(true));
                    }
                }
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
            .with_resizable(true)
            .with_fullscreen(true),
        ..Default::default()
    };

    eframe::run_native(
        "pupper-rs",
        options,
        Box::new(|cc| match ImageApp::new(cc) {
            Ok(app) => Box::new(app) as Box<dyn App>,
            Err(e) => {
                eprintln!("Failed to initialize application: {}", e);
                std::process::exit(1);
            }
        }),
    )
}
