use clap::Parser;
use eframe::{App, egui};
use egui::{Color32, Vec2};

mod config;
mod eyes;
mod system;
mod ui;

use config::{Config, load_config, print_config_info};
use eyes::{BlinkState, EyeTracker, draw_eye, draw_eyebrow};
use system::{BagRecorderMonitor, BatteryMonitor, CpuMonitor, InternetMonitor, LlmServiceMonitor, ServiceMonitor};
use ui::{
    draw_battery_indicator, draw_cpu_stats, draw_fullscreen_button, SimpleStatus, draw_status_badge,
};

/// Pupper robot GUI application
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Start in windowed mode instead of fullscreen
    #[arg(long, default_value_t = false)]
    windowed: bool,
}

struct ImageApp {
    config: Config,
    blink_state: BlinkState,
    bag_recorder_monitor: BagRecorderMonitor,
    battery_monitor: BatteryMonitor,
    cpu_monitor: CpuMonitor,
    service_monitor: ServiceMonitor,
    llm_service_monitor: LlmServiceMonitor,
    internet_monitor: InternetMonitor,
    eye_tracker: EyeTracker,
    is_fullscreen: bool,
    show_topbar: bool,
}

impl ImageApp {
    fn new(_cc: &eframe::CreationContext<'_>) -> Result<Self, String> {
        egui_extras::install_image_loaders(&_cc.egui_ctx);
        let config = load_config()?;
        print_config_info(&config);

        // Start fullscreen by default (controlled by CLI args in main)
        let is_fullscreen = true;

        Ok(Self {
            config,
            blink_state: BlinkState::new(),
            bag_recorder_monitor: BagRecorderMonitor::new(),
            battery_monitor: BatteryMonitor::new(),
            cpu_monitor: CpuMonitor::new(),
            service_monitor: ServiceMonitor::new(),
            llm_service_monitor: LlmServiceMonitor::new(),
            internet_monitor: InternetMonitor::new(),
            eye_tracker: EyeTracker::new(),
            is_fullscreen,
            show_topbar: true,
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
        // Top-bar visibility toggle control
        if self.config.ui.toggle_button_visible {
            // Visible button centered on the top bar
            egui::Area::new(egui::Id::new("topbar_toggle_button"))
                .anchor(egui::Align2::CENTER_TOP, [0.0, 6.0])
                .order(egui::Order::Foreground)
                .show(ctx, |ui| {
                    let label = if self.show_topbar { "Hide UI" } else { "Show UI" };
                    let resp = ui.add(
                        egui::Button::new(label)
                            .min_size(egui::vec2(110.0, 28.0))
                    );
                    if resp.clicked() {
                        self.show_topbar = !self.show_topbar;
                    }
                });
        } else {
            // Invisible hotzone for a clean look
            egui::Area::new(egui::Id::new("topbar_toggle_hotzone"))
                .anchor(egui::Align2::CENTER_TOP, [0.0, 5.0])
                .order(egui::Order::Foreground)
                .show(ctx, |ui| {
                    let desired_size = egui::vec2(200.0, 40.0);
                    let (_rect, response) = ui.allocate_exact_size(desired_size, egui::Sense::click());
                    if response.clicked() {
                        self.show_topbar = !self.show_topbar;
                    }
                });
        }

        if !self.show_topbar {
            return;
        }

        // Service status indicators in top-right
        egui::Area::new(egui::Id::new("service_status"))
            .anchor(egui::Align2::RIGHT_TOP, [-10.0, 10.0])
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    // Ensure consistent row height for icon alignment
                    ui.set_height(30.0);
                    // ROS, LLM, Internet, and Bag Recording — rendered using shared badge code
                    draw_status_badge(
                        ui,
                        "ROS",
                        SimpleStatus::from(self.service_monitor.get_status()),
                    );
                    ui.add_space(5.0);
                    draw_status_badge(
                        ui,
                        "LLM",
                        SimpleStatus::from(self.llm_service_monitor.get_status()),
                    );
                    ui.add_space(5.0);
                    draw_status_badge(
                        ui,
                        "NET",
                        SimpleStatus::from(self.internet_monitor.get_status()),
                    );
                    ui.add_space(5.0);
                    draw_status_badge(
                        ui,
                        "BAG",
                        SimpleStatus::from(self.bag_recorder_monitor.get_status()),
                    );

                    // Fullscreen button at the far right
                    ui.add_space(8.0);
                    if draw_fullscreen_button(ui) {
                        self.is_fullscreen = !self.is_fullscreen;
                        if self.is_fullscreen {
                            ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(true));
                        } else {
                            ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(false));
                            ctx.send_viewport_cmd(egui::ViewportCommand::Maximized(true));
                        }
                    }
                });
            });

        // Battery and CPU indicators in top-left
        egui::Area::new(egui::Id::new("battery_status"))
            .anchor(egui::Align2::LEFT_TOP, [10.0, 10.0])
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    draw_battery_indicator(
                        ui,
                        self.battery_monitor.percentage,
                        self.battery_monitor.should_flash(),
                        &self.config.battery,
                    );

                    if self.cpu_monitor.is_enabled() {
                        ui.add_space(10.0);

                        draw_cpu_stats(ui, self.cpu_monitor.usage, self.cpu_monitor.temperature);
                    }
                });
            });
    }
}

impl App for ImageApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint();

        // Update all subsystems
        self.bag_recorder_monitor.update(&self.config.bag_recorder);
        self.battery_monitor.update(&self.config.battery);
        self.cpu_monitor.update(&self.config.cpu);
        self.service_monitor.update(&self.config.service);
        self.llm_service_monitor.update(&self.config.service);
        self.internet_monitor.update(&self.config.service);
        self.blink_state.update(&self.config.blink);

        // Draw UI
        self.draw_main_ui(ctx);
        self.draw_status_ui(ctx);
    }
}

fn main() -> eframe::Result<()> {
    // Parse command line arguments
    let args = Args::parse();
    
    // Start fullscreen by default, unless --windowed is specified
    let fullscreen = !args.windowed;

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size(Vec2::new(720.0, 720.0))
            .with_min_inner_size(Vec2::new(720.0, 720.0))
            .with_maximize_button(true)
            .with_resizable(true)
            .with_fullscreen(fullscreen),
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
