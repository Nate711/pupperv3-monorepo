use clap::Parser;
use eframe::{App, egui};
use egui::{Color32, RichText, Vec2};
use tracing::debug;

mod config;
mod detection;
mod eyes;
mod qr;
mod system;
mod ui;

use config::{Config, load_config, print_config_info};
use detection::DetectionReceiver;
use eyes::{BlinkState, EyeTracker, draw_eye, draw_eyebrow};
use qr::{FrameReceiver, QrStatusReceiver, ScanStatus};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use system::{
    set_pairing, BagRecorderMonitor, BatteryMonitor, CpuMonitor, InternetMonitor,
    LlmServiceMonitor, PairingMode, PairingMonitor, ServiceMonitor,
};
use ui::{
    SimpleStatus, draw_battery_indicator, draw_cpu_stats, draw_fullscreen_button, draw_status_badge,
};

/// Which WiFi overlay (if any) is showing over the eyes.
#[derive(PartialEq, Clone, Copy)]
enum WifiView {
    None,
    PhoneQr,
    Scan,
}

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
    pairing_monitor: PairingMonitor,
    eye_tracker: EyeTracker,
    detection_receiver: DetectionReceiver,
    is_fullscreen: bool,
    show_topbar: bool,
    // QR WiFi pairing overlay
    wifi_view: WifiView,
    frame_receiver: Option<FrameReceiver>,
    qr_status_receiver: Option<QrStatusReceiver>,
    scan_status: Arc<Mutex<ScanStatus>>,
    scan_connected_at: Option<Instant>,
    last_scan_proc: Option<Instant>,
    qifi_tex: Option<egui::TextureHandle>,
    camera_tex: Option<egui::TextureHandle>,
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
            pairing_monitor: PairingMonitor::new(),
            eye_tracker: EyeTracker::new(),
            detection_receiver: DetectionReceiver::new(),
            is_fullscreen,
            show_topbar: true,
            wifi_view: WifiView::None,
            frame_receiver: None,
            qr_status_receiver: None,
            scan_status: Arc::new(Mutex::new(ScanStatus::Scanning)),
            scan_connected_at: None,
            last_scan_proc: None,
            qifi_tex: None,
            camera_tex: None,
        })
    }

    fn draw_main_ui(&mut self, ctx: &egui::Context) {
        egui::CentralPanel::default()
            .frame(egui::Frame::none().fill(Color32::BLACK))
            .show(ctx, |ui| {
                // Use the whole panel; place eyes relative to the center
                let rect = ui.max_rect();
                let painter = ui.painter();

                // Update eye tracker with latest person detections
                let people = self.detection_receiver.get_people_locations();
                self.eye_tracker.update(ctx, rect, people);

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

                // Get people positions for potential eye tracking
                let people = self.detection_receiver.get_people_locations();
                if people.is_some() {
                    debug!("Detected people: {:?}", people);
                }

                // TODO: Use people positions to update eye tracker target
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
                    let label = if self.show_topbar {
                        "Hide UI"
                    } else {
                        "Show UI"
                    };
                    let resp = ui.add(egui::Button::new(label).min_size(egui::vec2(110.0, 28.0)));
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
                    let (_rect, response) =
                        ui.allocate_exact_size(desired_size, egui::Sense::click());
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
                    ui.add_space(3.0);
                    draw_status_badge(
                        ui,
                        "LLM",
                        SimpleStatus::from(self.llm_service_monitor.get_status()),
                    );
                    ui.add_space(3.0);
                    draw_status_badge(
                        ui,
                        "NET",
                        SimpleStatus::from(self.internet_monitor.get_status()),
                    );
                    ui.add_space(3.0);
                    draw_status_badge(
                        ui,
                        "BAG",
                        SimpleStatus::from(self.bag_recorder_monitor.get_status()),
                    );

                    // WiFi pairing-mode dropdown (compact; pairing state shown by color)
                    ui.add_space(4.0);
                    let mode = self.pairing_monitor.get_mode();
                    let wifi_color = match mode {
                        PairingMode::Hotspot => Color32::from_rgb(251, 191, 36), // amber = pairing
                        PairingMode::Disconnected => Color32::from_rgb(239, 68, 68), // red = offline
                        _ => Color32::WHITE,
                    };
                    ui.menu_button(RichText::new("WiFi").size(16.0).color(wifi_color), |ui| {
                        ui.set_min_width(250.0);
                        if ui.button("Scan WiFi QR code").clicked() {
                            self.wifi_view = WifiView::Scan;
                            ui.close_menu();
                        }
                        if ui.button("Show QR for my phone").clicked() {
                            self.wifi_view = WifiView::PhoneQr;
                            ui.close_menu();
                        }
                        ui.separator();
                        match mode {
                            PairingMode::Hotspot => {
                                ui.label("Pairing mode is ACTIVE.");
                                ui.label("On your phone, join 'Pupper-Setup-…',");
                                ui.label("then open http://10.41.0.1");
                                ui.separator();
                                if ui.button("Exit pairing / reconnect").clicked() {
                                    set_pairing(false);
                                    ui.close_menu();
                                }
                            }
                            _ => {
                                ui.label("Broadcast the Pupper setup hotspot so");
                                ui.label("you can join a new WiFi from your phone.");
                                ui.label("(The current WiFi connection will drop.)");
                                ui.separator();
                                if ui.button("Enter pairing mode").clicked() {
                                    set_pairing(true);
                                    ui.close_menu();
                                }
                            }
                        }
                    });

                    // Fullscreen button at the far right
                    ui.add_space(5.0);
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

    /// Start/stop the camera frame stream to match the current view.
    fn sync_wifi_view(&mut self) {
        match self.wifi_view {
            WifiView::Scan => {
                if self.frame_receiver.is_none() {
                    self.frame_receiver = Some(FrameReceiver::start());
                    self.qr_status_receiver = Some(QrStatusReceiver::start());
                    if let Ok(mut s) = self.scan_status.lock() {
                        *s = ScanStatus::Scanning;
                    }
                    self.scan_connected_at = None;
                    self.last_scan_proc = None;
                }
            }
            _ => {
                if self.frame_receiver.is_some() {
                    self.frame_receiver = None; // Drop stops the ZMQ thread
                    self.qr_status_receiver = None;
                    self.camera_tex = None;
                }
            }
        }
    }

    fn draw_wifi_overlay(&mut self, ctx: &egui::Context) {
        let screen = ctx.screen_rect();
        egui::Area::new(egui::Id::new("wifi_overlay"))
            .order(egui::Order::Foreground)
            .fixed_pos(screen.min)
            .show(ctx, |ui| {
                ui.painter()
                    .rect_filled(screen, 0.0, Color32::from_black_alpha(238));
                ui.set_min_size(screen.size());
                ui.vertical_centered(|ui| match self.wifi_view {
                    WifiView::PhoneQr => self.draw_phone_qr(ui),
                    WifiView::Scan => self.draw_scan(ui, ctx),
                    WifiView::None => {}
                });
            });
    }

    fn draw_phone_qr(&mut self, ui: &mut egui::Ui) {
        ui.add_space(28.0);
        ui.label(
            RichText::new("Open the WiFi QR generator")
                .size(22.0)
                .color(Color32::WHITE),
        );
        ui.label(
            RichText::new("Scan this with your phone (qifi.org)")
                .size(15.0)
                .color(Color32::GRAY),
        );
        ui.add_space(16.0);
        if self.qifi_tex.is_none() {
            if let Some(img) = qr::qr_color_image("https://qifi.org/", 8, 4) {
                self.qifi_tex =
                    Some(ui.ctx().load_texture("qifi_qr", img, egui::TextureOptions::NEAREST));
            }
        }
        if let Some(tex) = &self.qifi_tex {
            ui.add(egui::Image::from_texture(egui::load::SizedTexture::new(
                tex.id(),
                egui::vec2(300.0, 300.0),
            )));
        }
        ui.add_space(14.0);
        ui.label(
            RichText::new("Generate a WiFi QR there, then choose")
                .size(15.0)
                .color(Color32::LIGHT_GRAY),
        );
        ui.label(
            RichText::new("\"Scan WiFi QR code\" to show it to Pupper.")
                .size(15.0)
                .color(Color32::LIGHT_GRAY),
        );
        ui.add_space(18.0);
        if ui
            .add(egui::Button::new(RichText::new("Close").size(18.0)).min_size(egui::vec2(150.0, 42.0)))
            .clicked()
        {
            self.wifi_view = WifiView::None;
        }
    }

    fn draw_scan(&mut self, ui: &mut egui::Ui, ctx: &egui::Context) {
        // Update the camera texture for display (throttled ~15 Hz).
        let due = self
            .last_scan_proc
            .map_or(true, |t| t.elapsed() > Duration::from_millis(66));
        if due {
            self.last_scan_proc = Some(Instant::now());
            if let Some(jpeg) = self.frame_receiver.as_ref().and_then(|r| r.latest()) {
                if let Some(img) = qr::jpeg_to_color_image(&jpeg) {
                    if let Some(tex) = &mut self.camera_tex {
                        tex.set(img, egui::TextureOptions::LINEAR);
                    } else {
                        self.camera_tex =
                            Some(ctx.load_texture("camera", img, egui::TextureOptions::LINEAR));
                    }
                }
            }
        }

        // Decode feedback comes from the detection node (zbar, over ZMQ 5558).
        let qr = self
            .qr_status_receiver
            .as_ref()
            .map(|r| r.latest())
            .unwrap_or_default();

        let status = self
            .scan_status
            .lock()
            .map(|s| s.clone())
            .unwrap_or(ScanStatus::Scanning);

        // Start connecting the moment the node decodes a WiFi QR.
        if matches!(status, ScanStatus::Scanning) && qr.decoded.starts_with("WIFI:") {
            if let Some(creds) = qr::parse_wifi_string(&qr.decoded) {
                qr::connect_async(creds, Arc::clone(&self.scan_status));
            }
        }

        ui.add_space(10.0);
        ui.label(
            RichText::new("Hold the WiFi QR code up to the camera")
                .size(19.0)
                .color(Color32::WHITE),
        );
        ui.add_space(8.0);
        if let Some(tex) = &self.camera_tex {
            let w = ui.available_width().min(540.0);
            let size = tex.size_vec2();
            let aspect = if size.x > 0.0 { size.y / size.x } else { 0.66 };
            ui.add(egui::Image::from_texture(egui::load::SizedTexture::new(
                tex.id(),
                egui::vec2(w, w * aspect),
            )));
        } else {
            ui.label(
                RichText::new("Waiting for camera…")
                    .size(16.0)
                    .color(Color32::GRAY),
            );
        }
        ui.add_space(10.0);
        match &status {
            ScanStatus::Scanning => {
                let (txt, col) = if !qr.located {
                    ("Searching for a QR code…", Color32::LIGHT_GRAY)
                } else if qr.sharpness < 40.0 {
                    (
                        "QR detected but too blurry — move back & hold steady",
                        Color32::from_rgb(251, 191, 36),
                    )
                } else {
                    ("QR detected — reading…", Color32::from_rgb(120, 200, 255))
                };
                ui.label(RichText::new(txt).size(17.0).color(col));
            }
            ScanStatus::Connecting(ssid) => {
                ui.label(
                    RichText::new(format!("Connecting to {ssid}…"))
                        .size(18.0)
                        .color(Color32::from_rgb(251, 191, 36)),
                );
            }
            ScanStatus::Connected(ssid) => {
                ui.label(
                    RichText::new(format!("Connected to {ssid}"))
                        .size(20.0)
                        .color(Color32::from_rgb(34, 197, 94)),
                );
                if self.scan_connected_at.is_none() {
                    self.scan_connected_at = Some(Instant::now());
                }
            }
            ScanStatus::Failed(e) => {
                ui.label(
                    RichText::new("Couldn't connect")
                        .size(18.0)
                        .color(Color32::from_rgb(239, 68, 68)),
                );
                if !e.is_empty() {
                    ui.label(RichText::new(e).size(12.0).color(Color32::GRAY));
                }
                if ui.button("Try again").clicked() {
                    if let Ok(mut s) = self.scan_status.lock() {
                        *s = ScanStatus::Scanning;
                    }
                }
            }
        }

        // Auto-close shortly after a successful connect.
        if let Some(t) = self.scan_connected_at {
            if t.elapsed() > Duration::from_secs(2) {
                self.wifi_view = WifiView::None;
            }
        }

        ui.add_space(10.0);
        if ui
            .add(egui::Button::new(RichText::new("Cancel").size(18.0)).min_size(egui::vec2(150.0, 42.0)))
            .clicked()
        {
            self.wifi_view = WifiView::None;
        }
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
        self.pairing_monitor.update();
        self.blink_state.update(&self.config.blink);

        // Draw UI
        self.draw_main_ui(ctx);
        self.draw_status_ui(ctx);

        // QR WiFi pairing overlay (drawn over the eyes when active)
        self.sync_wifi_view();
        if self.wifi_view != WifiView::None {
            self.draw_wifi_overlay(ctx);
        }
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
