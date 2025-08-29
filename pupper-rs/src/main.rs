use eframe::{App, egui};
use egui::{Color32, Pos2, RichText, Shape, Stroke, Vec2};
use std::process::Command;
use std::time::{Duration, Instant};

enum ServiceStatus {
    Active,
    Inactive,
    Unknown,
}

struct ImageApp {
    service_status: ServiceStatus,
    last_check: Instant,
}

impl ImageApp {
    fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        Self {
            service_status: ServiceStatus::Unknown,
            last_check: Instant::now(),
        }
    }

    fn poll_service_status(&mut self) {
        if self.last_check.elapsed() >= Duration::from_secs(1) {
            self.service_status = query_robot_status();
            self.last_check = Instant::now();
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
        Box::new(|cc| Box::new(ImageApp::new(cc))),
    )
}
