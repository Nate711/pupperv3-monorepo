use eframe::{App, egui};
use egui::{Color32, Pos2, Shape, Stroke, Vec2};

struct ImageApp;

impl ImageApp {
    fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        Self
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
    let iris_outer = Color32::from_rgb(0x08, 0x12, 0x20);
    let iris_mid = Color32::from_rgb(0x0b, 0x17, 0x27);
    let iris_inner = Color32::from_rgb(0x00, 0x84, 0xff);
    let iris_ring = Color32::from_rgba_unmultiplied(0x00, 0xa6, 0xff, 180);

    painter.add(Shape::circle_filled(center, 126.0, iris_outer));
    painter.add(Shape::circle_filled(center, 88.0, iris_mid));
    painter.add(Shape::circle_filled(center, 40.0, iris_inner));
    painter.add(Shape::circle_stroke(
        center,
        126.0,
        Stroke::new(6.0, iris_ring),
    ));

    painter.add(Shape::circle_filled(center, 96.0, Color32::BLACK));

    let iris_blue = Color32::from_rgb(0x00, 0xa6, 0xff);
    let arc_start = center + Vec2::new(-62.0, 38.0);
    let arc_ctrl = center + Vec2::new(0.0, -36.0);
    let arc_end = center + Vec2::new(58.0, 38.0);
    let arc_points = quadratic_bezier_points(arc_start, arc_ctrl, arc_end, 20);
    painter.add(Shape::line(arc_points, Stroke::new(17.0, iris_blue)));
    painter.add(Shape::circle_filled(
        center + Vec2::new(70.0, 42.0),
        8.0,
        iris_blue,
    ));

    painter.add(Shape::circle_filled(
        center + Vec2::new(-29.0, -51.0),
        31.0,
        Color32::from_rgba_unmultiplied(255, 255, 255, (0.96 * 255.0) as u8),
    ));
    painter.add(Shape::circle_filled(
        center + Vec2::new(18.0, -17.0),
        14.0,
        Color32::from_rgba_unmultiplied(255, 255, 255, (0.90 * 255.0) as u8),
    ));

    let brow_start = center + Vec2::new(-84.0, -180.0);
    let brow_ctrl = center + Vec2::new(0.0, -204.0);
    let brow_end = center + Vec2::new(84.0, -180.0);
    let brow_points = quadratic_bezier_points(brow_start, brow_ctrl, brow_end, 20);
    let brow_color = Color32::from_rgba_unmultiplied(0xb9, 0xb9, 0xb9, (0.8 * 255.0) as u8);
    painter.add(Shape::line(brow_points, Stroke::new(10.0, brow_color)));
}

impl App for ImageApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            let (rect, _) = ui.allocate_exact_size(Vec2::new(900.0, 420.0), egui::Sense::hover());
            let painter = ui.painter_at(rect);

            let halo_color = Color32::from_rgba_unmultiplied(0x00, 0xb4, 0xff, 60);
            painter.add(Shape::circle_filled(
                Pos2::new(220.0, 210.0),
                150.0,
                halo_color,
            ));
            painter.add(Shape::circle_filled(
                Pos2::new(680.0, 210.0),
                150.0,
                halo_color,
            ));

            draw_eye(&painter, Pos2::new(220.0, 210.0));
            draw_eye(&painter, Pos2::new(680.0, 210.0));
        });
    }
}

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_fullscreen(true),
        ..Default::default()
    };
    eframe::run_native(
        "pupper-rs",
        options,
        Box::new(|cc| Box::new(ImageApp::new(cc))),
    )
}
