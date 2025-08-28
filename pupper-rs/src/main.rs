use eframe::{App, egui};
use egui_extras::image::load_image_bytes;
use std::path::PathBuf;

struct ImageApp {
    // texture: egui::TextureHandle,
    // size: egui::Vec2,
}

impl ImageApp {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        egui_extras::install_image_loaders(&cc.egui_ctx);
        // let image_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        //     .join("..")
        //     .join("infra")
        //     .join("pupper_image_builder")
        //     .join("resources")
        //     .join("blue_eyes.jpg");
        // let bytes = std::fs::read(image_path).expect("failed to read blue_eyes.jpg");
        // let image = load_image_bytes(&bytes).expect("invalid image data");
        // let size = egui::Vec2::new(image.width() as f32, image.height() as f32);
        // let texture = cc
        //     .egui_ctx
        //     .load_texture("blue_eyes", image, egui::TextureOptions::default());
        // Self { texture, size }
        Self {}
    }
}

impl App for ImageApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.image(egui::include_image!("blue_eyes.png"));
        });
    }
}

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([720.0, 720.0]),
        ..Default::default()
    };
    eframe::run_native(
        "pupper-rs",
        options,
        Box::new(|cc| Box::new(ImageApp::new(cc))),
    )
}
