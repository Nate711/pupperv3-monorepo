use crate::config::BatteryConfig;
use crate::system::ServiceStatus;
use eframe::egui;
use egui::{Color32, RichText, Sense, Stroke, Vec2};

pub fn draw_service_status(ui: &mut egui::Ui, status: &ServiceStatus) -> Option<()> {
    let mut fullscreen_clicked = None;
    ui.horizontal(|ui| {
        // Draw status icon
        let (color, symbol, text) = match status {
            ServiceStatus::Active => (Color32::from_rgb(34, 197, 94), "✓", "Robot up"),
            ServiceStatus::Inactive => (Color32::from_rgb(239, 68, 68), "✗", "Robot down"),
            ServiceStatus::Unknown => (Color32::GRAY, "?", "Robot status unknown"),
        };

        // Draw colored circle with symbol
        let radius = 10.0;
        let (response, painter) =
            ui.allocate_painter(Vec2::new(radius * 2.0, radius * 2.0), Sense::hover());
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

        // Add fullscreen toggle button with SVG icon
        ui.add_space(8.0);

        // Load SVG icon using the modern egui::Image API
        // let svg_data = include_bytes!("../fullscreen_icon.svg");
        let button_size = Vec2::new(16.0, 16.0);

        if ui
            .add_sized(
                button_size,
                egui::ImageButton::new(egui::Image::from(egui::include_image!(
                    "../fullscreen_icon.svg"
                )))
                .frame(false),
            )
            .clicked()
        {
            fullscreen_clicked = Some(());
        }
    });
    fullscreen_clicked
}

pub fn draw_battery_indicator(
    ui: &mut egui::Ui,
    percentage: Option<u8>,
    should_flash: bool,
    config: &BatteryConfig,
) {
    ui.horizontal(|ui| {
        if let Some(percentage) = percentage {
            // Determine if we should show red (flashing or solid)
            let is_low = percentage < config.low_threshold;
            let show_red = is_low && should_flash;

            // Percentage text
            let text_color = if show_red {
                Color32::from_rgb(239, 68, 68)
            } else {
                Color32::WHITE
            };
            ui.label(
                RichText::new(format!("{}%", percentage))
                    .color(text_color)
                    .size(14.0),
            );

            ui.add_space(4.0);

            // Battery icon
            let battery_width = 30.0;
            let battery_height = 16.0;
            let terminal_width = 3.0;
            let terminal_height = 8.0;

            let (response, painter) = ui.allocate_painter(
                Vec2::new(battery_width + terminal_width, battery_height),
                Sense::hover(),
            );
            let rect = response.rect;

            // Battery outline color
            let outline_color = if show_red {
                Color32::from_rgb(239, 68, 68)
            } else {
                Color32::GRAY
            };

            // Draw battery body outline
            let battery_rect =
                egui::Rect::from_min_size(rect.min, Vec2::new(battery_width, battery_height));
            painter.rect_stroke(battery_rect, 2.0, Stroke::new(2.0, outline_color));

            // Draw battery terminal
            let terminal_rect = egui::Rect::from_min_size(
                rect.min + Vec2::new(battery_width, (battery_height - terminal_height) / 2.0),
                Vec2::new(terminal_width, terminal_height),
            );
            painter.rect_filled(terminal_rect, 0.0, outline_color);

            // Draw battery fill
            let padding = 2.0;
            let fill_width = (battery_width - 2.0 * padding) * (percentage as f32 / 100.0);
            let fill_color = if show_red {
                Color32::from_rgb(239, 68, 68)
            } else if percentage > 50 {
                Color32::from_rgb(34, 197, 94) // Green
            } else if percentage > 20 {
                Color32::from_rgb(251, 191, 36) // Yellow/Orange
            } else {
                Color32::from_rgb(239, 68, 68) // Red
            };

            if fill_width > 0.0 {
                let fill_rect = egui::Rect::from_min_size(
                    rect.min + Vec2::new(padding, padding),
                    Vec2::new(fill_width, battery_height - 2.0 * padding),
                );
                painter.rect_filled(fill_rect, 1.0, fill_color);
            }
        } else {
            ui.label(
                RichText::new("Battery: Unkown")
                    .color(Color32::GRAY)
                    .size(14.0),
            );
        }
    });
}
