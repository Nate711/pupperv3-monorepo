use crate::config::BatteryConfig;
use crate::system::{LlmServiceStatus, ServiceStatus};
use eframe::egui;
use egui::{Color32, RichText, Sense, Stroke, Vec2};

pub fn draw_service_status(ui: &mut egui::Ui, status: &ServiceStatus) -> Option<()> {
    let mut fullscreen_clicked = None;
    ui.horizontal(|ui| {
        // Draw status icon using SVG
        let (svg_path, text) = match status {
            ServiceStatus::Active => (egui::include_image!("../status_active.svg"), "Robot up"),
            ServiceStatus::Inactive => (egui::include_image!("../status_inactive.svg"), "Robot down"),
            ServiceStatus::Unknown => (egui::include_image!("../status_unknown.svg"), "Robot status unknown"),
        };

        // Draw SVG status icon
        let icon_size = Vec2::new(20.0, 20.0);
        ui.add(egui::Image::from(svg_path).fit_to_exact_size(icon_size));

        // Add some spacing and then the text
        ui.add_space(1.0);
        ui.label(RichText::new(text).color(Color32::WHITE).size(14.0));

        // Add fullscreen toggle button with SVG icon
        ui.add_space(8.0);

        // Load SVG icon using the modern egui::Image API
        let button_size = Vec2::new(16.0, 16.0);
        
        // Use allocate_response to get hover state before rendering
        let (rect, response) = ui.allocate_exact_size(button_size, Sense::click());
        
        // Determine tint color based on interaction state
        let tint_color = if response.is_pointer_button_down_on() {
            Color32::from_rgb(150, 150, 150) // Darker gray when clicked
        } else if response.hovered() {
            Color32::from_rgb(200, 200, 200) // Light gray when hovered
        } else {
            Color32::WHITE // White when idle
        };
        
        // Draw the image with appropriate tint
        egui::Image::from(egui::include_image!("../fullscreen_icon.svg"))
            .tint(tint_color)
            .paint_at(ui, rect);
        
        if response.clicked() {
            fullscreen_clicked = Some(());
        }
    });
    fullscreen_clicked
}

pub fn draw_llm_service_status(ui: &mut egui::Ui, status: &LlmServiceStatus) {
    ui.horizontal(|ui| {
        // Draw LLM service status icon using same SVGs as robot service
        let (svg_path, text) = match status {
            LlmServiceStatus::Active => (egui::include_image!("../status_active.svg"), "LLM up"),
            LlmServiceStatus::Inactive => (egui::include_image!("../status_inactive.svg"), "LLM down"),
            LlmServiceStatus::Unknown => (egui::include_image!("../status_unknown.svg"), "LLM status unknown"),
        };

        // Draw SVG status icon
        let icon_size = Vec2::new(20.0, 20.0);
        ui.add(egui::Image::from(svg_path).fit_to_exact_size(icon_size));

        // Add some spacing and then the text
        ui.add_space(1.0);
        ui.label(RichText::new(text).color(Color32::WHITE).size(14.0));
    });
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
