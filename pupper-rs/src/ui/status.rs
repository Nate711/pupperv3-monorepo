use crate::config::BatteryConfig;
use crate::system::{BagRecorderStatus, InternetStatus, LlmServiceStatus, ServiceStatus};
use eframe::egui;
use egui::{Color32, RichText, Sense, Stroke, Vec2};

// A simplified shared status enum so all systems render identically
pub enum SimpleStatus {
    Active,
    Inactive,
    Loading,
    Unknown,
}

impl From<&ServiceStatus> for SimpleStatus {
    fn from(s: &ServiceStatus) -> Self {
        match s {
            ServiceStatus::Active => SimpleStatus::Active,
            ServiceStatus::Inactive => SimpleStatus::Inactive,
            ServiceStatus::Unknown => SimpleStatus::Unknown,
        }
    }
}

impl From<&LlmServiceStatus> for SimpleStatus {
    fn from(s: &LlmServiceStatus) -> Self {
        match s {
            LlmServiceStatus::Active => SimpleStatus::Active,
            LlmServiceStatus::Inactive => SimpleStatus::Inactive,
            LlmServiceStatus::Loading => SimpleStatus::Loading,
            LlmServiceStatus::Unknown => SimpleStatus::Unknown,
        }
    }
}

impl From<&InternetStatus> for SimpleStatus {
    fn from(s: &InternetStatus) -> Self {
        match s {
            InternetStatus::Online => SimpleStatus::Active,
            InternetStatus::Offline => SimpleStatus::Inactive,
            InternetStatus::Unknown => SimpleStatus::Unknown,
        }
    }
}

impl From<&BagRecorderStatus> for SimpleStatus {
    fn from(s: &BagRecorderStatus) -> Self {
        match s {
            BagRecorderStatus::Recording => SimpleStatus::Active,
            BagRecorderStatus::Stopped => SimpleStatus::Inactive,
            BagRecorderStatus::Unknown => SimpleStatus::Unknown,
        }
    }
}

// Draw a single status badge (icon + label)
pub fn draw_status_badge(ui: &mut egui::Ui, label: &str, status: SimpleStatus) {
    // Draw inline to avoid nested layout vertical offsets
    let svg_path = match status {
        SimpleStatus::Active => egui::include_image!("../status_active.svg"),
        SimpleStatus::Inactive => egui::include_image!("../status_inactive.svg"),
        SimpleStatus::Loading => egui::include_image!("../status_loading.svg"),
        SimpleStatus::Unknown => egui::include_image!("../status_unknown.svg"),
    };

    let icon_size = Vec2::new(30.0, 30.0);
    ui.add(egui::Image::from(svg_path).fit_to_exact_size(icon_size));
    ui.add_space(1.0);
    ui.label(RichText::new(label).color(Color32::WHITE).size(21.0));
}

// Draw a fullscreen button and return true if clicked
pub fn draw_fullscreen_button(ui: &mut egui::Ui) -> bool {
    let button_size = Vec2::new(24.0, 24.0);
    let (rect, response) = ui.allocate_exact_size(button_size, Sense::click());

    let tint_color = if response.is_pointer_button_down_on() {
        Color32::from_rgb(150, 150, 150)
    } else if response.hovered() {
        Color32::from_rgb(200, 200, 200)
    } else {
        Color32::WHITE
    };

    egui::Image::from(egui::include_image!("../fullscreen_icon.svg"))
        .tint(tint_color)
        .paint_at(ui, rect);

    response.clicked()
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
                    .size(21.0),
            );

            ui.add_space(6.0);

            // Battery icon
            let battery_width = 45.0;
            let battery_height = 24.0;
            let terminal_width = 4.5;
            let terminal_height = 12.0;

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
            painter.rect_stroke(battery_rect, 2.0, Stroke::new(3.0, outline_color));

            // Draw battery terminal
            let terminal_rect = egui::Rect::from_min_size(
                rect.min + Vec2::new(battery_width, (battery_height - terminal_height) / 2.0),
                Vec2::new(terminal_width, terminal_height),
            );
            painter.rect_filled(terminal_rect, 0.0, outline_color);

            // Draw battery fill
            let padding = 3.0;
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
                    .size(21.0),
            );
        }
    });
}
