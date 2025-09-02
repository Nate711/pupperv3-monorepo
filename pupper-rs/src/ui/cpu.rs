use eframe::egui;
use egui::{Color32, RichText};

pub fn draw_cpu_stats(
    ui: &mut egui::Ui,
    usage: Option<f32>,
    temperature: Option<f32>,
) {
    // CPU usage
    if let Some(usage) = usage {
        let color = if usage > 80.0 {
            Color32::from_rgb(239, 68, 68) // Red for high usage
        } else if usage > 50.0 {
            Color32::from_rgb(251, 191, 36) // Yellow/Orange for medium
        } else {
            Color32::from_rgb(34, 197, 94) // Green for low
        };
        
        ui.label(
            RichText::new(format!("CPU: {:.0}%", usage))
                .color(color)
                .size(21.0)
        );
    } else {
        ui.label(
            RichText::new("CPU: --")
                .color(Color32::GRAY)
                .size(21.0)
        );
    }
    
    ui.add_space(8.0);
    
    // CPU temperature
    if let Some(temp) = temperature {
        let color = if temp > 80.0 {
            Color32::from_rgb(239, 68, 68) // Red for hot
        } else if temp > 60.0 {
            Color32::from_rgb(251, 191, 36) // Yellow/Orange for warm
        } else {
            Color32::from_rgb(34, 197, 94) // Green for cool
        };
        
        ui.label(
            RichText::new(format!("{:.0}°C", temp))
                .color(color)
                .size(21.0)
        );
    } else {
        ui.label(
            RichText::new("--°C")
                .color(Color32::GRAY)
                .size(21.0)
        );
    }
}
