use crate::config::BlinkConfig;
use eframe::egui;
use egui::{Color32, Pos2, Vec2};
use std::time::{Duration, Instant};

pub struct BlinkState {
    pub last_blink: Instant,
    pub blink_timer: Instant,
    pub is_blinking: bool,
    pub blink_progress: f32,
}

impl BlinkState {
    pub fn new() -> Self {
        Self {
            last_blink: Instant::now(),
            blink_timer: Instant::now(),
            is_blinking: false,
            blink_progress: 0.0,
        }
    }

    pub fn update(&mut self, config: &BlinkConfig) {
        // Blink based on configured interval
        let blink_interval = Duration::from_secs_f64(config.interval_secs);
        if !self.is_blinking && self.last_blink.elapsed() >= blink_interval {
            self.is_blinking = true;
            self.blink_timer = Instant::now();
            self.last_blink = Instant::now();
            self.blink_progress = 0.0;
        }

        // Animate the blink
        if self.is_blinking {
            let elapsed = self.blink_timer.elapsed().as_secs_f64();
            let blink_duration = config.duration_secs;
            let eye_delay = config.eye_delay_secs;
            let total_blink_time = blink_duration + eye_delay; // Total time needed for both eyes to complete

            if elapsed < total_blink_time {
                // Keep the blink active until both eyes are done
                let t = elapsed / blink_duration;
                if t < 0.5 {
                    self.blink_progress = (t * 2.0) as f32; // 0 to 1
                } else if t < 1.0 {
                    self.blink_progress = (2.0 - t * 2.0) as f32; // 1 to 0
                } else {
                    self.blink_progress = 0.0; // Left eye done, but right eye may still be blinking
                }
            } else {
                self.is_blinking = false;
                self.blink_progress = 0.0;
            }
        }
    }

    pub fn draw_blink_boxes(
        &self,
        painter: &egui::Painter,
        left_eye_center: Pos2,
        right_eye_center: Pos2,
        config: &BlinkConfig,
    ) {
        if !self.is_blinking {
            return;
        }

        let eye_radius = 140.0; // Match the outer ring radius
        let padding = 10.0; // Extra padding to make blink box larger
        let elapsed = self.blink_timer.elapsed().as_secs_f64();
        let blink_duration = config.duration_secs;
        let eye_delay = config.eye_delay_secs;

        // Left eye blink (starts first)
        let left_progress = elapsed / blink_duration;
        let left_blink_progress = if left_progress < 0.5 {
            (left_progress * 2.0) as f32 // 0 to 1
        } else if left_progress < 1.0 {
            (2.0 - left_progress * 2.0) as f32 // 1 to 0
        } else {
            0.0
        };

        if left_blink_progress > 0.0 {
            self.draw_eye_blink_box(
                painter,
                left_eye_center,
                left_blink_progress,
                eye_radius,
                padding,
            );
        }

        // Right eye blink (delayed by configured amount)
        if elapsed >= eye_delay {
            let right_elapsed = elapsed - eye_delay;
            let right_progress = right_elapsed / blink_duration;
            let right_blink_progress = if right_progress < 0.5 {
                (right_progress * 2.0) as f32 // 0 to 1
            } else if right_progress < 1.0 {
                (2.0 - right_progress * 2.0) as f32 // 1 to 0
            } else {
                0.0
            };

            if right_blink_progress > 0.0 {
                self.draw_eye_blink_box(
                    painter,
                    right_eye_center,
                    right_blink_progress,
                    eye_radius,
                    padding,
                );
            }
        }
    }

    fn draw_eye_blink_box(
        &self,
        painter: &egui::Painter,
        eye_center: Pos2,
        blink_progress: f32,
        eye_radius: f32,
        padding: f32,
    ) {
        let box_width = (eye_radius + padding) * 2.0;
        let box_height = (eye_radius + padding) * 2.0 * blink_progress;
        let rect = egui::Rect::from_min_size(
            egui::Pos2::new(
                eye_center.x - eye_radius - padding,
                eye_center.y - eye_radius - padding,
            ),
            Vec2::new(box_width, box_height),
        );
        painter.rect_filled(rect, 0.0, Color32::BLACK);
    }
}
