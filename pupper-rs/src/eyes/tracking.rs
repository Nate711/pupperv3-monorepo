use crate::config::{EyeTrackingConfig, EyeTrackingMode};
use eframe::egui;
use egui::{Pos2, Vec2};

pub struct EyeTracker {
    pub mouse_position: Option<Pos2>,
    pub window_center: Option<Pos2>,
}

impl EyeTracker {
    pub fn new() -> Self {
        Self {
            mouse_position: None,
            window_center: None,
        }
    }

    pub fn update(&mut self, ctx: &egui::Context, window_rect: egui::Rect) {
        self.window_center = Some(window_rect.center());
        
        if let Some(pointer_pos) = ctx.input(|i| i.pointer.hover_pos()) {
            self.mouse_position = Some(pointer_pos);
        }
    }

    fn calculate_base_offset(&self, config: &EyeTrackingConfig) -> Vec2 {
        if !config.enabled {
            return Vec2::ZERO;
        }

        if let (Some(mouse_pos), Some(window_center)) = (self.mouse_position, self.window_center) {
            // Calculate direction from window center to mouse
            let direction = mouse_pos - window_center;
            
            // Apply sensitivity
            direction * config.sensitivity
        } else {
            Vec2::ZERO
        }
    }

    pub fn get_pupil_offset(&self, config: &EyeTrackingConfig) -> Vec2 {
        let base_offset = self.calculate_base_offset(config);
        
        match config.mode {
            EyeTrackingMode::PupilsOnly => {
                // Only pupils move, clamp to pupil limits
                let length = base_offset.length();
                if length > config.max_pupil_offset {
                    base_offset * (config.max_pupil_offset / length)
                } else {
                    base_offset
                }
            }
            EyeTrackingMode::WholeEye => {
                // Pupils stay centered in eye when whole eye moves
                Vec2::ZERO
            }
            EyeTrackingMode::Combined => {
                // Pupils move a fraction of the eye movement
                let eye_offset = self.get_whole_eye_offset(config);
                let pupil_movement = eye_offset * config.combined_pupil_ratio;
                
                // Clamp pupil movement to its limits
                let length = pupil_movement.length();
                if length > config.max_pupil_offset {
                    pupil_movement * (config.max_pupil_offset / length)
                } else {
                    pupil_movement
                }
            }
        }
    }

    pub fn get_whole_eye_offset(&self, config: &EyeTrackingConfig) -> Vec2 {
        let base_offset = self.calculate_base_offset(config);
        
        match config.mode {
            EyeTrackingMode::PupilsOnly => {
                // Eyes stay in place when only pupils move
                Vec2::ZERO
            }
            EyeTrackingMode::WholeEye | EyeTrackingMode::Combined => {
                // Whole eyes move, clamp to eye limits
                let length = base_offset.length();
                if length > config.max_eye_offset {
                    base_offset * (config.max_eye_offset / length)
                } else {
                    base_offset
                }
            }
        }
    }
}