use crate::config::{EyeTrackingConfig, EyeTrackingMode, EyeTrackingSource};
use crate::detection::PersonLocation;
use eframe::egui;
use egui::{Pos2, Vec2};

pub struct EyeTracker {
    pub mouse_position: Option<Pos2>,
    pub window_center: Option<Pos2>,
    pub window_size: Vec2,
    pub latest_people: Option<Vec<PersonLocation>>,
}

impl EyeTracker {
    pub fn new() -> Self {
        Self {
            mouse_position: None,
            window_center: None,
            window_size: Vec2::new(800.0, 600.0), // Default size, will be updated
            latest_people: None,
        }
    }

    pub fn update(
        &mut self,
        ctx: &egui::Context,
        window_rect: egui::Rect,
        people: Option<Vec<PersonLocation>>,
    ) {
        self.window_center = Some(window_rect.center());
        self.window_size = window_rect.size();
        self.latest_people = people;

        if let Some(pointer_pos) = ctx.input(|i| i.pointer.hover_pos()) {
            self.mouse_position = Some(pointer_pos);
        }
    }

    fn get_largest_person(&self) -> Option<PersonLocation> {
        if let Some(ref people) = self.latest_people {
            // Find the person with the largest bounding box area
            people
                .iter()
                .max_by(|a, b| {
                    let area_a = a.width * a.height;
                    let area_b = b.width * b.height;
                    area_a
                        .partial_cmp(&area_b)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .cloned()
        } else {
            None
        }
    }

    fn get_tracking_position(&self, config: &EyeTrackingConfig) -> Option<Pos2> {
        match config.source {
            EyeTrackingSource::Mouse => self.mouse_position,
            EyeTrackingSource::Person => {
                if let Some(person) = self.get_largest_person() {
                    // Convert normalized coordinates (0-1) to window coordinates
                    // Person detection gives us center of bounding box
                    if let Some(window_center) = self.window_center {
                        let eye_offset_x = person.heading / 90.0 * (self.window_size.x / 2.0); // Assuming heading is in degrees
                        let eye_offset_y = person.elevation / 90.0 * (self.window_size.y / 2.0); // Assuming elevation is in degrees
                        Some(Pos2::new(
                            window_center.x + eye_offset_x,
                            window_center.y + eye_offset_y,
                        ))
                    } else {
                        None
                    }
                } else {
                    // No person detected, center the eyes
                    self.window_center
                }
            }
        }
    }

    fn calculate_base_offset(&self, config: &EyeTrackingConfig) -> Vec2 {
        if !config.enabled {
            return Vec2::ZERO;
        }

        if let (Some(tracking_pos), Some(window_center)) =
            (self.get_tracking_position(config), self.window_center)
        {
            // Calculate direction from window center to tracking position
            let direction = tracking_pos - window_center;

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
