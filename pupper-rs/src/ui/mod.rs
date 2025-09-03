pub mod cpu;
pub mod status;

pub use cpu::draw_cpu_stats;
pub use status::{
    draw_battery_indicator, draw_fullscreen_button, draw_status_badge, SimpleStatus,
};
