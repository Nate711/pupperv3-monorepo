use crate::config::ServiceConfig;
use std::net::{SocketAddr, TcpStream};
use std::time::{Duration, Instant};

#[derive(Debug, Clone)]
pub enum InternetStatus {
    Online,
    Offline,
    Unknown,
}

pub struct InternetMonitor {
    pub status: InternetStatus,
    pub last_check: Instant,
}

impl InternetMonitor {
    pub fn new() -> Self {
        Self {
            status: InternetStatus::Unknown,
            last_check: Instant::now(),
        }
    }

    pub fn update(&mut self, config: &ServiceConfig) {
        if self.last_check.elapsed() >= Duration::from_secs_f32(config.poll_interval_secs) {
            self.status = query_internet_status();
            self.last_check = Instant::now();
        }
    }

    pub fn get_status(&self) -> &InternetStatus {
        &self.status
    }
}

fn query_internet_status() -> InternetStatus {
    let addr: SocketAddr = match "1.1.1.1:53".parse() {
        Ok(a) => a,
        Err(_) => return InternetStatus::Unknown,
    };
    match TcpStream::connect_timeout(&addr, Duration::from_secs(1)) {
        Ok(_) => InternetStatus::Online,
        Err(_) => InternetStatus::Offline,
    }
}
