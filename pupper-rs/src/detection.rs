use serde::{Deserialize, Serialize};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PersonLocation {
    pub x: f32,      // Normalized x position (0.0 - 1.0)
    pub y: f32,      // Normalized y position (0.0 - 1.0)
    pub width: f32,  // Normalized width (0.0 - 1.0)
    pub height: f32, // Normalized height (0.0 - 1.0)
    pub heading: f32, // Heading angle in degrees
    pub elevation: f32, // Elevation angle in degrees
    pub id: i32,     // Tracker ID
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PeopleDetections {
    pub people: Vec<PersonLocation>,
    pub timestamp: f64,
}

pub struct DetectionReceiver {
    latest_people: Arc<Mutex<Option<PeopleDetections>>>,
    _zmq_thread: Option<thread::JoinHandle<()>>,
}

impl DetectionReceiver {
    pub fn new() -> Self {
        let latest_people = Arc::new(Mutex::new(None));
        let people_clone = Arc::clone(&latest_people);

        // Spawn ZMQ subscriber thread
        let zmq_thread = thread::spawn(move || {
            // Create ZMQ context and subscriber socket
            let ctx = zmq::Context::new();
            let socket = ctx.socket(zmq::SUB).unwrap();

            // Connect to the Hailo detection publisher
            if let Err(e) = socket.connect("tcp://127.0.0.1:5556") {
                eprintln!("Failed to connect to ZMQ publisher: {}", e);
                return;
            }

            // Subscribe to all messages
            socket.set_subscribe(b"").unwrap();
            socket.set_rcvtimeo(100).unwrap();

            println!("Connected to detection publisher at tcp://127.0.0.1:5556");

            loop {
                match socket.recv_string(0) {
                    Ok(Ok(msg)) => {
                        // Parse JSON message
                        match serde_json::from_str::<PeopleDetections>(&msg) {
                            Ok(people_msg) => {
                                // Update latest detections
                                if let Ok(mut latest) = people_clone.lock() {
                                    *latest = Some(people_msg);
                                }
                            }
                            Err(e) => {
                                eprintln!("Failed to parse people message: {}", e);
                            }
                        }
                    }
                    Ok(Err(_)) => {
                        // Invalid UTF-8, skip
                    }
                    Err(zmq::Error::EAGAIN) => {
                        // Timeout - no message available
                        thread::sleep(Duration::from_millis(10));
                    }
                    Err(e) => {
                        eprintln!("ZMQ error: {}", e);
                        thread::sleep(Duration::from_millis(100));
                    }
                }
            }
        });

        Self {
            latest_people,
            _zmq_thread: Some(zmq_thread),
        }
    }

    pub fn get_people_locations(&self) -> Option<Vec<PersonLocation>> {
        self.latest_people.lock().unwrap().as_ref().map(|d| d.people.clone())
    }
}

impl Drop for DetectionReceiver {
    fn drop(&mut self) {
        // The thread will naturally exit when the socket is closed
        // No need to explicitly join since we're using a non-blocking receive
    }
}