use clap::{Parser, Subcommand};
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use fdaf_aec::FdafAec;
use hound;
use rand::Rng;
use ringbuf::HeapRb;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

#[derive(Parser)]
#[command(name = "ui-rs")]
#[command(about = "Audio processing with echo cancellation", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Option<Commands>,
}

#[derive(Subcommand)]
enum Commands {
    /// List all available audio input devices and their supported sample rates
    ListInputs,
    /// List all available audio output devices and their supported sample rates
    ListOutputs,
    /// Run the echo cancellation processor (default)
    Run {
        /// Duration in seconds to run the processor
        #[arg(short, long, default_value = "4.0")]
        duration: f32,
    },
    /// Play white noise through the default speaker (for testing AEC)
    PlayNoise {
        /// Duration in seconds to play noise
        #[arg(short, long, default_value = "10")]
        duration: u64,
        /// Volume level (0.0 to 1.0)
        #[arg(short, long, default_value = "0.02")]
        volume: f32,
    },
}

fn list_inputs() {
    let host = cpal::default_host();

    println!("Available audio input devices:");
    println!("{:-<80}", "");

    let devices = host.input_devices().expect("Failed to get input devices");

    for (index, device) in devices.enumerate() {
        let name = device.name().unwrap_or_else(|_| "Unknown".to_string());
        println!("\n{}. {}", index + 1, name);

        // Check if this is the default device
        if let Some(default_device) = host.default_input_device() {
            if default_device.name().unwrap_or_default() == name {
                println!("   (DEFAULT DEVICE)");
            }
        }

        // Get supported configurations
        match device.supported_input_configs() {
            Ok(configs) => {
                for config in configs {
                    println!(
                        "   - Channels: {}, Format: {:?}, Sample Rate Range: {} - {} Hz",
                        config.channels(),
                        config.sample_format(),
                        config.min_sample_rate().0,
                        config.max_sample_rate().0
                    );
                }
            }
            Err(e) => {
                println!("   Error getting configurations: {}", e);
            }
        }
    }

    println!("\n{:-<80}", "");
}

fn list_outputs() {
    let host = cpal::default_host();

    println!("Available audio output devices:");
    println!("{:-<80}", "");

    let devices = host.output_devices().expect("Failed to get output devices");

    for (index, device) in devices.enumerate() {
        let name = device.name().unwrap_or_else(|_| "Unknown".to_string());
        println!("\n{}. {}", index + 1, name);

        // Check if this is the default device
        if let Some(default_device) = host.default_output_device() {
            if default_device.name().unwrap_or_default() == name {
                println!("   (DEFAULT DEVICE)");
            }
        }

        // Get supported configurations
        match device.supported_output_configs() {
            Ok(configs) => {
                for config in configs {
                    println!(
                        "   - Channels: {}, Format: {:?}, Sample Rate Range: {} - {} Hz",
                        config.channels(),
                        config.sample_format(),
                        config.min_sample_rate().0,
                        config.max_sample_rate().0
                    );
                }
            }
            Err(e) => {
                println!("   Error getting configurations: {}", e);
            }
        }
    }

    println!("\n{:-<80}", "");
}

fn run_processor(duration: f32) {
    // Configuration
    const FFT_SIZE: usize = 1024; // Must be a power of two. Determines filter length.
    const FRAME_SIZE: usize = FFT_SIZE / 2; // Should be half of FFT_SIZE.
    const STEP_SIZE: f32 = 0.1; // Learning rate. A small value between 0 and 1.
    const NOISE_VOLUME: f32 = 0.0; // Volume for the test noise
    const SAMPLE_RATE: u32 = 48000; // Fixed sample rate for both input and output

    // Create a new AEC instance
    let mut aec = FdafAec::new(FFT_SIZE, STEP_SIZE);

    // Set up audio
    let host = cpal::default_host();

    // Get default input device (microphone)
    let input_device = host
        .default_input_device()
        .expect("No input device available");

    // Get default output device (speaker)
    let output_device = host
        .default_output_device()
        .expect("No output device available");

    // Get supported configs and set to 48kHz
    let input_configs: Vec<_> = input_device
        .supported_input_configs()
        .expect("Failed to get input configs")
        .collect();

    let output_configs: Vec<_> = output_device
        .supported_output_configs()
        .expect("Failed to get output configs")
        .collect();

    // Find 48kHz configs
    let input_config_supported = input_configs
        .into_iter()
        .find(|c| c.min_sample_rate().0 <= SAMPLE_RATE && c.max_sample_rate().0 >= SAMPLE_RATE)
        .expect("Input device doesn't support 48kHz");

    let output_config_supported = output_configs
        .into_iter()
        .find(|c| c.min_sample_rate().0 <= SAMPLE_RATE && c.max_sample_rate().0 >= SAMPLE_RATE)
        .expect("Output device doesn't support 48kHz");

    let input_config = cpal::StreamConfig {
        channels: input_config_supported.channels(),
        sample_rate: cpal::SampleRate(SAMPLE_RATE),
        buffer_size: cpal::BufferSize::Default,
    };

    let output_config = cpal::StreamConfig {
        channels: output_config_supported.channels(),
        sample_rate: cpal::SampleRate(SAMPLE_RATE),
        buffer_size: cpal::BufferSize::Default,
    };

    println!(
        "Using input device: {}",
        input_device.name().unwrap_or("Unknown".to_string())
    );
    println!("Input sample rate: {} Hz", input_config.sample_rate.0);
    println!("Input channels: {}", input_config.channels);
    println!("Input format: {:?}", input_config_supported.sample_format());

    println!(
        "Using output device: {}",
        output_device.name().unwrap_or("Unknown".to_string())
    );
    println!("Output sample rate: {} Hz", output_config.sample_rate.0);
    println!("Output channels: {}", output_config.channels);
    println!(
        "Output format: {:?}",
        output_config_supported.sample_format()
    );

    println!(
        "Both devices set to {} Hz - no resampling needed",
        SAMPLE_RATE
    );

    // Calculate total frames to process based on duration
    let frames_per_second = SAMPLE_RATE as f64 / FRAME_SIZE as f64;
    let total_frames = (frames_per_second * duration as f64) as usize;
    println!(
        "Will process {} frames over {} seconds",
        total_frames, duration
    );

    // Create a shared buffer for the far-end (speaker) signal
    let far_end_rb = HeapRb::<(cpal::OutputCallbackInfo, f32)>::new(FRAME_SIZE * 8);
    let (far_end_producer, mut far_end_consumer) = far_end_rb.split();
    let far_end_producer = Arc::new(Mutex::new(far_end_producer));

    // Create ring buffer for microphone audio
    let rb = HeapRb::<(cpal::InputCallbackInfo, f32)>::new(FRAME_SIZE * 4);
    let (producer, mut consumer) = rb.split();
    let producer = Arc::new(Mutex::new(producer));

    // Build output stream for speaker (playing white noise)
    let far_end_producer_clone = far_end_producer.clone();
    let output_channels = output_config.channels as usize;
    let output_sample_format = output_config_supported.sample_format();

    let output_stream = match output_sample_format {
        cpal::SampleFormat::F32 => {
            output_device
                .build_output_stream(
                    &output_config,
                    move |data: &mut [f32], info: &cpal::OutputCallbackInfo| {
                        let mut rng = rand::thread_rng();
                        let mut producer = far_end_producer_clone.lock().unwrap();

                        // Process in stereo chunks but only store mono for AEC
                        for chunk in data.chunks_mut(output_channels) {
                            let noise = rng.gen_range(-1.0..1.0) * NOISE_VOLUME;

                            let _ = producer.push((info.clone(), noise)).unwrap_or_else(|e| {
                                eprintln!("Far-end buffer full, dropping sample: {:?}", e);
                            });

                            // Play the same noise through all channels
                            for channel in chunk {
                                *channel = noise;
                            }
                        }
                    },
                    |err| eprintln!("Output stream error: {}", err),
                    None,
                )
                .expect("Failed to build output stream")
        }
        _ => panic!("Unsupported output sample format"),
    };

    // Build input stream for microphone
    let producer_clone = producer.clone();
    let sample_format = input_config_supported.sample_format();
    let input_stream = match sample_format {
        cpal::SampleFormat::F32 => input_device
            .build_input_stream(
                &input_config,
                move |data: &[f32], info: &cpal::InputCallbackInfo| {
                    let mut producer = producer_clone.lock().unwrap();
                    for &sample in data {
                        let _ = producer.push((info.clone(), sample));
                    }
                },
                |err| eprintln!("Input stream error: {}", err),
                None,
            )
            .expect("Failed to build input stream"),

        _ => panic!("Unsupported sample format"),
    };

    // Start both streams
    output_stream.play().expect("Failed to play output stream");
    input_stream.play().expect("Failed to play input stream");

    println!(
        "Playing white noise through speakers at volume {}...",
        NOISE_VOLUME
    );
    println!("Recording audio from microphone...");
    println!("The AEC will attempt to cancel the echo from the speakers.");
    println!("Processing {} frames at a time...", FRAME_SIZE);

    // Set up WAV file writers for both input and output
    let spec = hound::WavSpec {
        channels: 1,
        sample_rate: SAMPLE_RATE,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };

    let mut output_writer = hound::WavWriter::create("output_processed.wav", spec)
        .expect("Failed to create output WAV file");

    let mut input_writer = hound::WavWriter::create("input_unprocessed.wav", spec)
        .expect("Failed to create input WAV file");

    println!("Unprocessed mic input will be saved to: input_unprocessed.wav");
    println!("Processed output will be saved to: output_processed.wav");

    // Audio processing loop
    let mut mic_buffer = vec![0.0f32; FRAME_SIZE];
    let mut far_end_buffer = vec![0.0f32; FRAME_SIZE];

    let mut frame_count = 0;
    loop {
        // Wait until we have enough samples
        while consumer.len() < FRAME_SIZE {
            thread::sleep(Duration::from_millis(1));
        }

        let mut first_mic_info = None;
        // Read microphone frames directly (no resampling needed)
        for i in 0..FRAME_SIZE {
            let (info, value) = consumer.pop().unwrap();
            mic_buffer[i] = value;
            if first_mic_info.is_none() {
                first_mic_info = Some(info);
            }
        }

        // Get the far-end signal (what's playing through the speakers)
        // Wait until we have enough far-end samples
        while far_end_consumer.len() < FRAME_SIZE {
            thread::sleep(Duration::from_millis(1));
        }

        // Read the far-end frames (the noise we're playing)
        let mut first_far_end_info = None;
        for i in 0..FRAME_SIZE {
            let (info, value) = far_end_consumer.pop().unwrap();
            far_end_buffer[i] = value;
            if first_far_end_info.is_none() {
                first_far_end_info = Some(info);
            }
        }

        println!("First far-end info: {:?}", first_far_end_info);
        println!("First mic info: {:?}", first_mic_info);

        println!("Processing {} frames at {} Hz...", FRAME_SIZE, SAMPLE_RATE);

        // Process the frames to get the echo-cancelled signal
        let output_frame = aec.process(&far_end_buffer, &mic_buffer);

        // Write unprocessed mic input to WAV file
        for &sample in &mic_buffer {
            let amplitude = (sample * i16::MAX as f32) as i16;
            input_writer
                .write_sample(amplitude)
                .expect("Failed to write input sample");
        }

        // Write processed output to WAV file
        for &sample in &output_frame {
            let amplitude = (sample * i16::MAX as f32) as i16;
            output_writer
                .write_sample(amplitude)
                .expect("Failed to write output sample");
        }

        // Calculate some basic statistics for demonstration
        let mic_energy: f32 = mic_buffer.iter().map(|x| x * x).sum::<f32>() / FRAME_SIZE as f32;
        let output_energy: f32 =
            output_frame.iter().map(|x| x * x).sum::<f32>() / FRAME_SIZE as f32;

        println!(
            "Mic energy: {:.6}, Output energy: {:.6}",
            mic_energy, output_energy
        );

        // In a real application, you would send output_frame to your output device or process it further

        // Process the specified number of frames
        frame_count += 1;
        if frame_count >= total_frames {
            println!(
                "Processed {} frames of {} samples each over {} seconds.",
                total_frames, FRAME_SIZE, duration
            );
            break;
        }
    }

    // Clean up
    drop(input_stream);
    drop(output_stream);
    input_writer
        .finalize()
        .expect("Failed to finalize input WAV file");
    output_writer
        .finalize()
        .expect("Failed to finalize output WAV file");
    println!("Audio processing complete.");
    println!("Unprocessed input saved to: input_unprocessed.wav");
    println!("Processed output saved to: output_processed.wav");
    println!("The processed output should have reduced echo from the speakers.");
}

fn play_noise(duration: u64, volume: f32) {
    let host = cpal::default_host();

    // Get default output device (speaker)
    let output_device = host
        .default_output_device()
        .expect("No output device available");

    let output_config = output_device
        .default_output_config()
        .expect("Failed to get output config");

    println!(
        "Using output device: {}",
        output_device.name().unwrap_or("Unknown".to_string())
    );
    println!("Sample rate: {} Hz", output_config.sample_rate().0);
    println!("Channels: {}", output_config.channels());
    println!(
        "Playing white noise for {} seconds at volume {}...",
        duration, volume
    );

    let config = output_config.config();
    let channels = config.channels as usize;

    // Create shared flag for stopping
    let running = Arc::new(Mutex::new(true));
    let running_clone = running.clone();

    // Build output stream for playing white noise
    let sample_format = output_config.sample_format();
    let output_stream = match sample_format {
        cpal::SampleFormat::F32 => output_device
            .build_output_stream(
                &config,
                move |data: &mut [f32], _: &cpal::OutputCallbackInfo| {
                    let mut rng = rand::thread_rng();
                    for sample in data.chunks_mut(channels) {
                        let noise = (rng.gen_range(-1.0..1.0)) * volume;
                        for channel in sample {
                            *channel = noise;
                        }
                    }
                },
                |err| eprintln!("Output stream error: {}", err),
                None,
            )
            .expect("Failed to build output stream"),
        _ => panic!("Unsupported sample format"),
    };

    // Start playing
    output_stream.play().expect("Failed to play output stream");

    // Play for the specified duration
    thread::sleep(Duration::from_secs(duration));

    // Stop and clean up
    drop(output_stream);
    *running_clone.lock().unwrap() = false;

    println!("White noise playback complete.");
}

fn main() {
    let cli = Cli::parse();

    match cli.command {
        Some(Commands::ListInputs) => {
            list_inputs();
        }
        Some(Commands::ListOutputs) => {
            list_outputs();
        }
        Some(Commands::Run { duration }) => {
            run_processor(duration);
        }
        Some(Commands::PlayNoise { duration, volume }) => {
            play_noise(duration, volume);
        }
        None => {
            // Default action: run processor with 16kHz for 10 seconds
            run_processor(10.0);
        }
    }
}
