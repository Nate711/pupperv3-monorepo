use clap::{Parser, Subcommand};
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use fdaf_aec::FdafAec;
use hound;
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
    /// Run the echo cancellation processor (default)
    Run {
        /// Target sample rate in Hz (will use closest supported rate)
        #[arg(short, long, default_value = "16000")]
        sample_rate: u32,
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

fn run_processor(target_sample_rate: u32) {
    // Configuration
    const FFT_SIZE: usize = 1024; // Must be a power of two. Determines filter length.
    const FRAME_SIZE: usize = FFT_SIZE / 2; // Should be half of FFT_SIZE.
    const STEP_SIZE: f32 = 0.02; // Learning rate. A small value between 0 and 1.

    // Create a new AEC instance
    let mut aec = FdafAec::new(FFT_SIZE, STEP_SIZE);

    // Set up audio
    let host = cpal::default_host();

    // Get default input device (microphone)
    let input_device = host
        .default_input_device()
        .expect("No input device available");

    // Just use the default config
    let supported_config = input_device
        .default_input_config()
        .expect("Failed to get default input config");

    let config = supported_config.config();

    println!(
        "Using input device: {}",
        input_device.name().unwrap_or("Unknown".to_string())
    );
    println!(
        "Sample rate: {} Hz (requested {}Hz)",
        config.sample_rate.0, target_sample_rate
    );
    println!("Channels: {}", config.channels);
    println!("Sample format: {:?}", supported_config.sample_format());

    // Calculate resampling ratio if needed
    let resample_ratio = config.sample_rate.0 as f32 / target_sample_rate as f32;
    if resample_ratio != 1.0 {
        println!(
            "Resampling ratio: {} (will resample by this factor)",
            resample_ratio
        );
    }

    // Create ring buffer for microphone audio (sized for higher sample rate)
    let buffer_size = ((FRAME_SIZE as f32 * resample_ratio).ceil() as usize) * 4;
    let rb = HeapRb::<f32>::new(buffer_size);
    let (producer, mut consumer) = rb.split();
    let producer = Arc::new(Mutex::new(producer));

    // Create resampling buffer
    let resampled_buffer_size = (FRAME_SIZE as f32 * resample_ratio).ceil() as usize;

    // Build input stream for microphone
    let producer_clone = producer.clone();
    let sample_format = supported_config.sample_format();
    let input_stream = match sample_format {
        cpal::SampleFormat::F32 => input_device
            .build_input_stream(
                &config,
                move |data: &[f32], _: &cpal::InputCallbackInfo| {
                    let mut producer = producer_clone.lock().unwrap();
                    for &sample in data {
                        let _ = producer.push(sample);
                    }
                },
                |err| eprintln!("Input stream error: {}", err),
                None,
            )
            .expect("Failed to build input stream"),
        cpal::SampleFormat::I16 => input_device
            .build_input_stream(
                &config,
                move |data: &[i16], _: &cpal::InputCallbackInfo| {
                    let mut producer = producer_clone.lock().unwrap();
                    for &sample in data {
                        let normalized = sample as f32 / i16::MAX as f32;
                        let _ = producer.push(normalized);
                    }
                },
                |err| eprintln!("Input stream error: {}", err),
                None,
            )
            .expect("Failed to build input stream"),
        cpal::SampleFormat::U16 => input_device
            .build_input_stream(
                &config,
                move |data: &[u16], _: &cpal::InputCallbackInfo| {
                    let mut producer = producer_clone.lock().unwrap();
                    for &sample in data {
                        let normalized = (sample as f32 / u16::MAX as f32) * 2.0 - 1.0;
                        let _ = producer.push(normalized);
                    }
                },
                |err| eprintln!("Input stream error: {}", err),
                None,
            )
            .expect("Failed to build input stream"),
        _ => panic!("Unsupported sample format"),
    };

    // Start the input stream
    input_stream.play().expect("Failed to play input stream");

    println!("Recording audio from microphone...");
    println!("Waiting to collect {} frames...", FRAME_SIZE);

    // Set up WAV file writers for both input and output
    let spec = hound::WavSpec {
        channels: 1,
        sample_rate: target_sample_rate,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };

    let mut output_writer =
        hound::WavWriter::create("output_processed.wav", spec).expect("Failed to create output WAV file");
    
    let mut input_writer =
        hound::WavWriter::create("input_unprocessed.wav", spec).expect("Failed to create input WAV file");

    println!("Unprocessed mic input will be saved to: input_unprocessed.wav");
    println!("Processed output will be saved to: output_processed.wav");

    // Audio processing loop
    let mut mic_buffer = vec![0.0f32; FRAME_SIZE];
    let mut far_end_buffer = vec![0.0f32; FRAME_SIZE];
    let mut raw_buffer = vec![0.0f32; resampled_buffer_size];

    // For this example, we'll use silence as far-end audio
    // In a real application, this would be the audio being played through speakers

    let mut frame_count = 0;
    loop {
        // Wait until we have enough samples at the higher sample rate
        while consumer.len() < resampled_buffer_size {
            thread::sleep(Duration::from_millis(10));
        }

        // Read samples at the actual sample rate
        for i in 0..resampled_buffer_size {
            raw_buffer[i] = consumer.pop().unwrap_or(0.0);
        }

        // Simple downsampling to target sample rate (512 samples)
        // This is a basic decimation - in production you'd want proper filtering
        for i in 0..FRAME_SIZE {
            let source_idx = (i as f32 * resample_ratio) as usize;
            if source_idx < raw_buffer.len() {
                mic_buffer[i] = raw_buffer[source_idx];
            } else {
                mic_buffer[i] = 0.0;
            }
        }

        // For demonstration, use silence or a test signal as far-end audio
        // In a real scenario, this would be the audio playing through your speakers
        for i in 0..FRAME_SIZE {
            // Using silence for now - you could generate a test tone here
            far_end_buffer[i] = 0.0;
        }

        println!(
            "Processing {} frames (downsampled to {}Hz from {} Hz)...",
            FRAME_SIZE, target_sample_rate, config.sample_rate.0
        );

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

        // For this demo, we'll process a few frames and then exit
        frame_count += 1;
        if frame_count >= 100 {
            println!(
                "Processed 100 frames of {} samples each. Exiting demo.",
                FRAME_SIZE
            );
            break;
        }
    }

    // Clean up
    drop(input_stream);
    input_writer.finalize().expect("Failed to finalize input WAV file");
    output_writer.finalize().expect("Failed to finalize output WAV file");
    println!("Audio processing complete.");
    println!("Unprocessed input saved to: input_unprocessed.wav");
    println!("Processed output saved to: output_processed.wav");
}

fn main() {
    let cli = Cli::parse();

    match cli.command {
        Some(Commands::ListInputs) => {
            list_inputs();
        }
        Some(Commands::Run { sample_rate }) => {
            run_processor(sample_rate);
        }
        None => {
            // Default action: run processor with 16kHz
            run_processor(16000);
        }
    }
}
