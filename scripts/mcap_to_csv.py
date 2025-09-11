#!/usr/bin/env python3

import argparse
import csv
import sys
from pathlib import Path
import time
from typing import Dict, List, Optional

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import JointState


def read_joint_states(mcap_file: str, topic_name: str = "/joint_states") -> List[tuple]:
    """
    Read joint state messages from MCAP file.

    Args:
        mcap_file: Path to MCAP file
        topic_name: Topic name to filter for (default: /joint_states)

    Returns:
        List of tuples: (timestamp_ns, joint_state_msg)
    """
    reader = rosbag2_py.SequentialReader()

    # Open the bag file
    storage_options = rosbag2_py.StorageOptions(uri=mcap_file, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")

    reader.open(storage_options, converter_options)

    # Get topic information
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    if topic_name not in type_map:
        print(f"Topic {topic_name} not found in bag file", file=sys.stderr)
        print(f"Available topics: {list(type_map.keys())}", file=sys.stderr)
        return []

    joint_states = []

    start = time.time()
    # Read messages
    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic == topic_name:
            # Deserialize the message
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            joint_states.append((timestamp, msg))
    end = time.time()
    print(f"Read {len(joint_states)} messages from {mcap_file} in {end - start:.2f} seconds")

    return joint_states


def filter_by_time_range(
    messages: List[tuple], start_time_sec: Optional[float] = None, end_time_sec: Optional[float] = None
) -> List[tuple]:
    """
    Filter messages by absolute time range.

    Args:
        messages: List of (timestamp_ns, msg) tuples
        start_time_sec: Start time as absolute timestamp in seconds (None = no start filter)
        end_time_sec: End time as absolute timestamp in seconds (None = no end filter)

    Returns:
        Filtered list of messages
    """
    if not messages:
        return []

    if start_time_sec is None and end_time_sec is None:
        return messages

    filtered = []
    for timestamp_ns, msg in messages:
        timestamp_sec = timestamp_ns / 1e9

        # Check start time filter
        if start_time_sec is not None and timestamp_sec < start_time_sec:
            continue

        # Check end time filter
        if end_time_sec is not None and timestamp_sec > end_time_sec:
            continue

        filtered.append((timestamp_ns, msg))

    return filtered


def subsample_messages(messages: List[tuple], target_freq_hz: float = 30.0) -> List[tuple]:
    """
    Subsample messages to target frequency.

    Args:
        messages: List of (timestamp_ns, msg) tuples
        target_freq_hz: Target frequency in Hz

    Returns:
        Subsampled list of messages
    """
    if not messages:
        return []

    # Convert target frequency to nanosecond interval
    target_interval_ns = int(1e9 / target_freq_hz)

    subsampled = []
    last_timestamp = 0

    for timestamp, msg in messages:
        if timestamp - last_timestamp >= target_interval_ns:
            subsampled.append((timestamp, msg))
            last_timestamp = timestamp

    return subsampled


def write_joint_states_csv(joint_states: List[tuple], output_file: str):
    """
    Write joint states to CSV file.

    Args:
        joint_states: List of (timestamp_ns, joint_state_msg) tuples
        output_file: Output CSV file path
    """
    if not joint_states:
        print("No joint state messages to write", file=sys.stderr)
        return

    # Get joint names from first message
    first_msg = joint_states[0][1]
    joint_names = first_msg.name

    with open(output_file, "w", newline="") as csvfile:
        # Create header: timestamp + joint names
        fieldnames = ["timestamp_ns", "timestamp_sec"] + list(joint_names)
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        # Write data rows
        for timestamp_ns, msg in joint_states:
            row = {"timestamp_ns": timestamp_ns, "timestamp_sec": timestamp_ns / 1e9}

            # Add joint positions
            for i, joint_name in enumerate(joint_names):
                if i < len(msg.position):
                    row[joint_name] = msg.position[i]
                else:
                    row[joint_name] = 0.0  # Default if position not available

            writer.writerow(row)

    print(f"Wrote {len(joint_states)} joint state samples to {output_file}")
    print(f"Joint names: {joint_names}")


def main():
    parser = argparse.ArgumentParser(description="Extract joint states from MCAP files to CSV")
    parser.add_argument("mcap_file", help="Input MCAP file path")
    parser.add_argument("-o", "--output", help="Output CSV file (default: same name as input with .csv extension)")
    parser.add_argument(
        "-t", "--topic", default="/joint_states", help="Joint states topic name (default: /joint_states)"
    )
    parser.add_argument("-f", "--frequency", type=float, default=30.0, help="Target frequency in Hz (default: 30.0)")
    parser.add_argument("-s", "--start-time", type=float, help="Start time as absolute timestamp in seconds (optional)")
    parser.add_argument("-e", "--end-time", type=float, help="End time as absolute timestamp in seconds (optional)")

    args = parser.parse_args()

    # Determine output file name
    if args.output:
        output_file = args.output
    else:
        input_path = Path(args.mcap_file)
        output_file = input_path.with_suffix(".csv")

    print(f"Reading joint states from: {args.mcap_file}")
    print(f"Topic: {args.topic}")
    print(f"Target frequency: {args.frequency} Hz")
    if args.start_time is not None:
        print(f"Start time: {args.start_time} seconds (absolute)")
    if args.end_time is not None:
        print(f"End time: {args.end_time} seconds (absolute)")
    print(f"Output file: {output_file}")

    # Read joint states from MCAP
    joint_states = read_joint_states(args.mcap_file, args.topic)

    if not joint_states:
        print("No joint state messages found", file=sys.stderr)
        return 1

    print(f"Found {len(joint_states)} joint state messages")

    # Filter by time range if specified
    if args.start_time is not None or args.end_time is not None:
        filtered = filter_by_time_range(joint_states, args.start_time, args.end_time)
        print(f"Filtered to {len(filtered)} messages in time range")
        joint_states = filtered

    # Subsample to target frequency
    subsampled = subsample_messages(joint_states, args.frequency)
    print(f"Subsampled to {len(subsampled)} messages at {args.frequency} Hz")

    # Write to CSV
    write_joint_states_csv(subsampled, output_file)

    return 0


if __name__ == "__main__":
    sys.exit(main())
