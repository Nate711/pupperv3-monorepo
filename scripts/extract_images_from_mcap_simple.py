#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path
import time
from mcap.reader import make_reader
from mcap_ros2.reader import read_ros2_messages


def extract_compressed_images(mcap_file: str, topic_name: str = "/camera/image_raw/compressed"):
    """
    Extract compressed images from MCAP file.

    Args:
        mcap_file: Path to MCAP file
        topic_name: Topic name to filter for

    Returns:
        Number of images extracted
    """
    mcap_path = Path(mcap_file)

    # Create output directory
    output_dir = mcap_path.parent / "extracted_images"
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Reading from: {mcap_file}")
    print(f"Looking for topic: {topic_name}")
    print(f"Output directory: {output_dir}")

    image_count = 0
    topics_seen = set()

    with open(mcap_file, "rb") as f:
        reader = make_reader(f)

        # First pass - list all topics to help with debugging
        for schema, channel, message in reader.iter_messages():
            topics_seen.add(channel.topic)

        if topic_name not in topics_seen:
            print(f"\nTopic '{topic_name}' not found in bag file", file=sys.stderr)
            print(f"Available topics:", file=sys.stderr)
            for topic in sorted(topics_seen):
                print(f"  - {topic}", file=sys.stderr)
            return 0

    # Second pass - extract images
    with open(mcap_file, "rb") as f:
        start_time = time.time()

        for msg in read_ros2_messages(mcap_file, topics=[topic_name]):
            if msg.channel.topic == topic_name:
                # The message data is already the compressed image
                ros_msg = msg.ros_msg

                # Determine file extension from format field
                format_str = ros_msg.format.lower() if hasattr(ros_msg, 'format') else 'jpeg'
                if 'jpeg' in format_str or 'jpg' in format_str:
                    ext = 'jpg'
                elif 'png' in format_str:
                    ext = 'png'
                else:
                    ext = 'jpg'

                # Create filename with timestamp
                timestamp_ns = msg.log_time_ns
                filename = f"image_{timestamp_ns:020d}.{ext}"
                filepath = output_dir / filename

                # Write the compressed image data
                with open(filepath, 'wb') as img_file:
                    img_file.write(ros_msg.data)

                image_count += 1

                if image_count % 100 == 0:
                    print(f"Extracted {image_count} images...")

        elapsed = time.time() - start_time
        print(f"\nExtracted {image_count} images in {elapsed:.2f} seconds")
        print(f"Images saved to: {output_dir}")

    return image_count


def main():
    parser = argparse.ArgumentParser(description="Extract compressed images from MCAP files")
    parser.add_argument("mcap_file", help="Input MCAP file path")
    parser.add_argument(
        "-t", "--topic",
        default="/camera/image_raw/compressed",
        help="Compressed image topic name (default: /camera/image_raw/compressed)"
    )

    args = parser.parse_args()

    if not Path(args.mcap_file).exists():
        print(f"Error: MCAP file not found: {args.mcap_file}", file=sys.stderr)
        return 1

    # Extract images
    count = extract_compressed_images(args.mcap_file, args.topic)

    if count == 0:
        print("No images were extracted", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())