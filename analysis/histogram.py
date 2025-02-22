#!/usr/bin/env python3

"""
Example usage:
python histogram.py --bagpath1 '/Users/nathankau/WOBBLY_feb_20'  --topic /neural_controller/observation --bagpath2 '/Users/nathankau/normal_walking_on_ground_rosbag2'
"""


import argparse
from pathlib import Path
import matplotlib.pyplot as plt

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from tqdm import tqdm
import ipdb

import numpy as np


def main():
    parser = argparse.ArgumentParser(
        description="Read two rosbags bags and overlay histograms for each index of an array message."
    )
    parser.add_argument(
        "--bagpath1", help="Path to the first rosbags bag (file or directory)"
    )
    parser.add_argument(
        "--bagpath2", help="Path to the second rosbags bag (file or directory)"
    )
    parser.add_argument(
        "--topic", help="Topic name that contains the array messages"
    )
    args = parser.parse_args()

    bag_paths = [Path(args.bagpath1), Path(args.bagpath2)]
    typestore = get_typestore(
        Stores.ROS2_HUMBLE
    )  # Adjust if you're using a different ROS version

    data_list = []

    for bag_path in bag_paths:
        with AnyReader([bag_path], default_typestore=typestore) as reader:
            connections = [
                conn for conn in reader.connections if conn.topic == args.topic
            ]
            if not connections:
                print(f"No connections found for topic: {args.topic}")
                return

            total_messages = sum(
                1 for _ in reader.messages(connections=connections)
            )

            data = []

            with tqdm(
                total=total_messages,
                desc=f"Processing messages from {bag_path}",
            ) as pbar:
                for connection, timestamp, rawdata in reader.messages(
                    connections=connections
                ):
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    try:
                        arr = msg.data
                    except AttributeError:
                        print("Message does not have a 'data' attribute.")
                        return

                    data.append(arr)
                    pbar.update(1)

        data = np.stack(data, axis=0)
        data_list.append(data)

    if len(data_list) != 2:
        print("Error processing bags.")
        return

    titles = None
    if "observation" in args.topic:
        titles = (
            [
                "Angular Velocity X",
                "Angular Velocity Y",
                "Angular Velocity Z",
                "Projected Gravity X",
                "Projected Gravity Y",
                "Projected Gravity Z",
                "Command Velocity X",
                "Command Velocity Y",
                "Command Yaw Velocity",
                "Desired Orientation X",
                "Desired Orientation Y",
                "Desired Orientation Z",
            ]
            + [f"Joint {i}" for i in range(12)]
            + [f"Action {i}" for i in range(12)]
        )
    plot_histograms(
        data_list[0],
        data_list[1],
        titles=titles,
        min_channels=0,
        max_channels=36,
    )


def plot_histograms(
    data1, data2, min_channels=0, max_channels=36, cols=6, titles=None
):
    print(data1.shape, data2.shape)
    assert data1.shape[1] == data2.shape[1]
    data1 = data1[:, min_channels:max_channels]
    data2 = data2[:, min_channels:max_channels]

    num_channels = data1.shape[1]
    fig, axes = plt.subplots(
        num_channels // cols, cols, figsize=(10, 2 * num_channels)
    )

    for i in range(num_channels):
        row = i // cols
        col = i % cols
        axes[row, col].hist(data1[:, i], bins=50, alpha=0.75, label="Bag 1")
        axes[row, col].hist(data2[:, i], bins=50, alpha=0.75, label="Bag 2")

        if titles is not None:
            axes[row, col].set_title(titles[i])

        # uncomment for tighter layout
        # axes[row, col].tick_params(axis="both", which="both", length=0)
        # axes[row, col].set_xticklabels([])
        # axes[row, col].set_yticklabels([])

    # plt.tight_layout()
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
