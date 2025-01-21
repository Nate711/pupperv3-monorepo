import argparse
import os
from pathlib import Path
import shutil


def remove_spaces_from_filenames(directory):
    # Convert Path object to a string, if necessary
    directory = str(directory)
    # Check if the directory exists
    if not os.path.isdir(directory):
        print(f"The directory {directory} does not exist.")
        return

    # List all files in the directory
    for filename in os.listdir(directory):
        # Construct full file path
        file_path = os.path.join(directory, filename)
        # Check if it's a file (and not a directory)
        if os.path.isfile(file_path):
            # Create a new filename by removing spaces
            new_filename = filename.replace(" ", "")
            new_file_path = os.path.join(directory, new_filename)
            # Rename the file if the new filename is different
            if new_filename != filename:
                shutil.copy2(file_path, new_file_path)
                print(f"Renamed '{filename}' to '{new_filename}'")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Remove spaces from filenames in the specified directory."
    )
    parser.add_argument(
        "--mesh_dir",
        type=Path,
        required=True,
        help="The directory containing the files to process.",
    )
    args = parser.parse_args()

    remove_spaces_from_filenames(args.mesh_dir)
