#!/usr/bin/env python3
"""
Serve the Three.js sphere viewer with camera parameters from YAML config.
"""

import http.server
import socketserver
import os
import yaml
import argparse
import webbrowser
from urllib.parse import urlencode
from pathlib import Path


def load_camera_params(config_path):
    """Load camera parameters from YAML file"""
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    params = config.get('camera_params', {})
    return {
        'fx': params.get('fx', 228.81030),
        'fy': params.get('fy', 228.81030),
        'cx': params.get('cx', 680.0),
        'cy': params.get('cy', 479.0),
        'xi': params.get('xi', -0.00092),
        'alpha': params.get('alpha', 0.62007)
    }


def select_image(directory):
    """Select an image file from the directory"""
    images = []
    for ext in ['*.jpg', '*.jpeg', '*.png', '*.bmp']:
        images.extend(Path(directory).glob(ext))

    # Filter out output files
    images = [img for img in images if not img.name.startswith('output') and not img.name.startswith('equirect')]

    if not images:
        raise ValueError(f"No images found in {directory}")

    if len(images) == 1:
        return images[0].name

    print("\nAvailable images:")
    for i, img_path in enumerate(images):
        print(f"{i + 1}. {img_path.name}")

    while True:
        try:
            choice = input(f"\nSelect image (1-{len(images)}): ")
            idx = int(choice) - 1
            if 0 <= idx < len(images):
                return images[idx].name
            else:
                print(f"Please enter a number between 1 and {len(images)}")
        except ValueError:
            print("Invalid input. Please enter a number.")


def select_yaml_config(directory):
    """Select a YAML config file from the directory"""
    configs = []
    for ext in ['*.yaml', '*.yml']:
        configs.extend(Path(directory).glob(ext))

    if not configs:
        return None

    if len(configs) == 1:
        print(f"Using config: {configs[0].name}")
        return str(configs[0])

    print("\nAvailable config files:")
    for i, config_path in enumerate(configs):
        print(f"{i + 1}. {config_path.name}")

    while True:
        try:
            choice = input(f"\nSelect config (1-{len(configs)}): ")
            idx = int(choice) - 1
            if 0 <= idx < len(configs):
                return str(configs[idx])
            else:
                print(f"Please enter a number between 1 and {len(configs)}")
        except ValueError:
            print("Invalid input. Please enter a number.")


class CORSRequestHandler(http.server.SimpleHTTPRequestHandler):
    """HTTP request handler with CORS support"""
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        super().end_headers()


def main():
    parser = argparse.ArgumentParser(description='Serve Three.js sphere viewer for fisheye images')
    parser.add_argument('--port', type=int, default=8000, help='Port to serve on')
    parser.add_argument('--config', '-c', help='Path to YAML config file')
    parser.add_argument('--image', '-i', help='Image file to view')
    parser.add_argument('--no-browser', action='store_true', help='Do not open browser automatically')

    args = parser.parse_args()

    # Default directory
    default_dir = "/Users/nathankau/pupperv3-monorepo/ai/playground/undistory"
    if os.path.exists(default_dir):
        os.chdir(default_dir)

    # Load camera parameters
    if args.config:
        config_path = args.config
    else:
        config_path = select_yaml_config(".")

    if not config_path:
        raise ValueError("No camera configuration file found. Please create a camera_params.yaml file or specify --config")

    camera_params = load_camera_params(config_path)
    print(f"\nLoaded camera parameters from {config_path}:")
    for key, value in camera_params.items():
        print(f"  {key}: {value}")

    # Select image
    if args.image:
        image_name = args.image
    else:
        image_name = select_image(".")

    print(f"\nUsing image: {image_name}")

    # Build URL with parameters
    params = {
        'fx': camera_params['fx'],
        'fy': camera_params['fy'],
        'cx': camera_params['cx'],
        'cy': camera_params['cy'],
        'xi': camera_params['xi'],
        'alpha': camera_params['alpha'],
        'image': image_name
    }

    url = f"http://localhost:{args.port}/sphere_viewer.html?{urlencode(params)}"

    # Start server
    with socketserver.TCPServer(("", args.port), CORSRequestHandler) as httpd:
        print(f"\nServing at http://localhost:{args.port}")
        print(f"\nViewer URL: {url}")

        if not args.no_browser:
            print("\nOpening browser...")
            webbrowser.open(url)

        print("\nPress Ctrl+C to stop the server")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServer stopped")


if __name__ == "__main__":
    main()