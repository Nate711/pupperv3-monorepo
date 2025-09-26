import cv2
import numpy as np
import argparse
import os
import yaml
from pathlib import Path


class CameraModel:
    def __init__(self, cx, cy, fx, fy, width=None, height=None):
        self.cx = cx
        self.cy = cy
        self.fx = fx
        self.fy = fy
        self.width = width
        self.height = height


class DoubleSphereModel(CameraModel):
    def __init__(self, cx, cy, fx, fy, xi, alpha, width=None, height=None):
        super().__init__(cx, cy, fx, fy, width, height)
        self.xi = xi
        self.alpha = alpha

    def project(self, x, y, z, eps=1e-9):
        r2 = x * x + y * y
        d1 = np.sqrt(r2 + z * z)
        k2 = self.xi * d1 + z
        d2 = np.sqrt(r2 + k2 * k2)
        denom_raw = self.alpha * d2 + (1.0 - self.alpha) * k2

        valid = denom_raw > 0
        denom = np.maximum(denom_raw, eps)

        mx = x / denom
        my = y / denom

        u = self.fx * mx + self.cx
        v = self.fy * my + self.cy

        return u, v, valid

    def unproject(self, u, v):
        mx = (u - self.cx) / self.fx
        my = (v - self.cy) / self.fy

        r2 = mx * mx + my * my
        mz = (1 - self.alpha * self.alpha * r2) / (self.alpha * np.sqrt(1 - (2 * self.alpha - 1) * r2) + 1 - self.alpha)

        scale = (mz * self.xi + np.sqrt(mz * mz + (1 - self.xi * self.xi) * r2)) / (mz * mz + r2)

        x = scale * mx
        y = scale * my
        z = scale * mz - self.xi

        norm = np.sqrt(x * x + y * y + z * z)
        return x / norm, y / norm, z / norm


class PinholeModel(CameraModel):
    def __init__(self, cx, cy, fx, fy, width=None, height=None):
        super().__init__(cx, cy, fx, fy, width, height)

    def project(self, x, y, z, eps=1e-9):
        valid = z > eps
        z_safe = np.maximum(z, eps)

        u = self.fx * (x / z_safe) + self.cx
        v = self.fy * (y / z_safe) + self.cy

        if self.width is not None and self.height is not None:
            valid = valid & (u >= 0) & (u < self.width) & (v >= 0) & (v < self.height)

        return u, v, valid

    def unproject(self, u, v):
        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy
        z = 1.0

        norm = np.sqrt(x * x + y * y + z * z)
        return x / norm, y / norm, z / norm


def create_equirectangular_rays(width, height, h_fov_deg=220.0):
    # Convert FOV to radians
    h_fov_rad = np.deg2rad(h_fov_deg)

    # Longitude range centered around 0, limited by h_fov
    lon = (np.linspace(0, width - 1, width) / (width - 1)) * h_fov_rad - (h_fov_rad / 2)
    lat = (np.linspace(0, height - 1, height) / (height - 1)) * np.pi - (np.pi / 2)
    lon_grid, lat_grid = np.meshgrid(lon, lat)

    x = np.cos(lat_grid) * np.sin(lon_grid)
    y = np.sin(lat_grid)
    z = np.cos(lat_grid) * np.cos(lon_grid)

    return x, y, z


def add_longitude_lines(img, h_fov_deg=220.0, num_lines=12, line_color=(0, 255, 0), text_color=(255, 255, 255)):
    """Add longitude lines and labels to an equirectangular image"""
    h, w = img.shape[:2]
    result = img.copy()

    # Calculate longitude range based on FOV
    lon_start = -h_fov_deg / 2
    lon_end = h_fov_deg / 2

    # Draw longitude lines and labels
    for i in range(num_lines):
        # Calculate longitude in degrees within the FOV range
        lon_deg = lon_start + (h_fov_deg * i / (num_lines - 1))

        # Skip if outside reasonable range
        if abs(lon_deg) > 180:
            continue

        # Calculate x position in image
        x = int(w * i / (num_lines - 1))

        # Draw vertical line
        cv2.line(result, (x, 0), (x, h - 1), line_color, 1)

        # Add text label at top and middle of image
        text = f"{lon_deg:+.0f}°"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1

        # Get text size for centering
        (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, thickness)

        # Draw text with background for visibility
        # Top position
        text_x = max(2, min(x - text_width // 2, w - text_width - 2))
        cv2.rectangle(result, (text_x - 2, 5), (text_x + text_width + 2, 5 + text_height + 4), (0, 0, 0), -1)
        cv2.putText(result, text, (text_x, 5 + text_height), font, font_scale, text_color, thickness)

        # Middle position
        mid_y = h // 2
        cv2.rectangle(
            result, (text_x - 2, mid_y - text_height - 2), (text_x + text_width + 2, mid_y + 2), (0, 0, 0), -1
        )
        cv2.putText(result, text, (text_x, mid_y), font, font_scale, text_color, thickness)

    # Add latitude lines (optional - equator and tropics)
    # Equator
    cv2.line(result, (0, h // 2), (w - 1, h // 2), (255, 0, 0), 1)

    # Tropic of Cancer/Capricorn (±23.5°)
    tropic_offset = int(h * 23.5 / 180)
    cv2.line(result, (0, h // 2 - tropic_offset), (w - 1, h // 2 - tropic_offset), (255, 0, 0), 1, cv2.LINE_AA)
    cv2.line(result, (0, h // 2 + tropic_offset), (w - 1, h // 2 + tropic_offset), (255, 0, 0), 1, cv2.LINE_AA)

    return result


def project_to_equirectangular(img, model, out_width, out_height=None, add_grid=False, h_fov_deg=220.0):
    if out_height is None:
        out_height = out_width // 2

    x, y, z = create_equirectangular_rays(out_width, out_height, h_fov_deg)

    u, v, valid = model.project(x, y, z)

    map_x = u.astype(np.float32)
    map_y = v.astype(np.float32)

    pano = cv2.remap(
        img, map_x, map_y, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0)
    )

    if pano.ndim == 3:
        pano[~valid] = (0, 0, 0)
    else:
        pano[~valid] = 0

    # Add longitude lines if requested
    if add_grid:
        pano = add_longitude_lines(pano, h_fov_deg)

    return pano


def project_to_pinhole(img, source_model, target_model, out_width, out_height):
    u_grid, v_grid = np.meshgrid(np.arange(out_width, dtype=np.float32), np.arange(out_height, dtype=np.float32))

    x, y, z = target_model.unproject(u_grid, v_grid)

    u_src, v_src, valid = source_model.project(x, y, z)

    map_x = u_src.astype(np.float32)
    map_y = v_src.astype(np.float32)

    projected = cv2.remap(
        img, map_x, map_y, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0)
    )

    if projected.ndim == 3:
        projected[~valid] = (0, 0, 0)
    else:
        projected[~valid] = 0

    return projected


def create_cube_face_model(face_size, direction):
    """Create a pinhole model for a cube face with 90° FOV"""
    # 90° FOV with given face size
    focal_length = face_size / (2 * np.tan(np.deg2rad(90) / 2))
    cx = cy = face_size / 2

    # Create rotation matrix for the face direction
    if direction == "front":
        # Looking forward (+Z direction)
        rotation = np.eye(3)
    elif direction == "up":
        # Looking up (+Y direction) - rotate around X axis by -90°
        rotation = np.array([
            [1, 0, 0],
            [0, 0, -1],
            [0, 1, 0]
        ])
    elif direction == "right":
        # Looking right (+X direction) - rotate around Y axis by +90°
        rotation = np.array([
            [0, 0, 1],
            [0, 1, 0],
            [-1, 0, 0]
        ])
    elif direction == "down":
        # Looking down (-Y direction) - rotate around X axis by +90°
        rotation = np.array([
            [1, 0, 0],
            [0, 0, 1],
            [0, -1, 0]
        ])
    elif direction == "left":
        # Looking left (-X direction) - rotate around Y axis by -90°
        rotation = np.array([
            [0, 0, -1],
            [0, 1, 0],
            [1, 0, 0]
        ])

    return PinholeModel(cx, cy, focal_length, focal_length, face_size, face_size), rotation


def project_cube_face(img, source_model, face_size, direction):
    """Project a single cube face"""
    target_model, rotation = create_cube_face_model(face_size, direction)

    # Create pixel grid for the face
    u_grid, v_grid = np.meshgrid(
        np.arange(face_size, dtype=np.float32),
        np.arange(face_size, dtype=np.float32)
    )

    # Unproject pixels to rays in cube face coordinate system
    x_face, y_face, z_face = target_model.unproject(u_grid, v_grid)

    # Apply rotation to get world coordinates
    rays = np.stack([x_face, y_face, z_face], axis=-1)  # [H, W, 3]
    rays_world = np.dot(rays, rotation.T)  # Rotate rays

    x_world = rays_world[..., 0]
    y_world = rays_world[..., 1]
    z_world = rays_world[..., 2]

    # Project using source camera model
    u_src, v_src, valid = source_model.project(x_world, y_world, z_world)

    map_x = u_src.astype(np.float32)
    map_y = v_src.astype(np.float32)

    # Sample from source image
    face_img = cv2.remap(
        img, map_x, map_y,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0)
    )

    # Mask invalid pixels
    if face_img.ndim == 3:
        face_img[~valid] = (0, 0, 0)
    else:
        face_img[~valid] = 0

    return face_img


def create_cube_projection(img, source_model, face_size):
    """Create a cube projection with 5 faces arranged in a cross pattern"""
    # Generate the 5 cube faces
    faces = {}
    directions = ["front", "up", "right", "down", "left"]

    print("Generating cube faces...")
    for direction in directions:
        print(f"  Processing {direction} face...")
        faces[direction] = project_cube_face(img, source_model, face_size, direction)

    # Arrange faces in cross pattern:
    #     [up]
    # [left][front][right]
    #     [down]

    cross_width = face_size * 3
    cross_height = face_size * 3

    if img.ndim == 3:
        cross_img = np.zeros((cross_height, cross_width, 3), dtype=np.uint8)
    else:
        cross_img = np.zeros((cross_height, cross_width), dtype=np.uint8)

    # Place faces in cross pattern
    # Up face (top center)
    cross_img[0:face_size, face_size:2*face_size] = faces["up"]

    # Left face (middle left)
    cross_img[face_size:2*face_size, 0:face_size] = faces["left"]

    # Front face (middle center)
    cross_img[face_size:2*face_size, face_size:2*face_size] = faces["front"]

    # Right face (middle right)
    cross_img[face_size:2*face_size, 2*face_size:3*face_size] = faces["right"]

    # Down face (bottom center)
    cross_img[2*face_size:3*face_size, face_size:2*face_size] = faces["down"]

    # Add labels to identify each face
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.7
    color = (255, 255, 255) if img.ndim == 3 else 255
    thickness = 2

    # Add text labels
    cv2.putText(cross_img, "UP", (face_size + 10, 30), font, font_scale, color, thickness)
    cv2.putText(cross_img, "LEFT", (10, face_size + 30), font, font_scale, color, thickness)
    cv2.putText(cross_img, "FRONT", (face_size + 10, face_size + 30), font, font_scale, color, thickness)
    cv2.putText(cross_img, "RIGHT", (2*face_size + 10, face_size + 30), font, font_scale, color, thickness)
    cv2.putText(cross_img, "DOWN", (face_size + 10, 2*face_size + 30), font, font_scale, color, thickness)

    return cross_img


def select_image(directory):
    images = []
    for ext in ["*.jpg", "*.jpeg", "*.png", "*.bmp"]:
        images.extend(Path(directory).glob(ext))

    if not images:
        raise ValueError(f"No images found in {directory}")

    print("\nAvailable images:")
    for i, img_path in enumerate(images):
        print(f"{i + 1}. {img_path.name}")

    while True:
        try:
            choice = input(f"\nSelect image (1-{len(images)}): ")
            idx = int(choice) - 1
            if 0 <= idx < len(images):
                return str(images[idx])
            else:
                print(f"Please enter a number between 1 and {len(images)}")
        except ValueError:
            print("Invalid input. Please enter a number.")


def load_camera_params(config_path):
    """Load camera parameters from YAML file"""
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    params = config.get("camera_params", {})
    return {
        "fx": params.get("fx", 228.81030),
        "fy": params.get("fy", 228.81030),
        "cx": params.get("cx", 680.0),
        "cy": params.get("cy", 479.0),
        "xi": params.get("xi", -0.00092),
        "alpha": params.get("alpha", 0.62007),
    }


def select_yaml_config(directory):
    """Select a YAML config file from the directory"""
    configs = []
    for ext in ["*.yaml", "*.yml"]:
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


def main():
    parser = argparse.ArgumentParser(description="Fisheye image undistortion and projection")
    parser.add_argument("--input", "-i", help="Input image path (if not specified, will prompt)")
    parser.add_argument("--output", "-o", default="output.jpg", help="Output image path")
    parser.add_argument(
        "--mode",
        "-m",
        choices=["equirect", "pinhole", "cube"],
        default="equirect",
        help="Output mode: equirect (equirectangular), pinhole, or cube (5-face cube projection)",
    )
    parser.add_argument("--width", "-w", type=int, default=4096, help="Output width")
    parser.add_argument("--height", type=int, help="Output height (default: width/2 for equirect)")

    parser.add_argument("--model", choices=["ds", "pinhole"], default="ds", help="Source camera model")

    parser.add_argument("--config", "-c", help="Path to YAML config file with camera params")

    # Target pinhole parameters
    parser.add_argument("--target-fov", type=float, default=90.0, help="Target FOV for pinhole output (degrees)")

    # Grid options
    parser.add_argument(
        "--grid", action="store_true", help="Add longitude/latitude grid lines to equirectangular output"
    )
    parser.add_argument(
        "--h-fov", type=float, default=220.0, help="Horizontal FOV for equirectangular output (degrees)"
    )

    args = parser.parse_args()

    # Default directory
    default_dir = "/Users/nathankau/pupperv3-monorepo/ai/playground/undistory"

    # Load camera parameters from YAML
    if args.config:
        config_path = args.config
    else:
        # Look for YAML files in undistory folder
        if os.path.exists(default_dir):
            config_path = select_yaml_config(default_dir)
        else:
            config_path = select_yaml_config(".")

    if config_path and os.path.exists(config_path):
        print(f"Loading parameters from: {config_path}")
        camera_params = load_camera_params(config_path)
    else:
        raise ValueError(
            "No camera configuration file found. Please create a camera_params.yaml file or specify --config"
        )

    # Select input image
    if args.input:
        in_path = args.input
    else:
        if os.path.exists(default_dir):
            in_path = select_image(default_dir)
        else:
            in_path = select_image(".")

    img = cv2.imread(in_path, cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(f"Could not load image: {in_path}")

    H_in, W_in = img.shape[:2]
    print(f"\nInput: {in_path}")
    print(f"Input size: {W_in}x{H_in}")

    if args.model == "ds":
        source_model = DoubleSphereModel(
            cx=camera_params["cx"],
            cy=camera_params["cy"],
            fx=camera_params["fx"],
            fy=camera_params["fy"],
            xi=camera_params["xi"],
            alpha=camera_params["alpha"],
            width=W_in,
            height=H_in,
        )
        print(
            f"Using DS model: fx={camera_params['fx']:.3f}, fy={camera_params['fy']:.3f}, "
            f"cx={camera_params['cx']:.1f}, cy={camera_params['cy']:.1f}, "
            f"xi={camera_params['xi']:.5f}, alpha={camera_params['alpha']:.5f}"
        )
    else:
        source_model = PinholeModel(
            cx=camera_params["cx"],
            cy=camera_params["cy"],
            fx=camera_params["fx"],
            fy=camera_params["fy"],
            width=W_in,
            height=H_in,
        )
        print(
            f"Using pinhole model: fx={camera_params['fx']:.3f}, fy={camera_params['fy']:.3f}, "
            f"cx={camera_params['cx']:.1f}, cy={camera_params['cy']:.1f}"
        )

    if args.mode == "equirect":
        # For square aspect ratio: height = width * (vertical_fov / horizontal_fov)
        # Vertical FOV is typically 180° (full sphere), horizontal FOV is customizable
        v_fov_deg = 180.0
        if args.height:
            out_height = args.height
        else:
            # Square aspect ratio: height proportional to vertical FOV coverage
            out_height = int(args.width * (v_fov_deg / args.h_fov))

        grid_text = " with grid overlay" if args.grid else ""
        print(f"Projecting to equirectangular: {args.width}x{out_height} (square aspect), {args.h_fov}° FOV{grid_text}")

        output = project_to_equirectangular(
            img, source_model, args.width, out_height, add_grid=args.grid, h_fov_deg=args.h_fov
        )
    elif args.mode == "pinhole":
        out_height = args.height if args.height else args.width

        target_fx = args.width / (2 * np.tan(np.deg2rad(args.target_fov) / 2))
        target_fy = out_height / (2 * np.tan(np.deg2rad(args.target_fov) / 2))
        target_model = PinholeModel(
            cx=args.width / 2, cy=out_height / 2, fx=target_fx, fy=target_fy, width=args.width, height=out_height
        )

        print(f"Projecting to pinhole: {args.width}x{out_height}, FOV={args.target_fov}°")
        output = project_to_pinhole(img, source_model, target_model, args.width, out_height)
    else:  # cube mode
        # For cube mode, width represents the face size
        face_size = args.width // 3  # Each face is 1/3 of the total output width
        print(f"Projecting to cube: {face_size}x{face_size} faces, 90° FOV per face")
        print(f"Output size: {face_size*3}x{face_size*3} (3x3 cross pattern)")

        output = create_cube_projection(img, source_model, face_size)

    cv2.imwrite(args.output, output)
    print(f"Saved: {args.output}")


if __name__ == "__main__":
    main()
