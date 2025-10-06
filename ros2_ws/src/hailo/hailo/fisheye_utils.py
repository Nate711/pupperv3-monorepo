import cv2
import numpy as np
import yaml
from PIL import Image
import os
from typing import Tuple, List, Dict, Any


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


def create_equirectangular_rays(width: float, height: float, h_fov_deg: float, v_fov_deg: float):
    # Convert FOV to radians
    h_fov_rad = np.deg2rad(h_fov_deg)
    v_fov_rad = np.deg2rad(v_fov_deg)

    # Longitude range centered around 0, limited by h_fov
    lon = (np.linspace(0, width - 1, width) / (width - 1)) * h_fov_rad - (h_fov_rad / 2)
    lat = (np.linspace(0, height - 1, height) / (height - 1)) * v_fov_rad - (v_fov_rad / 2)
    lon_grid, lat_grid = np.meshgrid(lon, lat)

    x = np.cos(lat_grid) * np.sin(lon_grid)
    y = np.sin(lat_grid)
    z = np.cos(lat_grid) * np.cos(lon_grid)

    return x, y, z


class FisheyeToEquirectangular:
    def __init__(self, out_width, out_height, h_fov_deg, v_fov_deg, fisheye_model):
        x, y, z = create_equirectangular_rays(out_width, out_height, h_fov_deg, v_fov_deg)
        u, v, self.valid = fisheye_model.project(x, y, z)
        self.map_x = u.astype(np.float32)
        self.map_y = v.astype(np.float32)

    def project(self, img):
        pano = cv2.remap(
            img,
            self.map_x,
            self.map_y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )

        if pano.ndim == 3:
            pano[~self.valid] = (0, 0, 0)
        else:
            pano[~self.valid] = 0

        return pano


def project_to_equirectangular(img, model, out_width, out_height=None, h_fov_deg=220.0):
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

    return pano


def load_camera_params(config_path):
    """Load camera parameters from YAML file"""
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    params = config.get("camera_params", {})
    return {
        "fx": params["fx"],
        "fy": params["fy"],
        "cx": params["cx"],
        "cy": params["cy"],
        "xi": params["xi"],
        "alpha": params["alpha"],
    }


def create_fisheye_model_from_params(config_path, img_width, img_height):
    """Create a DoubleSphereModel from camera parameters file"""
    params = load_camera_params(config_path)
    return DoubleSphereModel(
        cx=params["cx"],
        cy=params["cy"],
        fx=params["fx"],
        fy=params["fy"],
        xi=params["xi"],
        alpha=params["alpha"],
        width=img_width,
        height=img_height,
    )


def equirectangular_pixel_to_ray(u, v, width, height, h_fov_deg=220.0):
    """Convert equirectangular image pixel to 3D ray direction

    Args:
        u: pixel x coordinate (or array)
        v: pixel y coordinate (or array)
        width: image width
        height: image height
        h_fov_deg: horizontal field of view in degrees

    Returns:
        x, y, z: normalized 3D ray direction(s)
    """
    h_fov_rad = np.deg2rad(h_fov_deg)

    # Convert pixel coordinates to longitude/latitude
    lon = (u / (width - 1)) * h_fov_rad - (h_fov_rad / 2)
    lat = (v / (height - 1)) * np.pi - (np.pi / 2)

    # Convert to 3D ray
    x = np.cos(lat) * np.sin(lon)
    y = np.sin(lat)
    z = np.cos(lat) * np.cos(lon)

    return x, y, z


def equirectangular_pixel_to_elevation_heading(u, v, width, height, h_fov_deg, v_fov_deg):
    """Convert equirectangular image pixel to elevation/heading in degrees

    Args:
        u: pixel x coordinate (or array)
        v: pixel y coordinate (or array)
        width: image width
        height: image height
        h_fov_deg: horizontal field of view in degrees

    Returns:
        elevation_deg, heading_deg: elevation and heading in degrees
    """
    h_fov_rad = np.deg2rad(h_fov_deg)
    v_fov_rad = np.deg2rad(v_fov_deg)

    # Convert pixel coordinates to longitude/latitude in radians
    heading_rad = (u / (width - 1)) * h_fov_rad - (h_fov_rad / 2)
    elevation_rad = (1.0 - v / (height - 1)) * v_fov_rad - (v_fov_rad / 2)

    # Convert to degrees
    elevation_deg = np.rad2deg(elevation_rad)
    heading_deg = np.rad2deg(heading_rad)

    return elevation_deg, heading_deg


def bounding_box_to_rays(box_corners, equirect_width, equirect_height, h_fov_deg=220.0):
    """Convert bounding box corners from equirectangular image to 3D rays

    Args:
        box_corners: list of (x, y) pixel coordinates for box corners
        equirect_width: equirectangular image width
        equirect_height: equirectangular image height
        h_fov_deg: horizontal field of view in degrees

    Returns:
        rays: list of (x, y, z) normalized 3D ray directions
    """
    rays = []
    for u, v in box_corners:
        x, y, z = equirectangular_pixel_to_ray(u, v, equirect_width, equirect_height, h_fov_deg)
        rays.append((x, y, z))
    return rays


def fisheye_to_equirectangular(
    image: Image.Image, camera_params_path: str = None
) -> Tuple[Image.Image, int, int, float]:
    """Convert fisheye image to equirectangular projection.

    Args:
        image: PIL Image in fisheye projection
        camera_params_path: Path to camera parameters YAML file. If None, uses default location.

    Returns:
        Tuple of (equirect_image, width, height, h_fov_deg)
    """
    img_array = np.array(image)
    h, w = img_array.shape[:2]

    # Use default path if not provided
    if camera_params_path is None:
        camera_params_path = os.path.join(os.path.dirname(__file__), "camera_params.yaml")

    # Load camera parameters and create fisheye model
    fisheye_model = create_fisheye_model_from_params(camera_params_path, w, h)

    # Project to equirectangular
    equirect_width = w
    equirect_height = int(w * (180.0 / 200.0))  # Maintain aspect ratio for 200° horizontal FOV
    h_fov_deg = 200.0

    equirect_array = project_to_equirectangular(img_array, fisheye_model, equirect_width, equirect_height, h_fov_deg)
    # equirect_image = Image.fromarray(equirect_array.astype(np.uint8))

    return equirect_array, equirect_width, equirect_height, h_fov_deg


def convert_boxes_to_elevation_heading(
    boxes: List[Any], equirect_width: int, equirect_height: int, h_fov_deg: float
) -> List[Dict[str, Any]]:
    """Convert bounding box centroids to elevation/heading coordinates.

    Args:
        boxes: List of bounding box objects with x1, y1, x2, y2, label
        equirect_width: Width of equirectangular image
        equirect_height: Height of equirectangular image
        h_fov_deg: Horizontal field of view in degrees

    Returns:
        List of dictionaries containing label, centroid, lat_deg, lon_deg
    """
    objects_list = []
    for box in boxes:
        # Calculate centroid
        centroid_u = (box.x1 + box.x2) / 2.0
        centroid_v = (box.y1 + box.y2) / 2.0

        print(f"Centroid (u, v): ({centroid_u}, {centroid_v})")

        # Convert to elevation/heading
        elevation_deg, heading_deg = equirectangular_pixel_to_elevation_heading(
            centroid_u, centroid_v, equirect_width, equirect_height, h_fov_deg
        )

        print(f"Elevation/Heading: ({elevation_deg}, {heading_deg})")
        objects_list.append(
            {
                "label": box.label,
                "centroid": (centroid_u, centroid_v),
                "elevation_deg": elevation_deg,
                "heading_deg": heading_deg,
            }
        )
    return objects_list
