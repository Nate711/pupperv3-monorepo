import cv2
import numpy as np
import yaml


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
