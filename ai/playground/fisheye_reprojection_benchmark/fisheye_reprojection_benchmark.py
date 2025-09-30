#!/usr/bin/env python3
"""
Fisheye to Pinhole Reprojection Benchmark
Uses Double Sphere Camera Model for accurate fisheye undistortion
Reference: https://vision.in.tum.de/research/vslam/double-sphere
"""

import numpy as np
import cv2
import time
from typing import Tuple, Optional
import argparse
from pathlib import Path


class CameraModel:
    """Base camera model class"""
    def __init__(self, cx: float, cy: float, fx: float, fy: float,
                 width: Optional[int] = None, height: Optional[int] = None):
        self.cx = cx
        self.cy = cy
        self.fx = fx
        self.fy = fy
        self.width = width
        self.height = height

        # Lookup tables for remapping
        self.map_x = None
        self.map_y = None


class DoubleSphereModel(CameraModel):
    """
    Implementation of the Double Sphere Camera Model
    Paper: The Double Sphere Camera Model, ICRA 2018
    """
    def __init__(self, cx: float, cy: float, fx: float, fy: float,
                 xi: float, alpha: float,
                 width: Optional[int] = None, height: Optional[int] = None):
        super().__init__(cx, cy, fx, fy, width, height)
        self.xi = xi
        self.alpha = alpha

    def project(self, x: np.ndarray, y: np.ndarray, z: np.ndarray,
                eps: float = 1e-9) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Project 3D points to fisheye image using double sphere model

        Args:
            x, y, z: 3D coordinates (can be arrays)
            eps: Small value to avoid division by zero

        Returns:
            u, v: Image coordinates
            valid: Boolean mask of valid projections
        """
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

    def unproject(self, u: np.ndarray, v: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Unproject fisheye image points to 3D rays using double sphere model

        Args:
            u, v: Image coordinates (can be arrays)

        Returns:
            x, y, z: Normalized 3D ray directions
        """
        mx = (u - self.cx) / self.fx
        my = (v - self.cy) / self.fy

        r2 = mx * mx + my * my
        mz = (1 - self.alpha * self.alpha * r2) / \
             (self.alpha * np.sqrt(1 - (2 * self.alpha - 1) * r2) + 1 - self.alpha)

        scale = (mz * self.xi + np.sqrt(mz * mz + (1 - self.xi * self.xi) * r2)) / \
                (mz * mz + r2)

        x = scale * mx
        y = scale * my
        z = scale * mz - self.xi

        norm = np.sqrt(x * x + y * y + z * z)
        return x / norm, y / norm, z / norm


class PinholeModel(CameraModel):
    """Standard pinhole camera model"""
    def __init__(self, cx: float, cy: float, fx: float, fy: float,
                 width: Optional[int] = None, height: Optional[int] = None):
        super().__init__(cx, cy, fx, fy, width, height)

    def project(self, x: np.ndarray, y: np.ndarray, z: np.ndarray,
                eps: float = 1e-9) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Project 3D points to pinhole image

        Args:
            x, y, z: 3D coordinates (can be arrays)
            eps: Small value to avoid division by zero

        Returns:
            u, v: Image coordinates
            valid: Boolean mask of valid projections
        """
        valid = z > eps
        z_safe = np.maximum(z, eps)

        u = self.fx * (x / z_safe) + self.cx
        v = self.fy * (y / z_safe) + self.cy

        if self.width is not None and self.height is not None:
            valid = valid & (u >= 0) & (u < self.width) & (v >= 0) & (v < self.height)

        return u, v, valid

    def unproject(self, u: np.ndarray, v: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Unproject pinhole image points to 3D rays

        Args:
            u, v: Image coordinates (can be arrays)

        Returns:
            x, y, z: Normalized 3D ray directions
        """
        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy
        z = np.ones_like(u)

        norm = np.sqrt(x * x + y * y + z * z)
        return x / norm, y / norm, z / norm


class FisheyeReprojector:
    """Class for reprojecting fisheye images to pinhole"""

    def __init__(self, fisheye_model: DoubleSphereModel,
                 pinhole_model: Optional[PinholeModel] = None):
        self.fisheye_model = fisheye_model
        self.pinhole_model = pinhole_model
        self.map_x = None
        self.map_y = None

    def create_remapping_lookup_tables(self, target_width: int, target_height: int,
                                      target_fx: Optional[float] = None,
                                      target_fy: Optional[float] = None,
                                      target_cx: Optional[float] = None,
                                      target_cy: Optional[float] = None,
                                      fov_scale: float = 1.0) -> Tuple[np.ndarray, np.ndarray]:
        """
        Create remapping lookup tables from fisheye to pinhole

        Args:
            target_width, target_height: Target image size
            target_fx, target_fy: Target focal lengths (if None, auto-computed)
            target_cx, target_cy: Target principal point (if None, uses center)
            fov_scale: Scale factor for FOV (< 1.0 reduces FOV)

        Returns:
            map_x, map_y: Remapping lookup tables for cv2.remap()
        """
        # Default pinhole parameters if not provided
        if target_fx is None:
            target_fx = min(target_width, target_height) * fov_scale
        if target_fy is None:
            target_fy = target_fx  # Square pixels
        if target_cx is None:
            target_cx = target_width / 2.0
        if target_cy is None:
            target_cy = target_height / 2.0

        print(f"Target intrinsics: fx={target_fx}, fy={target_fy}, cx={target_cx}, cy={target_cy}")

        # Create pinhole model for target
        # if self.pinhole_model is None:
        self.pinhole_model = PinholeModel(target_cx, target_cy, target_fx, target_fy,
                                            target_width, target_height)

        print(f"Creating remapping tables ({target_width}x{target_height}, FOV scale: {fov_scale:.2f})...")

        # Create grid of target pixel coordinates
        u_target, v_target = np.meshgrid(np.arange(target_width, dtype=np.float32),
                                        np.arange(target_height, dtype=np.float32))
        u_target = u_target.flatten()
        v_target = v_target.flatten()

        # Unproject from pinhole to 3D rays
        x_ray, y_ray, z_ray = self.pinhole_model.unproject(u_target, v_target)

        # Project rays to fisheye image
        u_fisheye, v_fisheye, valid = self.fisheye_model.project(x_ray, y_ray, z_ray)

        # Create mapping arrays
        map_x = np.full((target_height, target_width), -1, dtype=np.float32)
        map_y = np.full((target_height, target_width), -1, dtype=np.float32)

        # Fill valid mappings
        valid_mask = valid.reshape(target_height, target_width)
        map_x[valid_mask] = u_fisheye[valid].astype(np.float32)
        map_y[valid_mask] = v_fisheye[valid].astype(np.float32)

        # Check bounds if fisheye dimensions are known
        if self.fisheye_model.width and self.fisheye_model.height:
            bounds_valid = (map_x >= 0) & (map_x < self.fisheye_model.width) & \
                          (map_y >= 0) & (map_y < self.fisheye_model.height)
            map_x[~bounds_valid] = -1
            map_y[~bounds_valid] = -1

        print(f"  Valid pixels: {np.sum(map_x >= 0)} / {target_width * target_height}")
        return map_x, map_y

    def reproject_to_pinhole(self, fisheye_image: np.ndarray,
                            target_width: int = 640,
                            target_height: int = 480,
                            fov_scale: float = 0.6) -> np.ndarray:
        """
        Reproject fisheye image to pinhole camera image

        Args:
            fisheye_image: Input fisheye image
            target_width, target_height: Output dimensions
            fov_scale: Scale factor for FOV (< 1.0 reduces FOV)

        Returns:
            Reprojected pinhole image
        """
        # Update fisheye dimensions if needed
        if self.fisheye_model.width is None:
            self.fisheye_model.width = fisheye_image.shape[1]
        if self.fisheye_model.height is None:
            self.fisheye_model.height = fisheye_image.shape[0]

        # Create or reuse mapping tables
        if self.map_x is None or self.map_x.shape != (target_height, target_width):
            self.map_x, self.map_y = self.create_remapping_lookup_tables(
                target_width, target_height, fov_scale=fov_scale
            )

        # Apply remapping
        pinhole_image = cv2.remap(fisheye_image, self.map_x, self.map_y,
                                 interpolation=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_CONSTANT,
                                 borderValue=(0, 0, 0))

        return pinhole_image


def benchmark_reprojection(fisheye_model: DoubleSphereModel,
                          test_image: Optional[np.ndarray] = None,
                          num_iterations: int = 100,
                          target_sizes: list = [(640, 480), (1280, 720), (1920, 1080)],
                          fov_scales: list = [0.4, 0.6, 0.8]):
    """
    Benchmark the reprojection performance

    Args:
        fisheye_model: DoubleSphereModel instance
        test_image: Test image (if None, creates synthetic)
        num_iterations: Number of iterations for timing
        target_sizes: List of target resolutions to test
        fov_scales: List of FOV scale factors to test
    """
    # Create test image if not provided
    if test_image is None:
        print("Creating synthetic test image...")
        width = fisheye_model.width or 1360
        height = fisheye_model.height or 960
        test_image = np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)
        # Add some patterns for visual validation
        cv2.circle(test_image, (width//2, height//2), 200, (0, 255, 0), 5)
        for i in range(0, width, 100):
            cv2.line(test_image, (i, 0), (i, height), (255, 0, 0), 1)
        for i in range(0, height, 100):
            cv2.line(test_image, (0, i), (width, i), (255, 0, 0), 1)

    # Create reprojector
    reprojector = FisheyeReprojector(fisheye_model)

    print("\n" + "="*60)
    print("FISHEYE TO PINHOLE REPROJECTION BENCHMARK")
    print("="*60)
    print(f"Input image size: {test_image.shape[1]}x{test_image.shape[0]}")
    print(f"Double Sphere params: xi={fisheye_model.xi:.5f}, alpha={fisheye_model.alpha:.5f}")
    print(f"Iterations per test: {num_iterations}")
    print("-"*60)

    results = []

    for target_width, target_height in target_sizes:
        for fov_scale in fov_scales:
            print(f"\nTarget: {target_width}x{target_height}, FOV scale: {fov_scale:.2f}")

            # First run to create lookup tables
            print("  Creating lookup tables...")
            start_init = time.perf_counter()
            reprojector.map_x, reprojector.map_y = reprojector.create_remapping_lookup_tables(
                target_width, target_height, fov_scale=fov_scale
            )
            init_time = time.perf_counter() - start_init
            print(f"  Lookup table creation: {init_time*1000:.2f} ms")

            # Benchmark remapping operation
            print(f"  Running {num_iterations} iterations...")
            times = []
            for _ in range(num_iterations):
                start = time.perf_counter()
                output = cv2.remap(test_image, reprojector.map_x, reprojector.map_y,
                                 interpolation=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_CONSTANT)
                elapsed = time.perf_counter() - start
                times.append(elapsed)

            # Calculate statistics
            times = np.array(times) * 1000  # Convert to milliseconds
            mean_time = np.mean(times)
            std_time = np.std(times)
            min_time = np.min(times)
            max_time = np.max(times)
            fps = 1000.0 / mean_time

            print(f"  Performance:")
            print(f"    Mean: {mean_time:.3f} ms (±{std_time:.3f} ms)")
            print(f"    Min/Max: {min_time:.3f} / {max_time:.3f} ms")
            print(f"    FPS: {fps:.1f}")

            results.append({
                'resolution': f'{target_width}x{target_height}',
                'fov_scale': fov_scale,
                'init_time_ms': init_time * 1000,
                'mean_time_ms': mean_time,
                'fps': fps
            })

            # Save sample output
            output_path = f'fisheye_to_pinhole_{target_width}x{target_height}_fov{fov_scale:.1f}.jpg'
            cv2.imwrite(output_path, output)
            print(f"  Sample output saved to: {output_path}")

            # Only test one FOV scale for larger resolutions (to save time)
            if target_width >= 1920:
                break

    # Summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    print(f"{'Resolution':<15} {'FOV Scale':<10} {'Init (ms)':<12} {'Process (ms)':<12} {'FPS':<10}")
    print("-"*60)
    for r in results:
        print(f"{r['resolution']:<15} {r['fov_scale']:<10.2f} {r['init_time_ms']:<12.2f} {r['mean_time_ms']:<12.3f} {r['fps']:<10.1f}")

    return results


def main():
    parser = argparse.ArgumentParser(description='Benchmark fisheye to pinhole reprojection')
    parser.add_argument('--image', type=str, help='Path to fisheye image')
    parser.add_argument('--iterations', type=int, default=100,
                       help='Number of iterations for benchmark')
    parser.add_argument('--visualize', action='store_true',
                       help='Show visualization of reprojection')
    parser.add_argument('--quick', action='store_true',
                       help='Quick test with fewer sizes/iterations')
    args = parser.parse_args()

    # Initialize fisheye camera model with provided Double Sphere parameters
    fisheye_model = DoubleSphereModel(
        cx=680.0,
        cy=479.0,
        fx=228.81030,
        fy=228.81030,
        xi=-0.00092,
        alpha=0.62007,
        width=1400,
        height=1050
    )

    # Load test image if provided
    test_image = None
    if args.image:
        test_image = cv2.imread(args.image)
        if test_image is not None:
            print(f"Loaded image: {args.image}")
            fisheye_model.width = test_image.shape[1]
            fisheye_model.height = test_image.shape[0]

    # Configure benchmark parameters
    if args.quick:
        target_sizes = [(640, 480)]
        fov_scales = [0.6]
        iterations = 10
    else:
        # target_sizes = [(640, 480), (1280, 720), (1920, 1080), (1400, 1050)]
        target_sizes = [(1400, 1050)]
        fov_scales = [0.4, 0.6, 0.8]
        iterations = args.iterations

    # Run benchmark
    results = benchmark_reprojection(fisheye_model, test_image, iterations,
                                    target_sizes, fov_scales)

    # Visualization if requested
    if args.visualize:
        print("\nShowing visualization (press any key to continue)...")

        # Create reprojector for visualization
        reprojector = FisheyeReprojector(fisheye_model)

        if test_image is None:
            # Create test pattern for visualization
            width = fisheye_model.width or 1360
            height = fisheye_model.height or 960
            test_image = np.zeros((height, width, 3), dtype=np.uint8)
            # Create checkerboard pattern
            square_size = 50
            for i in range(0, height, square_size):
                for j in range(0, width, square_size):
                    if (i // square_size + j // square_size) % 2 == 0:
                        test_image[i:i+square_size, j:j+square_size] = [255, 255, 255]
            # Add circles
            cv2.circle(test_image, (width//2, height//2), 300, (0, 255, 0), 3)
            cv2.circle(test_image, (width//2, height//2), 200, (0, 0, 255), 3)
            cv2.circle(test_image, (width//2, height//2), 100, (255, 0, 0), 3)

        # Create reprojections at different FOVs
        for fov_scale in [0.4, 0.6, 0.8]:
            output = reprojector.reproject_to_pinhole(test_image,
                                                     target_width=640,
                                                     target_height=480,
                                                     fov_scale=fov_scale)
            cv2.imshow(f'FOV scale: {fov_scale:.1f}', output)

        cv2.imshow('Original Fisheye', cv2.resize(test_image, (680, 480)))
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()