"""ROS-independent geometry. Python 3.8 / NumPy / OpenCV 4.2 compatible.

Ground: X forward, Y left, Z up. Camera: x right, y down, z forward.
BEV columns run right (-Y); rows run backward (-X). Samples are cell centers.
The distortion domain is deliberately bounded: this calibration folds at large
normalized radii. Pixels outside the central invertible domain are not invented.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


def read_yaml(path):
    with Path(path).open('r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def calibration_fingerprint(calibration):
    keys = ('image_size', 'distortion_model', 'camera_matrix', 'distortion_coefficients')
    data = {k: calibration[k] for k in keys}
    return hashlib.sha256(json.dumps(data, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def camera_rotation(pitch_deg, roll_deg, yaw_deg):
    """Ground -> optical rotation; positive pitch looks down, yaw looks left.

    Mount convention: R_ground_body = Rz(yaw) Ry(pitch) Rx(roll).
    """
    p, r, y = np.deg2rad([pitch_deg, roll_deg, yaw_deg])
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0],
                   [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    optical = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
    return optical @ (rz @ ry @ rx).T


class CameraModel:
    def __init__(self, calibration):
        if calibration.get('distortion_model') != 'plumb_bob':
            raise ValueError('Only the calibrated five-coefficient plumb_bob model is supported')
        self.k = np.asarray(calibration['camera_matrix'], dtype=np.float64)
        self.d = np.asarray(calibration['distortion_coefficients'], dtype=np.float64)
        self.size = tuple(calibration['image_size'])
        if (self.k.shape != (3, 3) or self.d.shape != (5,)
                or not np.all(np.isfinite(self.k)) or not np.all(np.isfinite(self.d))
                or self.k[0, 0] <= 0 or self.k[1, 1] <= 0
                or not np.allclose(self.k[2], [0, 0, 1])
                or self.k[0, 1] != 0 or self.k[1, 0] != 0
                or len(self.size) != 2
                or any(not isinstance(s, int) or s < 2 for s in self.size)):
            raise ValueError('Invalid calibration matrix, coefficients or image size')
        k1, k2, _, _, k3 = self.d
        roots = np.roots([7 * k3, 5 * k2, 3 * k1, 1])
        positive = [v.real for v in roots if abs(v.imag) < 1e-8 and v.real > 0]
        self.radial_limit = np.sqrt(min(positive)) if positive else float('inf')
        # For every direction, lambda_min(J_tangential) >= -6*||p||*r.
        # Bound both radial-Jacobian eigenvalues on the entire central disk.
        tangent_norm = float(np.linalg.norm(self.d[2:4]))
        limits = []
        for coeffs in ([k3, 0, k2, 0, k1, -6 * tangent_norm, 1],
                       [7 * k3, 0, 5 * k2, 0, 3 * k1, -6 * tangent_norm, 1]):
            limits.extend(v.real for v in np.roots(coeffs) if abs(v.imag) < 1e-8 and v.real > 0)
        self.invertible_limit = min(limits) if limits else float('inf')

    def distort(self, x, y):
        k1, k2, p1, p2, k3 = self.d
        r2 = x * x + y * y
        radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
        slope = k1 + r2 * (2 * k2 + r2 * 3 * k3)
        xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        j11 = radial + 2 * x * x * slope + 2 * p1 * y + 6 * p2 * x
        j22 = radial + 2 * y * y * slope + 6 * p1 * y + 2 * p2 * x
        j12 = 2 * x * y * slope + 2 * p1 * x + 2 * p2 * y
        return xd, yd, j11, j22, j12

    def project(self, xyz, max_radius=1.2):
        """Project rays to raw; validity also excludes behind-camera/folded rays.

        The configured radius must be within the central positive-Jacobian disk,
        checked when constructing the mapper. Geometry-valid is not a guarantee
        of unoccluded ground or of physical calibration accuracy.
        """
        xyz = np.asarray(xyz, dtype=np.float64)
        z = xyz[..., 2]
        x = np.divide(xyz[..., 0], z, out=np.zeros_like(z), where=z > 1e-9)
        y = np.divide(xyz[..., 1], z, out=np.zeros_like(z), where=z > 1e-9)
        domain = ((z > 1e-9) & np.isfinite(x) & np.isfinite(y)
                  & (x * x + y * y < min(max_radius, self.radial_limit) ** 2))
        # Avoid evaluating the polynomial on huge near-horizon rays.
        x = np.where(domain, x, 0.0)
        y = np.where(domain, y, 0.0)
        xd, yd, a, b, c = self.distort(x, y)
        u = self.k[0, 0] * xd + self.k[0, 2]
        v = self.k[1, 1] * yd + self.k[1, 2]
        valid = (domain & (a > 1e-6) & (a * b - c * c > 1e-6)
                 & (u >= 0) & (u < self.size[0] - 1)
                 & (v >= 0) & (v < self.size[1] - 1))
        return u, v, valid

    def validate_radius(self, radius):
        """Conservative global positive-Jacobian disk, including a 1% margin."""
        if not np.isfinite(radius) or radius <= 0 or radius >= self.radial_limit:
            raise ValueError('max_normalized_radius exceeds the radial fold or is invalid')
        if radius * 1.01 >= self.invertible_limit:
            raise ValueError('max_normalized_radius exceeds the central invertible domain')

    def undistort_points(self, pixels, radius=1.2):
        """Newton inverse for diagnostics, with residual/domain checks.

        Does not rely on newer OpenCV undistortPointsIter Python bindings.
        """
        pixels = np.asarray(pixels, dtype=float).reshape(-1, 2)
        target = (pixels - self.k[:2, 2]) / [self.k[0, 0], self.k[1, 1]]
        xy = target.copy()
        for _ in range(80):
            xd, yd, a, b, c = self.distort(xy[:, 0], xy[:, 1])
            ex, ey = xd - target[:, 0], yd - target[:, 1]
            det = a * b - c * c
            good = det > 1e-8
            step = np.zeros_like(xy)
            step[good, 0] = (b[good] * ex[good] - c[good] * ey[good]) / det[good]
            step[good, 1] = (a[good] * ey[good] - c[good] * ex[good]) / det[good]
            # Limit steps and keep invalid inputs from diverging.
            step /= np.maximum(1, np.linalg.norm(step, axis=1) / 0.1)[:, None]
            xy -= step
            xy *= np.minimum(1, radius / np.maximum(np.linalg.norm(xy, axis=1), 1e-9))[:, None]
        u, v, valid = self.project(np.c_[xy, np.ones(len(xy))], radius)
        residual = np.linalg.norm(np.c_[u, v] - pixels, axis=1)
        return xy, valid & (residual < 0.01), residual


class BEVMapper:
    def __init__(self, calibration, settings):
        self.settings = deepcopy(settings)
        self.camera = CameraModel(calibration)
        g = settings['ground']
        self.ground = deepcopy(g)
        values = [v for k, v in g.items() if k != 'measured']
        if not np.all(np.isfinite(values)) or g['height_m'] <= 0:
            raise ValueError('Ground parameters must be finite; camera height must be positive')
        if not -89 < g['pitch_deg'] < 89 or abs(g['roll_deg']) >= 89:
            raise ValueError('Pitch/roll must be between -89 and 89 degrees')
        self.radius = float(settings['max_normalized_radius'])
        self.camera.validate_radius(self.radius)
        self.rotation = camera_rotation(g['pitch_deg'], g['roll_deg'], g['yaw_deg'])
        self.center = np.array([g['offset_x_m'], g['offset_y_m'], g['height_m']])
        self.ground_projection = deepcopy(settings.get('ground_projection'))
        self.ground_homography = None
        if self.ground_projection is not None:
            projection = self.ground_projection
            if not isinstance(projection, dict) or projection.get('type') != 'normalized_homography':
                raise ValueError('Unsupported ground projection')
            if projection.get('reference_ground') != g:
                raise ValueError('Calibrated ground projection is fixed; regenerate the profile to change ground pose')
            if projection.get('calibration_fingerprint') != calibration_fingerprint(calibration):
                raise ValueError('Ground projection was fitted with different camera intrinsics')
            matrix = np.asarray(projection.get('ground_to_normalized'), dtype=float)
            if (matrix.shape != (3, 3) or not np.isfinite(matrix).all()
                    or np.linalg.cond(matrix) > 1e8):
                raise ValueError('Ground homography must be finite, nonsingular and well-conditioned')
            self.ground_homography = matrix
            self.inverse_ground_homography = np.linalg.inv(matrix)
        o = settings['output']
        self.resolution = float(o['resolution_m'])
        self.x_min, self.x_max = float(o['x_min_m']), float(o['x_max_m'])
        self.y_min, self.y_max = float(o['y_min_m']), float(o['y_max_m'])
        bounds = [self.x_min, self.x_max, self.y_min, self.y_max, self.resolution]
        if not np.all(np.isfinite(bounds)) or self.resolution <= 0:
            raise ValueError('Output bounds/resolution must be finite; resolution must be positive')
        wf = (self.y_max - self.y_min) / self.resolution
        hf = (self.x_max - self.x_min) / self.resolution
        self.width, self.height = int(round(wf)), int(round(hf))
        if (min(self.width, self.height) < 2 or max(self.width, self.height) > 2048
                or abs(wf - self.width) > 1e-6 or abs(hf - self.height) > 1e-6):
            raise ValueError('Grid dimensions must be integral and between 2 and 2048 pixels')
        self.x = self.x_max - (np.arange(self.height) + 0.5) * self.resolution
        self.y = self.y_max - (np.arange(self.width) + 0.5) * self.resolution
        xx, yy = np.meshgrid(self.x, self.y, indexing='ij')
        self.ground_points = np.stack([xx, yy, np.zeros_like(xx)], axis=-1)
        if self.ground_homography is None:
            self.camera_points = (self.ground_points - self.center) @ self.rotation.T
        else:
            # Homogeneous normalized camera rays, not a newly estimated 3-D pose.
            # Lens distortion is still applied below, before the single raw remap.
            self.camera_points = np.stack([xx, yy, np.ones_like(xx)], axis=-1) @ self.ground_homography.T
        u, v, valid = self.camera.project(self.camera_points, self.radius)
        self.map_x = np.where(valid, u, -1).astype(np.float32)
        self.map_y = np.where(valid, v, -1).astype(np.float32)
        self.valid_mask = (valid * 255).astype(np.uint8)
        self.maps = cv2.convertMaps(self.map_x, self.map_y, cv2.CV_16SC2)
        r = settings['rectification']
        rw, rh = int(r['width']), int(r['height'])
        focal = float(r['focal_px'])
        if min(rw, rh) < 2 or max(rw, rh) > 2048 or not np.isfinite(focal) or focal <= 0:
            raise ValueError('Invalid rectification output size/focal length')
        self.rect_k = np.array([[focal, 0, (rw - 1) / 2], [0, focal, (rh - 1) / 2], [0, 0, 1]])
        ru, rv = np.meshgrid(np.arange(rw), np.arange(rh))
        rays = np.stack([(ru - self.rect_k[0, 2]) / focal,
                         (rv - self.rect_k[1, 2]) / focal, np.ones_like(ru)], axis=-1)
        u, v, valid = self.camera.project(rays, self.radius)
        self.rect_valid_mask = (valid * 255).astype(np.uint8)
        self.rect_maps = cv2.convertMaps(np.where(valid, u, -1).astype(np.float32),
                                        np.where(valid, v, -1).astype(np.float32), cv2.CV_16SC2)
        self.debug_overlay, self.debug_alpha = self._overlay()

    def check_image(self, image):
        if image is None or image.dtype != np.uint8 or image.shape != (self.camera.size[1], self.camera.size[0], 3):
            raise ValueError('Expected calibrated {}x{} BGR8 image; input is never silently resized'.format(*self.camera.size))

    def render(self, image):
        self.check_image(image)
        return cv2.remap(image, *self.maps, interpolation=cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=0)

    def rectify(self, image):
        self.check_image(image)
        return cv2.remap(image, *self.rect_maps, interpolation=cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=0)

    def pixel_for_ground(self, x, y):
        return ((self.y_max - np.asarray(y)) / self.resolution - 0.5,
                (self.x_max - np.asarray(x)) / self.resolution - 0.5)

    def raw_to_ground(self, pixels):
        xy, valid, residual = self.camera.undistort_points(pixels, self.radius)
        if self.ground_homography is not None:
            homogeneous = np.c_[xy, np.ones(len(xy))] @ self.inverse_ground_homography.T
            denominator = homogeneous[:, 2]
            good = denominator > 1e-9
            points = np.zeros((len(xy), 3))
            points[:, :2] = np.divide(homogeneous[:, :2], denominator[:, None],
                                     out=np.zeros((len(xy), 2)), where=good[:, None])
            return points, valid & good & np.isfinite(points).all(axis=1), residual
        rays = np.c_[xy, np.ones(len(xy))] @ self.rotation
        dz = rays[:, 2]
        scale = np.divide(-self.center[2], dz, out=np.zeros_like(dz), where=dz < -1e-9)
        points = self.center + scale[:, None] * rays
        return points, valid & (dz < -1e-9), residual

    def _overlay(self):
        overlay = np.zeros((self.height, self.width, 3), np.uint8)
        for x in np.arange(np.ceil(self.x_min / .1) * .1, self.x_max, .1):
            _, row = self.pixel_for_ground(x, 0)
            row = int(round(float(row)))
            cv2.line(overlay, (0, row), (self.width - 1, row), (75, 75, 75), 1)
            cv2.putText(overlay, '{:.1f}'.format(x), (5, row - 3), cv2.FONT_HERSHEY_SIMPLEX, .35, (200, 200, 200), 1)
        for y in np.arange(np.ceil(self.y_min / .1) * .1, self.y_max, .1):
            col, _ = self.pixel_for_ground(0, y)
            cv2.line(overlay, (int(round(float(col))), 0), (int(round(float(col))), self.height - 1), (75, 75, 75), 1)
        col, _ = self.pixel_for_ground(0, 0)
        col = int(round(float(col)))
        cv2.arrowedLine(overlay, (col, self.height - 30), (col, max(35, self.height - 100)), (255, 200, 40), 2)
        cv2.putText(overlay, '+X forward | +Y left', (8, self.height - 10), cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 200, 40), 1)
        contours = cv2.findContours(self.valid_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cv2.drawContours(overlay, contours, -1, (230, 120, 30), 1)
        label = ('BOARD SCALE / APPROX ORIGIN' if self.ground_homography is not None
                 else 'MEASURED POSE' if self.ground['measured'] else 'ESTIMATED SCALE / POSE')
        cv2.rectangle(overlay, (0, 0), (self.width - 1, 29), (28, 28, 28), -1)
        cv2.putText(overlay, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .52, (0, 200, 255), 1)
        return overlay, np.any(overlay != 0, axis=2)

    def debug(self, bev):
        output = bev.copy()
        invalid = self.valid_mask == 0
        output[invalid] = (28, 20, 28)
        output[self.debug_alpha] = self.debug_overlay[self.debug_alpha]
        return output

    def info(self):
        return dict(ground=self.ground, output=self.settings['output'],
                    ground_projection=self.ground_projection,
                    shape=[self.height, self.width], valid_fraction=float(np.mean(self.valid_mask > 0)),
                    max_normalized_radius=self.radius, radial_fold_radius=float(self.camera.radial_limit),
                    conservative_invertible_radius=float(self.camera.invertible_limit),
                    mask_meaning='geometric support only; not ground/occlusion/lane confidence',
                    scale=('checkerboard metric plane; approximate camera-relative origin; not a calibrated robot transform'
                           if self.ground_homography is not None else
                           'measured parameters' if self.ground['measured'] else 'assumed height; estimated pitch'))
