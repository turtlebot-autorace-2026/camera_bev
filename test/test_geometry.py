from copy import deepcopy
from pathlib import Path

import cv2
import numpy as np
import pytest

from camera_bev.geometry import BEVMapper, CameraModel, camera_rotation, read_yaml


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def calibration():
    return read_yaml(ROOT / 'config/camera_1280x720.yaml')


@pytest.fixture(scope='module')
def settings():
    return read_yaml(ROOT / 'config/initial_20261028.yaml')


@pytest.fixture(scope='module')
def mapper(calibration, settings):
    return BEVMapper(calibration, settings)


def test_projection_matches_opencv_for_nonzero_pose(calibration, settings):
    settings = deepcopy(settings)
    settings['ground'].update(roll_deg=3.0, yaw_deg=-7.0, offset_x_m=.04, offset_y_m=-.03)
    m = BEVMapper(calibration, settings)
    points = m.ground_points[::13, ::13].reshape(-1, 3)
    projected = cv2.projectPoints(points, cv2.Rodrigues(m.rotation)[0],
                                  -m.rotation @ m.center, m.camera.k, m.camera.d)[0][:, 0]
    u, v, valid = m.camera.project((points - m.center) @ m.rotation.T, m.radius)
    assert np.sum(valid) > 500
    np.testing.assert_allclose(np.c_[u, v][valid], projected[valid], atol=1e-8)


def test_ground_axes_and_optical_center(mapper):
    np.testing.assert_allclose(camera_rotation(0, 0, 0) @ [1, 0, 0], [0, 0, 1])
    np.testing.assert_allclose(camera_rotation(0, 0, 0) @ [0, 1, 0], [-1, 0, 0])
    g, valid, _ = mapper.raw_to_ground([mapper.camera.k[:2, 2]])
    assert valid[0]
    np.testing.assert_allclose(g[0], [.12 / np.tan(np.deg2rad(31.6)), 0, 0], atol=1e-9)
    u1, v1 = mapper.pixel_for_ground(.2, .1)
    u2, v2 = mapper.pixel_for_ground(.4, -.1)
    assert u1 < u2 and v1 > v2  # left ground is left image, forward is up


def test_no_fold_or_behind_camera_samples(mapper):
    camera = mapper.camera
    _, _, valid = camera.project(np.array([[0, 0, -1], [2, 0, 1], [0, 0, 0], [.1, .1, 1]]), mapper.radius)
    assert valid.tolist() == [False, False, False, True]
    with pytest.raises(ValueError):
        camera.validate_radius(1.2)  # tangential fold precedes radial-only fold


def test_inverse_roundtrip_and_unrepresentable_raw_edges(mapper):
    points = np.array([[187, 660], [1089, 660], [185, 710], [1090, 710]], float)
    ground, valid, residual = mapper.raw_to_ground(points)
    assert valid.tolist() == [True, True, False, False]
    assert max(residual[:2]) < .001
    u, v, ok = mapper.camera.project((ground - mapper.center) @ mapper.rotation.T, mapper.radius)
    assert ok[:2].all()
    np.testing.assert_allclose(np.c_[u, v][:2], points[:2], atol=.001)


def test_remap_recovers_synthetic_pixel_coordinates_and_masks(mapper):
    u, v = np.meshgrid(np.arange(1280), np.arange(720))
    image = np.stack([u % 251, v % 251, np.full_like(u, 200)], axis=-1).astype('uint8')
    result = mapper.render(image)
    valid = mapper.valid_mask > 0
    assert np.all(result[~valid] == 0)
    assert np.all(result[valid, 2] == 200)
    # Independent sampling check in locations away from the modulo wrap boundary.
    smooth = valid & (np.mod(mapper.map_x, 251) < 249) & (np.mod(mapper.map_y, 251) < 249)
    assert np.max(abs(result[:, :, 0][smooth].astype(float) - np.mod(mapper.map_x[smooth], 251))) < 1


def test_height_changes_scale_and_not_ray_orientation(calibration, settings, mapper):
    other = deepcopy(settings)
    other['ground']['height_m'] *= 2
    m2 = BEVMapper(calibration, other)
    pts = [[450, 430], [850, 400]]
    p1, valid1, _ = mapper.raw_to_ground(pts)
    p2, valid2, _ = m2.raw_to_ground(pts)
    assert valid1.all() and valid2.all()
    np.testing.assert_allclose(p2, p1 * 2, atol=1e-9)


@pytest.mark.parametrize('group,key,value', [
    ('ground', 'height_m', 0.0), ('ground', 'pitch_deg', float('nan')),
    ('output', 'resolution_m', 0.0), ('output', 'x_max_m', -1.0),
    ('output', 'resolution_m', .003), ('rectification', 'focal_px', -1.0)])
def test_invalid_settings_rejected(calibration, settings, group, key, value):
    broken = deepcopy(settings)
    broken[group][key] = value
    with pytest.raises(ValueError):
        BEVMapper(calibration, broken)


def test_wrong_resolution_rejected_without_rescaling(mapper):
    with pytest.raises(ValueError):
        mapper.render(np.zeros((480, 640, 3), np.uint8))
    with pytest.raises(ValueError):
        mapper.render(None)


def test_invalid_model_is_not_assumed_fisheye(calibration):
    broken = deepcopy(calibration)
    broken['distortion_model'] = 'equidistant'
    with pytest.raises(ValueError):
        CameraModel(broken)
