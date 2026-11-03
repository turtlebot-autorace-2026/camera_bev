"""Metric-plane projection contracts, independent of checkerboard fit quality."""
from copy import deepcopy
from pathlib import Path

import cv2
import numpy as np
import pytest

from camera_bev.geometry import BEVMapper, calibration_fingerprint, read_yaml

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def inputs():
    return (read_yaml(ROOT / 'config/camera_1280x720.yaml'),
            read_yaml(ROOT / 'config/ground_stationary_20261030.yaml'))


def test_plane_projection_matches_opencv_and_roundtrips(inputs):
    calibration, settings = inputs
    mapper = BEVMapper(calibration, settings)
    # Test points span the checkerboard and both visible lane boundaries.
    xy = np.array([[.2, .08], [.3, -.1], [.4, .1], [.5, 0], [.8, -.15]])
    rays = np.c_[xy, np.ones(len(xy))] @ mapper.ground_homography.T
    raw = cv2.projectPoints(rays, np.zeros(3), np.zeros(3),
                            mapper.camera.k, mapper.camera.d)[0][:, 0]
    ground, valid, residual = mapper.raw_to_ground(raw)
    assert valid.all() and max(residual) < .001
    np.testing.assert_allclose(ground[:, :2], xy, atol=1e-8)
    np.testing.assert_allclose(ground[:, 2], 0)
    u, v, valid = mapper.camera.project(rays, mapper.radius)
    assert valid.all()
    np.testing.assert_allclose(np.c_[u, v], raw, atol=1e-8)


def test_pose_equivalent_homography_preserves_legacy_map(inputs):
    calibration, _ = inputs
    legacy = read_yaml(ROOT / 'config/initial_20261028.yaml')
    original = BEVMapper(calibration, legacy)
    equivalent = deepcopy(legacy)
    matrix = np.column_stack([original.rotation[:, :2], -original.rotation @ original.center])
    equivalent['ground_projection'] = dict(type='normalized_homography',
        ground_to_normalized=matrix.tolist(), reference_ground=deepcopy(legacy['ground']),
        calibration_fingerprint=calibration_fingerprint(calibration))
    mapped = BEVMapper(calibration, equivalent)
    np.testing.assert_array_equal(mapped.valid_mask, original.valid_mask)
    np.testing.assert_allclose(mapped.map_x, original.map_x, atol=1e-5)
    np.testing.assert_allclose(mapped.map_y, original.map_y, atol=1e-5)
    raw = np.random.RandomState(1).randint(0, 256, (720, 1280, 3), dtype=np.uint8)
    np.testing.assert_array_equal(mapped.render(raw), original.render(raw))


@pytest.mark.parametrize('matrix', [np.zeros((3, 3)), np.eye(2),
                                    np.full((3, 3), np.nan), np.diag([1., 1., 1e-10])])
def test_invalid_homography_is_rejected(inputs, matrix):
    calibration, settings = deepcopy(inputs)
    settings['ground_projection']['ground_to_normalized'] = matrix.tolist()
    with pytest.raises(ValueError):
        BEVMapper(calibration, settings)


def test_changed_intrinsics_or_reference_rejected(inputs):
    calibration, settings = deepcopy(inputs)
    calibration['camera_matrix'][0][0] += 1
    with pytest.raises(ValueError, match='intrinsics'):
        BEVMapper(calibration, settings)
    calibration, settings = deepcopy(inputs)
    settings['ground']['height_m'] += .01
    with pytest.raises(ValueError, match='fixed'):
        BEVMapper(calibration, settings)


def test_output_crop_preserves_metric_plane(inputs):
    calibration, settings = deepcopy(inputs)
    full = BEVMapper(calibration, settings)
    settings['output'].update(x_min_m=.2, x_max_m=.8, y_min_m=-.2, y_max_m=.2)
    cropped = BEVMapper(calibration, settings)
    np.testing.assert_array_equal(cropped.map_x, full.map_x[100:400, 150:350])
    np.testing.assert_array_equal(cropped.map_y, full.map_y[100:400, 150:350])
    assert cropped.info()['ground_projection'] == full.info()['ground_projection']
    assert not cropped.info()['ground']['measured']
