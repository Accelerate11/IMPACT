import numpy as np
import pytest

from xq_autonomy.surfel_map import TemporalVoxelSurfelMap


def _plane_points(z: float = 0.0) -> np.ndarray:
    x, y = np.meshgrid(np.linspace(0.05, 0.95, 10), np.linspace(0.05, 0.95, 10))
    return np.column_stack((x.ravel(), y.ravel(), np.full(x.size, z)))


def test_repeated_plane_becomes_high_confidence_surfel():
    surfels = TemporalVoxelSurfelMap(
        voxel_size_m=1.0,
        minimum_points=20,
        confidence_observations=4,
        minimum_geometry_quality=0.5,
    )
    for index in range(4):
        surfels.update(_plane_points(), float(index))
    snapshot = surfels.snapshot()
    assert len(snapshot.positions) == 1
    assert abs(snapshot.normals[0, 2]) == pytest.approx(1.0, abs=1.0e-8)
    assert snapshot.static_confidence[0] == pytest.approx(1.0)
    assert snapshot.geometry_quality[0] > 0.9


def test_line_geometry_is_not_published_as_plane():
    surfels = TemporalVoxelSurfelMap(
        voxel_size_m=2.0,
        minimum_points=8,
        minimum_geometry_quality=0.2,
    )
    points = np.column_stack((np.linspace(0.0, 1.0, 20), np.zeros(20), np.zeros(20)))
    surfels.update(points, 0.0)
    assert len(surfels.snapshot().positions) == 0


def test_stale_voxels_are_pruned_and_invalid_inputs_fail_closed():
    surfels = TemporalVoxelSurfelMap(voxel_size_m=1.0, stale_after_s=2.0)
    surfels.update(_plane_points(), 1.0)
    assert surfels.voxel_count == 1
    surfels.update(np.empty((0, 3)), 3.1)
    assert surfels.voxel_count == 0
    with pytest.raises(ValueError):
        surfels.update(np.array([[np.nan, 0.0, 0.0]]), 4.0)
