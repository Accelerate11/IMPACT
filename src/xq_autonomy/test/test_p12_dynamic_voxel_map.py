import math

import numpy as np
import pytest

from xq_autonomy.dynamic_planning import (
    DynamicPassageGate,
    path_obstruction,
    polyline_obstruction,
    polyline_proximity,
    resample_polyline,
    supported_polyline_obstruction,
)
from xq_autonomy.dynamic_voxel_map import TemporalDynamicVoxelMap


def _scan_line(x_values, y=0.0, z=1.0):
    return np.asarray([(x, y, z) for x in x_values], dtype=float)


def test_transient_hit_in_confirmed_free_space_decays_but_static_wall_remains():
    voxel_map = TemporalDynamicVoxelMap(
        voxel_size_m=0.25,
        dynamic_ttl_s=2.0,
        static_confirmation_hits=4,
        free_confirmation_rays=2,
    )
    origin = np.asarray((0.0, 0.0, 1.0))
    wall = _scan_line([5.0], z=1.0)
    for frame in range(6):
        voxel_map.update_scan(origin, wall, float(frame) * 0.1)
    assert len(voxel_map.static_points()) >= 1

    obstacle = _scan_line([2.0], z=1.0)
    for stamp in (1.0, 1.1, 1.2):
        voxel_map.update_scan(origin, obstacle, stamp)
    assert len(voxel_map.dynamic_points()) >= 1
    static_before = len(voxel_map.static_points())

    voxel_map.decay(8.0)
    assert len(voxel_map.dynamic_points()) == 0
    assert len(voxel_map.static_points()) == static_before


def test_ttl_decay_is_incremental_not_reapplied_from_original_age():
    voxel_map = TemporalDynamicVoxelMap(
        voxel_size_m=0.25, dynamic_ttl_s=2.0, free_confirmation_rays=1
    )
    origin = np.asarray((0.0, 0.0, 1.0))
    voxel_map.update_scan(origin, _scan_line([5.0]), 0.0)
    for stamp in (0.1, 0.2, 0.3):
        voxel_map.update_scan(origin, _scan_line([2.0]), stamp)
    _, confidence0 = voxel_map.confidence_points()
    voxel_map.decay(1.1)
    _, confidence1 = voxel_map.confidence_points()
    voxel_map.decay(2.1)
    _, confidence2 = voxel_map.confidence_points()
    assert confidence1.max() == pytest.approx(confidence0.max() * math.exp(-0.4))
    assert confidence2.max() == pytest.approx(confidence1.max() * math.exp(-0.5))


def test_path_gate_brakes_then_reopens_after_clear_confirmation():
    points = np.asarray(((2.0, 0.1, 1.1), (2.0, 1.5, 1.1)))
    blocked, distance = path_obstruction(
        points,
        np.asarray((0.0, 0.0, 1.0)),
        np.asarray((5.0, 0.0, 1.0)),
        clearance_radius_m=0.6,
        lookahead_m=4.0,
    )
    assert blocked
    assert distance == pytest.approx(2.0)
    gate = DynamicPassageGate(clear_confirmation_s=1.0)
    assert gate.update(True, 2.0).brake
    assert gate.update(False, 3.0).brake
    reopened = gate.update(False, 4.0)
    assert not reopened.brake
    assert reopened.passage_reopened
    assert not gate.update(False, 4.1).passage_reopened


def test_off_path_and_beyond_lookahead_voxels_do_not_block():
    points = np.asarray(((2.0, 1.2, 1.0), (8.0, 0.0, 1.0)))
    blocked, distance = path_obstruction(
        points,
        np.asarray((0.0, 0.0, 1.0)),
        np.asarray((10.0, 0.0, 1.0)),
        clearance_radius_m=0.6,
        lookahead_m=4.0,
    )
    assert not blocked
    assert math.isinf(distance)


def test_polyline_guard_follows_commanded_lateral_route_not_world_x_axis():
    path = np.asarray(
        ((0.0, 0.0, 1.0), (1.0, 0.8, 1.0), (3.0, 0.8, 1.0)), dtype=float
    )
    points = np.asarray(((2.0, 0.0, 1.0), (2.5, 0.82, 1.0)), dtype=float)
    blocked, along = polyline_obstruction(
        points,
        path,
        clearance_radius_m=0.30,
        lookahead_m=4.0,
    )
    assert blocked
    assert 2.0 < along < 3.5
    distances, coordinates = polyline_proximity(
        points, path, lookahead_m=4.0
    )
    assert distances[0] > 0.30
    assert distances[1] < 0.05
    assert coordinates[1] == pytest.approx(along, abs=0.05)


def test_polyline_guard_avoids_false_brake_below_vertical_candidate():
    path = np.asarray(
        ((0.0, 0.0, 1.0), (1.0, 0.0, 2.0), (3.0, 0.0, 2.0)), dtype=float
    )
    horizontal_only = np.asarray(((2.0, 0.0, 1.0),), dtype=float)
    blocked, distance = polyline_obstruction(
        horizontal_only,
        path,
        clearance_radius_m=0.40,
        lookahead_m=4.0,
    )
    assert not blocked
    assert math.isinf(distance)


def test_supported_obstruction_rejects_sparse_registration_outliers():
    path = np.asarray(((0.0, 0.0, 1.0), (4.0, 0.0, 1.0)), dtype=float)
    sparse = np.asarray(
        ((1.0, 0.05, 1.0), (2.0, -0.04, 1.0), (3.0, 0.03, 1.0)),
        dtype=float,
    )
    blocked, distance, support = supported_polyline_obstruction(
        sparse,
        path,
        clearance_radius_m=0.70,
        lookahead_m=4.0,
        minimum_support_points=5,
        support_radius_m=0.45,
    )
    assert not blocked
    assert math.isinf(distance)
    assert support == 1


def test_supported_obstruction_retains_compact_dynamic_object():
    path = np.asarray(((0.0, 0.0, 1.0), (4.0, 0.0, 1.0)), dtype=float)
    obstacle = np.asarray(
        (
            (2.00, -0.20, 0.85),
            (2.00, 0.00, 0.85),
            (2.00, 0.20, 0.85),
            (2.00, -0.20, 1.10),
            (2.00, 0.00, 1.10),
            (2.00, 0.20, 1.10),
        ),
        dtype=float,
    )
    blocked, distance, support = supported_polyline_obstruction(
        obstacle,
        path,
        clearance_radius_m=0.70,
        lookahead_m=4.0,
        minimum_support_points=5,
        support_radius_m=0.45,
    )
    assert blocked
    assert distance == pytest.approx(2.0)
    assert support == 6


def test_polyline_resampling_preserves_endpoints_and_curved_route():
    phase = np.linspace(0.0, 1.0, 48)
    path = np.column_stack((7.5 * phase, 0.7 * np.sin(np.pi * phase), phase))
    reduced = resample_polyline(path, 16)
    assert reduced.shape == (16, 3)
    assert np.array_equal(reduced[0], path[0])
    assert np.array_equal(reduced[-1], path[-1])
    assert np.max(reduced[:, 1]) > 0.68


def test_all_endpoints_contribute_hits_when_free_space_rays_are_subsampled():
    voxel_map = TemporalDynamicVoxelMap(
        voxel_size_m=0.25, dynamic_ttl_s=3.0, free_confirmation_rays=1
    )
    origin = np.asarray((0.0, 0.0, 1.0))
    background = np.column_stack(
        (np.full(200, 5.0), np.linspace(-2.0, 2.0, 200), np.ones(200))
    )
    voxel_map.update_scan(origin, background, 0.0, maximum_rays=10)
    # This endpoint is deliberately at an index that the 10-ray selection
    # does not need to retain; occupied evidence must still see it.
    obstacle = np.vstack((background, np.asarray((2.0, 0.0, 1.0))))
    for stamp in (0.1, 0.2, 0.3):
        voxel_map.update_scan(origin, obstacle, stamp, maximum_rays=10)
    assert any(np.linalg.norm(point - np.asarray((2.125, 0.125, 1.125))) < 0.3
               for point in voxel_map.dynamic_points())


def test_vectorized_free_ray_keys_match_scalar_definition():
    voxel_map = TemporalDynamicVoxelMap(voxel_size_m=0.25)
    origin = np.asarray((0.1, -0.2, 1.0), dtype=float)
    endpoints = np.asarray(
        (
            (0.2, -0.2, 1.0),
            (2.8, 0.4, 1.2),
            (5.1, -1.3, 2.0),
            (3.7, 1.8, 0.4),
        ),
        dtype=float,
    )
    expected = set()
    step_m = 0.75 * voxel_map.voxel_size_m
    for endpoint in endpoints:
        delta = endpoint - origin
        distance = float(np.linalg.norm(delta))
        if distance <= voxel_map.voxel_size_m:
            continue
        steps = max(
            1,
            int(math.floor((distance - voxel_map.voxel_size_m) / step_m)),
        )
        for ratio in np.linspace(0.0, 1.0, steps, endpoint=False)[1:]:
            expected.add(voxel_map._key(origin + ratio * delta))
    assert voxel_map._free_keys(origin, endpoints) == expected


def test_unseen_free_space_uses_compact_columns_without_allocating_voxels():
    voxel_map = TemporalDynamicVoxelMap(
        voxel_size_m=0.25, dynamic_ttl_s=3.0, free_confirmation_rays=2
    )
    origin = np.asarray((0.0, 0.0, 1.0))
    background = np.asarray(((8.0, 0.0, 1.0),))
    for stamp in (0.0, 0.1, 0.2):
        outcome = voxel_map.update_scan(origin, background, stamp)
    # The ray traverses many cells, but only its occupied endpoint belongs in
    # the sparse voxel dictionary. Free history remains available by column.
    assert outcome["free_voxels"] > 20
    assert voxel_map.stats()["voxel_count"] == 1
    assert voxel_map.stats()["free_column_count"] > 20

    obstacle = np.asarray(((2.0, 0.0, 1.0),))
    for stamp in (0.3, 0.4, 0.5):
        voxel_map.update_scan(origin, obstacle, stamp)
    assert len(voxel_map.dynamic_points()) >= 1


def test_status_snapshot_matches_public_statistics_and_path_points():
    voxel_map = TemporalDynamicVoxelMap(
        voxel_size_m=0.25,
        dynamic_ttl_s=2.0,
        free_confirmation_rays=1,
        dynamic_confirmation_hits=1,
    )
    point = np.asarray(((2.0, 0.0, 1.0),))
    voxel_map.promote_dynamic(point, 1.0)
    payload, path_points = voxel_map.status_snapshot()
    assert payload == voxel_map.stats()
    assert np.array_equal(path_points, voxel_map.path_dynamic_points())


def test_neighboring_confirmed_free_cell_handles_ray_endpoint_quantization():
    voxel_map = TemporalDynamicVoxelMap(
        voxel_size_m=0.25, dynamic_ttl_s=3.0, free_confirmation_rays=2
    )
    origin = np.asarray((0.0, 0.0, 1.0))
    # The prior rays pass one y voxel beside the later obstacle endpoint.
    for stamp in (0.0, 0.1, 0.2):
        voxel_map.update_scan(origin, np.asarray(((5.0, 0.24, 1.0),)), stamp)
    obstacle = np.asarray(((2.0, 0.26, 1.0),))
    for stamp in (0.3, 0.4, 0.5):
        voxel_map.update_scan(origin, obstacle, stamp)
    assert len(voxel_map.dynamic_points()) >= 1


def test_columnar_free_evidence_detects_tall_object_across_lidar_elevation_layers():
    voxel_map = TemporalDynamicVoxelMap(
        voxel_size_m=0.25, dynamic_ttl_s=3.0, free_confirmation_rays=2
    )
    origin = np.asarray((0.0, 0.0, 1.0))
    for stamp in (0.0, 0.1, 0.2):
        voxel_map.update_scan(origin, np.asarray(((5.0, 0.0, 1.8),)), stamp)
    low_object_return = np.asarray(((2.0, 0.0, 0.35),))
    for stamp in (0.3, 0.4, 0.5):
        voxel_map.update_scan(origin, low_object_return, stamp)
    assert len(voxel_map.dynamic_points()) >= 1


def test_planned_sweep_promotion_is_dynamic_but_never_overwrites_static_structure():
    voxel_map = TemporalDynamicVoxelMap(
        voxel_size_m=0.25, dynamic_ttl_s=2.0, static_confirmation_hits=4
    )
    transient = np.asarray(((2.0, 0.0, 1.0),))
    assert voxel_map.promote_dynamic(transient, 1.0) == 1
    assert len(voxel_map.dynamic_points()) == 1
    voxel_map.decay(8.0)
    assert len(voxel_map.dynamic_points()) == 0

    origin = np.asarray((0.0, 0.0, 1.0))
    wall = np.asarray(((5.0, 0.0, 1.0),))
    for stamp in (9.0, 9.1, 9.2, 9.3, 9.4):
        voxel_map.update_scan(origin, wall, stamp)
    voxel_map.freeze_static_baseline()
    static_count = len(voxel_map.static_points())
    assert static_count >= 1
    assert voxel_map.promote_dynamic(wall, 10.0) == 0
    assert len(voxel_map.static_points()) == static_count
    assert len(voxel_map.promotable_points(wall)) == 0


def test_stationary_post_baseline_object_becomes_reversible_static_then_clears():
    voxel_map = TemporalDynamicVoxelMap(
        voxel_size_m=0.25,
        dynamic_ttl_s=2.0,
        static_confirmation_hits=4,
        free_confirmation_rays=2,
    )
    origin = np.asarray((0.0, 0.0, 1.0))
    background = np.asarray(((5.0, 0.0, 1.0),))
    for stamp in (0.0, 0.1, 0.2, 0.3):
        voxel_map.update_scan(origin, background, stamp)
    voxel_map.freeze_static_baseline()

    stopped_object = np.asarray(((2.0, 0.0, 1.0),))
    for stamp in (1.0, 1.1, 1.2, 1.3, 1.4):
        voxel_map.update_scan(origin, stopped_object, stamp)
        voxel_map.promote_dynamic(stopped_object, stamp)
    assert not any(point[0] < 3.0 for point in voxel_map.dynamic_points())
    assert any(point[0] < 3.0 for point in voxel_map.static_points())

    for stamp in (2.0, 2.1, 2.2):
        voxel_map.update_scan(origin, background, stamp)
    voxel_map.decay(8.0)
    assert not any(point[0] < 3.0 for point in voxel_map.static_points())
    assert not any(point[0] < 3.0 for point in voxel_map.dynamic_points())
    assert not any(point[0] < 3.0 for point in voxel_map.path_dynamic_points())
    # A later stable residual follows the same lifecycle: conservative dynamic
    # first, then reversible static after the full confirmation horizon.
    for stamp in (8.1, 8.2, 8.3, 8.4, 8.5):
        voxel_map.update_scan(origin, stopped_object, stamp)
    assert any(point[0] < 3.0 for point in voxel_map.static_points())
    assert not any(point[0] < 3.0 for point in voxel_map.dynamic_points())


def test_confirmed_mover_cannot_leave_static_ghost_before_dwell_gate():
    voxel_map = TemporalDynamicVoxelMap(
        voxel_size_m=0.25,
        dynamic_ttl_s=2.0,
        static_confirmation_hits=4,
        free_confirmation_rays=2,
        post_dynamic_static_confirmation_s=30.0,
    )
    origin = np.asarray((0.0, 0.0, 1.0))
    background = np.asarray(((5.0, 0.0, 1.0),))
    for stamp in (0.0, 0.1, 0.2, 0.3):
        voxel_map.update_scan(origin, background, stamp)
    voxel_map.freeze_static_baseline()

    mover = np.asarray(((2.0, 0.0, 1.0),))
    for stamp in (1.0, 1.1, 1.2, 1.3, 1.4):
        voxel_map.update_scan(origin, mover, stamp)
        voxel_map.promote_dynamic(mover, stamp)
    assert any(point[0] < 3.0 for point in voxel_map.dynamic_points())
    assert not any(point[0] < 3.0 for point in voxel_map.static_points())

    for stamp in (2.0, 2.1, 2.2):
        voxel_map.update_scan(origin, background, stamp)
    voxel_map.decay(8.0)
    assert not any(point[0] < 3.0 for point in voxel_map.static_points())
    assert not any(point[0] < 3.0 for point in voxel_map.dynamic_points())


def test_reversible_static_expires_without_a_clearing_ray():
    voxel_map = TemporalDynamicVoxelMap(
        voxel_size_m=0.25,
        dynamic_ttl_s=2.0,
        static_confirmation_hits=4,
        free_confirmation_rays=2,
        reversible_static_ttl_s=2.0,
    )
    origin = np.asarray((0.0, 0.0, 1.0))
    background = np.asarray(((5.0, 0.0, 1.0),))
    for stamp in (0.0, 0.1, 0.2, 0.3):
        voxel_map.update_scan(origin, background, stamp)
    voxel_map.freeze_static_baseline()

    stopped_mover = np.asarray(((2.0, 0.0, 1.0),))
    for stamp in (1.0, 1.1, 1.2, 1.3, 1.4):
        voxel_map.update_scan(origin, stopped_mover, stamp)
        voxel_map.promote_dynamic(stopped_mover, stamp)
    assert any(point[0] < 3.0 for point in voxel_map.static_points())
    assert voxel_map.stats()["reversible_static_voxel_count"] >= 1

    # No scan passes through the old object cell.  Last-observation expiry,
    # rather than a free ray, must still remove the provisional static ghost.
    voxel_map.decay(4.0)
    assert not any(point[0] < 3.0 for point in voxel_map.static_points())
    assert any(point[0] >= 4.0 for point in voxel_map.static_points())
    assert not any(point[0] < 3.0 for point in voxel_map.path_dynamic_points())


def test_path_obstacle_cannot_transfer_static_before_full_dwell_window():
    voxel_map = TemporalDynamicVoxelMap(
        voxel_size_m=0.25,
        dynamic_ttl_s=2.0,
        static_confirmation_hits=3,
        free_confirmation_rays=2,
        post_dynamic_static_confirmation_s=2.0,
    )
    origin = np.asarray((0.0, 0.0, 1.0))
    background = np.asarray(((5.0, 0.0, 1.0),))
    for stamp in (0.0, 0.1, 0.2):
        voxel_map.update_scan(origin, background, stamp)
    voxel_map.freeze_static_baseline()
    obstacle = np.asarray(((2.0, 0.0, 1.0),))
    for stamp in (1.0, 1.2, 1.4, 1.6):
        voxel_map.update_scan(origin, obstacle, stamp)
        voxel_map.promote_dynamic(obstacle, stamp)
    assert any(point[0] < 3.0 for point in voxel_map.path_dynamic_points())
    assert not any(point[0] < 3.0 for point in voxel_map.static_points())
    voxel_map.update_scan(origin, obstacle, 3.2)
    assert not any(point[0] < 3.0 for point in voxel_map.path_dynamic_points())
    assert any(point[0] < 3.0 for point in voxel_map.static_points())


def test_newly_revealed_occupancy_without_prior_free_evidence_stays_static():
    voxel_map = TemporalDynamicVoxelMap(
        voxel_size_m=0.25,
        dynamic_ttl_s=2.0,
        static_confirmation_hits=4,
        free_confirmation_rays=3,
    )
    origin = np.asarray((0.0, 0.0, 1.0))
    baseline = np.asarray(((5.0, 3.0, 1.0),))
    for stamp in (0.0, 0.1, 0.2, 0.3):
        voxel_map.update_scan(origin, baseline, stamp)
    voxel_map.freeze_static_baseline()
    newly_visible_wall = np.asarray(((2.0, 0.0, 1.0),))
    for stamp in (1.0, 1.1, 1.2, 1.3, 1.4):
        voxel_map.update_scan(origin, newly_visible_wall, stamp)
    assert any(point[0] < 3.0 for point in voxel_map.static_points())
    assert len(voxel_map.promotable_points(newly_visible_wall)) == 0


def test_pre_mission_baseline_is_immutable_while_new_hit_remains_promotable():
    voxel_map = TemporalDynamicVoxelMap(voxel_size_m=0.25, dynamic_ttl_s=2.0)
    origin = np.asarray((0.0, 0.0, 1.0))
    panel = np.asarray(((3.0, 0.8, 1.0),))
    clear_corridor = np.asarray(((5.0, 0.0, 1.0),))
    for stamp in (0.0, 0.1, 0.2):
        voxel_map.update_scan(origin, np.vstack((panel, clear_corridor)), stamp)
    assert voxel_map.freeze_static_baseline() >= 2
    assert len(voxel_map.promotable_points(panel)) == 0
    new_obstacle = np.asarray(((2.0, 0.0, 1.0),))
    voxel_map.update_scan(origin, new_obstacle, 0.4)
    assert len(voxel_map.promotable_points(new_obstacle)) == 1
    assert voxel_map.promote_dynamic(new_obstacle, 1.0) == 1

    adjacent_panel_return = panel + np.asarray((0.0, 0.24, 0.0))
    assert len(voxel_map.promotable_points(adjacent_panel_return)) == 0


def test_only_path_promoted_dynamic_enters_safety_layer():
    voxel_map = TemporalDynamicVoxelMap(
        voxel_size_m=0.25, dynamic_ttl_s=2.0,
        free_confirmation_rays=1, dynamic_confirmation_hits=1,
    )
    origin = np.asarray((0.0, 0.0, 1.0))
    voxel_map.update_scan(origin, np.asarray(((4.0, 0.0, 1.0),)), 0.0)
    transient = np.asarray(((2.0, 0.0, 1.0),))
    voxel_map.update_scan(origin, transient, 0.2)
    assert len(voxel_map.dynamic_points()) >= 1
    assert len(voxel_map.path_dynamic_points()) == 0
    assert voxel_map.promote_dynamic(transient, 0.3) == 1
    assert len(voxel_map.path_dynamic_points()) == 1
    voxel_map.decay(8.0)
    assert len(voxel_map.path_dynamic_points()) == 0
