from xq_autonomy.p2_sensor_validator_node import (
    StreamStats,
    has_observed_irreversible_failure,
)


def _failed(lidar: StreamStats, imu: StreamStats, **overrides: int) -> bool:
    values = {
        "lidar_frame_mismatches": 0,
        "imu_frame_mismatches": 0,
        "nan_or_inf_violations": 0,
        "layout_violations": 0,
    }
    values.update(overrides)
    return has_observed_irreversible_failure(lidar, imu, **values)


def test_missing_startup_samples_are_not_a_failure():
    assert not _failed(StreamStats(), StreamStats())


def test_observed_layout_failure_is_irreversible():
    assert _failed(StreamStats(), StreamStats(), layout_violations=1)


def test_non_monotonic_timestamp_is_irreversible():
    lidar = StreamStats()
    lidar.add(1_000_000_000)
    lidar.add(1_000_000_000)
    assert _failed(lidar, StreamStats())
