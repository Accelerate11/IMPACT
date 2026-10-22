"""Pixel motion and gyro-window contracts, with no simulator or truth inputs."""

import json
import math

import cv2
import numpy as np
import pytest

from xq_autonomy.downward_flow import (
    BODY_FROM_DOWN_OPTICAL, CameraModel, DownwardFlowProcessor, GyroBuffer,
    estimate_angular_flow, rotational_displacements,
)


def _camera():
    focal = 320.0 / (2.0 * math.tan(math.pi / 6.0))
    return CameraModel(320, 240, focal, focal, 160.0, 120.0)


def _texture():
    rng = np.random.default_rng(812)
    image = np.zeros((240, 320), dtype=np.uint8)
    for y in range(22, 220, 17):
        for x in range(22, 300, 17):
            intensity = int(rng.integers(90, 256))
            cv2.rectangle(image, (x - 3, y - 3), (x + 3, y + 3), intensity, -1)
    return image


@pytest.mark.parametrize("shift", [(3.0, -2.0), (-2.0, 3.0)])
def test_known_pixel_shift_preserves_optical_sign_and_has_no_metric_scale(shift):
    camera = _camera()
    previous = _texture()
    current = cv2.warpAffine(previous, np.asarray(((1.0, 0.0, shift[0]), (0.0, 1.0, shift[1]))),
                             (camera.width, camera.height))
    result = estimate_angular_flow(previous, current, camera, 1_000_000_000, 1_066_666_667, np.zeros(3))

    assert result["valid"]
    assert result["inlier_count"] >= 12
    assert 0 < result["quality"] <= 255
    assert np.allclose(result["raw_integrated_flow_rad"], np.asarray(shift) / camera.fx, atol=0.0004)
    assert np.allclose(result["compensated_integrated_flow_rad"], np.asarray(shift) / camera.fx, atol=0.0004)
    assert result["acquisition_start_ns"] == 1_000_000_000
    assert result["acquisition_end_ns"] == 1_066_666_667
    assert result["source_image_stamps_ns"] == [1_000_000_000, 1_066_666_667]
    assert result["metric_velocity_valid"] is False
    assert result["metric_velocity_mps"] is None
    assert result["scale_source"] == "unavailable"
    assert result["dependencies"] == ["down_camera", "livox_imu"]
    assert result["ground_truth_used"] is False
    json.dumps(result, allow_nan=False)


def test_pure_camera_rotation_cancels_stationary_scene_pixel_motion():
    camera = _camera()
    previous = _texture()
    integral = np.asarray((0.020, -0.015, 0.005))
    rotation, _ = cv2.Rodrigues(-integral)
    homography = camera.matrix @ rotation @ np.linalg.inv(camera.matrix)
    current = cv2.warpPerspective(previous, homography, (camera.width, camera.height))

    result = estimate_angular_flow(previous, current, camera, 1_000_000_000, 1_066_666_667, integral)

    assert result["valid"]
    assert np.linalg.norm(result["raw_integrated_flow_rad"]) > 0.01
    assert np.linalg.norm(result["compensated_integrated_flow_rad"]) < 0.001
    assert result["gyro_integral_optical_rad"] == pytest.approx(integral)


def test_stationary_rays_use_inverse_camera_rotation_with_correct_axes():
    camera = _camera()
    centre = np.asarray(((camera.cx, camera.cy),))
    displacement = rotational_displacements(centre, np.asarray((0.02, 0.0, 0.0)), camera)
    assert displacement[0, 0] == pytest.approx(0.0)
    assert displacement[0, 1] == pytest.approx(camera.fy * math.tan(0.02))


def test_featureless_images_are_invalid_despite_available_gyro():
    image = np.zeros((240, 320), dtype=np.uint8)
    result = estimate_angular_flow(image, image, _camera(), 100, 66_666_767, np.zeros(3))
    assert result["valid"] is False
    assert result["reason"] == "INSUFFICIENT_FEATURES"
    assert result["track_count"] == result["inlier_count"] == result["quality"] == 0
    assert result["gyro_valid"] is True


def test_gyro_window_interpolates_endpoints_and_trapezoids_in_optical_axes():
    buffer = GyroBuffer()
    origin = 1_790_000_000_000_000_000
    for elapsed_ms in range(0, 101, 10):
        elapsed = elapsed_ms / 1000.0
        optical_rate = np.asarray((1.0 + elapsed, 2.0 - 2.0 * elapsed, 0.5))
        assert buffer.append(origin + elapsed_ms * 1_000_000, BODY_FROM_DOWN_OPTICAL @ optical_rate)

    integral, reason = buffer.integrate(origin + 5_000_000, origin + 95_000_000)

    assert reason == "OK"
    assert integral == pytest.approx((0.0945, 0.171, 0.045), abs=1.0e-12)


def test_gyro_requires_both_endpoint_coverage_and_rejects_large_internal_gap():
    buffer = GyroBuffer()
    buffer.append(10_000_000, (0.0, 0.0, 0.0))
    buffer.append(20_000_000, (0.0, 0.0, 0.0))
    assert buffer.integrate(0, 20_000_000) == (None, "GYRO_WINDOW_UNCOVERED")
    assert buffer.integrate(10_000_000, 30_000_000) == (None, "GYRO_WINDOW_UNCOVERED")
    buffer.append(50_000_000, (0.0, 0.0, 0.0))
    assert buffer.integrate(10_000_000, 50_000_000) == (None, "GYRO_GAP_EXCEEDED")
    assert buffer.append(20_000_000, (1.0, 0.0, 0.0)) is False
    assert buffer.append(60_000_000, (math.nan, 0.0, 0.0)) is False


def test_image_waits_for_late_gyro_without_retimestamping_the_measurement():
    processor = DownwardFlowProcessor(_camera())
    previous = _texture()
    shifted = cv2.warpAffine(previous, np.asarray(((1.0, 0.0, 2.0), (0.0, 1.0, 0.0))), (320, 240))
    first = processor.push_image(1_000_000_000, previous, "impact_down_optical", 10_000_000_000)
    assert first[0]["reason"] == "INITIALIZING_FIRST_IMAGE"
    assert processor.push_image(1_060_000_000, shifted, "impact_down_optical", 10_060_000_000) == []
    outputs = []
    for elapsed_ms in range(0, 61, 10):
        outputs += processor.push_gyro(1_000_000_000 + elapsed_ms * 1_000_000,
                                      (0.0, 0.0, 0.0), 10_070_000_000)
    assert len(outputs) == 1
    assert outputs[0]["valid"]
    assert outputs[0]["acquisition_end_ns"] == 1_060_000_000
    assert processor.pending is None
    assert processor.expire(10_100_000_000) == []


def test_uncovered_gyro_times_out_with_bounded_queue_and_invalid_measurement():
    processor = DownwardFlowProcessor(_camera())
    image = _texture()
    processor.push_image(1_000_000_000, image, "impact_down_optical", 10_000_000_000)
    processor.push_image(1_060_000_000, image, "impact_down_optical", 10_060_000_000)
    assert processor.expire(10_259_999_999) == []
    outputs = processor.expire(10_260_000_000)
    assert len(outputs) == 1
    assert outputs[0]["valid"] is False
    assert outputs[0]["reason"] == "GYRO_WINDOW_UNCOVERED"
    assert processor.pending is None
    assert processor.previous.stamp_ns == 1_060_000_000


def test_latest_frame_queue_drops_uncovered_pair_without_reusing_source_pixels():
    processor = DownwardFlowProcessor(_camera())
    image = _texture()
    processor.push_image(1_000_000_000, image, "impact_down_optical", 10_000_000_000)
    processor.push_image(1_060_000_000, image, "impact_down_optical", 10_060_000_000)
    outputs = processor.push_image(1_120_000_000, image, "impact_down_optical", 10_120_000_000)
    assert len(outputs) == 1
    assert outputs[0]["reason"] == "GYRO_WINDOW_UNCOVERED_IMAGE_SUPERSEDED"
    assert processor.previous.stamp_ns == 1_060_000_000
    assert processor.pending.stamp_ns == 1_120_000_000


@pytest.mark.parametrize("replayed_stamp", [1_000_000_000, 999_999_999])
def test_repeated_or_old_image_cannot_replace_the_original_frame(replayed_stamp):
    processor = DownwardFlowProcessor(_camera())
    image = _texture()
    processor.push_image(1_000_000_000, image, "impact_down_optical", 10_000_000_000)
    image[:] = 0
    rejected = processor.push_image(replayed_stamp, image, "impact_down_optical", 10_010_000_000)
    assert rejected[0]["valid"] is False
    assert rejected[0]["reason"] == "IMAGE_STAMP_NOT_INCREASING"
    assert processor.previous.stamp_ns == 1_000_000_000
    assert np.count_nonzero(processor.previous.pixels) > 0
    assert processor.pending is None


@pytest.mark.parametrize("interval_ns", [0, -1, 200_000_000, 200_000_001])
def test_flow_integration_interval_is_strictly_positive_and_below_point_two_seconds(interval_ns):
    image = _texture()
    result = estimate_angular_flow(image, image, _camera(), 1_000_000_000,
                                   1_000_000_000 + interval_ns, np.zeros(3))
    assert result["valid"] is False
    assert result["reason"] == "INVALID_IMAGE_INTERVAL"


def test_camera_model_uses_received_intrinsics_instead_of_default_focal_length():
    from sensor_msgs.msg import CameraInfo
    from xq_autonomy.downward_flow_node import camera_from_info

    info = CameraInfo()
    info.header.frame_id = "impact_down_optical"
    info.width, info.height = 320, 240
    info.k = [311.0, 0.0, 153.0, 0.0, 317.0, 118.0, 0.0, 0.0, 1.0]
    camera = camera_from_info(info)
    assert (camera.fx, camera.fy, camera.cx, camera.cy) == (311.0, 317.0, 153.0, 118.0)
    info.d = [0.01, 0.0, 0.0, 0.0, 0.0]
    with pytest.raises(ValueError):
        camera_from_info(info)


def test_rgb_conversion_respects_row_padding_and_does_not_change_source_stamp():
    from sensor_msgs.msg import Image
    from xq_autonomy.downward_flow_node import image_to_gray

    message = Image()
    message.header.stamp.sec, message.header.stamp.nanosec = 7, 12345
    message.height, message.width, message.step = 2, 2, 8
    message.encoding = "rgb8"
    message.data = bytes([255, 0, 0, 0, 255, 0, 42, 43, 0, 0, 255, 255, 255, 255, 44, 45])
    gray = image_to_gray(message)
    assert gray == pytest.approx(np.asarray(((76, 150), (29, 255))))
    assert (message.header.stamp.sec, message.header.stamp.nanosec) == (7, 12345)
    message.data = message.data[:-1]
    with pytest.raises(ValueError):
        image_to_gray(message)


def test_ros_boundary_replayed_image_has_no_new_measurement():
    from types import SimpleNamespace
    from sensor_msgs.msg import Image
    from xq_autonomy.downward_flow_node import DownwardFlowNode

    messages = []
    node = SimpleNamespace(counts={"images_received": 0, "images_rejected": 0},
                           last_image_stamp_ns=1_000_000_000, _publish=messages.extend)
    message = Image()
    message.header.stamp.sec = 1
    DownwardFlowNode._image_cb(node, message)
    assert messages == []
    assert node.counts == {"images_received": 1, "images_rejected": 1}
    assert node.last_reason == "IMAGE_STAMP_NOT_INCREASING"


def test_ros_boundary_rejects_gyro_marked_unavailable_without_fabricating_zero_rate():
    from types import SimpleNamespace
    from sensor_msgs.msg import Imu
    from xq_autonomy.downward_flow_node import DownwardFlowNode

    node = SimpleNamespace(counts={"gyro_received": 0, "gyro_rejected": 0}, gyro=GyroBuffer())
    message = Imu()
    message.header.frame_id = "livox_imu"
    message.header.stamp.sec = 1
    message.angular_velocity_covariance[0] = -1.0
    DownwardFlowNode._imu_cb(node, message)
    assert len(node.gyro.samples) == 0
    assert node.counts["gyro_rejected"] == 1
