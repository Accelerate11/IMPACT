#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <google/protobuf/struct.pb.h>
#include <google/protobuf/util/json_util.h>

#include "xq_gz_bridge/range_conversions.hpp"

namespace
{
namespace bridge = xq_gz_bridge::r2b3;

gz::msgs::LaserScan native_scan()
{
  gz::msgs::LaserScan scan;
  scan.mutable_header()->mutable_stamp()->set_sec(12);
  scan.mutable_header()->mutable_stamp()->set_nsec(345678901);
  scan.set_frame("xq_p4_uav1::xq_lio_imu_link::down_range_noisy");
  scan.set_count(1);
  scan.set_vertical_count(1);
  scan.set_range_min(0.05);
  scan.set_range_max(10.0);
  scan.add_ranges(1.23456789123);
  scan.add_intensities(3.25);
  // A native transport pose must never become an estimator depth shortcut.
  scan.mutable_world_pose()->mutable_position()->set_z(999.0);
  return scan;
}

gz::msgs::IMU native_imu()
{
  gz::msgs::IMU imu;
  imu.mutable_header()->mutable_stamp()->set_sec(7);
  imu.mutable_header()->mutable_stamp()->set_nsec(987654321);
  imu.set_entity_name("xq_p4_uav1::xq_lio_imu_link::xq_lio_imu");
  imu.mutable_orientation()->set_x(0.1);
  imu.mutable_orientation()->set_y(0.2);
  imu.mutable_orientation()->set_z(0.3);
  imu.mutable_orientation()->set_w(0.9273618495495703);
  imu.mutable_angular_velocity()->set_x(0.125);
  imu.mutable_angular_velocity()->set_y(-0.375);
  imu.mutable_angular_velocity()->set_z(1.25);
  imu.mutable_linear_acceleration()->set_x(-0.25);
  imu.mutable_linear_acceleration()->set_y(0.5);
  imu.mutable_linear_acceleration()->set_z(9.80665);
  return imu;
}
}  // namespace

TEST(PencilScanConversion, PreservesSourceStampAndSameFloatMeasurementInBothMessages)
{
  const auto native = native_scan();
  const auto scan = bridge::to_ros_pencil_scan(native, "down_range_noisy");
  const auto range = bridge::pencil_scan_to_range(scan);
  EXPECT_EQ(scan.header.stamp.sec, 12);
  EXPECT_EQ(scan.header.stamp.nanosec, 345678901U);
  EXPECT_EQ(scan.header.frame_id, "impact_down_range");
  EXPECT_EQ(range.header, scan.header);
  ASSERT_EQ(scan.ranges.size(), 1U);
  EXPECT_FLOAT_EQ(scan.ranges[0], static_cast<float>(native.ranges(0)));
  EXPECT_EQ(range.range, scan.ranges[0]);
  EXPECT_EQ(range.radiation_type, sensor_msgs::msg::Range::INFRARED);
  EXPECT_FLOAT_EQ(range.field_of_view, 0.0F);
  EXPECT_FLOAT_EQ(range.min_range, 0.05F);
  EXPECT_FLOAT_EQ(range.max_range, 10.0F);
  ASSERT_EQ(scan.intensities.size(), 1U);
  EXPECT_FLOAT_EQ(scan.intensities[0], 3.25F);
  EXPECT_FLOAT_EQ(scan.angle_min, 0.0F);
  EXPECT_FLOAT_EQ(scan.angle_max, 0.0F);
  EXPECT_FLOAT_EQ(scan.angle_increment, 0.0F);
  EXPECT_FLOAT_EQ(scan.time_increment, 0.0F);
  EXPECT_FLOAT_EQ(scan.scan_time, 0.02F);
  EXPECT_DOUBLE_EQ(native.world_pose().position().z(), 999.0);
}

TEST(PencilScanConversion, SupportsEmptyIntensityAndClosedFiniteRangeBounds)
{
  auto scan = native_scan();
  scan.clear_intensities();
  for (const double bound : {0.05, 10.0}) {
    scan.set_ranges(0, bound);
    const auto result = bridge::to_ros_pencil_scan(scan, "down_range_noisy");
    EXPECT_TRUE(result.intensities.empty());
    EXPECT_FLOAT_EQ(bridge::pencil_scan_to_range(result).range, static_cast<float>(bound));
  }
}

TEST(PencilScanConversion, ChecksSensorScopedFrameInsteadOfSilentlyRelabelingUnknownAxes)
{
  auto scan = native_scan();
  for (const auto & frame : {"down_range_noisy", "model::link::down_range_noisy", "model/link/down_range_noisy"}) {
    scan.set_frame(frame);
    EXPECT_NO_THROW(bridge::to_ros_pencil_scan(scan, "down_range_noisy"));
  }
  for (const auto & frame : {"", "unknown_link", "prefixdown_range_noisy", "down_range_reference", "model::down_range_noisy_extra"}) {
    scan.set_frame(frame);
    EXPECT_THROW(bridge::to_ros_pencil_scan(scan, "down_range_noisy"), std::invalid_argument);
  }
  scan.set_frame("down_range_reference");
  EXPECT_NO_THROW(bridge::to_ros_pencil_scan(scan, "down_range_reference"));
  EXPECT_THROW(bridge::to_ros_pencil_scan(scan, ""), std::invalid_argument);
}

class InvalidRangeTime : public ::testing::TestWithParam<int> {};
TEST_P(InvalidRangeTime, RejectsMalformedAcquisitionTimeWithoutNormalizingOrFallingBack)
{
  auto scan = native_scan();
  switch (GetParam()) {
    case 0: scan.clear_header(); break;
    case 1: scan.mutable_header()->clear_stamp(); break;
    case 2: scan.mutable_header()->mutable_stamp()->set_sec(-1); break;
    case 3: scan.mutable_header()->mutable_stamp()->set_sec(2147483648LL); break;
    case 4: scan.mutable_header()->mutable_stamp()->set_nsec(-1); break;
    case 5: scan.mutable_header()->mutable_stamp()->set_nsec(1000000000); break;
  }
  EXPECT_THROW(bridge::to_ros_pencil_scan(scan, "down_range_noisy"), std::invalid_argument);
}
INSTANTIATE_TEST_SUITE_P(StrictClock, InvalidRangeTime, ::testing::Range(0, 6));

TEST(AcquisitionTime, AcceptsZeroAndLargestRosTimeWithoutPrecisionLoss)
{
  gz::msgs::Header header;
  header.mutable_stamp()->set_sec(0);
  header.mutable_stamp()->set_nsec(0);
  EXPECT_EQ(bridge::strict_acquisition_stamp_ns(header), 0);
  header.mutable_stamp()->set_sec(2147483647);
  header.mutable_stamp()->set_nsec(999999999);
  EXPECT_EQ(bridge::strict_acquisition_stamp_ns(header), 2147483647999999999LL);
}

class InvalidPencilShape : public ::testing::TestWithParam<int> {};
TEST_P(InvalidPencilShape, RejectsNonPencilOrIncompleteNativeSamples)
{
  auto scan = native_scan();
  switch (GetParam()) {
    case 0: scan.set_count(0); break;
    case 1: scan.set_count(2); break;
    case 2: scan.set_vertical_count(0); break;
    case 3: scan.set_vertical_count(2); break;
    case 4: scan.clear_ranges(); break;
    case 5: scan.add_ranges(2.0); break;
    case 6: scan.add_intensities(4.0); break;
  }
  EXPECT_THROW(bridge::to_ros_pencil_scan(scan, "down_range_noisy"), std::invalid_argument);
}
INSTANTIATE_TEST_SUITE_P(FrozenBeam, InvalidPencilShape, ::testing::Range(0, 7));

class InvalidPencilAngle : public ::testing::TestWithParam<int> {};
TEST_P(InvalidPencilAngle, RejectsHorizontalAndVerticalAxisChanges)
{
  auto scan = native_scan();
  switch (GetParam()) {
    case 0: scan.set_angle_min(-0.01); break;
    case 1: scan.set_angle_max(0.01); break;
    case 2: scan.set_angle_step(0.01); break;
    case 3: scan.set_vertical_angle_min(-0.01); break;
    case 4: scan.set_vertical_angle_max(0.01); break;
    case 5: scan.set_vertical_angle_step(0.01); break;
    case 6: scan.set_vertical_angle_step(std::numeric_limits<double>::infinity()); break;
  }
  EXPECT_THROW(bridge::to_ros_pencil_scan(scan, "down_range_noisy"), std::invalid_argument);
}
INSTANTIATE_TEST_SUITE_P(FrozenAxis, InvalidPencilAngle, ::testing::Range(0, 7));

class SingleRayNanSpacing : public ::testing::TestWithParam<int> {};
TEST_P(SingleRayNanSpacing, AllowsOnlyUndefinedDerivedSpacingAndRetainsDistanceAndTime)
{
  auto scan = native_scan();
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  if (GetParam() != 1) {scan.set_angle_step(nan);}
  if (GetParam() != 0) {scan.set_vertical_angle_step(nan);}
  const auto before = scan.SerializeAsString();
  const auto converted = bridge::to_ros_pencil_scan(scan, "down_range_noisy");
  EXPECT_TRUE(bridge::has_undefined_single_sample_spacing(scan));
  EXPECT_FLOAT_EQ(converted.angle_increment, 0.0F);
  ASSERT_EQ(converted.ranges.size(), 1U);
  EXPECT_EQ(converted.ranges[0], static_cast<float>(scan.ranges(0)));
  EXPECT_EQ(converted.header.stamp.sec, 12);
  EXPECT_EQ(converted.header.stamp.nanosec, 345678901U);
  EXPECT_EQ(scan.SerializeAsString(), before);
}
INSTANTIATE_TEST_SUITE_P(UndefinedSpacing, SingleRayNanSpacing, ::testing::Range(0, 3));

class NonzeroOrInfiniteSpacing : public ::testing::TestWithParam<int> {};
TEST_P(NonzeroOrInfiniteSpacing, RejectsEvenTinyNonzeroOrInfiniteDerivedSpacing)
{
  auto scan = native_scan();
  const double values[] = {1.0e-15, -1.0e-15,
    std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
  if (GetParam() < 4) {scan.set_angle_step(values[GetParam()]);}
  else {scan.set_vertical_angle_step(values[GetParam() - 4]);}
  EXPECT_THROW(bridge::to_ros_pencil_scan(scan, "down_range_noisy"), std::invalid_argument);
}
INSTANTIATE_TEST_SUITE_P(NoStepSubstitution, NonzeroOrInfiniteSpacing, ::testing::Range(0, 8));

class NonzeroOrNanPencilEndpoint : public ::testing::TestWithParam<int> {};
TEST_P(NonzeroOrNanPencilEndpoint, RejectsInvalidBoundsEvenIfSpacingIsUndefined)
{
  auto scan = native_scan();
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  scan.set_angle_step(nan);
  scan.set_vertical_angle_step(nan);
  const double invalid = GetParam() < 4 ? nan : 1.0e-15;
  switch (GetParam() % 4) {
    case 0: scan.set_angle_min(invalid); break;
    case 1: scan.set_angle_max(invalid); break;
    case 2: scan.set_vertical_angle_min(invalid); break;
    case 3: scan.set_vertical_angle_max(invalid); break;
  }
  EXPECT_THROW(bridge::to_ros_pencil_scan(scan, "down_range_noisy"), std::invalid_argument);
}
INSTANTIATE_TEST_SUITE_P(NoBoundSubstitution, NonzeroOrNanPencilEndpoint, ::testing::Range(0, 8));

TEST(PencilScanConversion, UndefinedSpacingCannotMaskMissingDistanceOrMultipleRays)
{
  auto scan = native_scan();
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  scan.set_angle_step(nan);
  scan.set_vertical_angle_step(nan);
  scan.set_ranges(0, nan);
  EXPECT_THROW(bridge::to_ros_pencil_scan(scan, "down_range_noisy"), std::invalid_argument);
  scan.set_ranges(0, 2.0);
  scan.set_count(2);
  EXPECT_THROW(bridge::to_ros_pencil_scan(scan, "down_range_noisy"), std::invalid_argument);
  scan.set_count(1);
  scan.set_vertical_count(2);
  EXPECT_THROW(bridge::to_ros_pencil_scan(scan, "down_range_noisy"), std::invalid_argument);
  scan.set_vertical_count(1);
  scan.add_ranges(2.0);
  EXPECT_THROW(bridge::to_ros_pencil_scan(scan, "down_range_noisy"), std::invalid_argument);
}

TEST(NativeSourceLedger, ValidZeroSpacingRetainsOriginalDoubleAndAcquisitionFields)
{
  const auto scan = native_scan();
  const auto payload = bridge::native_scan_source_json(scan, "noisy");
  google::protobuf::Struct parsed;
  ASSERT_TRUE(google::protobuf::util::JsonStringToMessage(payload, &parsed).ok());
  const auto & fields = parsed.fields();
  EXPECT_EQ(fields.at("stream").string_value(), "noisy");
  EXPECT_EQ(fields.at("stamp_ns").number_value(), 12345678901.0);
  EXPECT_EQ(fields.at("native_frame").string_value(), scan.frame());
  EXPECT_EQ(fields.at("count").number_value(), 1.0);
  EXPECT_EQ(fields.at("vertical_count").number_value(), 1.0);
  EXPECT_DOUBLE_EQ(fields.at("native_range_m").number_value(), scan.ranges(0));
  EXPECT_EQ(fields.at("native_angle_step").number_value(), 0.0);
  EXPECT_EQ(fields.at("native_vertical_angle_step").number_value(), 0.0);
  EXPECT_EQ(fields.at("derived_ros_increment").number_value(), 0.0);
  EXPECT_EQ(fields.at("normalization_reason").string_value(), "already_zero");
  EXPECT_EQ(payload.find("world"), std::string::npos);
  EXPECT_FALSE(bridge::has_undefined_single_sample_spacing(scan));
}

TEST(NativeSourceLedger, EmitsJsonStringNanForUndefinedSpacingWithoutChangingOriginalRange)
{
  auto scan = native_scan();
  scan.set_frame("down_range_reference");
  scan.set_angle_step(std::numeric_limits<double>::quiet_NaN());
  scan.set_vertical_angle_step(std::numeric_limits<double>::quiet_NaN());
  const auto before = scan.SerializeAsString();
  const auto payload = bridge::native_scan_source_json(scan, "reference");
  google::protobuf::Struct parsed;
  ASSERT_TRUE(google::protobuf::util::JsonStringToMessage(payload, &parsed).ok());
  EXPECT_EQ(parsed.fields().at("native_angle_step").string_value(), "nan");
  EXPECT_EQ(parsed.fields().at("native_vertical_angle_step").string_value(), "nan");
  EXPECT_EQ(parsed.fields().at("normalization_reason").string_value(), "single_sample_spacing_undefined");
  EXPECT_DOUBLE_EQ(parsed.fields().at("native_range_m").number_value(), scan.ranges(0));
  EXPECT_EQ(scan.SerializeAsString(), before);
}

TEST(NativeSourceLedger, EscapesRawFrameAndPreservesLargeTimestampAsIntegerText)
{
  auto scan = native_scan();
  scan.set_frame("quoted\"scope\\with\nnewline::down_range_noisy");
  scan.mutable_header()->mutable_stamp()->set_sec(2147483647);
  scan.mutable_header()->mutable_stamp()->set_nsec(999999999);
  const auto payload = bridge::native_scan_source_json(scan, "noisy");
  google::protobuf::Struct parsed;
  ASSERT_TRUE(google::protobuf::util::JsonStringToMessage(payload, &parsed).ok());
  EXPECT_EQ(parsed.fields().at("native_frame").string_value(), scan.frame());
  EXPECT_NE(payload.find("\"stamp_ns\":2147483647999999999"), std::string::npos);
}

TEST(NativeSourceLedger, RejectsUnknownStreamAndMalformedObservationsInsteadOfLoggingSyntheticData)
{
  auto scan = native_scan();
  EXPECT_THROW(bridge::native_scan_source_json(scan, "not_a_stream"), std::invalid_argument);
  EXPECT_THROW(bridge::native_scan_source_json(scan, "reference"), std::invalid_argument);
  scan.set_ranges(0, std::numeric_limits<double>::infinity());
  EXPECT_THROW(bridge::native_scan_source_json(scan, "noisy"), std::invalid_argument);
}

class InvalidRangeBounds : public ::testing::TestWithParam<int> {};
TEST_P(InvalidRangeBounds, RejectsChangedOrNonfiniteSensorLimits)
{
  auto scan = native_scan();
  switch (GetParam()) {
    case 0: scan.set_range_min(0.01); break;
    case 1: scan.set_range_max(20.0); break;
    case 2: scan.set_range_min(std::numeric_limits<double>::quiet_NaN()); break;
    case 3: scan.set_range_max(std::numeric_limits<double>::infinity()); break;
  }
  EXPECT_THROW(bridge::to_ros_pencil_scan(scan, "down_range_noisy"), std::invalid_argument);
}
INSTANTIATE_TEST_SUITE_P(FrozenBounds, InvalidRangeBounds, ::testing::Range(0, 4));

class InvalidRangeValue : public ::testing::TestWithParam<int> {};
TEST_P(InvalidRangeValue, RejectsMissingHitsAndNonfiniteOrOutOfRangeMeasurements)
{
  auto scan = native_scan();
  const double values[] = {std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
    0.049999, 10.000001};
  scan.set_ranges(0, values[GetParam()]);
  EXPECT_THROW(bridge::to_ros_pencil_scan(scan, "down_range_noisy"), std::invalid_argument);
}
INSTANTIATE_TEST_SUITE_P(FiniteOnly, InvalidRangeValue, ::testing::Range(0, 5));

TEST(PencilScanConversion, RejectsNonfiniteAndFloatOverflowIntensity)
{
  auto scan = native_scan();
  for (const auto value : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), std::numeric_limits<double>::max()})
  {
    scan.set_intensities(0, value);
    EXPECT_THROW(bridge::to_ros_pencil_scan(scan, "down_range_noisy"), std::invalid_argument);
  }
}

TEST(RangeConversion, RejectsUnvalidatedRosShapeFrameTimestampAndBounds)
{
  const auto valid = bridge::to_ros_pencil_scan(native_scan(), "down_range_noisy");
  auto corrupted = valid;
  corrupted.ranges.clear();
  EXPECT_THROW(bridge::pencil_scan_to_range(corrupted), std::invalid_argument);
  corrupted = valid;
  corrupted.header.frame_id = "unknown_axis";
  EXPECT_THROW(bridge::pencil_scan_to_range(corrupted), std::invalid_argument);
  corrupted = valid;
  corrupted.header.stamp.nanosec = 1000000000U;
  EXPECT_THROW(bridge::pencil_scan_to_range(corrupted), std::invalid_argument);
  corrupted = valid;
  corrupted.range_max = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(bridge::pencil_scan_to_range(corrupted), std::invalid_argument);
}

TEST(KnownNoiseImuConversion, PreservesNativeTimeQuaternionAndAllSixMeasurements)
{
  const auto native = native_imu();
  const auto before = native.SerializeAsString();
  const auto imu = bridge::to_ros_known_noise_imu(native);
  EXPECT_EQ(native.SerializeAsString(), before);
  EXPECT_EQ(imu.header.stamp.sec, 7);
  EXPECT_EQ(imu.header.stamp.nanosec, 987654321U);
  EXPECT_EQ(imu.header.frame_id, "livox_imu");
  EXPECT_DOUBLE_EQ(imu.orientation.x, native.orientation().x());
  EXPECT_DOUBLE_EQ(imu.orientation.y, native.orientation().y());
  EXPECT_DOUBLE_EQ(imu.orientation.z, native.orientation().z());
  EXPECT_DOUBLE_EQ(imu.orientation.w, native.orientation().w());
  EXPECT_DOUBLE_EQ(imu.angular_velocity.x, 0.125);
  EXPECT_DOUBLE_EQ(imu.angular_velocity.y, -0.375);
  EXPECT_DOUBLE_EQ(imu.angular_velocity.z, 1.25);
  EXPECT_DOUBLE_EQ(imu.linear_acceleration.x, -0.25);
  EXPECT_DOUBLE_EQ(imu.linear_acceleration.y, 0.5);
  EXPECT_DOUBLE_EQ(imu.linear_acceleration.z, 9.80665);
  for (size_t index = 0; index < 9; ++index) {
    const bool diagonal = index == 0 || index == 4 || index == 8;
    EXPECT_DOUBLE_EQ(imu.orientation_covariance[index], index == 0 ? -1.0 : 0.0);
    EXPECT_DOUBLE_EQ(imu.angular_velocity_covariance[index], diagonal ? 4.0e-8 : 0.0);
    EXPECT_DOUBLE_EQ(imu.linear_acceleration_covariance[index], diagonal ? 4.0e-6 : 0.0);
  }
}

TEST(KnownNoiseImuConversion, MissingOrientationStaysExplicitlyUnknown)
{
  auto native = native_imu();
  native.clear_orientation();
  const auto imu = bridge::to_ros_known_noise_imu(native);
  EXPECT_DOUBLE_EQ(imu.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(imu.orientation_covariance[0], -1.0);
}

TEST(KnownNoiseImuConversion, RejectsAbsentOrMalformedTimeAndMissingMeasurementVectors)
{
  auto imu = native_imu();
  imu.clear_header();
  EXPECT_THROW(bridge::to_ros_known_noise_imu(imu), std::invalid_argument);
  imu = native_imu();
  imu.mutable_header()->mutable_stamp()->set_nsec(-1);
  EXPECT_THROW(bridge::to_ros_known_noise_imu(imu), std::invalid_argument);
  imu = native_imu();
  imu.clear_angular_velocity();
  EXPECT_THROW(bridge::to_ros_known_noise_imu(imu), std::invalid_argument);
  imu = native_imu();
  imu.clear_linear_acceleration();
  EXPECT_THROW(bridge::to_ros_known_noise_imu(imu), std::invalid_argument);
}

class NonfiniteImuComponent : public ::testing::TestWithParam<int> {};
TEST_P(NonfiniteImuComponent, RejectsEveryNonfiniteQuaternionOrMeasurementComponent)
{
  auto imu = native_imu();
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  const auto inf = std::numeric_limits<double>::infinity();
  switch (GetParam()) {
    case 0: imu.mutable_orientation()->set_x(nan); break;
    case 1: imu.mutable_orientation()->set_y(inf); break;
    case 2: imu.mutable_orientation()->set_z(nan); break;
    case 3: imu.mutable_orientation()->set_w(inf); break;
    case 4: imu.mutable_angular_velocity()->set_x(nan); break;
    case 5: imu.mutable_angular_velocity()->set_y(inf); break;
    case 6: imu.mutable_angular_velocity()->set_z(nan); break;
    case 7: imu.mutable_linear_acceleration()->set_x(inf); break;
    case 8: imu.mutable_linear_acceleration()->set_y(nan); break;
    case 9: imu.mutable_linear_acceleration()->set_z(inf); break;
  }
  EXPECT_THROW(bridge::to_ros_known_noise_imu(imu), std::invalid_argument);
}
INSTANTIATE_TEST_SUITE_P(FiniteOnly, NonfiniteImuComponent, ::testing::Range(0, 10));

TEST(StrictStampSequence, RejectsDuplicatesRegressionsAndNegativeTimes)
{
  bridge::StrictStampSequence sequence;
  EXPECT_FALSE(sequence.has_stamp());
  EXPECT_FALSE(sequence.would_accept(-1));
  EXPECT_THROW(sequence.commit(-1), std::invalid_argument);
  sequence.commit(0);
  EXPECT_TRUE(sequence.has_stamp());
  EXPECT_FALSE(sequence.would_accept(0));
  EXPECT_THROW(sequence.commit(0), std::invalid_argument);
  sequence.commit(10000000001LL);
  EXPECT_FALSE(sequence.would_accept(10000000000LL));
  EXPECT_THROW(sequence.commit(10000000000LL), std::invalid_argument);
  EXPECT_EQ(sequence.last_stamp_ns(), 10000000001LL);
  EXPECT_TRUE(sequence.would_accept(10000000002LL));
}

TEST(StrictStampSequence, IndependentSourcesMayShareAnAcquisitionEpoch)
{
  bridge::StrictStampSequence noisy, reference, imu;
  noisy.commit(123456789);
  reference.commit(123456789);
  imu.commit(123456789);
  EXPECT_TRUE(noisy.would_accept(123456790));
  EXPECT_TRUE(reference.would_accept(123456790));
  EXPECT_TRUE(imu.would_accept(123456790));
}
