#include <gtest/gtest.h>
#include <cmath>
#include "robstride_motor/can_protocol.hpp"

using namespace robstride_motor;

// --- CAN ID packing, verified against real candump traces from the motor ---

TEST(CanProtocol, EnableFrameId)
{
  EXPECT_EQ(make_can_id(kCommTypeEnable, kHostCanId, 127), 0x0300FF7Fu);
}

TEST(CanProtocol, StopFrameId)
{
  EXPECT_EQ(make_can_id(kCommTypeStop, kHostCanId, 127), 0x0400FF7Fu);
}

TEST(CanProtocol, ParamWriteFrameId)
{
  EXPECT_EQ(make_can_id(kCommTypeParamWrite, kHostCanId, 127), 0x1200FF7Fu);
}

TEST(CanProtocol, ParamReadFrameId)
{
  EXPECT_EQ(make_can_id(kCommTypeParamRead, kHostCanId, 127), 0x1100FF7Fu);
}

// --- Parameter write encoding, byte-for-byte against real captured frames --

TEST(CanProtocol, EncodeParamIndexOnly)
{
  auto data = encode_param_index(kParamMechPos);
  std::array<uint8_t, 8> expected = {0x19, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(data, expected);
}

TEST(CanProtocol, EncodeRunModeU8)
{
  auto data = encode_param_u8(kParamRunMode, kCspRunMode);
  std::array<uint8_t, 8> expected = {0x05, 0x70, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00};
  EXPECT_EQ(data, expected);
}

TEST(CanProtocol, EncodeSpeedLimitFloat)
{
  auto data = encode_param_float(kParamSpeedLimit, 1.0f);
  std::array<uint8_t, 8> expected = {0x17, 0x70, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F};
  EXPECT_EQ(data, expected);
}

TEST(CanProtocol, EncodeCurrentLimitFloat)
{
  auto data = encode_param_float(kParamCurrentLimit, 3.0f);
  std::array<uint8_t, 8> expected = {0x18, 0x70, 0x00, 0x00, 0x00, 0x00, 0x40, 0x40};
  EXPECT_EQ(data, expected);
}

// --- Parameter decoding (the reply direction) -------------------------------

TEST(CanProtocol, DecodeMechPosFloat)
{
  // Real captured reply: motor returned 0x3E85EEA5 for mechPos (~0.2616 rad)
  std::array<uint8_t, 8> reply = {0x19, 0x70, 0x00, 0x00, 0xA5, 0xEE, 0x85, 0x3E};
  EXPECT_NEAR(decode_param_float(reply), 0.2616f, 1e-4f);
}

TEST(CanProtocol, DecodeParamIndex)
{
  std::array<uint8_t, 8> reply = {0x19, 0x70, 0x00, 0x00, 0xA5, 0xEE, 0x85, 0x3E};
  EXPECT_EQ(decode_param_index(reply), kParamMechPos);
}

TEST(CanProtocol, EncodeDecodeRoundTrip)
{
  auto data = encode_param_float(kParamLocRef, 0.76f);
  EXPECT_NEAR(decode_param_float(data), 0.76f, 1e-6f);
}

// --- Watchdog, fault clearing and active reporting --------------------------

TEST(CanProtocol, EncodeCanTimeoutU32)
{
  // 0x7028 counts 20000 per second: 0.5 s = 10000 = 0x2710.
  EXPECT_EQ(can_timeout_counts(0.5), 10000u);
  EXPECT_EQ(can_timeout_counts(0.0), 0u);
  auto data = encode_param_u32(kParamCanTimeout, can_timeout_counts(0.5));
  std::array<uint8_t, 8> expected = {0x28, 0x70, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00};
  EXPECT_EQ(data, expected);
}

TEST(CanProtocol, StopFrameCanClearFaults)
{
  EXPECT_EQ(encode_stop(false), (std::array<uint8_t, 8>{0, 0, 0, 0, 0, 0, 0, 0}));
  EXPECT_EQ(encode_stop(true), (std::array<uint8_t, 8>{1, 0, 0, 0, 0, 0, 0, 0}));
}

TEST(CanProtocol, ActiveReportFrame)
{
  EXPECT_EQ(make_can_id(kCommTypeActiveReport, kHostCanId, 127), 0x1800FF7Fu);
  EXPECT_EQ(encode_active_report(true), (std::array<uint8_t, 8>{1, 2, 3, 4, 5, 6, 1, 0}));
  EXPECT_EQ(encode_active_report(false), (std::array<uint8_t, 8>{1, 2, 3, 4, 5, 6, 0, 0}));
}

// --- Status and fault frames (the motor -> host direction) ------------------

TEST(CanProtocol, DecodesStatusFrame)
{
  // Motor 1 to host 0xFE, running, overtemperature flag set.
  const uint32_t id = (2u << 24) | (kModeRunning << 22) | (0x04u << 16) | (1u << 8) | 0xFE;
  EXPECT_EQ(frame_comm_type(id), kCommTypeStatus);
  EXPECT_EQ(frame_motor_id(id), 1);
  // Position mid-range (0 rad), velocity full scale, torque minimum, 25.0 C.
  const std::array<uint8_t, 8> data = {0x7F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFA};
  const auto status = decode_status(id, data, 4.0 * M_PI, 50.0, 36.0);
  EXPECT_EQ(status.mode, kModeRunning);
  EXPECT_EQ(status.faults, 0x04);
  EXPECT_NEAR(status.position, 0.0, 1e-9);
  EXPECT_NEAR(status.velocity, 50.0, 0.01);
  EXPECT_NEAR(status.torque, -36.0, 1e-9);
  EXPECT_NEAR(status.temperature, 25.0, 1e-9);
  EXPECT_EQ(describe_status_faults(status.faults), "overtemperature");
  EXPECT_EQ(describe_status_faults(0x3F),
    "undervoltage, overcurrent, overtemperature, magnetic encoder fault, stall, uncalibrated");
  EXPECT_STREQ(mode_name(kModeReset), "reset (disabled)");
}

TEST(CanProtocol, DescribesFaultReport)
{
  EXPECT_EQ(describe_fault_report({0, 0, 0, 0, 0, 0, 0, 0}), "none");
  // Faults: overtemperature (bit 0), overvoltage (bit 3), phase A overcurrent (bit 16).
  EXPECT_EQ(describe_fault_report({0x09, 0x00, 0x01, 0x00, 0x01, 0, 0, 0}),
    "overtemperature, overvoltage, phase A overcurrent, overtemperature warning");
}

// --- Position clamping -------------------------------------------------------

TEST(PositionClamp, WithinRangeUnchanged)
{
  EXPECT_DOUBLE_EQ(clamp_position(1.0, -4.0 * M_PI, 4.0 * M_PI), 1.0);
}

TEST(PositionClamp, AboveMaxClampsToMax)
{
  const double max = 4.0 * M_PI;
  EXPECT_DOUBLE_EQ(clamp_position(100.0, -max, max), max);
}

TEST(PositionClamp, BelowMinClampsToMin)
{
  const double min = -4.0 * M_PI;
  EXPECT_DOUBLE_EQ(clamp_position(-100.0, min, 4.0 * M_PI), min);
}

TEST(PositionClamp, ExactlyAtBoundaryUnchanged)
{
  const double max = 4.0 * M_PI;
  EXPECT_DOUBLE_EQ(clamp_position(max, -max, max), max);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
