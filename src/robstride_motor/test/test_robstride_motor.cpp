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
