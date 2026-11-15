#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "vectornav_imu/protocol.hpp"

namespace vn = vectornav_imu;

TEST(Ascii, ChecksumMatchesRealReply)
{
  // Replies read from the team's VN-100T: "$VNRRG,01,VN-100T-CR*32", "$VNRRG,05,115200*5D".
  EXPECT_EQ(vn::ascii_checksum("VNRRG,01,VN-100T-CR"), 0x32);
  EXPECT_EQ(vn::ascii_checksum("VNRRG,05,115200"), 0x5D);
  EXPECT_EQ(vn::ascii_command("VNRRG,05,115200"), "$VNRRG,05,115200*5D\r\n");
}

TEST(Ascii, BinaryOutputCommand)
{
  EXPECT_EQ(vn::binary_output_command(4), "VNWRG,75,1,4,01,0130");
  EXPECT_EQ(vn::binary_output_command(4, false), "VNWRG,75,0,4,01,0130");
}

vn::Sample real_sample()
{
  // Register 15 on the VN-100T, tilted (pitch -19.9 deg, roll +11.4 deg, yaw 32.0 deg).
  return {{0.141211, -0.138512, 0.286449, 0.937454}, {0.001468, -0.000279, 0.000587},
    {-3.313, -1.789, -8.912}};
}

TEST(Binary, EncodeDecodeRoundTripAndCrc)
{
  const auto packet = vn::encode_packet(real_sample());
  ASSERT_EQ(packet.size(), vn::kPacketSize);
  EXPECT_EQ(vn::crc16(&packet[1], packet.size() - 1), 0);
  const auto s = vn::decode_packet(packet.data());
  ASSERT_TRUE(s.has_value());
  EXPECT_NEAR(s->quaternion_xyzw[3], 0.937454, 1e-6);
  EXPECT_NEAR(s->acceleration[2], -8.912, 1e-5);

  auto corrupt = packet;
  corrupt[10] ^= 0x01;
  EXPECT_FALSE(vn::decode_packet(corrupt.data()).has_value());
}

TEST(Binary, FramerSkipsAsciiAndSplitBytes)
{
  const auto packet = vn::encode_packet(real_sample());
  std::vector<uint8_t> stream;
  const std::string ascii = "$VNWRG,75,1,4,01,0130*XX\r\n";
  stream.insert(stream.end(), ascii.begin(), ascii.end());
  stream.insert(stream.end(), packet.begin(), packet.end());
  stream.push_back(0xFA);  // stray sync byte
  stream.insert(stream.end(), packet.begin(), packet.end());

  vn::PacketFramer framer;
  size_t samples = 0;
  for (size_t i = 0; i < stream.size(); i += 7) {  // arbitrary chunking
    const size_t n = std::min<size_t>(7, stream.size() - i);
    samples += framer.push(&stream[i], n).size();
  }
  EXPECT_EQ(samples, 2u);
}

TEST(Frames, RealSampleGravityAgreesInRosFrames)
{
  const auto r = vn::to_ros_frames(real_sample());
  EXPECT_LT(vn::gravity_disagreement(r), 0.5 * M_PI / 180.0);
  // Nose down -> gravity has +x in FLU; right side down -> -y.
  const auto & q = r.orientation_wxyz;
  const double gx = 2.0 * (q[1] * q[3] - q[0] * q[2]) * -1.0;  // (R^T * (0,0,-1)).x
  EXPECT_GT(gx, 0.3);
  EXPECT_GT(r.linear_acceleration[2], 8.0);  // upright-ish: z up reads +g
  EXPECT_NEAR(r.linear_acceleration[1], 1.789, 1e-9);
}

TEST(Frames, LevelFacingNorthIsYaw90InEnu)
{
  // FRD body level, x pointing north: NED orientation is identity.
  const auto r = vn::to_ros_frames({{0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.1}, {0.0, 0.0, -9.81}});
  const auto & q = r.orientation_wxyz;
  const double yaw = std::atan2(
    2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
  EXPECT_NEAR(yaw, M_PI / 2.0, 1e-9);
  EXPECT_NEAR(r.angular_velocity[2], -0.1, 1e-12);  // yaw right (FRD +z) is -z in FLU
  EXPECT_NEAR(r.linear_acceleration[2], 9.81, 1e-12);
  EXPECT_LT(vn::gravity_disagreement(r), 1e-9);
}
