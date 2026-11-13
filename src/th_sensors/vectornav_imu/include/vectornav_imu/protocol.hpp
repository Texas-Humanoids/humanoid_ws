#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

namespace vectornav_imu
{

// ASCII commands: "$<body>*<XOR of body bytes>\r\n".
inline uint8_t ascii_checksum(const std::string & body)
{
  uint8_t sum = 0;
  for (char c : body) {
    sum ^= static_cast<uint8_t>(c);
  }
  return sum;
}

inline std::string ascii_command(const std::string & body)
{
  char tail[8];
  std::snprintf(tail, sizeof(tail), "*%02X\r\n", ascii_checksum(body));
  return "$" + body + tail;
}

// Binary output register 1 (75): serial port 1, rate = 800 Hz / divisor, common group
// with quaternion, angular rate and acceleration.
constexpr uint8_t kSync = 0xFA;
constexpr uint8_t kGroupCommon = 0x01;
constexpr uint16_t kCommonFields = 0x0130;  // bit 4 quaternion, bit 5 angular rate, bit 8 accel
constexpr size_t kPayloadSize = 40;  // 4 + 3 + 3 little-endian floats
constexpr size_t kPacketSize = 1 + 1 + 2 + kPayloadSize + 2;
constexpr double kImuRate = 800.0;  // VN-100 internal rate the divisor applies to

inline std::string binary_output_command(int rate_divisor, bool enable = true)
{
  return "VNWRG,75," + std::string(enable ? "1" : "0") + "," + std::to_string(rate_divisor) +
         ",01,0130";
}

// CRC-16-CCITT as specified by VectorNav; a valid packet (after the sync byte,
// including its big-endian CRC) checks to 0.
inline uint16_t crc16(const uint8_t * data, size_t size)
{
  uint16_t crc = 0;
  for (size_t i = 0; i < size; ++i) {
    crc = static_cast<uint16_t>((crc >> 8) | (crc << 8));
    crc ^= data[i];
    crc ^= static_cast<uint16_t>((crc & 0xFF) >> 4);
    crc ^= static_cast<uint16_t>(crc << 12);
    crc ^= static_cast<uint16_t>((crc & 0x00FF) << 5);
  }
  return crc;
}

// One sample in the sensor's own frames: body x forward, y right, z down (FRD);
// the quaternion (scalar last) is the body's orientation in a North-East-Down world.
// (Checked on the VN-100T: gravity from the quaternion matches the accelerometer.)
struct Sample
{
  std::array<double, 4> quaternion_xyzw;
  std::array<double, 3> angular_rate;  // rad/s, FRD
  std::array<double, 3> acceleration;  // m/s^2, FRD (specific force)
};

inline float read_float(const uint8_t * p)
{
  float value;
  std::memcpy(&value, p, sizeof(value));  // little-endian on the sensor and on arm64/x86
  return value;
}

inline void write_float(uint8_t * p, float value) {std::memcpy(p, &value, sizeof(value));}

// Decodes a packet starting at its sync byte; nullopt for another layout or a bad CRC.
inline std::optional<Sample> decode_packet(const uint8_t * p)
{
  if (p[0] != kSync || p[1] != kGroupCommon ||
    static_cast<uint16_t>(p[2] | (p[3] << 8)) != kCommonFields ||
    crc16(p + 1, kPacketSize - 1) != 0)
  {
    return std::nullopt;
  }
  const uint8_t * f = p + 4;
  Sample s{};
  for (int i = 0; i < 4; ++i) {
    s.quaternion_xyzw[i] = read_float(f + 4 * i);
  }
  for (int i = 0; i < 3; ++i) {
    s.angular_rate[i] = read_float(f + 16 + 4 * i);
  }
  for (int i = 0; i < 3; ++i) {
    s.acceleration[i] = read_float(f + 28 + 4 * i);
  }
  return s;
}

// The inverse of decode_packet, for tests.
inline std::vector<uint8_t> encode_packet(const Sample & s)
{
  std::vector<uint8_t> p(kPacketSize, 0);
  p[0] = kSync;
  p[1] = kGroupCommon;
  p[2] = kCommonFields & 0xFF;
  p[3] = kCommonFields >> 8;
  for (int i = 0; i < 4; ++i) {
    write_float(&p[4 + 4 * i], static_cast<float>(s.quaternion_xyzw[i]));
  }
  for (int i = 0; i < 3; ++i) {
    write_float(&p[20 + 4 * i], static_cast<float>(s.angular_rate[i]));
  }
  for (int i = 0; i < 3; ++i) {
    write_float(&p[32 + 4 * i], static_cast<float>(s.acceleration[i]));
  }
  const uint16_t crc = crc16(&p[1], kPacketSize - 3);
  p[kPacketSize - 2] = static_cast<uint8_t>(crc >> 8);
  p[kPacketSize - 1] = static_cast<uint8_t>(crc & 0xFF);
  return p;
}

// Splits a byte stream (binary packets mixed with ASCII replies) into samples.
class PacketFramer
{
public:
  std::vector<Sample> push(const uint8_t * data, size_t size)
  {
    buffer_.insert(buffer_.end(), data, data + size);
    std::vector<Sample> out;
    size_t i = 0;
    while (i + kPacketSize <= buffer_.size()) {
      if (buffer_[i] != kSync) {
        ++i;
        continue;
      }
      if (auto sample = decode_packet(&buffer_[i])) {
        out.push_back(*sample);
        i += kPacketSize;
      } else {
        ++bad_packets_;
        ++i;  // a 0xFA inside other data; resync on the next one
      }
    }
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(i));
    if (buffer_.size() > 4096) {
      buffer_.clear();
    }
    return out;
  }

  size_t bad_packets() const {return bad_packets_;}

private:
  std::vector<uint8_t> buffer_;
  size_t bad_packets_{0};
};

// ROS conventions (REP 103): body x forward, y left, z up (FLU); world East-North-Up.
struct RosSample
{
  std::array<double, 4> orientation_wxyz;
  std::array<double, 3> angular_velocity;
  std::array<double, 3> linear_acceleration;
};

using Quat = std::array<double, 4>;  // w, x, y, z

inline Quat multiply(const Quat & a, const Quat & b)
{
  return {
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

inline RosSample to_ros_frames(const Sample & s)
{
  const double h = std::sqrt(0.5);
  const Quat enu_from_ned{0.0, h, h, 0.0};  // swaps x/y, flips z
  const Quat frd_from_flu{0.0, 1.0, 0.0, 0.0};  // 180 deg about x
  const Quat ned_from_frd{
    s.quaternion_xyzw[3], s.quaternion_xyzw[0], s.quaternion_xyzw[1], s.quaternion_xyzw[2]};
  Quat q = multiply(multiply(enu_from_ned, ned_from_frd), frd_from_flu);
  const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (q[0] < 0.0) {
    for (double & v : q) {
      v = -v;
    }
  }
  RosSample r{};
  for (int i = 0; i < 4; ++i) {
    r.orientation_wxyz[i] = q[i] / n;
  }
  r.angular_velocity = {s.angular_rate[0], -s.angular_rate[1], -s.angular_rate[2]};
  r.linear_acceleration = {s.acceleration[0], -s.acceleration[1], -s.acceleration[2]};
  return r;
}

// Angle (rad) between gravity predicted by the orientation and gravity measured by the
// accelerometer, both in the body frame. Near zero when the robot is still and the
// frame handling is right.
inline double gravity_disagreement(const RosSample & r)
{
  const auto & q = r.orientation_wxyz;
  // Body-frame "down" = conj(q) * (0, 0, -1).
  const double w = q[0], x = -q[1], y = -q[2], z = -q[3];
  const double vx = 0.0, vy = 0.0, vz = -1.0;
  const double tx = 2.0 * (y * vz - z * vy), ty = 2.0 * (z * vx - x * vz),
    tz = 2.0 * (x * vy - y * vx);
  const std::array<double, 3> down{
    vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz),
    vz + w * tz + (x * ty - y * tx)};
  const auto & a = r.linear_acceleration;
  const double norm = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  if (norm < 1e-6) {
    return M_PI;
  }
  // Specific force points up, opposite to gravity.
  const double dot = -(down[0] * a[0] + down[1] * a[1] + down[2] * a[2]) / norm;
  return std::acos(std::fmax(-1.0, std::fmin(1.0, dot)));
}

}  // namespace vectornav_imu
