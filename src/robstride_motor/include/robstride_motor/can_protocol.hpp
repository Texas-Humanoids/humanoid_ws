#ifndef ROBSTRIDE_MOTOR__CAN_PROTOCOL_HPP_
#define ROBSTRIDE_MOTOR__CAN_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <string>
#include <utility>

namespace robstride_motor
{

constexpr uint8_t kHostCanId = 0xFF;
constexpr uint8_t kCommTypeStatus = 2;
constexpr uint8_t kCommTypeEnable = 3;
constexpr uint8_t kCommTypeStop = 4;
constexpr uint8_t kCommTypeParamRead = 0x11;
constexpr uint8_t kCommTypeParamWrite = 0x12;
constexpr uint8_t kCommTypeFaultReport = 0x15;
constexpr uint8_t kCommTypeActiveReport = 0x18;

constexpr uint16_t kParamRunMode = 0x7005;
constexpr uint16_t kParamLocRef = 0x7016;
constexpr uint16_t kParamSpeedLimit = 0x7017;
constexpr uint16_t kParamCurrentLimit = 0x7018;
constexpr uint16_t kParamMechPos = 0x7019;
constexpr uint16_t kParamMechVel = 0x701B;
constexpr uint16_t kParamCanTimeout = 0x7028;  // uint32, 20000 counts per second

// mode field of status frames
constexpr uint8_t kModeReset = 0;
constexpr uint8_t kModeCalibration = 1;
constexpr uint8_t kModeRunning = 2;

constexpr uint8_t kCspRunMode = 5;

inline uint32_t make_can_id(uint8_t comm_type, uint16_t middle_field, uint8_t motor_id)
{
  return (static_cast<uint32_t>(comm_type) << 24) |
         (static_cast<uint32_t>(middle_field) << 8) |
         static_cast<uint32_t>(motor_id);
}

inline std::array<uint8_t, 8> encode_param_index(uint16_t index)
{
  std::array<uint8_t, 8> data = {0, 0, 0, 0, 0, 0, 0, 0};
  data[0] = static_cast<uint8_t>(index & 0xFF);
  data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
  return data;
}

inline std::array<uint8_t, 8> encode_param_u8(uint16_t index, uint8_t value)
{
  std::array<uint8_t, 8> data = encode_param_index(index);
  data[4] = value;
  return data;
}

inline std::array<uint8_t, 8> encode_param_u32(uint16_t index, uint32_t value)
{
  std::array<uint8_t, 8> data = encode_param_index(index);
  for (int i = 0; i < 4; ++i) {
    data[4 + i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFF);
  }
  return data;
}

inline uint32_t can_timeout_counts(double seconds)
{
  return static_cast<uint32_t>(std::lround(seconds * 20000.0));
}

// Stop frame payload; byte 0 = 1 also clears latched faults.
inline std::array<uint8_t, 8> encode_stop(bool clear_faults)
{
  return {static_cast<uint8_t>(clear_faults ? 1 : 0), 0, 0, 0, 0, 0, 0, 0};
}

// Turns the motor's periodic status broadcast (every 10 ms by default) on or off.
inline std::array<uint8_t, 8> encode_active_report(bool enabled)
{
  return {1, 2, 3, 4, 5, 6, static_cast<uint8_t>(enabled ? 1 : 0), 0};
}

inline uint8_t frame_comm_type(uint32_t can_id) {return (can_id >> 24) & 0x1F;}

// Replies and status frames carry the sending motor's ID in bits 8..15.
inline uint8_t frame_motor_id(uint32_t can_id) {return (can_id >> 8) & 0xFF;}

struct MotorStatus
{
  uint8_t mode;  // kModeReset, kModeCalibration or kModeRunning
  uint8_t faults;  // bits listed in kStatusFaultNames
  double position;  // rad
  double velocity;  // rad/s
  double torque;  // Nm
  double temperature;  // degrees C
};

constexpr std::array<const char *, 6> kStatusFaultNames = {
  "undervoltage", "overcurrent", "overtemperature", "magnetic encoder fault", "stall",
  "uncalibrated"};

inline double decode_u16_range(uint8_t high, uint8_t low, double max)
{
  const uint16_t raw = static_cast<uint16_t>((high << 8) | low);
  return (raw / 32767.0 - 1.0) * max;
}

// Status (type 2) and active report (type 24) frames. Position, velocity and
// torque are big-endian uint16 mapped over each model's +/- range.
inline MotorStatus decode_status(
  uint32_t can_id, const std::array<uint8_t, 8> & data,
  double position_max, double velocity_max, double torque_max)
{
  return {
    static_cast<uint8_t>((can_id >> 22) & 0x03),
    static_cast<uint8_t>((can_id >> 16) & 0x3F),
    decode_u16_range(data[0], data[1], position_max),
    decode_u16_range(data[2], data[3], velocity_max),
    decode_u16_range(data[4], data[5], torque_max),
    static_cast<uint16_t>((data[6] << 8) | data[7]) * 0.1};
}

inline const char * mode_name(uint8_t mode)
{
  switch (mode) {
    case kModeReset: return "reset (disabled)";
    case kModeCalibration: return "calibrating";
    case kModeRunning: return "running";
    default: return "unknown";
  }
}

inline std::string describe_status_faults(uint8_t faults)
{
  std::string text;
  for (size_t bit = 0; bit < kStatusFaultNames.size(); ++bit) {
    if (faults & (1U << bit)) {
      text += (text.empty() ? "" : ", ") + std::string(kStatusFaultNames[bit]);
    }
  }
  return text;
}

// Fault report (type 21): fault bits in bytes 0..3, warning bits in 4..7, little-endian.
inline std::string describe_fault_report(const std::array<uint8_t, 8> & data)
{
  const auto word = [&data](int start) {
      uint32_t value = 0;
      for (int i = 0; i < 4; ++i) {
        value |= static_cast<uint32_t>(data[start + i]) << (8 * i);
      }
      return value;
    };
  static const std::array<std::pair<int, const char *>, 11> fault_names = {{
    {0, "overtemperature"}, {1, "driver fault"}, {2, "undervoltage"}, {3, "overvoltage"},
    {4, "phase B overcurrent"}, {5, "phase C overcurrent"}, {7, "encoder uncalibrated"},
    {8, "hardware ID fault"}, {9, "position init fault"}, {14, "stall overload"},
    {16, "phase A overcurrent"}}};
  const uint32_t faults = word(0);
  std::string text;
  for (const auto & [bit, name] : fault_names) {
    if (faults & (1U << bit)) {
      text += (text.empty() ? "" : ", ") + std::string(name);
    }
  }
  if (word(4) & 1U) {
    text += (text.empty() ? "" : ", ") + std::string("overtemperature warning");
  }
  return text.empty() ? "none" : text;
}

inline std::array<uint8_t, 8> encode_param_float(uint16_t index, float value)
{
  std::array<uint8_t, 8> data = encode_param_index(index);

  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  data[4] = static_cast<uint8_t>(bits & 0xFF);
  data[5] = static_cast<uint8_t>((bits >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>((bits >> 16) & 0xFF);
  data[7] = static_cast<uint8_t>((bits >> 24) & 0xFF);
  return data;
}

inline float decode_param_float(const std::array<uint8_t, 8> & data)
{
  uint32_t bits =
    static_cast<uint32_t>(data[4]) |
    (static_cast<uint32_t>(data[5]) << 8) |
    (static_cast<uint32_t>(data[6]) << 16) |
    (static_cast<uint32_t>(data[7]) << 24);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

inline uint16_t decode_param_index(const std::array<uint8_t, 8> & data)
{
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

inline double clamp_position(double target, double position_min, double position_max)
{
  if (target < position_min) {return position_min;}
  if (target > position_max) {return position_max;}
  return target;
}

}  // namespace robstride_motor

#endif  // ROBSTRIDE_MOTOR__CAN_PROTOCOL_HPP_
