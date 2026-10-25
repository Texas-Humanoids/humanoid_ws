#ifndef ROBSTRIDE_MOTOR__CAN_PROTOCOL_HPP_
#define ROBSTRIDE_MOTOR__CAN_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <cstring>

namespace robstride_motor
{

constexpr uint8_t kHostCanId = 0xFF;
constexpr uint8_t kCommTypeEnable = 3;
constexpr uint8_t kCommTypeStop = 4;
constexpr uint8_t kCommTypeParamRead = 0x11;
constexpr uint8_t kCommTypeParamWrite = 0x12;

constexpr uint16_t kParamRunMode = 0x7005;
constexpr uint16_t kParamLocRef = 0x7016;
constexpr uint16_t kParamSpeedLimit = 0x7017;
constexpr uint16_t kParamCurrentLimit = 0x7018;
constexpr uint16_t kParamMechPos = 0x7019;

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
