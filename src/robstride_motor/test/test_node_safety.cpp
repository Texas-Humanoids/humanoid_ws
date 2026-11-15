#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "robstride_motor/can_protocol.hpp"

namespace test_can
{
std::vector<can_frame> sent_frames;
int write_count = 0;
int fail_write = 0;
int throw_write = 0;
int close_count = 0;
int socket_count = 0;
float starting_position = 0.25f;
uint8_t motor_id = 1;
std::vector<can_frame> queued_frames;  // returned by read_frame before the reply

int open_socket(int, int, int)
{
  ++socket_count;
  return 42;
}

int get_interface(int, unsigned long, ifreq * request)
{
  request->ifr_ifindex = 1;
  return 0;
}

int bind_socket(int, const sockaddr *, socklen_t) {return 0;}
int set_option(int, int, int, const void *, socklen_t) {return 0;}
int close_socket(int) {++close_count; return 0;}

ssize_t write_frame(int, const void * data, size_t size)
{
  ++write_count;
  if (write_count == throw_write) {
    throw std::runtime_error("Injected transport exception");
  }
  if (write_count == fail_write) {
    return -1;
  }
  sent_frames.push_back(*static_cast<const can_frame *>(data));
  return static_cast<ssize_t>(size);
}

ssize_t read_frame(int, void * data, size_t size)
{
  if (!queued_frames.empty()) {
    std::memcpy(data, &queued_frames.front(), sizeof(can_frame));
    queued_frames.erase(queued_frames.begin());
    return static_cast<ssize_t>(size);
  }
  can_frame frame{};
  frame.can_id = CAN_EFF_FLAG |
    robstride_motor::make_can_id(robstride_motor::kCommTypeParamRead, motor_id, 0xFF);
  frame.can_dlc = 8;
  auto payload = robstride_motor::encode_param_float(
    robstride_motor::kParamMechPos, starting_position);
  std::memcpy(frame.data, payload.data(), payload.size());
  std::memcpy(data, &frame, sizeof(frame));
  return static_cast<ssize_t>(size);
}
}  // namespace test_can

// Replace only the operating-system boundary; exercise the production node.
// All dependencies are included first so the macros do not alter their declarations.
#define socket test_can::open_socket
#define ioctl test_can::get_interface
#define bind test_can::bind_socket
#define setsockopt test_can::set_option
#define close test_can::close_socket
#define write test_can::write_frame
#define read test_can::read_frame
#define main robstride_motor_program_main
#include "../src/robstride_motor_node.cpp"
#undef main
#undef read
#undef write
#undef close
#undef setsockopt
#undef bind
#undef ioctl
#undef socket

class NodeSafety : public testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    reset_transport();
  }

  void reset_transport()
  {
    test_can::sent_frames.clear();
    test_can::write_count = 0;
    test_can::fail_write = 0;
    test_can::throw_write = 0;
    test_can::close_count = 0;
    test_can::socket_count = 0;
    test_can::starting_position = 0.25f;
    test_can::motor_id = 1;
    test_can::queued_frames.clear();
  }

  rclcpp::NodeOptions options(int64_t id = 1)
  {
    return rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("motor_id", id)});
  }

  int count_frames(uint8_t type)
  {
    int count = 0;
    for (const auto & frame : test_can::sent_frames) {
      if (((frame.can_id >> 24) & 0x1F) == type) {
        ++count;
      }
    }
    return count;
  }
};

TEST_F(NodeSafety, RejectsInvalidIdsBeforeOpeningCan)
{
  for (const int64_t id : {-1LL, 128LL, 257LL, 9223372036854775807LL}) {
    EXPECT_THROW(RobstrideMotorNode node(options(id)), std::invalid_argument);
  }
  EXPECT_EQ(test_can::socket_count, 0);
  EXPECT_TRUE(test_can::sent_frames.empty());
}

TEST_F(NodeSafety, AcceptsBoundaryIdsWithoutTruncation)
{
  for (const int64_t id : {0LL, 127LL}) {
    reset_transport();
    test_can::motor_id = static_cast<uint8_t>(id);
    {
      RobstrideMotorNode node(options(id));
      EXPECT_EQ(count_frames(robstride_motor::kCommTypeEnable), 1);
    }
    for (const auto & frame : test_can::sent_frames) {
      EXPECT_EQ(frame.can_id & 0xFF, id);
    }
    EXPECT_EQ(test_can::close_count, 1);
  }
}

TEST_F(NodeSafety, AcceptsRs03AndRejectsInvalidLimits)
{
  {
    auto rs03 = options();
    rs03.append_parameter_override("motor_model", "RS03");
    RobstrideMotorNode node(rs03);
    EXPECT_EQ(count_frames(robstride_motor::kCommTypeEnable), 1);
  }
  for (const char * name : {"speed_limit", "current_limit", "feedback_rate"}) {
    reset_transport();
    auto invalid = options();
    invalid.append_parameter_override(name, 0.0);
    EXPECT_THROW(RobstrideMotorNode node(invalid), std::invalid_argument);
    EXPECT_EQ(test_can::socket_count, 0);
  }
  auto unknown = options();
  unknown.append_parameter_override("motor_model", "RS99");
  EXPECT_THROW(RobstrideMotorNode node(unknown), std::runtime_error);
}

TEST_F(NodeSafety, StartupClearsFaultsArmsWatchdogAndShutdownStopsReporting)
{
  {
    RobstrideMotorNode node(options());
    ASSERT_GE(test_can::sent_frames.size(), 9u);
    const auto & first = test_can::sent_frames.front();
    EXPECT_EQ((first.can_id >> 24) & 0x1F, robstride_motor::kCommTypeStop);
    EXPECT_EQ(first.data[0], 1);  // clear faults
    const auto timeout = robstride_motor::encode_param_u32(
      robstride_motor::kParamCanTimeout, robstride_motor::can_timeout_counts(0.5));
    const auto active = robstride_motor::encode_active_report(true);
    bool saw_timeout = false;
    bool saw_active = false;
    for (const auto & frame : test_can::sent_frames) {
      saw_timeout |= std::memcmp(frame.data, timeout.data(), 8) == 0;
      saw_active |= ((frame.can_id >> 24) & 0x1F) == robstride_motor::kCommTypeActiveReport &&
        std::memcmp(frame.data, active.data(), 8) == 0;
    }
    EXPECT_TRUE(saw_timeout);
    EXPECT_TRUE(saw_active);
    test_can::sent_frames.clear();
  }
  ASSERT_EQ(test_can::sent_frames.size(), 2u);
  EXPECT_EQ(
    (test_can::sent_frames[0].can_id >> 24) & 0x1F, robstride_motor::kCommTypeActiveReport);
  EXPECT_EQ(test_can::sent_frames[0].data[6], 0);  // reporting off
  EXPECT_EQ((test_can::sent_frames[1].can_id >> 24) & 0x1F, robstride_motor::kCommTypeStop);
}

TEST_F(NodeSafety, StatusAndFaultFramesDoNotDisturbTheStartingPositionRead)
{
  const auto frame = [](uint8_t type, uint32_t middle, std::array<uint8_t, 8> data) {
      can_frame f{};
      f.can_id = CAN_EFF_FLAG | (static_cast<uint32_t>(type) << 24) | (middle << 8) | 0xFE;
      f.can_dlc = 8;
      std::memcpy(f.data, data.data(), 8);
      return f;
    };
  // A faulted status frame and a fault report from this motor, plus another motor's
  // parameter reply that looks like the requested one.
  test_can::queued_frames = {
    frame(robstride_motor::kCommTypeStatus, (0x04u << 8) | 1, {0x7F, 0xFF, 0, 0, 0, 0, 0, 0}),
    frame(robstride_motor::kCommTypeFaultReport, 1, {0x01, 0, 0, 0, 0, 0, 0, 0}),
    frame(robstride_motor::kCommTypeParamRead, 2,
      robstride_motor::encode_param_float(robstride_motor::kParamMechPos, 3.0f)),
  };
  RobstrideMotorNode node(options());
  EXPECT_TRUE(test_can::queued_frames.empty());
  bool saw_initial_target = false;
  for (const auto & sent : test_can::sent_frames) {
    std::array<uint8_t, 8> data;
    std::memcpy(data.data(), sent.data, 8);
    if (((sent.can_id >> 24) & 0x1F) == robstride_motor::kCommTypeParamWrite &&
      robstride_motor::decode_param_index(data) == robstride_motor::kParamLocRef)
    {
      EXPECT_FLOAT_EQ(robstride_motor::decode_param_float(data), 0.25f);
      saw_initial_target = true;
    }
  }
  EXPECT_TRUE(saw_initial_target);
}

TEST_F(NodeSafety, RejectsInvalidCanTimeout)
{
  for (const double timeout : {-0.1, 10.5, 0.03}) {  // 0.03 s is under two 50 Hz polls
    reset_transport();
    auto invalid = options();
    invalid.append_parameter_override("can_timeout", timeout);
    EXPECT_THROW(RobstrideMotorNode node(invalid), std::invalid_argument);
    EXPECT_EQ(test_can::socket_count, 0);
  }
  auto disabled = options();
  disabled.append_parameter_override("can_timeout", 0.0);
  EXPECT_NO_THROW(RobstrideMotorNode node(disabled));
}

TEST_F(NodeSafety, EveryFailedStartupWritePreventsEnableAndClosesSocket)
{
  // Stop, mode, speed, current, CAN timeout, active report, position read request,
  // initial target, enable.
  for (int failed_write = 1; failed_write <= 9; ++failed_write) {
    reset_transport();
    test_can::fail_write = failed_write;
    EXPECT_THROW(RobstrideMotorNode node(options()), std::runtime_error);
    EXPECT_EQ(count_frames(robstride_motor::kCommTypeEnable), 0);
    EXPECT_EQ(test_can::close_count, 1);
    ASSERT_FALSE(test_can::sent_frames.empty());
    EXPECT_EQ(
      (test_can::sent_frames.back().can_id >> 24) & 0x1F,
      robstride_motor::kCommTypeStop);
  }
}

TEST_F(NodeSafety, RejectsNonFiniteStartingPosition)
{
  test_can::starting_position = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(RobstrideMotorNode node(options()), std::runtime_error);
  EXPECT_EQ(count_frames(robstride_motor::kCommTypeEnable), 0);
  EXPECT_EQ(test_can::close_count, 1);
}

TEST_F(NodeSafety, ConstructorExceptionStopsAndClosesSocket)
{
  test_can::throw_write = 7;
  EXPECT_THROW(RobstrideMotorNode node(options()), std::runtime_error);
  EXPECT_EQ(count_frames(robstride_motor::kCommTypeStop), 2);
  EXPECT_EQ(test_can::close_count, 1);
}

TEST_F(NodeSafety, ExceptionAfterEnableStopsAndClosesSocket)
{
  EXPECT_THROW(
  {
    RobstrideMotorNode node(options());
    throw std::runtime_error("Injected executor exception");
    }, std::runtime_error);
  EXPECT_EQ(count_frames(robstride_motor::kCommTypeEnable), 1);
  EXPECT_EQ(count_frames(robstride_motor::kCommTypeStop), 2);
  EXPECT_EQ(test_can::close_count, 1);
}

TEST_F(NodeSafety, ExplicitStopAndDestructorAreIdempotent)
{
  {
    RobstrideMotorNode node(options());
    node.stop();
    node.stop();
  }
  EXPECT_EQ(count_frames(robstride_motor::kCommTypeStop), 2);
  EXPECT_EQ(test_can::close_count, 1);
}
