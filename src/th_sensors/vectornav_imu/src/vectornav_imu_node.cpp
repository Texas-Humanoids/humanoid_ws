#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstring>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "vectornav_imu/protocol.hpp"

namespace vn = vectornav_imu;
using SteadyClock = std::chrono::steady_clock;

// Reads a VectorNav VN-100 over its serial port and publishes sensor_msgs/Imu in ROS
// frames (body x forward, y left, z up; world East-North-Up). At startup it switches the
// sensor from its default ASCII output to a binary quaternion + angular rate +
// acceleration stream. These register writes are not saved to flash, so a power cycle
// restores the sensor's own defaults.
class VectorNavImu : public rclcpp::Node
{
public:
  VectorNavImu()
  : Node("vectornav_imu")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    const int baud = declare_parameter<int>("baud", 115200);
    const double rate = declare_parameter<double>("rate", 200.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    restore_on_exit_ = declare_parameter<bool>("restore_on_exit", true);
    max_gravity_error_ = declare_parameter<double>("max_gravity_error", 0.1);

    divisor_ = static_cast<int>(std::lround(vn::kImuRate / rate));
    if (!(rate > 0.0) || divisor_ < 1 ||
      std::fabs(vn::kImuRate / divisor_ - rate) > 1e-6 * rate)
    {
      throw std::invalid_argument("rate must be 800 Hz divided by a whole number (e.g. 200, 100)");
    }
    // 10 bits per byte on the wire; keep the stream under 90 % of the link.
    if (vn::kPacketSize * 10.0 * rate > 0.9 * baud) {
      throw std::invalid_argument(
              "rate " + std::to_string(rate) + " Hz does not fit " + std::to_string(baud) +
              " baud; lower rate (200 Hz fits 115200)");
    }

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      declare_parameter<std::string>("imu_topic", "/imu/data"), rclcpp::SensorDataQoS());

    open_port(baud);
    try {
      configure();
    } catch (...) {
      close(fd_);
      throw;
    }
    RCLCPP_INFO(
      get_logger(), "VectorNav on %s at %d baud: binary output at %.0f Hz on %s",
      port_.c_str(), baud, vn::kImuRate / divisor_, publisher_->get_topic_name());

    running_ = true;
    reader_ = std::thread([this]() {read_loop();});
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {check_health();});
  }

  ~VectorNavImu() override
  {
    running_ = false;
    if (reader_.joinable()) {
      reader_.join();
    }
    if (restore_on_exit_) {
      // Best effort: stop the binary stream and bring back the default 40 Hz ASCII output.
      write_all(vn::ascii_command(vn::binary_output_command(divisor_, false)));
      write_all(vn::ascii_command("VNWRG,06,14"));
    }
    close(fd_);
  }

private:
  void open_port(int baud)
  {
    static const std::map<int, speed_t> speeds = {
      {115200, B115200}, {230400, B230400}, {460800, B460800}, {921600, B921600}};
    const auto speed = speeds.find(baud);
    if (speed == speeds.end()) {
      throw std::invalid_argument("baud must be 115200, 230400, 460800 or 921600");
    }
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error(
              "Cannot open " + port_ + ": " + std::strerror(errno) +
              " (in the dialout group, or in the robot container?)");
    }
    termios tio{};
    tcgetattr(fd_, &tio);
    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~CRTSCTS;
    cfsetispeed(&tio, speed->second);
    cfsetospeed(&tio, speed->second);
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
      close(fd_);
      throw std::runtime_error("Cannot configure " + port_);
    }
    tcflush(fd_, TCIOFLUSH);
  }

  bool write_all(const std::string & data)
  {
    size_t done = 0;
    while (done < data.size()) {
      const ssize_t n = write(fd_, data.data() + done, data.size() - done);
      if (n < 0) {
        if (errno == EAGAIN) {
          continue;
        }
        return false;
      }
      done += static_cast<size_t>(n);
    }
    return true;
  }

  // Sends a register write and waits for its echo. Binary data may already be flowing,
  // so the reply is searched for in the raw stream.
  void write_register(const std::string & body)
  {
    const std::string echo = "$" + body.substr(0, 8);  // "$VNWRG,NN"
    for (int attempt = 0; attempt < 3; ++attempt) {
      if (!write_all(vn::ascii_command(body))) {
        throw std::runtime_error("Write to " + port_ + " failed");
      }
      std::string seen;
      const auto deadline = SteadyClock::now() + std::chrono::milliseconds(300);
      while (SteadyClock::now() < deadline) {
        char buf[512];
        const ssize_t n = read(fd_, buf, sizeof(buf));
        if (n > 0) {
          seen.append(buf, static_cast<size_t>(n));
          if (seen.find(echo) != std::string::npos) {
            return;
          }
          const auto err = seen.find("$VNERR,");
          if (err != std::string::npos && seen.size() >= err + 9) {
            throw std::runtime_error(
                    "VectorNav rejected '" + body + "' with " + seen.substr(err, 9));
          }
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
      }
    }
    throw std::runtime_error(
            "No reply from a VectorNav on " + port_ + " (wrong port or baud, or not powered?)");
  }

  void configure()
  {
    write_register("VNWRG,06,0");  // async ASCII output off
    write_register(vn::binary_output_command(divisor_));
  }

  void read_loop()
  {
    uint8_t buf[1024];
    while (running_) {
      fd_set set;
      FD_ZERO(&set);
      FD_SET(fd_, &set);
      timeval tv{0, 50000};
      if (select(fd_ + 1, &set, nullptr, nullptr, &tv) <= 0) {
        continue;
      }
      const ssize_t n = read(fd_, buf, sizeof(buf));
      if (n <= 0) {
        continue;
      }
      for (const auto & sample : framer_.push(buf, static_cast<size_t>(n))) {
        publish(vn::to_ros_frames(sample));
      }
    }
  }

  void publish(const vn::RosSample & r)
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.orientation.w = r.orientation_wxyz[0];
    msg.orientation.x = r.orientation_wxyz[1];
    msg.orientation.y = r.orientation_wxyz[2];
    msg.orientation.z = r.orientation_wxyz[3];
    msg.angular_velocity.x = r.angular_velocity[0];
    msg.angular_velocity.y = r.angular_velocity[1];
    msg.angular_velocity.z = r.angular_velocity[2];
    msg.linear_acceleration.x = r.linear_acceleration[0];
    msg.linear_acceleration.y = r.linear_acceleration[1];
    msg.linear_acceleration.z = r.linear_acceleration[2];
    publisher_->publish(msg);

    std::lock_guard<std::mutex> lock(mutex_);
    ++count_;
    last_sample_ = SteadyClock::now();
    // Only judge the frame check while nearly still (accel magnitude close to g).
    const auto & a = r.linear_acceleration;
    const double g = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (std::fabs(g - 9.81) < 0.3) {
      gravity_error_sum_ += vn::gravity_disagreement(r);
      ++gravity_error_count_;
    }
  }

  void check_health()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now_time = SteadyClock::now();
    if (count_ == 0 || now_time - last_sample_ > std::chrono::milliseconds(500)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "No IMU packets for 0.5 s (%zu bad packets)",
        framer_.bad_packets());
    }
    if (++seconds_ % 5 == 0) {
      RCLCPP_INFO(
        get_logger(), "%.0f Hz, %zu bad packets", (count_ - last_count_) / 5.0,
        framer_.bad_packets());
      last_count_ = count_;
    }
    if (!gravity_checked_ && gravity_error_count_ >= 200) {
      gravity_checked_ = true;
      const double mean = gravity_error_sum_ / gravity_error_count_;
      if (mean > max_gravity_error_) {
        RCLCPP_ERROR(
          get_logger(), "Orientation and accelerometer disagree on gravity by %.1f deg: the "
          "frame conversion is wrong for this sensor. Do not walk.", mean * 180.0 / M_PI);
      } else {
        RCLCPP_INFO(
          get_logger(), "Frame check passed: orientation and accelerometer agree within %.1f deg",
          mean * 180.0 / M_PI);
      }
    }
  }

  std::string port_;
  std::string frame_id_;
  bool restore_on_exit_{true};
  double max_gravity_error_{};
  int divisor_{4};
  int fd_{-1};
  vn::PacketFramer framer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> running_{false};
  std::thread reader_;

  std::mutex mutex_;
  size_t count_{0};
  size_t last_count_{0};
  size_t seconds_{0};
  SteadyClock::time_point last_sample_{};
  double gravity_error_sum_{0.0};
  size_t gravity_error_count_{0};
  bool gravity_checked_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<VectorNavImu>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("vectornav_imu"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
