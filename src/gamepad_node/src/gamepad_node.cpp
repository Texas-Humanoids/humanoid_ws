// gamepad_node: publishes /cmd_vel and /gamepad/button_event from a gamepad (via /joy)
// or from the keyboard. Mode and speeds are ROS parameters; the node never waits for input.

#include <dirent.h>
#include <fcntl.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"

#include "gamepad_node/keyboard_input.hpp"
#include "gamepad_node/teleop_mapping.hpp"

namespace
{

namespace gt = gamepad_node;
using SteadyClock = std::chrono::steady_clock;

// ---------- Keyboard device helpers (Linux evdev) ----------

constexpr size_t kBitsPerLong = 8 * sizeof(unsigned long);

bool has_key(const unsigned long * bits, int key)
{
  return bits[key / kBitsPerLong] & (1UL << (key % kBitsPerLong));
}

// Returns an open fd, -1 if the device can't be opened, -2 if it isn't a keyboard.
int open_keyboard_device(const std::string & path, std::string & name)
{
  int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
  if (fd < 0) {
    return -1;
  }
  unsigned long bits[(KEY_MAX + kBitsPerLong) / kBitsPerLong] = {0};
  if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(bits)), bits) < 0 ||
    !has_key(bits, KEY_W) || !has_key(bits, KEY_A) || !has_key(bits, KEY_LEFT))
  {
    close(fd);
    return -2;
  }
  char buf[256] = "unknown";
  ioctl(fd, EVIOCGNAME(sizeof(buf)), buf);
  name = buf;
  return fd;
}

int find_keyboard(std::string & name, std::string & path)
{
  DIR * dir = opendir("/dev/input");
  if (!dir) {
    return -1;
  }
  bool permission_denied = false;
  int fd = -1;
  while (dirent * e = readdir(dir)) {
    if (std::strncmp(e->d_name, "event", 5) != 0) {
      continue;
    }
    const std::string p = std::string("/dev/input/") + e->d_name;
    const int r = open_keyboard_device(p, name);
    if (r >= 0) {
      fd = r;
      path = p;
      break;
    }
    if (r == -1 && errno == EACCES) {
      permission_denied = true;
    }
  }
  closedir(dir);
  if (fd < 0 && permission_denied) {
    throw std::runtime_error(
            "permission denied reading /dev/input. Add the user to the input group: "
            "sudo usermod -aG input $USER, then log out and back in");
  }
  return fd;
}

// ---------- Node ----------

class GamepadNode : public rclcpp::Node
{
public:
  GamepadNode()
  : Node("gamepad_node")
  {
    const std::string input_mode = declare_parameter<std::string>("input_mode", "controller");
    scales_.linear_x = declare_parameter<double>("scale.linear.x", 0.5);
    scales_.linear_y = declare_parameter<double>("scale.linear.y", 0.5);
    scales_.angular_z = declare_parameter<double>("scale.angular.z", 1.0);
    scales_.keyboard_scaling = declare_parameter<double>("keyboard_scaling", 0.8);
    print_velocity_ = declare_parameter<bool>("print_velocity", true);
    gt::validate(scales_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    btn_pub_ = create_publisher<std_msgs::msg::String>("gamepad/button_event", 10);

    if (input_mode == "controller") {
      setup_controller();
    } else if (input_mode == "keyboard") {
      setup_keyboard();
    } else {
      throw std::invalid_argument(
              "input_mode must be 'controller' or 'keyboard', got '" + input_mode + "'");
    }

    RCLCPP_INFO(
      get_logger(), "Full-input speeds: x = %.2f m/s, y = %.2f m/s, turning = %.2f rad/s",
      scales_.linear_x, scales_.linear_y, scales_.angular_z);
  }

  ~GamepadNode() override
  {
    stop();
    if (fd_ >= 0) {
      close(fd_);
    }
    if (term_changed_) {
      tcflush(STDIN_FILENO, TCIFLUSH);  // discard keys typed while running
      tcsetattr(STDIN_FILENO, TCSANOW, &old_term_);
    }
  }

  // Called by the context's pre-shutdown hook while the publisher is still valid.
  // The same lock gates normal output so a racing callback cannot restart motion.
  void stop() noexcept
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    if (stopping_) {
      return;
    }
    stopping_ = true;
    if (!get_node_base_interface()->get_context()->is_valid()) {
      return;
    }
    try {
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
      if (!cmd_pub_->wait_for_all_acked(std::chrono::milliseconds(250))) {
        RCLCPP_WARN(get_logger(), "Timed out waiting for stop command acknowledgement");
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Could not publish shutdown stop: %s", e.what());
    }
  }

private:
  // ---------- Shared output ----------

  void publish_velocity(const gt::Velocity & v)
  {
    std::lock_guard<std::mutex> lock(output_mutex_);
    if (stopping_) {
      return;
    }
    geometry_msgs::msg::Twist twist;  // all other fields stay 0
    twist.linear.x = v.linear_x;
    twist.linear.y = v.linear_y;
    twist.angular.z = v.angular_z;
    cmd_pub_->publish(twist);
    if (print_velocity_) {
      log_velocity(v);
    }
  }

  void publish_button_event(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    btn_pub_->publish(msg);
  }

  void log_velocity(const gt::Velocity & v)
  {
    auto r = [](double value) {return std::round(value * 100.0) / 100.0 + 0.0;};
    const double x = r(v.linear_x);
    const double y = r(v.linear_y);
    const double z = r(v.angular_z);
    if (x == last_x_ && y == last_y_ && z == last_z_) {
      return;
    }
    const bool stopped = (x == 0.0 && y == 0.0 && z == 0.0);
    const auto now_time = SteadyClock::now();
    if (!stopped && now_time - last_log_time_ < std::chrono::milliseconds(250)) {
      return;  // at most 4 lines per second while moving; always show the stop
    }
    RCLCPP_INFO(
      get_logger(), "Linear: x = %+.2f m/s, y = %+.2f m/s | Turning Velocity: %+.2f rad/s",
      x, y, z);
    last_x_ = x;
    last_y_ = y;
    last_z_ = z;
    last_log_time_ = now_time;
  }

  // ---------- Controller mode ----------

  void setup_controller()
  {
    axes_.forward = declare_parameter<int>("axis_forward", 1);
    axes_.lateral = declare_parameter<int>("axis_lateral", 0);
    axes_.turn = declare_parameter<int>("axis_turn", 3);
    enable_button_ = declare_parameter<int>("enable_button", 4);
    joy_timeout_ = declare_parameter<double>("joy_timeout", 0.5);
    if (!std::isfinite(joy_timeout_) || joy_timeout_ < 0.0) {
      throw std::invalid_argument("joy_timeout must be >= 0 (0 disables the timeout)");
    }

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) {joy_callback(*msg);});
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {joy_watchdog();});

    RCLCPP_INFO(
      get_logger(), "Controller mode: left stick = x/y, right stick = turning, enable button = %d",
      enable_button_);
  }

  void joy_callback(const sensor_msgs::msg::Joy & msg)
  {
    last_joy_time_ = SteadyClock::now();

    for (std::size_t i : gt::newly_pressed(prev_buttons_, msg.buttons)) {
      publish_button_event(gt::button_name(i) + " pressed");
    }
    prev_buttons_ = msg.buttons;

    const bool enabled = gt::is_enabled(msg.buttons, enable_button_);
    if (enabled) {
      publish_velocity(gt::joy_to_velocity(msg.axes, axes_, scales_));
    } else if (was_enabled_) {
      publish_velocity(gt::Velocity{});  // one stop message on release
    }
    was_enabled_ = enabled;
  }

  // Stops the robot if /joy goes quiet (e.g. joy_node crashed) while the enable button was held.
  void joy_watchdog()
  {
    if (!was_enabled_ || joy_timeout_ <= 0.0) {
      return;
    }
    const std::chrono::duration<double> since = SteadyClock::now() - last_joy_time_;
    if (since.count() > joy_timeout_) {
      RCLCPP_WARN(get_logger(), "No /joy message for %.2f s, sending stop", since.count());
      publish_velocity(gt::Velocity{});
      was_enabled_ = false;
    }
  }

  // ---------- Keyboard mode ----------

  void setup_keyboard()
  {
    const std::string device = declare_parameter<std::string>("keyboard_device", "");
    const double rate = declare_parameter<double>("publish_rate", 20.0);
    if (!std::isfinite(rate) || rate <= 0.0) {
      throw std::invalid_argument("publish_rate must be greater than 0");
    }

    std::string name;
    std::string path = device;
    fd_ = device.empty() ? find_keyboard(name, path) : open_keyboard_device(device, name);
    if (fd_ < 0) {
      throw std::runtime_error(
              device.empty() ?
              "no keyboard found; set keyboard_device (see cat /proc/bus/input/devices)" :
              "keyboard_device '" + device + "' is missing, not a keyboard, or not readable");
    }

    // When run from a terminal, hide typed keys so they don't pile up on screen
    if (isatty(STDIN_FILENO)) {
      tcgetattr(STDIN_FILENO, &old_term_);
      termios t = old_term_;
      t.c_lflag &= ~ECHO;
      tcsetattr(STDIN_FILENO, TCSANOW, &t);
      term_changed_ = true;
    }

    RCLCPP_INFO(get_logger(), "Keyboard mode: %s (%s)", name.c_str(), path.c_str());
    RCLCPP_INFO(
      get_logger(), "W/S = x, A/D = y, Left/Right arrows = turning, keyboard_scaling = %.2f",
      scales_.keyboard_scaling);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate));
    keyboard_timer_ = create_wall_timer(period, [this]() {keyboard_update();});
  }

  bool read_keyboard_state(gt::KeyState & state)
  {
    unsigned long bits[(KEY_MAX + kBitsPerLong) / kBitsPerLong] = {0};
    if (ioctl(fd_, EVIOCGKEY(sizeof(bits)), bits) < 0) {
      RCLCPP_WARN(get_logger(), "Cannot resynchronize keyboard: %s", std::strerror(errno));
      return false;
    }
    state.forward = has_key(bits, KEY_W);
    state.back = has_key(bits, KEY_S);
    state.left = has_key(bits, KEY_A);
    state.right = has_key(bits, KEY_D);
    state.turn_left = has_key(bits, KEY_LEFT);
    state.turn_right = has_key(bits, KEY_RIGHT);
    return true;
  }

  void keyboard_update()
  {
    input_event ev;
    while (fd_ >= 0) {
      const ssize_t n = read(fd_, &ev, sizeof(ev));
      if (n == static_cast<ssize_t>(sizeof(ev))) {
        const char * pressed = keyboard_input_.handle_event(
          ev, [this](gt::KeyState & state) {return read_keyboard_state(state);});
        if (pressed) {
          publish_button_event(pressed);
        }
        continue;
      }
      if (n < 0 && errno == EINTR) {
        continue;
      }
      if (n >= 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
        RCLCPP_ERROR(
          get_logger(), "Lost keyboard device: %s",
          n < 0 ? std::strerror(errno) : "unexpected end of input");
        keyboard_input_.reset();
        close(fd_);
        fd_ = -1;
      }
      break;
    }
    publish_velocity(gt::keys_to_velocity(keyboard_input_.keys(), scales_));
  }

  // Shared
  gt::ScaleConfig scales_;
  std::mutex output_mutex_;
  bool stopping_{false};
  bool print_velocity_{true};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr btn_pub_;
  double last_x_{0.0};
  double last_y_{0.0};
  double last_z_{0.0};
  SteadyClock::time_point last_log_time_{};

  // Controller
  gt::JoyAxisConfig axes_;
  int enable_button_{4};
  double joy_timeout_{0.5};
  bool was_enabled_{false};
  std::vector<int32_t> prev_buttons_;
  SteadyClock::time_point last_joy_time_{};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Keyboard
  int fd_{-1};
  gt::KeyboardInput keyboard_input_;
  bool term_changed_{false};
  termios old_term_{};
  rclcpp::TimerBase::SharedPtr keyboard_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  std::shared_ptr<GamepadNode> node;
  try {
    node = std::make_shared<GamepadNode>();
    const std::weak_ptr<GamepadNode> weak_node = node;
    node->get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      [weak_node]() {
        if (const auto live_node = weak_node.lock()) {
          live_node->stop();
        }
      });
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("gamepad_node"), "%s", e.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
