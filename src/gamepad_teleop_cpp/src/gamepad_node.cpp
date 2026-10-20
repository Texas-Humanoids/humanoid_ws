#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// ======================= Startup prompts =======================

static std::string trim(const std::string & s)
{
  const auto first = s.find_first_not_of(" \t");
  if (first == std::string::npos) {
    return "";
  }
  return s.substr(first, s.find_last_not_of(" \t") - first + 1);
}

static char ask_mode()
{
  while (true) {
    std::cout << "Input mode: keyboard or controller? (k/c): " << std::flush;
    std::string line;
    if (!std::getline(std::cin, line)) {
      throw std::runtime_error("no input received");
    }
    line = trim(line);
    if (line.size() == 1) {
      char c = static_cast<char>(std::tolower(static_cast<unsigned char>(line[0])));
      if (c == 'k' || c == 'c') {
        return c;
      }
    }
    std::cout << "  Please enter k or c\n";
  }
}

static double ask_scale(const std::string & label, const std::string & unit)
{
  while (true) {
    std::cout << label << " scaling factor (1 = 1 " << unit << ", press Enter for 1): " << std::flush;
    std::string line;
    if (!std::getline(std::cin, line)) {
      throw std::runtime_error("no input received");
    }
    line = trim(line);
    if (line.empty()) {
      return 1.0;
    }
    try {
      size_t used = 0;
      double v = std::stod(line, &used);
      if (used == line.size() && std::isfinite(v)) {
        return v;
      }
    } catch (const std::exception &) {
    }
    std::cout << "  Please enter a number, e.g. 1 or 0.5\n";
  }
}

// ======================= Keyboard device helpers =======================

static constexpr size_t BITS_PER_LONG = 8 * sizeof(unsigned long);

static bool has_key(const unsigned long * bits, int key)
{
  return bits[key / BITS_PER_LONG] & (1UL << (key % BITS_PER_LONG));
}

static int open_device(const std::string & path, std::string & name)
{
  int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
  if (fd < 0) {
    return -1;
  }
  unsigned long bits[(KEY_MAX + BITS_PER_LONG) / BITS_PER_LONG] = {0};
  if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(bits)), bits) < 0 ||
    !has_key(bits, KEY_W) || !has_key(bits, KEY_A) || !has_key(bits, KEY_LEFT))
  {
    close(fd);
    return -2;  // not a keyboard
  }
  char buf[256] = "unknown";
  ioctl(fd, EVIOCGNAME(sizeof(buf)), buf);
  name = buf;
  return fd;
}

static int find_keyboard(std::string & name, std::string & path)
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
    std::string p = std::string("/dev/input/") + e->d_name;
    int r = open_device(p, name);
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
      "permission denied reading /dev/input. Run: sudo usermod -aG input $USER, then log out and back in");
  }
  return fd;
}

// ======================= Node =======================

static const std::vector<std::string> BUTTON_NAMES = {
  "A", "B", "X", "Y", "LB", "RB", "BACK", "START", "GUIDE", "L_STICK", "R_STICK"};

class GamepadNode : public rclcpp::Node
{
public:
  GamepadNode(char mode, double scale_x, double scale_y, double scale_yaw)
  : Node("gamepad_node"), scale_x_(scale_x), scale_y_(scale_y), scale_yaw_(scale_yaw)
  {
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    btn_pub_ = create_publisher<std_msgs::msg::String>("gamepad/button_event", 10);

    if (mode == 'c') {
      setup_controller();
    } else {
      setup_keyboard();
    }
  }

  ~GamepadNode() override
  {
    if (fd_ >= 0) {
      close(fd_);
    }
    if (term_changed_) {
      tcflush(STDIN_FILENO, TCIFLUSH);  // discard keys typed while running
      tcsetattr(STDIN_FILENO, TCSANOW, &old_term_);
    }
  }

private:
  // ---------------- Controller mode ----------------

  void setup_controller()
  {
    axis_linear_ = declare_parameter<int>("axis_linear", 1);
    axis_angular_ = declare_parameter<int>("axis_angular", 0);
    scale_linear_ = declare_parameter<double>("scale_linear", 0.5);
    scale_angular_ = declare_parameter<double>("scale_angular", 1.0);
    enable_button_ = declare_parameter<int>("enable_button", 4);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&GamepadNode::joy_cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Controller mode: listening on /joy (start joy_node in another terminal).");
    RCLCPP_INFO(get_logger(), "Hold LB and move the left stick. Ctrl+C to quit.");
  }

  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Button press events (rising edge only)
    if (prev_buttons_.size() != msg->buttons.size()) {
      prev_buttons_.assign(msg->buttons.size(), 0);
    }
    for (size_t i = 0; i < msg->buttons.size(); ++i) {
      if (msg->buttons[i] && !prev_buttons_[i]) {
        std::string name = i < BUTTON_NAMES.size() ? BUTTON_NAMES[i]
                                                   : "BUTTON_" + std::to_string(i);
        std_msgs::msg::String out;
        out.data = name + " pressed";
        btn_pub_->publish(out);
      }
    }
    prev_buttons_ = msg->buttons;

    // Velocity command
    bool enabled = enable_button_ >= 0 &&
      static_cast<size_t>(enable_button_) < msg->buttons.size() &&
      msg->buttons[enable_button_] == 1;

    geometry_msgs::msg::Twist twist;
    if (enabled) {
      if (static_cast<size_t>(axis_linear_) < msg->axes.size()) {
        twist.linear.x = msg->axes[axis_linear_] * scale_linear_;
      }
      if (static_cast<size_t>(axis_angular_) < msg->axes.size()) {
        twist.angular.z = msg->axes[axis_angular_] * scale_angular_;
      }
      cmd_pub_->publish(twist);
    } else if (was_enabled_) {
      cmd_pub_->publish(twist);  // one stop message on release
    }
    was_enabled_ = enabled;
  }

  // ---------------- Keyboard mode ----------------

  enum Key { W, A, S, D, LEFT, RIGHT, NUM_KEYS };

  void setup_keyboard()
  {
    const std::string device = declare_parameter<std::string>("device", "");

    std::string name, path;
    if (!device.empty()) {
      path = device;
      fd_ = open_device(device, name);
    } else {
      fd_ = find_keyboard(name, path);
    }
    if (fd_ < 0) {
      throw std::runtime_error("no keyboard found. Find one with: cat /proc/bus/input/devices, "
        "then run with --ros-args -p device:=/dev/input/eventN");
    }

    // Hide typed keys in this terminal so they don't pile up on screen
    if (isatty(STDIN_FILENO)) {
      tcgetattr(STDIN_FILENO, &old_term_);
      termios t = old_term_;
      t.c_lflag &= ~ECHO;
      tcsetattr(STDIN_FILENO, TCSANOW, &t);
      term_changed_ = true;
    }

    RCLCPP_INFO(get_logger(), "Keyboard mode: %s (%s)", name.c_str(), path.c_str());
    RCLCPP_INFO(get_logger(), "Scales: x = %.2f, y = %.2f, yaw = %.2f", scale_x_, scale_y_, scale_yaw_);
    RCLCPP_INFO(get_logger(), "W/S = y, A/D = x, Left/Right arrows = yaw. Ctrl+C to quit.");

    timer_ = create_wall_timer(25ms, std::bind(&GamepadNode::keyboard_update, this));  // 20 Hz
  }

  void handle_key(int code, int value)
  {
    static const char * names[NUM_KEYS] = {"w", "a", "s", "d", "left", "right"};
    int k;
    switch (code) {
      case KEY_W: k = W; break;
      case KEY_A: k = A; break;
      case KEY_S: k = S; break;
      case KEY_D: k = D; break;
      case KEY_LEFT: k = LEFT; break;
      case KEY_RIGHT: k = RIGHT; break;
      default: return;
    }
    held_[k] = (value != 0);  // 1 = press, 2 = auto-repeat, 0 = release
    if (value == 1) {
      std_msgs::msg::String msg;
      msg.data = names[k];
      btn_pub_->publish(msg);
    }
  }

  void keyboard_update()
  {
    // Read all pending key events
    input_event ev;
    while (true) {
      ssize_t n = read(fd_, &ev, sizeof(ev));
      if (n == static_cast<ssize_t>(sizeof(ev))) {
        if (ev.type == EV_KEY) {
          handle_key(ev.code, ev.value);
        }
        continue;
      }
      if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && !read_error_) {
        RCLCPP_ERROR(get_logger(), "Lost keyboard device: %s", std::strerror(errno));
        read_error_ = true;
        for (bool & h : held_) {
          h = false;  // fail safe: stop
        }
      }
      break;
    }

    // Opposite keys cancel: (+1) + (-1) = 0
    const double x = (static_cast<int>(held_[D]) - static_cast<int>(held_[A])) * scale_x_;
    const double y = (static_cast<int>(held_[W]) - static_cast<int>(held_[S])) * scale_y_;
    const double yaw = (static_cast<int>(held_[LEFT]) - static_cast<int>(held_[RIGHT])) * scale_yaw_;

    geometry_msgs::msg::Twist twist;  // every field starts at 0
    twist.linear.x = x;
    twist.linear.y = y;
    twist.angular.z = yaw;
    cmd_pub_->publish(twist);

    if (x != last_x_ || y != last_y_ || yaw != last_yaw_) {
      RCLCPP_INFO(get_logger(), "x: %+.2f m/s   y: %+.2f m/s   yaw: %+.2f rad/s", x, y, yaw);
      last_x_ = x;
      last_y_ = y;
      last_yaw_ = yaw;
    }
  }

  // ---------------- Shared ----------------
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr btn_pub_;

  // ---------------- Controller state ----------------
  int axis_linear_{1}, axis_angular_{0}, enable_button_{4};
  double scale_linear_{0.5}, scale_angular_{1.0};
  bool was_enabled_{false};
  std::vector<int32_t> prev_buttons_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  // ---------------- Keyboard state ----------------
  double scale_x_, scale_y_, scale_yaw_;
  int fd_{-1};
  bool held_[NUM_KEYS] = {false, false, false, false, false, false};
  bool read_error_{false};
  bool term_changed_{false};
  termios old_term_{};
  double last_x_{0.0}, last_y_{0.0}, last_yaw_{0.0};
  rclcpp::TimerBase::SharedPtr timer_;
};

// ======================= main =======================

int main(int argc, char ** argv)
{
  char mode = 'c';
  double sx = 1.0, sy = 1.0, syaw = 1.0;
  try {
    mode = ask_mode();
    if (mode == 'k') {
      sx = ask_scale("x-linear", "m/s");
      sy = ask_scale("y-linear", "m/s");
      syaw = ask_scale("yaw", "rad/s");
    }
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<GamepadNode>(mode, sx, sy, syaw);
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("gamepad_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
