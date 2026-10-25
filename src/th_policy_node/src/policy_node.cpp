#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class PolicyNode : public rclcpp::Node
{
public:
  PolicyNode()
  : Node("policy_node"), env_(ORT_LOGGING_LEVEL_WARNING, "th_policy_node"), session_(nullptr)
  {
    const auto model_path = declare_parameter<std::string>("model_path", "");
    const auto observation_topic = declare_parameter<std::string>(
      "observation_topic", "/policy/observation");
    const auto target_topic = declare_parameter<std::string>(
      "joint_target_topic", "/policy/joint_targets");
    const auto observation_size = declare_parameter<int64_t>("expected_observation_size", 75);
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{});
    default_positions_ = declare_parameter<std::vector<double>>(
      "default_joint_positions", std::vector<double>{});
    action_scale_ = declare_parameter<double>("action_scale", 0.25);
    action_lower_ = declare_parameter<double>("action_limit_lower", -10000.0);
    action_upper_ = declare_parameter<double>("action_limit_upper", 10000.0);

    if (model_path.empty() || !std::filesystem::is_regular_file(model_path)) {
      throw std::invalid_argument("model_path must point to an existing ONNX file");
    }
    if (observation_topic.empty() || target_topic.empty() || observation_size <= 0 ||
      joint_names_.empty() || joint_names_.size() != default_positions_.size() ||
      !std::isfinite(action_scale_) || !std::isfinite(action_lower_) ||
      !std::isfinite(action_upper_) || action_lower_ >= action_upper_ ||
      !std::all_of(default_positions_.begin(), default_positions_.end(),
        [](double value) {return std::isfinite(value);}))
    {
      throw std::invalid_argument("Invalid policy profile parameters");
    }
    auto unique_names = joint_names_;
    std::sort(unique_names.begin(), unique_names.end());
    if (unique_names.front().empty() ||
      std::adjacent_find(unique_names.begin(), unique_names.end()) != unique_names.end())
    {
      throw std::invalid_argument("Policy joint names must be nonempty and unique");
    }
    observation_size_ = static_cast<size_t>(observation_size);

    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    session_ = Ort::Session(env_, model_path.c_str(), options);
    if (session_.GetInputCount() != 1 || session_.GetOutputCount() != 1) {
      throw std::invalid_argument("Policy must have exactly one input and one output");
    }
    const auto input_type = session_.GetInputTypeInfo(0);
    const auto output_type = session_.GetOutputTypeInfo(0);
    const auto input_info = input_type.GetTensorTypeAndShapeInfo();
    const auto output_info = output_type.GetTensorTypeAndShapeInfo();
    if (input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
    {
      throw std::invalid_argument("Policy input and output must be float32 tensors");
    }
    input_shape_ = validate_shape(input_info.GetShape(), observation_size_, "input");
    validate_shape(output_info.GetShape(), joint_names_.size(), "output");
    Ort::AllocatorWithDefaultOptions allocator;
    input_name_ = session_.GetInputNameAllocated(0, allocator).get();
    output_name_ = session_.GetOutputNameAllocated(0, allocator).get();

    publisher_ = create_publisher<sensor_msgs::msg::JointState>(target_topic, 10);
    subscription_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      observation_topic, 10,
      std::bind(&PolicyNode::on_observation, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Loaded %s: %zu inputs, %zu joint targets",
      model_path.c_str(), observation_size_, joint_names_.size());
    RCLCPP_WARN(get_logger(), "Policy targets are diagnostic; no motor is connected");
  }

private:
  static std::vector<int64_t> validate_shape(
    const std::vector<int64_t> & shape, size_t width, const char * label)
  {
    if ((shape.size() != 1 && shape.size() != 2) ||
      (shape.size() == 2 && shape[0] != 1 && shape[0] != -1) ||
      shape.back() != static_cast<int64_t>(width))
    {
      throw std::invalid_argument(std::string("Unexpected policy ") + label + " shape");
    }
    return shape.size() == 1 ? std::vector<int64_t>{static_cast<int64_t>(width)} :
           std::vector<int64_t>{1, static_cast<int64_t>(width)};
  }

  void on_observation(const std_msgs::msg::Float32MultiArray::SharedPtr message)
  {
    if (message->data.size() != observation_size_ ||
      !std::all_of(message->data.begin(), message->data.end(),
        [](float value) {return std::isfinite(value);}))
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid policy observation");
      return;
    }
    try {
      auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
      // ONNX Runtime's API takes a mutable pointer but does not modify this input.
      auto input = Ort::Value::CreateTensor<float>(
        memory, const_cast<float *>(message->data.data()), message->data.size(),
        input_shape_.data(), input_shape_.size());
      const char * input_names[] = {input_name_.c_str()};
      const char * output_names[] = {output_name_.c_str()};
      auto results = session_.Run(
        Ort::RunOptions{nullptr}, input_names, &input, 1, output_names, 1);
      if (results.size() != 1 || !results[0].IsTensor() ||
        results[0].GetTensorTypeAndShapeInfo().GetElementCount() != joint_names_.size())
      {
        throw std::runtime_error("Policy returned the wrong action count");
      }
      const auto * actions = results[0].GetTensorData<float>();
      sensor_msgs::msg::JointState targets;
      targets.header.stamp = now();
      targets.name = joint_names_;
      targets.position.reserve(joint_names_.size());
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        if (!std::isfinite(actions[i])) {
          throw std::runtime_error("Policy returned a non-finite action");
        }
        const double target = default_positions_[i] + action_scale_ *
          std::clamp(static_cast<double>(actions[i]), action_lower_, action_upper_);
        if (!std::isfinite(target)) {
          throw std::runtime_error("Policy computed a non-finite target");
        }
        targets.position.push_back(target);
      }
      publisher_->publish(targets);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Policy inference failed: %s", error.what());
    }
  }

  Ort::Env env_;
  Ort::Session session_;
  size_t observation_size_{};
  std::vector<std::string> joint_names_;
  std::vector<double> default_positions_;
  double action_scale_{};
  double action_lower_{};
  double action_upper_{};
  std::vector<int64_t> input_shape_;
  std::string input_name_;
  std::string output_name_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PolicyNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("policy_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
