#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace th_policy_node
{

inline std::vector<int64_t> validate_vector_shape(
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

inline std::vector<double> scale_actions(
  const float * actions, size_t count, const std::vector<double> & defaults,
  double scale, double lower, double upper)
{
  if (actions == nullptr || count != defaults.size() ||
    !std::isfinite(scale) || !std::isfinite(lower) || !std::isfinite(upper) ||
    lower >= upper)
  {
    throw std::invalid_argument("Invalid policy action profile");
  }
  std::vector<double> targets;
  targets.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    if (!std::isfinite(actions[i]) || !std::isfinite(defaults[i])) {
      throw std::invalid_argument("Policy action or default position is non-finite");
    }
    const double target = defaults[i] + scale *
      std::clamp(static_cast<double>(actions[i]), lower, upper);
    if (!std::isfinite(target)) {
      throw std::overflow_error("Policy target is non-finite");
    }
    targets.push_back(target);
  }
  return targets;
}

}  // namespace th_policy_node
