#ifndef GAMEPAD_NODE__KEYBOARD_INPUT_HPP_
#define GAMEPAD_NODE__KEYBOARD_INPUT_HPP_

#include <linux/input.h>

#include "gamepad_node/teleop_mapping.hpp"

namespace gamepad_node
{

// Tracks evdev state, including recovery when the kernel drops queued events.
class KeyboardInput
{
public:
  const KeyState & keys() const {return keys_;}

  void reset()
  {
    keys_ = KeyState{};
    syncing_ = false;
  }

  // read_state must return a complete EVIOCGKEY snapshot, or false on failure.
  // Recovery does not generate press events for keys that were already held.
  template<typename StateReader>
  const char * handle_event(const input_event & event, StateReader read_state)
  {
    if (event.type == EV_SYN && event.code == SYN_DROPPED) {
      keys_ = KeyState{};
      syncing_ = true;
      return nullptr;
    }
    if (syncing_) {
      if (event.type == EV_SYN && event.code == SYN_REPORT) {
        KeyState current;
        if (read_state(current)) {
          keys_ = current;
          syncing_ = false;
        }
      }
      return nullptr;
    }
    if (event.type != EV_KEY) {
      return nullptr;
    }

    const bool down = event.value != 0;
    const char * name = nullptr;
    switch (event.code) {
      case KEY_W: keys_.forward = down; name = "w"; break;
      case KEY_S: keys_.back = down; name = "s"; break;
      case KEY_A: keys_.left = down; name = "a"; break;
      case KEY_D: keys_.right = down; name = "d"; break;
      case KEY_LEFT: keys_.turn_left = down; name = "left"; break;
      case KEY_RIGHT: keys_.turn_right = down; name = "right"; break;
      default: return nullptr;
    }
    return event.value == 1 ? name : nullptr;
  }

private:
  KeyState keys_;
  bool syncing_{false};
};

}  // namespace gamepad_node

#endif  // GAMEPAD_NODE__KEYBOARD_INPUT_HPP_
