#include <gtest/gtest.h>

#include "gamepad_node/keyboard_input.hpp"

namespace
{

using gamepad_node::KeyboardInput;
using gamepad_node::KeyState;

input_event event(unsigned short type, unsigned short code, int value = 0)
{
  input_event ev{};
  ev.type = type;
  ev.code = code;
  ev.value = value;
  return ev;
}

bool released_snapshot(KeyState & state)
{
  state = KeyState{};
  return true;
}

TEST(KeyboardEvents, PressRepeatAndRelease)
{
  KeyboardInput input;
  EXPECT_STREQ(input.handle_event(event(EV_KEY, KEY_W, 1), released_snapshot), "w");
  EXPECT_TRUE(input.keys().forward);
  EXPECT_EQ(input.handle_event(event(EV_KEY, KEY_W, 2), released_snapshot), nullptr);
  EXPECT_TRUE(input.keys().forward);
  EXPECT_EQ(input.handle_event(event(EV_KEY, KEY_W, 0), released_snapshot), nullptr);
  EXPECT_FALSE(input.keys().forward);
}

TEST(KeyboardEvents, AllMovementKeysAreTrackedIndependently)
{
  KeyboardInput input;
  const unsigned short codes[] = {KEY_W, KEY_S, KEY_A, KEY_D, KEY_LEFT, KEY_RIGHT};
  const char * names[] = {"w", "s", "a", "d", "left", "right"};
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_STREQ(input.handle_event(event(EV_KEY, codes[i], 1), released_snapshot), names[i]);
  }
  const auto & keys = input.keys();
  EXPECT_TRUE(keys.forward && keys.back && keys.left && keys.right);
  EXPECT_TRUE(keys.turn_left && keys.turn_right);
  for (auto code : codes) {
    input.handle_event(event(EV_KEY, code, 0), released_snapshot);
  }
  EXPECT_FALSE(keys.forward || keys.back || keys.left || keys.right);
  EXPECT_FALSE(keys.turn_left || keys.turn_right);
}

TEST(KeyboardEvents, IgnoresUnmappedKeysAndOtherEventTypes)
{
  KeyboardInput input;
  EXPECT_EQ(input.handle_event(event(EV_KEY, KEY_SPACE, 1), released_snapshot), nullptr);
  EXPECT_EQ(input.handle_event(event(EV_MSC, KEY_W, 1), released_snapshot), nullptr);
  EXPECT_FALSE(input.keys().forward);
}

TEST(KeyboardEvents, OverflowStopsImmediatelyAndRecoversDroppedRelease)
{
  KeyboardInput input;
  input.handle_event(event(EV_KEY, KEY_W, 1), released_snapshot);
  input.handle_event(event(EV_SYN, SYN_DROPPED), released_snapshot);
  EXPECT_FALSE(input.keys().forward);
  // Unreliable events must be discarded until the synchronization boundary.
  input.handle_event(event(EV_KEY, KEY_W, 1), released_snapshot);
  EXPECT_FALSE(input.keys().forward);
  EXPECT_EQ(input.handle_event(event(EV_SYN, SYN_REPORT), released_snapshot), nullptr);
  EXPECT_FALSE(input.keys().forward);
  EXPECT_STREQ(input.handle_event(event(EV_KEY, KEY_W, 1), released_snapshot), "w");
}

TEST(KeyboardEvents, RecoveryRestoresTheCompleteSnapshotWithoutPressEvents)
{
  KeyboardInput input;
  input.handle_event(event(EV_SYN, SYN_DROPPED), released_snapshot);
  int reads = 0;
  const auto snapshot = [&reads](KeyState & state) {
      ++reads;
      state.back = true;
      state.left = true;
      state.turn_right = true;
      return true;
    };
  EXPECT_EQ(input.handle_event(event(EV_SYN, SYN_REPORT), snapshot), nullptr);
  EXPECT_EQ(reads, 1);
  EXPECT_FALSE(input.keys().forward);
  EXPECT_TRUE(input.keys().back && input.keys().left && input.keys().turn_right);
  input.handle_event(event(EV_SYN, SYN_REPORT), snapshot);
  EXPECT_EQ(reads, 1);
  input.handle_event(event(EV_KEY, KEY_S, 0), snapshot);
  EXPECT_FALSE(input.keys().back);
}

TEST(KeyboardEvents, FailedSnapshotKeepsMotionStoppedUntilRecoverySucceeds)
{
  KeyboardInput input;
  input.handle_event(event(EV_KEY, KEY_W, 1), released_snapshot);
  input.handle_event(event(EV_SYN, SYN_DROPPED), released_snapshot);
  input.handle_event(event(EV_SYN, SYN_REPORT), [](KeyState & state) {
      state.forward = true;  // Even a partially filled failed snapshot is discarded.
      return false;
    });
  input.handle_event(event(EV_KEY, KEY_W, 1), released_snapshot);
  EXPECT_FALSE(input.keys().forward);
  input.handle_event(event(EV_SYN, SYN_REPORT), released_snapshot);
  EXPECT_STREQ(input.handle_event(event(EV_KEY, KEY_A, 1), released_snapshot), "a");
  EXPECT_TRUE(input.keys().left);
}

TEST(KeyboardEvents, RepeatedOverflowWaitsForTheNextReport)
{
  KeyboardInput input;
  input.handle_event(event(EV_SYN, SYN_DROPPED), released_snapshot);
  input.handle_event(event(EV_SYN, SYN_DROPPED), released_snapshot);
  input.handle_event(event(EV_KEY, KEY_W, 1), released_snapshot);
  EXPECT_FALSE(input.keys().forward);
  input.handle_event(event(EV_SYN, SYN_REPORT), released_snapshot);
  EXPECT_STREQ(input.handle_event(event(EV_KEY, KEY_W, 1), released_snapshot), "w");
}

TEST(KeyboardEvents, DeviceLossClearsAllKeysAndRecoveryState)
{
  KeyboardInput input;
  input.handle_event(event(EV_KEY, KEY_W, 1), released_snapshot);
  input.handle_event(event(EV_KEY, KEY_LEFT, 1), released_snapshot);
  input.reset();
  EXPECT_FALSE(input.keys().forward || input.keys().turn_left);
  input.handle_event(event(EV_SYN, SYN_DROPPED), released_snapshot);
  input.reset();
  EXPECT_STREQ(input.handle_event(event(EV_KEY, KEY_D, 1), released_snapshot), "d");
}

}  // namespace
