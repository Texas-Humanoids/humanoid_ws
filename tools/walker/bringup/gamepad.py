"""Minimal Linux joystick reader (/dev/input/js*), no dependencies, never blocks.

Default mapping is the Xbox controller under the xpad driver (what the Jetson sees). Check yours
with `python3 tools/walker/bringup/gamepad.py` (prints every axis/button as you press it) and
pass --map to jog.py if the numbers differ.
"""
from __future__ import annotations

import glob
import json
import os
import struct
import sys
import time

JS_EVENT_BUTTON, JS_EVENT_AXIS, JS_EVENT_INIT = 0x01, 0x02, 0x80

XBOX = {
    "axes": {"lx": 0, "ly": 1, "lt": 2, "rx": 3, "ry": 4, "rt": 5, "dpad_x": 6, "dpad_y": 7},
    "buttons": {"a": 0, "b": 1, "x": 2, "y": 3, "lb": 4, "rb": 5, "back": 6, "start": 7, "guide": 8},
}


class Gamepad:
    def __init__(self, path: str | None = None, mapping: dict | None = None):
        self.map = mapping or XBOX
        self.path = path or next(iter(sorted(glob.glob("/dev/input/js*"))), None)
        self.fd = None
        self.axes: dict[int, float] = {}
        self.buttons: dict[int, int] = {}
        self.pressed: list[str] = []     # button names pressed since the last poll()
        self.last_event = 0.0
        if self.path and hasattr(os, "O_NONBLOCK"):   # Linux only (no joystick API on Windows)
            try:
                self.fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK)
            except OSError:
                self.fd = None

    @property
    def ok(self) -> bool:
        return self.fd is not None

    def poll(self) -> list[str]:
        """Read every queued event. Returns the names of buttons pressed since the last call."""
        self.pressed = []
        if self.fd is None:
            return []
        names = {v: k for k, v in self.map["buttons"].items()}
        while True:
            try:
                ev = os.read(self.fd, 8)
            except BlockingIOError:
                break
            except OSError:          # unplugged
                os.close(self.fd)
                self.fd = None
                self.axes.clear()
                self.buttons.clear()
                break
            if len(ev) < 8:
                break
            _, value, etype, number = struct.unpack("IhBB", ev)
            init = etype & JS_EVENT_INIT
            etype &= ~JS_EVENT_INIT
            self.last_event = time.monotonic()
            if etype == JS_EVENT_AXIS:
                self.axes[number] = value / 32767.0
            elif etype == JS_EVENT_BUTTON:
                was = self.buttons.get(number, 0)
                self.buttons[number] = value
                if value and not was and not init and number in names:
                    self.pressed.append(names[number])
        return self.pressed

    def axis(self, name: str, deadzone: float = 0.12) -> float:
        v = self.axes.get(self.map["axes"][name], 0.0)
        if abs(v) < deadzone:
            return 0.0
        return (abs(v) - deadzone) / (1 - deadzone) * (1 if v > 0 else -1)

    def held(self, name: str) -> bool:
        return bool(self.buttons.get(self.map["buttons"][name], 0))


def load_map(path: str | None) -> dict:
    if not path:
        return XBOX
    with open(path) as f:
        m = json.load(f)
    return {"axes": {**XBOX["axes"], **m.get("axes", {})}, "buttons": {**XBOX["buttons"], **m.get("buttons", {})}}


if __name__ == "__main__":
    g = Gamepad(sys.argv[1] if len(sys.argv) > 1 else None)
    if not g.ok:
        sys.exit(f"no joystick device ({g.path or 'no /dev/input/js*'}). Is the controller paired / plugged in?")
    print(f"reading {g.path}; press buttons and move sticks (Ctrl-C to stop)")
    last = None
    while True:
        g.poll()
        state = (tuple(sorted((k, round(v, 2)) for k, v in g.axes.items() if abs(v) > 0.05)),
                 tuple(sorted(k for k, v in g.buttons.items() if v)))
        if state != last:
            print(f"axes {dict(state[0])}  buttons held {list(state[1])}")
            last = state
        time.sleep(0.02)
