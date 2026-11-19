# These bench scripts are named *_test.py but are not pytest tests: they import rclpy and command
# real motors (hold_test.py and move_test.py do it at import). Keep `pytest tools/walker` from
# collecting them.
collect_ignore = [
    'wiggle_test.py',
    'bench/hold_test.py',
    'bench/move_test.py',
]
