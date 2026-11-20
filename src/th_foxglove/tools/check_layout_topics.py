"""Static check: every topic a layout uses is (1) allowed by foxglove_bridge's whitelist in
launch/foxglove.launch.py and (2) published by something in the walker stack.

  python3 src/th_foxglove/tools/check_layout_topics.py src/th_foxglove/layouts/walker_demo.json
Exit 1 if a topic is not whitelisted or has no known publisher (optional ones are listed).
"""
import ast
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
JOINTS = [f'{s}_{j}' for s in ('left', 'right')
          for j in ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee_pitch', 'ankle_pitch')]

# Who publishes what (robot.launch.py + `ros.sh dashboard`).
PUBLISHERS = {
    'robot_state_publisher': ['/tf', '/tf_static', '/robot_description'],
    'policy_bridge': ['/joint_states', '/policy_bridge/mode', '/policy_bridge/projected_gravity',
                      '/policy_bridge/stand_lean', '/policy/observation'],
    'policy_node': ['/policy/joint_targets'],
    'robstride_motor_node': [f'/{j}/motor/{t}' for j in JOINTS for t in (
        'actual_position', 'actual_velocity', 'actual_torque', 'temperature', 'bus_voltage',
        'status', 'target_position')],
    'robot_monitor': ['/battery', '/diagnostics', '/robot/battery_voltage', '/robot/battery_percent',
                      '/robot/mechanical_power', '/robot/max_motor_temperature', '/robot/tilt_deg',
                      '/robot/lean_forward_deg', '/robot/lean_left_deg', '/robot/current_left',
                      '/robot/current_right', '/robot/current_total', '/robot/cpu_percent',
                      '/robot/health', '/dashboard/command', '/dashboard/lean',
                      '/dashboard/telemetry'],
    'gamepad_node': ['/gamepad/button_event', '/cmd_vel'],
    'every node': ['/rosout'],
}
# Shown only when that part runs: the camera (camera_url) and th_autonomy's follower.
OPTIONAL = {'/camera/image/compressed': 'camera_url', '/follower/annotations': 'follower',
            '/follower/markers': 'follower', '/follower/state': 'follower',
            '/follower/target': 'follower'}


def whitelist():
    src = open(os.path.join(HERE, '..', 'launch', 'foxglove.launch.py'), encoding='utf-8').read()
    for node in ast.parse(src).body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == 'TOPIC_WHITELIST':
            return [re.compile(p) for p in ast.literal_eval(node.value)]
    raise SystemExit('TOPIC_WHITELIST not found in foxglove.launch.py')


def layout_topics(layout):
    """Topics named by message paths, topic lists, images and log/diagnostic panels."""
    topics = set()

    def visit(v):
        if isinstance(v, dict):
            for k, x in v.items():
                if k in ('value', 'path', 'topicPath') and isinstance(x, str) and x.startswith('/'):
                    m = re.match(r'(/[A-Za-z0-9_/]+?)(\.|\[|$)', x)
                    topics.add(m.group(1))
                elif k in ('topicToRender', 'imageTopic') and isinstance(x, str) and x:
                    topics.add(x)
                elif k in ('topics', 'annotations') and isinstance(x, dict):
                    topics.update(t for t in x if t.startswith('/'))
                visit(x)
        elif isinstance(v, list):
            for x in v:
                visit(x)

    visit(layout['configById'])
    return topics


def main():
    layout = json.load(open(sys.argv[1], encoding='utf-8'))
    allowed = whitelist()
    published = {t: who for who, ts in PUBLISHERS.items() for t in ts}
    bad = 0
    for topic in sorted(layout_topics(layout)):
        ok_wl = any(p.search(topic) for p in allowed)
        who = published.get(topic) or (f'optional ({OPTIONAL[topic]})' if topic in OPTIONAL else None)
        flag = 'ok' if ok_wl and who else 'PROBLEM'
        if flag != 'ok':
            bad += 1
        print(f'{flag:8s} {topic:48s} whitelisted={ok_wl!s:5s} publisher={who}')
    print(f'{bad} problem(s)')
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
