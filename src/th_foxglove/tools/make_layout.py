"""Generate the presentation layout: python3 tools/make_layout.py layouts/walker_v2.json."""

import json, sys
FRAMES = ['world', 'base_link', 'base_footprint', 'base_smooth', 'dashboard', 'imu_link',
          'follow_camera'] + [
    f'{s}_{l}_link' for s in ('left', 'right')
    for l in ('hip_pitch', 'hip_roll', 'thigh', 'shank', 'foot')]
BLUE, BLUE_T, ORANGE, ORANGE_T = '#4e98e2', '#a9cdf2', '#f5774d', '#fbbca6'
GREEN, YELLOW, PINK, CYAN = '#5cd6a9', '#f7df71', '#e05ea8', '#61cbff'
VIEW_THETA = float(sys.argv[3]) if len(sys.argv) > 3 else 90.0
cfg = {}


def series(value, label, color, width=1.5):
    return {'value': value, 'enabled': True, 'timestampMethod': 'receiveTime', 'label': label,
            'color': color, 'lineSize': width}


def plot(pid, title, paths, ymin=None, ymax=None, window=20, legend='floating'):
    c = {'foxglovePanelTitle': title, 'paths': paths, 'showLegend': legend != 'none',
         'legendDisplay': 'top' if legend == 'none' else legend,
         'showPlotValuesInLegend': False, 'showXAxisLabels': True, 'showYAxisLabels': True,
         'isSynced': True, 'xAxisVal': 'timestamp', 'followingViewWidth': window,
         'sidebarDimension': 240}
    if ymin is not None:
        c['minYValue'] = ymin
    if ymax is not None:
        c['maxYValue'] = ymax
    cfg[pid] = c
    return pid


def split(direction, first, second, pct):
    return {'direction': direction, 'first': first, 'second': second, 'splitPercentage': pct}


def indicator(pid, title, path, rules):
    cfg[pid] = {'foxglovePanelTitle': title, 'path': path, 'style': 'background',
                'fallbackColor': '#3a3d44', 'fallbackLabel': 'NO DATA', 'fontSize': 26,
                'rules': [{'operator': '=', 'rawValue': v, 'color': c, 'label': v}
                          for v, c in rules]}
    return pid


PUBLISH = {'type': 'point', 'poseTopic': '/move_base_simple/goal', 'pointTopic': '/clicked_point',
           'poseEstimateTopic': '/initialpose', 'poseEstimateXDeviation': 0.5,
           'poseEstimateYDeviation': 0.5, 'poseEstimateThetaDeviation': 0.26179939}
GREEN_BG, YELLOW_BG, RED_BG = '#2fa84f', '#d39b00', '#b03a2e'

# -- Overview ---------------------------------------------------------------------------------
cfg['3D!robot'] = {
    'foxglovePanelTitle': 'Robot',
    # Smooth the followed position and heading so each step does not shake the camera.
    'followTf': 'base_smooth', 'followMode': 'follow-pose',
    'cameraState': {'perspective': True, 'distance': 2.4, 'phi': 72, 'thetaOffset': 90,
                    'targetOffset': [0, 0, 0.5], 'target': [0, 0, 0],
                    'targetOrientation': [0, 0, 0, 1], 'fovy': 45, 'near': 0.01, 'far': 5000},
    # The CAD STL meshes are z-up; Foxglove's default (y-up) turns every part 90 degrees.
    'scene': {'enableStats': False, 'backgroundColor': '#262a31', 'meshUpAxis': 'z_up',
              'transforms': {'showLabel': False, 'axisSize': 0.08, 'lineWidth': 0}},
    # Hide every TF frame's axes and the parent lines; the URDF still uses the transforms.
    'transforms': {f'frame:{f}': {'visible': False} for f in FRAMES},
    'topics': {'/robot_description': {'visible': True}, '/tf': {'visible': True},
               '/tf_static': {'visible': True}, '/dashboard/command': {'visible': False},
               # th_autonomy's follower: the target it sees and where it will stop.
               '/follower/markers': {'visible': True},
               '/dashboard/lean': {'visible': False},
               '/dashboard/telemetry': {'visible': False}},
    'layers': {'grid': {'layerId': 'foxglove.Grid', 'instanceId': 'grid', 'label': 'Floor',
                        'visible': True, 'frameId': 'world', 'size': 20, 'divisions': 40,
                        'lineWidth': 1, 'color': '#ffffff30', 'position': [0, 0, 0],
                        'rotation': [0, 0, 0], 'order': 1}},
    'publish': PUBLISH, 'synchronize': False, 'imageMode': {},
}


def top_down(pid, title, topic, distance):
    # Follows the fixed "dashboard" frame, so the view never moves.
    cfg[pid] = {
        'foxglovePanelTitle': title, 'followTf': 'dashboard', 'followMode': 'follow-pose',
        'cameraState': {'perspective': False, 'distance': distance, 'phi': 0.01,
                        'thetaOffset': VIEW_THETA, 'targetOffset': [0, 0, 0],
                        'target': [0, 0, 0], 'targetOrientation': [0, 0, 0, 1], 'fovy': 45,
                        'near': 0.01, 'far': 5000},
        'scene': {'enableStats': False, 'backgroundColor': '#1d2026',
                  'transforms': {'showLabel': False, 'axisSize': 0.01, 'lineWidth': 0}},
        'transforms': {f'frame:{f}': {'visible': False} for f in FRAMES},
        'topics': {**{t: {'visible': t == topic} for t in (
            '/dashboard/command', '/dashboard/lean', '/dashboard/telemetry')},
            '/robot_description': {'visible': False}},
        'layers': {}, 'publish': PUBLISH, 'synchronize': False, 'imageMode': {},
    }
    return pid


top_down('3D!command', 'Command · blue (vx, vy, wz) / green (vx, vy)', '/dashboard/command', 1.75)
top_down('3D!lean', 'Lean · tilt / (forward, left) °', '/dashboard/lean', 1.4)
def image(pid, title, topic, annotations=()):
    cfg[pid] = {
        'foxglovePanelTitle': title,
        'cameraState': {'distance': 20, 'perspective': True, 'phi': 60, 'target': [0, 0, 0],
                        'targetOffset': [0, 0, 0], 'targetOrientation': [0, 0, 0, 1],
                        'thetaOffset': 45, 'fovy': 45, 'near': 0.5, 'far': 5000},
        'followMode': 'follow-pose', 'scene': {}, 'transforms': {}, 'topics': {}, 'layers': {},
        'publish': PUBLISH, 'synchronize': False,
        'imageMode': {'imageTopic': topic,
                      'annotations': {a: {'visible': True} for a in annotations}},
    }
    return pid


# A follower (th_autonomy) outlines what it detects over the camera's own stream; the one
# it follows is green, labeled with distance and bearing. Nothing extra when none runs.
image('Image!camera', 'Camera · green = followed', '/camera/image/compressed',
      ['/follower/annotations'])
indicator('Indicator!mode', 'State', '/policy_bridge/mode.data',
          [('WALK', GREEN_BG), ('STAND', YELLOW_BG), ('DISARMED', RED_BG)])
indicator('Indicator!health', 'Health (details on System)', '/robot/health.data',
          [('OK', GREEN_BG), ('WARNING', YELLOW_BG), ('FAULT', RED_BG)])
top_down('3D!telemetry', 'Battery & current estimates', '/dashboard/telemetry', 3.1)

overview = split('row',
    split('column', '3D!robot', split('row', '3D!command', '3D!lean', 50), 60),
    split('column', split('row', 'Indicator!mode', 'Indicator!health', 50),
          split('column',
                '3D!telemetry',
                'Image!camera', 44),
          9),
    60)

# -- Joints: one plot per joint, both legs, measured vs target ------------------------------
names = {'hip_pitch': 'Hip pitch', 'hip_roll': 'Hip roll', 'hip_yaw': 'Hip yaw',
         'knee_pitch': 'Knee', 'ankle_pitch': 'Ankle'}
joint_plots = []
for k, (short, title) in enumerate(names.items()):
    l, r = k, k + 5
    joint_plots.append(plot(f'Plot!{short}', f'{title} (deg)', [
        series(f'/joint_states.position[{l}].@degrees', 'L measured', BLUE, 2),
        series(f'/policy/joint_targets.position[{l}].@degrees', 'L target', BLUE_T, 1),
        series(f'/joint_states.position[{r}].@degrees', 'R measured', ORANGE, 2),
        series(f'/policy/joint_targets.position[{r}].@degrees', 'R target', ORANGE_T, 1)], window=8, legend='floating'))
cfg['StateTransitions!mode'] = {
    'foxglovePanelTitle': 'State history', 'isSynced': True,
    'paths': [{'value': '/policy_bridge/mode.data', 'timestampMethod': 'receiveTime',
               'label': 'state'}]}
joints = split('column',
    split('row', joint_plots[0], joint_plots[3], 50),
    split('column',
          split('row', joint_plots[1], joint_plots[4], 50),
          split('row', joint_plots[2], 'StateTransitions!mode', 50), 50),
    33)

# -- System: power, heat, computer, health, log ---------------------------------------------
plot('Plot!battery', 'Battery voltage (V)', [
    series('/robot/battery_voltage.data', 'battery', YELLOW, 2)], ymin=30, ymax=43, window=300, legend='none')
plot('Plot!current', 'Estimated current (A)', [
    series('/robot/current_total.data', 'total', YELLOW, 2),
    series('/robot/current_left.data', 'left leg', BLUE, 1.5),
    series('/robot/current_right.data', 'right leg', ORANGE, 1.5)], ymin=0, window=60)
plot('Plot!torque', 'Knee and ankle torque (Nm)', [
    series('/left_knee_pitch/motor/actual_torque.data', 'L knee', BLUE, 1.5),
    series('/right_knee_pitch/motor/actual_torque.data', 'R knee', ORANGE, 1.5),
    series('/left_ankle_pitch/motor/actual_torque.data', 'L ankle', CYAN, 1.5),
    series('/right_ankle_pitch/motor/actual_torque.data', 'R ankle', YELLOW, 1.5)], window=20)
plot('Plot!computer', 'Computer CPU (%)', [
    series('/foxglove_bridge/sysinfo.total_cpu_percent', 'CPU', BLUE, 1.5)],
    ymin=0, ymax=100, window=120, legend='none')
plot('Plot!temperature', 'Hottest motor (°C; warning at 70)', [
    series('/robot/max_motor_temperature.data', 'motor', ORANGE, 2)],
    ymin=0, ymax=90, window=120, legend='none')
cfg['DiagnosticSummary!health'] = {
    'foxglovePanelTitle': 'Health', 'topicToRender': '/diagnostics', 'pinnedIds': [],
    'hardwareIdFilter': '', 'sortByLevel': True, 'minLevel': 0, 'secondsUntilStale': 5}
cfg['RosOut!log'] = {
    'foxglovePanelTitle': 'Log (warnings and errors)', 'topicToRender': '/rosout',
    'searchTerms': [], 'minLogLevel': 3, 'nameFilter': {}, 'fontSize': 12,
    'showLevel': True, 'showDate': False, 'showTime': True, 'showTimezone': False,
    'showName': True, 'showSourceLocation': False}
system = split('row',
    split('column', split('column', 'Plot!battery', 'Plot!current', 50),
          split('column', 'Plot!torque',
                split('row', 'Plot!computer', 'Plot!temperature', 50), 50), 50),
    split('column', 'DiagnosticSummary!health', 'RosOut!log', 45), 55)

# -- Follow: a backup view for tuning th_autonomy's tag or person follower ------------------
# The Overview already shows its detections (camera) and target (3D). Here: the same camera,
# its state, and how the target distance and the command evolve.
image('Image!follow', 'Follower view · green = followed', '/camera/image/compressed',
      ['/follower/annotations'])
indicator('Indicator!follow', 'Follower', '/follower/state.data',
          [('TRACKING', GREEN_BG), ('LOST', YELLOW_BG), ('SEARCH', '#2f6fb0'),
           ('OVERRIDE', '#7a4fb0'), ('DISABLED', '#3a3d44')])
plot('Plot!follow_target', 'Target (m): ahead, right of center', [
    series('/follower/target.point.z', 'ahead', GREEN, 2),
    series('/follower/target.point.x', 'right', PINK, 1.5)], window=30)
plot('Plot!follow_cmd', 'Command: forward (m/s), turn left (rad/s)', [
    series('/cmd_vel.linear.x', 'forward', BLUE, 2),
    series('/cmd_vel.angular.z', 'turn', ORANGE, 1.5)], ymin=-0.5, ymax=0.5, window=30)
follow = split('row', 'Image!follow',
               split('column', 'Indicator!follow',
                     split('column', 'Plot!follow_target', 'Plot!follow_cmd', 50), 14),
               68)

cfg['Tab!main'] = {'activeTabIdx': int(sys.argv[2]) if len(sys.argv) > 2 else 0, 'tabs': [
    {'title': 'Overview', 'layout': overview},
    {'title': 'Joints', 'layout': joints},
    {'title': 'System', 'layout': system},
    {'title': 'Follow', 'layout': follow}]}
layout = {'configById': cfg, 'globalVariables': {}, 'userNodes': {},
          'playbackConfig': {'speed': 1}, 'layout': 'Tab!main'}
json.dump(layout, open(sys.argv[1], 'w'), indent=2)
print('panels', len(cfg))
