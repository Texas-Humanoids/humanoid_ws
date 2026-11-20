"""Collects robot health into a few dashboard topics and places base_link in the world.

Inputs (per motor namespace <ns>, as robstride_motor_node or th_sim publish them):
  <ns>/motor/actual_velocity, actual_torque, temperature, bus_voltage, status
  /policy_bridge/mode, /imu/data, /cmd_vel, /mujoco_sim/base_pose (sim only)

Load: the motor nodes publish each of those at their feedback rate (100 Hz x 10 motors x 5
topics, plus the IMU at 200 Hz: ~5000 messages/s). Taking every one in Python cost ~75% of a
Jetson core (2026-11-19). So by default (sample_inputs: true) the high-rate inputs are
*sampled*: their subscriptions live on a helper node that is never spun, with a depth-1 queue,
so DDS keeps only the newest message and this node takes it when it needs it:
  IMU at pose_rate, velocity/torque/command at 10 Hz, temperature/voltage/status at 1 Hz.
If taking fails (an rclpy API change), it logs that once and falls back to callbacks.

Outputs:
  /battery                       sensor_msgs/BatteryState  voltage (median of the motors'
                                 supply readings) and percentage (needs battery.cells)
  /robot/battery_voltage         std_msgs/Float64  V, 0.1 V steps (only with a reading)
  /robot/battery_percent         std_msgs/Float64  0-100, whole percent (needs battery.cells)
  /robot/mechanical_power        std_msgs/Float64  sum of |torque * velocity| over joints, W
  /robot/max_motor_temperature   std_msgs/Float64  hottest motor, C
  /robot/tilt_deg                std_msgs/Float64  base tilt from upright, degrees
  /robot/lean_forward_deg,       std_msgs/Float64  lean toward +x (nose down) and +y (left
  /robot/lean_left_deg                             side down), degrees
  /robot/current_left, _right,   std_msgs/Float64  estimated battery current (A) of each leg's
  /robot/current_total           motors and in total (see telemetry.battery_current)
  /robot/cpu_percent             std_msgs/Float64  whole-computer CPU use (%), 1 Hz
  The /robot/* numbers are rounded so the dashboard shows them as they are read.
  /robot/health                  std_msgs/String  worst of the motors, IMU and battery: OK,
                                 WARNING, FAULT or NO DATA
  /diagnostics                   diagnostic_msgs/DiagnosticArray  battery, bridge, IMU, motors
  /tf                            world -> base_link: the sim's ground-truth pose when running
                                 in MuJoCo, otherwise the IMU orientation at base_height
                                 (no position estimate on hardware); world -> base_footprint
                                 (base_link on the floor, heading only); world -> base_smooth
                                 (the same, low-passed, for a steady camera); world ->
                                 dashboard (fixed, for the top-down views)
  /dashboard/command, /dashboard/lean   visualization_msgs/MarkerArray (see dashboard.py)
"""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Float64, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import MarkerArray

from th_foxglove import dashboard, telemetry

# DiagnosticStatus levels as ints (rclpy's DiagnosticStatus.OK etc. are one-byte bytes objects).
OK, WARN, ERROR, STALE = 0, 1, 2, 3

# Only the newest message is kept for sampled inputs; best effort matches any publisher.
LATEST = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                    reliability=ReliabilityPolicy.BEST_EFFORT)


class Reading:
    """Last value of a topic and when it arrived (or was sampled)."""

    def __init__(self):
        self.value = None
        self.stamp = 0.0
        self.count = 0

    def set(self, value):
        self.value = value
        self.stamp = time.monotonic()
        self.count += 1

    def fresh(self, timeout):
        return self.value is not None and time.monotonic() - self.stamp <= timeout

    def get(self, timeout, default=None):
        return self.value if self.fresh(timeout) else default


class Motor:

    def __init__(self, joint, namespace):
        self.joint = joint
        self.namespace = namespace
        self.velocity = Reading()
        self.torque = Reading()
        self.temperature = Reading()
        self.bus_voltage = Reading()
        self.status = Reading()


def take_latest(subscription, limit=64):
    """The newest message waiting on a subscription that is not spun, or None.

    Same calls as rclpy's executor (Executor._take_subscription)."""
    latest = None
    with subscription.handle:
        for _ in range(limit):
            taken = subscription.handle.take_message(subscription.msg_type, subscription.raw)
            if taken is None:
                break
            latest = taken[0]
    return latest


class RobotMonitor(Node):

    def __init__(self):
        super().__init__('robot_monitor')
        p = self.declare_parameter
        joints = list(p('joints', Parameter.Type.STRING_ARRAY).value or [])
        namespaces = list(p('motor_namespaces', Parameter.Type.STRING_ARRAY).value or [])
        if len(joints) != len(namespaces):
            raise ValueError('joints and motor_namespaces need the same length')
        self.cells = int(p('battery_cells', 0).value)
        self.low_cell_voltage = float(p('battery_low_cell_voltage', 3.3).value)
        self.hot_temperature = float(p('hot_temperature', 70.0).value)
        rotation = list(p('imu_rotation', [1.0, 0.0, 0.0, 0.0]).value)
        self.imu_to_base = telemetry.quat_normalize(rotation)
        self.imu_ned = p('imu_world_frame', 'enu').value == 'ned'
        self.base_height = float(p('base_height', 0.66).value)
        self.world_frame = p('world_frame', 'world').value
        self.base_frame = p('base_frame', 'base_link').value
        self.footprint_frame = p('footprint_frame', 'base_footprint').value
        self.max_tilt_deg = math.degrees(float(p('max_tilt', 1.0).value))
        self.efficiency = float(p('drive_efficiency', 0.85).value)
        self.base_current = float(p('battery_base_current', 0.0).value)
        # Voltage for the current estimate when the motors report none (sim): nominal 3.6 V/cell.
        self.nominal_voltage = 3.6 * self.cells if self.cells > 0 else 36.0
        pose_rate = float(p('pose_rate', 20.0).value)

        # High-rate inputs: sampled (see the module docstring) or, with sample_inputs false,
        # one callback per message as before.
        self.sampled = []  # (subscription, setter, group, msg_type, topic, callback_qos)
        self.sampler = (rclpy.create_node('robot_monitor_sampler')
                        if bool(p('sample_inputs', True).value) else None)

        self.motors = [Motor(j, ns) for j, ns in zip(joints, namespaces)]
        for motor in self.motors:
            for name, reading, group in (('actual_velocity', motor.velocity, 'fast'),
                                         ('actual_torque', motor.torque, 'fast'),
                                         ('temperature', motor.temperature, 'slow'),
                                         ('bus_voltage', motor.bus_voltage, 'slow')):
                self.watch(Float64, f'{motor.namespace}/motor/{name}',
                           lambda msg, r=reading: r.set(msg.data), group, 10)
            self.watch(DiagnosticStatus, f'{motor.namespace}/motor/status', motor.status.set,
                       'slow', 10)

        self.mode = Reading()
        self.imu = Reading()
        self.sim_pose = Reading()
        self.command = Reading()
        self.watch(Twist, p('cmd_vel_topic', '/cmd_vel').value,
                   lambda msg: self.command.set((msg.linear.x, msg.linear.y, msg.angular.z)),
                   'views', 10)
        self.velocity = dashboard.VelocityEstimate(float(p('velocity_window', 1.4).value))
        self.smooth = dashboard.SmoothedPose(float(p('camera_smoothing', 0.8).value))
        self.measured_velocity = None
        self.lean_trail = []
        self.create_subscription(
            String, p('mode_topic', '/policy_bridge/mode').value,
            lambda msg: self.mode.set(msg.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.watch(Imu, p('imu_topic', '/imu/data').value, self.imu.set, 'imu',
                   qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, p('base_pose_topic', '/mujoco_sim/base_pose').value,
            self.sim_pose.set, 10)

        self.battery_pub = self.create_publisher(BatteryState, '/battery', 10)
        self.power_pub = self.create_publisher(Float64, '/robot/mechanical_power', 10)
        self.temperature_pub = self.create_publisher(Float64, '/robot/max_motor_temperature', 10)
        self.tilt_pub = self.create_publisher(Float64, '/robot/tilt_deg', 10)
        self.voltage_pub = self.create_publisher(Float64, '/robot/battery_voltage', 10)
        self.percent_pub = self.create_publisher(Float64, '/robot/battery_percent', 10)
        self.lean_forward_pub = self.create_publisher(Float64, '/robot/lean_forward_deg', 10)
        self.lean_left_pub = self.create_publisher(Float64, '/robot/lean_left_deg', 10)
        self.current_pubs = {
            side: self.create_publisher(Float64, f'/robot/current_{side}', 10)
            for side in ('left', 'right', 'total')}
        self.cpu_pub = self.create_publisher(Float64, '/robot/cpu_percent', 10)
        self.cpu_previous = None
        self.health_pub = self.create_publisher(String, '/robot/health', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.tf = TransformBroadcaster(self)
        # A fixed frame for the top-down views, well below the floor so it stays out of the
        # robot view. They follow it, so nothing in them moves with the robot's steps.
        self.static_tf = StaticTransformBroadcaster(self)
        d = TransformStamped()
        d.header.stamp = self.get_clock().now().to_msg()
        d.header.frame_id = self.world_frame
        d.child_frame_id = 'dashboard'
        d.transform.translation.z = -20.0
        d.transform.rotation.w = 1.0
        self.static_tf.sendTransform(d)
        self.command_markers_pub = self.create_publisher(MarkerArray, '/dashboard/command', 10)
        self.lean_markers_pub = self.create_publisher(MarkerArray, '/dashboard/lean', 10)
        self.telemetry_pub = self.create_publisher(MarkerArray, '/dashboard/telemetry', 10)
        self.create_timer(0.1, self.publish_views)
        self.imu_count_at = (0, time.monotonic())
        self.imu_rate = 0.0

        self.create_timer(1.0 / pose_rate, self.publish_pose)
        self.create_timer(0.1, self.publish_summary)
        self.create_timer(1.0, self.publish_diagnostics)
        self.get_logger().info(
            f'Monitoring {len(self.motors)} motors '
            f'({"sampled inputs" if self.sampler is not None else "every message"}); battery: '
            + (f'{self.cells} cells' if self.cells > 0 else
               'cell count unknown (set battery.cells in the robot yaml for a percentage)'))

    # -- inputs -----------------------------------------------------------------------------

    def watch(self, msg_type, topic, setter, group, callback_qos):
        if self.sampler is not None:
            sub = self.sampler.create_subscription(msg_type, topic, lambda _msg: None, LATEST)
            self.sampled.append((sub, setter, group, msg_type, topic, callback_qos))
        else:
            self.create_subscription(msg_type, topic, setter, callback_qos)

    def poll(self, group):
        """Take the newest message of every sampled input in this group."""
        if self.sampler is None:
            return
        try:
            for sub, setter, g, *_ in self.sampled:
                if g == group:
                    msg = take_latest(sub)
                    if msg is not None:
                        setter(msg)
        except Exception as error:  # noqa: BLE001  rclpy internals changed: use callbacks
            self.get_logger().error(
                f'Sampling inputs failed ({error!r}); falling back to a callback per message')
            for sub, setter, _group, msg_type, topic, callback_qos in self.sampled:
                self.sampler.destroy_subscription(sub)
                self.create_subscription(msg_type, topic, setter, callback_qos)
            self.sampled = []
            self.sampler.destroy_node()
            self.sampler = None

    def sampled_imu(self):
        return self.sampler is not None

    # -- pose -------------------------------------------------------------------------------

    def base_pose(self):
        """(position, orientation [w, x, y, z], source) of base_link in the world."""
        pose = self.sim_pose.get(0.5)
        if pose is not None:
            o = pose.pose.orientation
            p = pose.pose.position
            return (p.x, p.y, p.z), (o.w, o.x, o.y, o.z), 'sim'
        imu = self.imu.get(0.5)
        if imu is not None:
            o = imu.orientation
            try:
                q = telemetry.base_orientation(
                    (o.w, o.x, o.y, o.z), self.imu_to_base, self.imu_ned)
                return (0.0, 0.0, self.base_height), q, 'imu'
            except ValueError:
                pass
        return (0.0, 0.0, self.base_height), (1.0, 0.0, 0.0, 0.0), 'none'

    def publish_pose(self):
        self.poll('imu')
        position, q, source = self.base_pose()
        stamp = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = position
        r = t.transform.rotation
        r.w, r.x, r.y, r.z = q
        yaw = dashboard.yaw_of(q)
        f = TransformStamped()
        f.header.stamp = stamp
        f.header.frame_id = self.world_frame
        f.child_frame_id = self.footprint_frame
        f.transform.translation.x, f.transform.translation.y = position[0], position[1]
        f.transform.rotation.w, f.transform.rotation.z = math.cos(yaw / 2), math.sin(yaw / 2)
        sx, sy, syaw = self.smooth.update(time.monotonic(), position[0], position[1], yaw)
        c = TransformStamped()
        c.header.stamp = stamp
        c.header.frame_id = self.world_frame
        c.child_frame_id = 'base_smooth'
        c.transform.translation.x, c.transform.translation.y = sx, sy
        c.transform.rotation.w, c.transform.rotation.z = math.cos(syaw / 2), math.sin(syaw / 2)
        self.tf.sendTransform([t, f, c])
        if source == 'sim':
            self.measured_velocity = self.velocity.update(
                time.monotonic(), position[0], position[1], yaw)
        else:
            self.velocity.reset()
            self.measured_velocity = None

    def publish_views(self):
        """Top-down command and lean views, 10 Hz."""
        self.poll('views')
        stamp = self.get_clock().now().to_msg()
        command = self.command.get(0.5, (0.0, 0.0, 0.0))  # the bridge treats stale as stop
        self.command_markers_pub.publish(dashboard.command_markers(
            'dashboard', stamp, command, self.measured_velocity))
        _, q, source = self.base_pose()
        lean = telemetry.lean(q) if source != 'none' else None
        if lean is not None:
            self.lean_trail = (self.lean_trail + [lean])[-20:]  # last 2 s
        else:
            self.lean_trail = []
        self.lean_markers_pub.publish(dashboard.lean_markers(
            'dashboard', stamp, lean, self.lean_trail, self.max_tilt_deg))

    # -- 10 Hz numbers ----------------------------------------------------------------------

    def voltage(self):
        return telemetry.pack_voltage([m.bus_voltage.get(3.0) for m in self.motors])

    def publish_summary(self):
        self.poll('fast')
        volts = self.voltage()
        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.voltage = float(volts)
        battery.percentage = float(telemetry.battery_fraction(volts, self.cells))
        battery.temperature = battery.current = battery.charge = math.nan
        battery.capacity = battery.design_capacity = math.nan
        battery.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery.present = math.isfinite(volts)
        self.voltage_pub.publish(Float64(data=round(volts, 1)))
        percent = round(battery.percentage * 100, 0)
        self.percent_pub.publish(Float64(data=percent))

        torques = [m.torque.get(0.5) for m in self.motors]
        velocities = [m.velocity.get(0.5) for m in self.motors]
        bus = volts if math.isfinite(volts) else self.nominal_voltage
        amps = {}
        for side in ('left', 'right'):
            pick = [i for i, m in enumerate(self.motors) if m.joint.startswith(side + '_')]
            amps[side] = telemetry.battery_current(
                [torques[i] for i in pick], [velocities[i] for i in pick], bus,
                self.efficiency)
        amps['total'] = amps['left'] + amps['right'] + self.base_current
        for side, value in amps.items():
            self.current_pubs[side].publish(Float64(data=round(value, 1)))
        power = telemetry.battery_current(torques, velocities, 1.0, 1.0)
        self.power_pub.publish(Float64(data=round(power, 0)))
        # Keep BatteryState.current unknown: this topic is a measurement, not our estimate.
        self.telemetry_pub.publish(dashboard.telemetry_markers(
            'dashboard', battery.header.stamp, volts, amps, nominal=not math.isfinite(volts)))
        self.battery_pub.publish(battery)
        temperatures = [m.temperature.get(3.0) for m in self.motors]
        temperatures = [t for t in temperatures if t is not None and math.isfinite(t)]
        hottest = max(temperatures, default=math.nan)
        self.temperature_pub.publish(Float64(data=round(float(hottest), 1)))
        _, q, source = self.base_pose()
        if source != 'none':
            forward, left = telemetry.lean(q)
            self.tilt_pub.publish(Float64(data=round(math.degrees(telemetry.tilt(q)), 1)))
            self.lean_forward_pub.publish(Float64(data=round(forward, 1)))
            self.lean_left_pub.publish(Float64(data=round(left, 1)))

    # -- 1 Hz diagnostics -------------------------------------------------------------------

    def publish_cpu(self):
        try:
            with open('/proc/stat') as f:
                current = telemetry.cpu_times(f.read())
        except (OSError, ValueError):
            return
        if self.cpu_previous is not None:
            percent = telemetry.cpu_percent(self.cpu_previous, current)
            self.cpu_pub.publish(Float64(data=round(percent, 1)))
        self.cpu_previous = current

    def publish_diagnostics(self):
        self.poll('slow')
        self.publish_cpu()
        now = time.monotonic()
        count, since = self.imu_count_at
        self.imu_rate = (self.imu.count - count) / max(now - since, 1e-3)
        self.imu_count_at = (self.imu.count, now)

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [self.battery_status(), self.bridge_status(), self.imu_status()]
        array.status += [self.motor_status(m) for m in self.motors]
        self.diagnostics_pub.publish(array)
        self.health_pub.publish(String(data=health_summary(array.status)))

    def battery_status(self):
        volts = self.voltage()
        fraction = telemetry.battery_fraction(volts, self.cells)
        values = {'voltage_v': f'{volts:.2f}', 'cells': str(self.cells or 'unknown')}
        if not math.isfinite(volts):
            level, message = STALE, 'no voltage reading'
        elif self.cells > 0 and volts < self.low_cell_voltage * self.cells:
            level, message = ERROR, f'LOW {volts:.1f} V ({fraction:.0%})'
        elif self.cells > 0:
            level, message = OK, f'{volts:.1f} V ({fraction:.0%})'
        else:
            level, message = OK, f'{volts:.1f} V'
        if math.isfinite(fraction):
            values['percentage'] = f'{fraction * 100:.0f}'
        return status('robot: battery', level, message, values)

    def bridge_status(self):
        mode = self.mode.value
        if mode is None:
            return status('robot: policy_bridge', STALE, 'no mode')
        return status('robot: policy_bridge', OK if mode != 'DISARMED' else WARN, mode,
                      {'mode': mode})

    def imu_status(self):
        # Sampled, the count is samples taken (at pose_rate), not the IMU's own rate.
        sampled = self.sampled_imu()
        values = {'rate_hz': 'sampled' if sampled else f'{self.imu_rate:.0f}'}
        if not self.imu.fresh(0.5):
            return status('robot: imu', STALE, 'no data', values)
        _, q, source = self.base_pose()
        values['tilt_deg'] = f'{math.degrees(telemetry.tilt(q)):.1f}'
        values['pose_source'] = source
        return status('robot: imu', OK, 'OK' if sampled else f'{self.imu_rate:.0f} Hz', values)

    def motor_status(self, motor):
        name = f'motor: {motor.joint.removesuffix("_joint")}'
        reported = motor.status.get(1.5)
        if reported is not None:
            values = {kv.key: kv.value for kv in reported.values}
            level, message = level_of(reported), reported.message
            temperature = motor.temperature.get(3.0)
            if level == OK and temperature is not None and temperature >= self.hot_temperature:
                level, message = WARN, f'hot: {temperature:.0f} C'
            return status(name, level, message, values, reported.hardware_id)
        if motor.velocity.fresh(0.5):  # th_sim, or a motor whose status frames stopped
            return status(name, OK, 'OK (no status frames)')
        return status(name, STALE, 'no feedback')


def health_summary(statuses):
    """One word for the dashboard light: the worst motor/IMU/battery level.

    The policy bridge's row (DISARMED is a WARN) and a battery with no reading (the sim) are
    left out: the state banner and the battery gauge already show those. Anything that
    stopped reporting makes it at least WARNING.
    """
    levels = [level_of(s) for s in statuses
              if s.name != 'robot: policy_bridge' and
              not (s.name == 'robot: battery' and level_of(s) == STALE)]
    if not levels or all(level == STALE for level in levels):
        return 'NO DATA'
    worst = max(level for level in levels if level != STALE)
    if STALE in levels:
        worst = max(worst, WARN)
    return {OK: 'OK', WARN: 'WARNING', ERROR: 'FAULT'}[worst]


def level_of(msg):
    """DiagnosticStatus.level as an int (rclpy gives a one-byte bytes object)."""
    level = msg.level
    return level[0] if isinstance(level, (bytes, bytearray)) else int(level)


def status(name, level, message, values=None, hardware_id=''):
    msg = DiagnosticStatus(name=name, level=bytes([level]), message=message,
                           hardware_id=hardware_id)
    msg.values = [KeyValue(key=k, value=str(v)) for k, v in (values or {}).items()]
    return msg


def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.sampler is not None:
            node.sampler.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
