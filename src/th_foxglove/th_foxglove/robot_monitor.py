"""Collects robot health into a few dashboard topics and places base_link in the world.

Inputs (per motor namespace <ns>, as robstride_motor_node or th_sim publish them):
  <ns>/motor/actual_velocity, actual_torque, temperature, bus_voltage, status
  /policy_bridge/mode, /imu/data, /mujoco_sim/base_pose (sim only)

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
  /robot/summary                 diagnostic_msgs/DiagnosticStatus  the readings above as
                                 labeled text (key/value rows), for a table on the dashboard
  The /robot/* numbers are rounded so the dashboard shows them as they are read.
  /diagnostics                   diagnostic_msgs/DiagnosticArray  battery, bridge, IMU, motors
  /tf                            world -> base_link: the sim's ground-truth pose when running
                                 in MuJoCo, otherwise the IMU orientation at base_height
                                 (no position estimate on hardware)
"""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Float64, String
from tf2_ros import TransformBroadcaster

from th_foxglove import telemetry

# DiagnosticStatus levels as ints (rclpy's DiagnosticStatus.OK etc. are one-byte bytes objects).
OK, WARN, ERROR, STALE = 0, 1, 2, 3


class Reading:
    """Last value of a topic and when it arrived."""

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

        self.motors = [Motor(j, ns) for j, ns in zip(joints, namespaces)]
        for motor in self.motors:
            for name, reading in (('actual_velocity', motor.velocity),
                                  ('actual_torque', motor.torque),
                                  ('temperature', motor.temperature),
                                  ('bus_voltage', motor.bus_voltage)):
                self.create_subscription(
                    Float64, f'{motor.namespace}/motor/{name}',
                    lambda msg, r=reading: r.set(msg.data), 10)
            self.create_subscription(
                DiagnosticStatus, f'{motor.namespace}/motor/status', motor.status.set, 10)

        self.mode = Reading()
        self.imu = Reading()
        self.sim_pose = Reading()
        self.create_subscription(
            String, p('mode_topic', '/policy_bridge/mode').value,
            lambda msg: self.mode.set(msg.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            Imu, p('imu_topic', '/imu/data').value, self.imu.set, qos_profile_sensor_data)
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
        self.summary_pub = self.create_publisher(DiagnosticStatus, '/robot/summary', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.tf = TransformBroadcaster(self)
        self.imu_count_at = (0, time.monotonic())
        self.imu_rate = 0.0

        self.create_timer(1.0 / float(p('pose_rate', 50.0).value), self.publish_pose)
        self.create_timer(0.1, self.publish_summary)
        self.create_timer(1.0, self.publish_diagnostics)
        self.get_logger().info(
            f'Monitoring {len(self.motors)} motors; battery: '
            + (f'{self.cells} cells' if self.cells > 0 else
               'cell count unknown (set battery.cells in the robot yaml for a percentage)'))

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
        position, q, _ = self.base_pose()
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = position
        r = t.transform.rotation
        r.w, r.x, r.y, r.z = q
        self.tf.sendTransform(t)

    # -- 10 Hz numbers ----------------------------------------------------------------------

    def voltage(self):
        return telemetry.pack_voltage([m.bus_voltage.get(3.0) for m in self.motors])

    def publish_summary(self):
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
        self.battery_pub.publish(battery)
        if math.isfinite(volts):
            self.voltage_pub.publish(Float64(data=round(volts, 1)))
        if math.isfinite(battery.percentage):
            self.percent_pub.publish(Float64(data=float(round(battery.percentage * 100))))

        torques = [m.torque.get(0.5) for m in self.motors]
        velocities = [m.velocity.get(0.5) for m in self.motors]
        if any(t is not None for t in torques):
            self.power_pub.publish(
                Float64(data=float(round(telemetry.mechanical_power(torques, velocities)))))
        temperatures = [m.temperature.get(3.0) for m in self.motors]
        temperatures = [t for t in temperatures if t is not None]
        if temperatures:
            self.temperature_pub.publish(Float64(data=round(float(max(temperatures)), 1)))
        _, q, source = self.base_pose()
        tilt = forward = left = None
        if source != 'none':
            tilt = math.degrees(telemetry.tilt(q))
            forward, left = telemetry.lean(q)
            self.tilt_pub.publish(Float64(data=round(tilt, 1)))
            self.lean_forward_pub.publish(Float64(data=round(forward, 1)))
            self.lean_left_pub.publish(Float64(data=round(left, 1)))

        dash = '-'
        if math.isfinite(volts):
            battery_text = f'{volts:.1f} V'
            if math.isfinite(battery.percentage):
                battery_text += f'  ({battery.percentage * 100:.0f}%)'
        else:
            battery_text = dash
        power = (telemetry.mechanical_power(torques, velocities)
                 if any(t is not None for t in torques) else None)
        rows = {
            'Battery': battery_text,
            'Motor power': f'{power:.0f} W' if power is not None else dash,
            'Tilt': f'{tilt:.1f} deg' if tilt is not None else dash,
            'Lean fwd / left': (f'{forward:+.1f} / {left:+.1f} deg'
                                if forward is not None else dash),
            'Hottest motor': f'{max(temperatures):.0f} C' if temperatures else dash,
        }
        self.summary_pub.publish(status('robot: summary', OK, '', rows))

    # -- 1 Hz diagnostics -------------------------------------------------------------------

    def publish_diagnostics(self):
        now = time.monotonic()
        count, since = self.imu_count_at
        self.imu_rate = (self.imu.count - count) / max(now - since, 1e-3)
        self.imu_count_at = (self.imu.count, now)

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [self.battery_status(), self.bridge_status(), self.imu_status()]
        array.status += [self.motor_status(m) for m in self.motors]
        self.diagnostics_pub.publish(array)

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
        values = {'rate_hz': f'{self.imu_rate:.0f}'}
        if not self.imu.fresh(0.5):
            return status('robot: imu', STALE, 'no data', values)
        _, q, source = self.base_pose()
        values['tilt_deg'] = f'{math.degrees(telemetry.tilt(q)):.1f}'
        values['pose_source'] = source
        return status('robot: imu', OK, f'{self.imu_rate:.0f} Hz', values)

    def motor_status(self, motor):
        name = f'motor: {motor.joint.removesuffix("_joint")}'
        reported = motor.status.get(1.0)
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
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
