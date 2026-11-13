"""MuJoCo stand-in for the robot: RobStride motors (MIT mode) and the IMU.

Speaks the same topics as robstride_motor_node and the IMU driver, so
policy_bridge and policy_node run unchanged against a simulated WalkerV2:

  /<ns>/motor/target_position  (sub, motor space)
  /<ns>/motor/actual_position  (pub, motor space, feedback_rate)
  /<ns>/motor/actual_velocity  (pub, motor space, feedback_rate)
  /imu/data                    (pub, sensor_msgs/Imu, imu_rate, frame per the robot yaml)
  ~/base_pose                  (pub, ground-truth base_link pose in the world, 50 Hz)

Physics runs at the model timestep (1 kHz) paced to wall time; the motor PD is
evaluated every physics step, like the motor's own loop. A "gantry" (a weld from
base_link to a fixed anchor) holds the robot up until policy_bridge reaches the
mode named by `gantry_release`.
"""

import importlib
import socket
import threading
import time

import mujoco
import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

from th_sim.emulation import ImuMount, MitMotors, TORQUE_MAX, KD_MAX, KP_MAX


class MujocoSim(Node):

    def __init__(self):
        super().__init__('mujoco_sim')
        p = self.declare_parameter
        model_path = p('model_path', '').value
        self.joints = list(p('joints', ['']).value)
        namespaces = list(p('motor_namespaces', ['']).value)
        models = list(p('motor_models', ['']).value)
        directions = list(p('directions', [0.0]).value)
        offsets = list(p('offsets', [0.0]).value)
        kp = list(p('kp', [0.0]).value)
        kd = list(p('kd', [0.0]).value)
        torque_limits = list(p('torque_limits', [0.0]).value)  # 0 = model peak
        damping_kd = p('damping_kd', 1.0).value
        target_timeout = p('target_timeout', 0.1).value  # as robstride_motor_node
        command_delay = p('command_delay', 0.0).value  # s, before a target reaches the motor
        initial = list(p('initial_joint_positions', [0.0]).value)
        self.feedback_period = 1.0 / p('feedback_rate', 100.0).value
        self.imu_period = 1.0 / p('imu_rate', 200.0).value
        self.real_time_factor = p('real_time_factor', 1.0).value
        # Bridge mode that drops the gantry: STAND, WALK, never (stays up), or off (no gantry).
        self.gantry_release = p('gantry_release', 'WALK').value
        viewer = p('viewer', False).value
        stream_to = p('stream_to', '').value  # host:port for tools/walker/sim_viewer.py
        self.imu = ImuMount(
            p('imu_rotation', [1.0, 0.0, 0.0, 0.0]).value, p('imu_world_frame', 'enu').value)

        n = len(self.joints)
        if not model_path or n == 0 or any(
                len(v) != n for v in (namespaces, models, directions, offsets, kp, kd,
                                      torque_limits, initial)):
            raise ValueError(
                'model_path, and joints, motor_namespaces, motor_models, directions, offsets, '
                'kp, kd, torque_limits, initial_joint_positions with one entry per joint')
        for joint, model, kp_j, kd_j in zip(self.joints, models, kp, kd):
            if model not in TORQUE_MAX:
                raise ValueError(f'{joint}: unknown motor model {model}')
            # robstride_motor_node refuses the same out-of-range gains.
            if not (0 <= kp_j <= KP_MAX[model] and 0 <= kd_j <= KD_MAX[model]
                    and 0 <= damping_kd <= KD_MAX[model]):
                raise ValueError(f'{joint}: kp/kd/damping_kd out of range for {model}')
        limits = [t if t > 0 else TORQUE_MAX[m] for t, m in zip(torque_limits, models)]
        for joint, limit, model in zip(self.joints, limits, models):
            if limit > TORQUE_MAX[model]:
                raise ValueError(f'{joint}: torque_limit above the {model} peak')
        self.motors = MitMotors(
            directions, offsets, kp, kd, damping_kd, limits, target_timeout, command_delay)

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.qadr, self.vadr, self.act = [], [], []
        for joint in self.joints:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            if jid < 0:
                raise ValueError(f'{joint} is not a joint in {model_path}')
            self.qadr.append(self.model.jnt_qposadr[jid])
            self.vadr.append(self.model.jnt_dofadr[jid])
            act = [a for a in range(self.model.nu) if self.model.actuator_trnid[a, 0] == jid]
            if len(act) != 1:
                raise ValueError(f'{joint} needs exactly one motor actuator in the model')
            self.act.append(act[0])
        self.initial = np.asarray(initial, float)
        self.base = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')
        self.gantry = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, 'gantry')
        self.sensor = {
            name: self.model.sensor_adr[
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)]
            for name in ('imu_quat', 'imu_gyro', 'imu_acc')}
        self.lock = threading.Lock()
        self.reset()

        self.position_pubs, self.velocity_pubs = [], []
        for i, ns in enumerate(namespaces):
            self.position_pubs.append(
                self.create_publisher(Float64, ns + '/motor/actual_position', 10))
            self.velocity_pubs.append(
                self.create_publisher(Float64, ns + '/motor/actual_velocity', 10))
            self.create_subscription(
                Float64, ns + '/motor/target_position',
                lambda msg, i=i: self.on_target(i, msg), 10)
        self.imu_pub = self.create_publisher(
            Imu, p('imu_topic', '/imu/data').value, qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, '~/base_pose', 10)
        self.create_subscription(
            String, p('mode_topic', '/policy_bridge/mode').value, self.on_mode,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.create_service(Trigger, '~/reset', self.on_reset)
        self.create_service(Trigger, '~/release', self.on_release)

        self.viewer = None
        if viewer:
            viewer_module = importlib.import_module('mujoco.viewer')  # needs a display
            self.viewer = viewer_module.launch_passive(self.model, self.data)
        self.stream = None
        if stream_to:
            host, port = stream_to.rsplit(':', 1)
            self.stream = (socket.socket(socket.AF_INET, socket.SOCK_DGRAM), (host, int(port)))

        self.running = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()
        self.get_logger().info(
            f'Simulating {n} MIT motors and the IMU at {1 / self.model.opt.timestep:.0f} Hz '
            f'({model_path}); gantry release: {self.gantry_release}. '
            'Motors damp until targets arrive.')

    def on_target(self, index, msg):
        with self.lock:
            self.motors.command(index, msg.data, time.monotonic())

    def reset(self):
        with self.lock:
            mujoco.mj_resetData(self.model, self.data)
            self.data.qpos[self.qadr] = self.initial
            if self.gantry >= 0:
                self.data.eq_active[self.gantry] = int(self.gantry_release != 'off')
            mujoco.mj_forward(self.model, self.data)
            self.next_feedback = self.next_imu = self.next_stream = self.next_pose = 0.0

    def set_gantry(self, hold):
        if self.gantry >= 0 and bool(self.data.eq_active[self.gantry]) != hold:
            self.data.eq_active[self.gantry] = int(hold)
            self.get_logger().warn('Gantry ' + ('holds the robot' if hold else 'released'))

    def on_mode(self, msg):
        if msg.data == self.gantry_release:
            self.set_gantry(False)

    def on_reset(self, _request, response):
        self.reset()
        response.success, response.message = True, 'Reset to the initial pose on the gantry'
        return response

    def on_release(self, _request, response):
        self.set_gantry(False)
        response.success, response.message = True, 'Gantry released'
        return response

    def run(self):
        dt = self.model.opt.timestep
        start_wall = time.monotonic()
        start_sim = self.data.time
        while self.running and rclpy.ok():
            # Step until sim time catches up with (scaled) wall time, at most 20 ms per batch.
            due = start_sim + (time.monotonic() - start_wall) * self.real_time_factor
            lag = due - self.data.time
            if lag > 0.03:
                self.get_logger().warn(
                    f'Sim is {lag * 1000:.0f} ms behind real time', throttle_duration_sec=1.0)
            if lag > 0.1:  # fell behind (e.g. a stall or reset): drop the backlog
                start_wall, start_sim = time.monotonic(), self.data.time
                due = start_sim
            steps = 0
            while self.data.time < due and steps < 20:
                self.step(dt)
                steps += 1
            if steps == 0:
                time.sleep(dt / 2)

    def step(self, dt):
        d = self.data
        with self.lock:
            q, qd = d.qpos[self.qadr], d.qvel[self.vadr]
            d.ctrl[self.act] = self.motors.joint_torque(q, qd, time.monotonic())
            mujoco.mj_step(self.model, d)
            now = d.time
            if now >= self.next_feedback:
                self.next_feedback = max(self.next_feedback + self.feedback_period, now)
                self.publish_feedback()
            if now >= self.next_imu:
                self.next_imu = max(self.next_imu + self.imu_period, now)
                self.publish_imu()
            if now >= self.next_pose:
                self.next_pose = max(self.next_pose + 0.02, now)
                self.publish_pose()
            if now >= self.next_stream and (self.viewer or self.stream):
                self.next_stream = now + 1.0 / 60.0
                if self.viewer is not None:
                    self.viewer.sync()
                if self.stream is not None:
                    self.stream[0].sendto(d.qpos.astype(np.float64).tobytes(), self.stream[1])

    def publish_feedback(self):
        position, velocity = self.motors.motor_feedback(
            self.data.qpos[self.qadr], self.data.qvel[self.vadr])
        for i in range(len(self.joints)):
            self.position_pubs[i].publish(Float64(data=float(position[i])))
            self.velocity_pubs[i].publish(Float64(data=float(velocity[i])))

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        x, y, z = (float(v) for v in self.data.xpos[self.base])
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        w, qx, qy, qz = (float(v) for v in self.data.xquat[self.base])
        msg.pose.orientation.w, msg.pose.orientation.x = w, qx
        msg.pose.orientation.y, msg.pose.orientation.z = qy, qz
        self.pose_pub.publish(msg)

    def publish_imu(self):
        s, a = self.data.sensordata, self.sensor
        orientation, gyro, accel = self.imu.measure(
            s[a['imu_quat']:a['imu_quat'] + 4], s[a['imu_gyro']:a['imu_gyro'] + 3],
            s[a['imu_acc']:a['imu_acc'] + 3])
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = (
            float(v) for v in orientation)
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
            float(v) for v in gyro)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (
            float(v) for v in accel)
        self.imu_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        self.thread.join(timeout=1.0)
        if self.viewer is not None:
            self.viewer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MujocoSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
