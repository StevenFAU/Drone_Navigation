"""Shared MAVROS plumbing for the mission nodes (ArduCopter in GUIDED mode).

Frames: MAVROS's local frame is ENU (x = East, y = North, z = Up), metres from the EKF origin
(where ArduPilot first got a position fix, which stays put across flights in one session).
Yaw is ENU too: 0 = facing East, +pi/2 = North. Mission nodes work in a "mission frame": the
same axes, with the origin at the spot the drone armed (`self.origin`, set by connect_and_arm).
With heading_frame:=true the mission frame is instead turned to the drone's heading at arming
(x = forward, y = left), which is easier to lay out indoors than East/North.

Pattern: a node subclasses FlightNode and implements mission(); run_node() spins ROS callbacks
in a background thread while mission() runs as plain blocking code, switches to LAND if the
mission fails after arming, and always closes the run log.

Run log (start_recording): logs/<prefix>_<time>.csv at 10 Hz with the phase, the drone's own
estimate (mission frame), Gazebo's true pose when the sim publishes it (/ground_truth/drone),
and whatever the current phase puts in self.rec (errors, commands). Plus <prefix>_<time>.json
with self.summary (per-phase results, success flag, true marker pose) for the experiment tools.
"""
import csv
import json
import math
import os
import threading
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL, SetMode, StreamRate
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class MissionError(RuntimeError):
    pass


class PilotTakeover(MissionError):
    """The flight mode left GUIDED without us asking: the pilot has the drone. Stop commanding."""


REC_FIELDS = ['range', 'lateral', 'height', 'bearing_deg', 'seen', 'ctrl',
              'marker_x', 'marker_y', 'marker_z', 'target_x', 'target_y',
              'cmd_vx', 'cmd_vy', 'cmd_vz', 'cmd_yaw_rate_deg']


def yaw_of(q):
    """ENU yaw from a quaternion with .x .y .z .w"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FlightNode(Node):
    def __init__(self, name):
        super().__init__(name)
        # Multiplies every wait. Gazebo with its GUI open can run at ~0.3x real time
        # on a laptop, so give it slack; set to 1.0 for headless runs.
        self.timeout_scale = self.param('timeout_scale', 4.0)
        # EKF source set to select before arming (0 = leave it to the RC switch). On the T27,
        # set 1 = GPS (outdoors), set 2 = optical flow + rangefinder (indoors).
        self.ekf_source_set = int(self.param('ekf_source_set', 0))
        self.heading_frame = bool(self.param('heading_frame', False))
        log_dir = self.param('log_dir', '')
        self.log_dir = log_dir or os.path.join(os.environ.get('DRONE_MISSION_ROOT', '.'), 'logs')

        self.state = State()
        self.pose = None          # geometry_msgs/Pose, local ENU
        self.true_pose = None     # geometry_msgs/Pose from Gazebo (world frame), sim only
        self.true_marker = None   # same, for the marker model
        self.origin = (0.0, 0.0, 0.0)
        self.origin_yaw = 0.0     # mission frame rotation (heading_frame)
        self.true_origin = None
        self.phase = 'init'
        self.expect_guided = False   # True while we are flying the drone in GUIDED
        self.waypoint_index = 0
        self.rec = {}             # extra columns for the run log (see REC_FIELDS)
        self.summary = {'node': name, 'success': False}
        self._rec_thread = None
        self._recording = False
        self.rec_t0 = time.time()   # t = 0 of the run log

        # Best-effort QoS matches any MAVROS publisher.
        self.create_subscription(State, '/mavros/state', self._on_state, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._on_pose,
                                 qos_profile_sensor_data)
        # Ground truth from the sim (nothing publishes these on the real drone).
        self.create_subscription(PoseStamped, '/ground_truth/drone',
                                 lambda m: setattr(self, 'true_pose', m.pose), 10)
        self.create_subscription(PoseStamped, '/ground_truth/marker',
                                 lambda m: setattr(self, 'true_marker', m.pose), 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.raw_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)

        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_cli = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.stream_cli = self.create_client(StreamRate, '/mavros/set_stream_rate')
        self.command_cli = self.create_client(CommandLong, '/mavros/cmd/command')

    def param(self, name, default):
        """Declare a parameter once (several mission pieces may ask for the same one)."""
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _on_state(self, msg):
        self.state = msg

    def _on_pose(self, msg):
        self.pose = msg.pose

    # --- state helpers ---
    @property
    def position(self):
        p = self.pose.position
        return (p.x, p.y, p.z)

    @property
    def yaw(self):
        return yaw_of(self.pose.orientation)

    def distance_to(self, x, y, z):
        return math.dist(self.position, (x, y, z))

    def to_local(self, x, y, z):
        """Mission frame -> MAVROS local frame."""
        ox, oy, oz = self.origin
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return c * x - s * y + ox, s * x + c * y + oy, z + oz

    def to_mission(self, x, y, z):
        ox, oy, oz = self.origin
        dx, dy = x - ox, y - oy
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return c * dx + s * dy, -s * dx + c * dy, z - oz

    def rotation(self):
        """3x3 rotation matrix body (FLU) -> local ENU from the current attitude."""
        q = self.pose.orientation
        w, x, y, z = q.w, q.x, q.y, q.z
        return (
            (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
        )

    # --- blocking helpers (called from the mission thread while the executor spins elsewhere) ---
    def call(self, client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise MissionError(f'service {client.srv_name} not available')
        future = client.call_async(request)
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                raise MissionError(f'service {client.srv_name} timed out')
            time.sleep(0.05)
        return future.result()

    def check_pilot(self):
        """Raise PilotTakeover if the mode left GUIDED while we were flying. On the real drone
        that's the pilot flipping the RC mode switch; never fight it."""
        if self.expect_guided and self.state.mode and self.state.mode != 'GUIDED':
            self.expect_guided = False
            raise PilotTakeover(f'mode changed to {self.state.mode}: pilot has control, '
                                'mission stopped')

    def wait_for(self, condition, timeout, what):
        self.get_logger().info(f'waiting for {what}...')
        deadline = time.time() + timeout * self.timeout_scale
        while not condition():
            self.check_pilot()
            if time.time() > deadline:
                raise MissionError(f'timed out waiting for {what}')
            time.sleep(0.1)

    def set_mode(self, mode):
        res = self.call(self.mode_cli, SetMode.Request(custom_mode=mode))
        if not res.mode_sent:
            raise MissionError(f'set_mode {mode} rejected')
        self.wait_for(lambda: self.state.mode == mode, 10.0, f'mode {mode}')

    def send_setpoint(self, x, y, z, yaw=None):
        """Position target in the local ENU frame. yaw=None keeps the current heading."""
        self.check_pilot()
        if yaw is None:
            yaw = self.yaw
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.setpoint_pub.publish(msg)

    def send_velocity(self, vx, vy, vz, yaw_rate=0.0):
        """Velocity target in ENU (m/s) plus yaw rate (rad/s, + = counter-clockwise from above).

        ArduPilot stops if these stop arriving for ~3 s, so stream them at ~10 Hz.
        MAVROS converts ENU to ArduPilot's NED.
        """
        self.check_pilot()
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
                         | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY
                         | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW)
        msg.velocity.x, msg.velocity.y, msg.velocity.z = float(vx), float(vy), float(vz)
        msg.yaw_rate = float(yaw_rate)
        self.raw_pub.publish(msg)
        self.rec.update(cmd_vx=vx, cmd_vy=vy, cmd_vz=vz, cmd_yaw_rate_deg=math.degrees(yaw_rate))

    # --- run log ---
    def start_recording(self, prefix):
        os.makedirs(self.log_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_base = os.path.join(self.log_dir, f'{prefix}_{stamp}')
        self.summary.update(log_csv=self.log_base + '.csv', started=stamp)
        self._recording = True
        self.rec_t0 = time.time()
        self._rec_thread = threading.Thread(target=self._record_loop, daemon=True)
        self._rec_thread.start()

    def _record_loop(self):
        with open(self.log_base + '.csv', 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'phase', 'waypoint', 'mode', 'armed', 'x', 'y', 'z', 'yaw_deg',
                        'gt_x', 'gt_y', 'gt_z', 'gt_qx', 'gt_qy', 'gt_qz', 'gt_qw'] + REC_FIELDS)
            while self._recording:
                if self.pose is not None:
                    x, y, z = self.to_mission(*self.position)
                    row = [f'{time.time() - self.rec_t0:.2f}', self.phase, self.waypoint_index,
                           self.state.mode, int(self.state.armed), f'{x:.3f}', f'{y:.3f}',
                           f'{z:.3f}', f'{math.degrees(self.yaw):.1f}']
                    t = self.true_pose
                    if t is not None:
                        row += [f'{t.position.x:.4f}', f'{t.position.y:.4f}',
                                f'{t.position.z:.4f}', f'{t.orientation.x:.5f}',
                                f'{t.orientation.y:.5f}', f'{t.orientation.z:.5f}',
                                f'{t.orientation.w:.5f}']
                    else:
                        row += [''] * 7
                    for k in REC_FIELDS:
                        v = self.rec.get(k)
                        row.append('' if v is None else (f'{v:.4f}' if isinstance(v, float) else v))
                    w.writerow(row)
                time.sleep(0.1)

    def finish_recording(self):
        if self._rec_thread is None:
            return
        self._recording = False
        self._rec_thread.join(timeout=1.0)
        if self.true_marker is not None:
            m = self.true_marker
            self.summary['true_marker'] = {
                'x': m.position.x, 'y': m.position.y, 'z': m.position.z,
                'yaw': yaw_of(m.orientation)}
        with open(self.log_base + '.json', 'w') as f:
            json.dump(self.summary, f, indent=2)
        self.get_logger().info(f'run log: {self.log_base}.csv / .json')
        self._rec_thread = None

    # --- flight phases ---
    def connect_and_arm(self):
        """Wait for MAVROS, switch to GUIDED and arm (retrying through SITL warm-up).
        Records the arming spot as the mission-frame origin."""
        self.phase = 'arming'
        self.wait_for(lambda: self.state.connected, 60.0, 'MAVROS connection to ArduPilot')
        # Ask ArduPilot to stream all telemetry at 10 Hz on this link.
        self.call(self.stream_cli, StreamRate.Request(stream_id=0, message_rate=10, on_off=True))
        if self.ekf_source_set:
            # MAV_CMD_SET_EKF_SOURCE_SET (42007). Not saved: a reboot or the RC switch changes it.
            res = self.call(self.command_cli, CommandLong.Request(command=42007,
                                                                  param1=float(self.ekf_source_set)))
            if not res.success:
                raise MissionError(f'flight controller refused EKF source set {self.ekf_source_set}')
            self.get_logger().info(f'EKF source set {self.ekf_source_set} selected')
        self.wait_for(lambda: self.pose is not None, 30.0, 'local position')
        self.set_mode('GUIDED')
        self.expect_guided = True

        # ArduPilot refuses to arm until its pre-arm checks pass (GPS lock, EKF ready).
        # In SITL that takes ~30 s after start, so keep retrying.
        deadline = time.time() + 120.0 * self.timeout_scale
        while not self.state.armed:
            if time.time() > deadline:
                raise MissionError('could not arm (check pre-arm messages in logs/sim_ardupilot.log)')
            self.call(self.arm_cli, CommandBool.Request(value=True))
            time.sleep(2.0)
        self.origin = self.position
        self.origin_yaw = self.yaw if self.heading_frame else 0.0
        if self.true_pose is not None:
            t = self.true_pose.position
            self.true_origin = (t.x, t.y, t.z)
            self.summary['true_origin'] = {'x': t.x, 'y': t.y, 'z': t.z,
                                           'yaw': yaw_of(self.true_pose.orientation)}
        self.get_logger().info('armed')

    def takeoff(self, altitude):
        self.phase = 'takeoff'
        res = self.call(self.takeoff_cli, CommandTOL.Request(altitude=float(altitude)))
        if not res.success:
            raise MissionError('takeoff rejected')
        self.wait_for(lambda: self.to_mission(*self.position)[2] > altitude - 0.2, 30.0,
                      f'altitude {altitude} m')

    def land(self):
        self.phase = 'land'
        self.rec = {}
        self.expect_guided = False
        self.set_mode('LAND')
        self.wait_for(lambda: not self.state.armed, 60.0, 'landing and disarm')
        self.phase = 'landed'
        self.get_logger().info('landed and disarmed')

    def mission(self):
        raise NotImplementedError


def run_node(node_cls):
    """Entry-point helper: spin callbacks in the background, run node.mission() here."""
    rclpy.init()
    try:
        node = node_cls()
    except MissionError as e:   # bad parameters: nothing is flying yet
        rclpy.logging.get_logger(node_cls.__name__).error(str(e))
        rclpy.try_shutdown()
        raise SystemExit(2)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        t0 = time.time()
        node.mission()
        node.summary['success'] = True
        node.summary['duration_s'] = round(time.time() - t0, 1)
    except PilotTakeover as e:
        node.get_logger().warning(str(e))
        node.summary.update(success=False, error=str(e), failed_in=node.phase)
        raise SystemExit(1)
    except (MissionError, KeyboardInterrupt) as e:
        msg = str(e) or type(e).__name__
        node.get_logger().error(msg)
        node.summary.update(success=False, error=msg, failed_in=node.phase)
        if node.state.armed and node.state.mode in ('GUIDED', ''):
            # Never leave the drone hovering with nobody in charge.
            node.get_logger().warning('aborting: switching to LAND')
            try:
                node.call(node.mode_cli, SetMode.Request(custom_mode='LAND'))
            except Exception as land_err:  # e.g. context already shut down by Ctrl-C
                node.get_logger().error(f'could not switch to LAND: {land_err}')
        raise SystemExit(1)
    finally:
        node.finish_recording()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
