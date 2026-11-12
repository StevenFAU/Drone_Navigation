"""Shared MAVROS plumbing for the mission nodes (ArduCopter in GUIDED mode).

Positions are in MAVROS's local ENU frame (x = East, y = North, z = Up), in metres,
relative to where the drone armed. Yaw is ENU too: 0 = facing East, +pi/2 = North.

Pattern: a node subclasses FlightNode and implements mission(); run_node() spins
ROS callbacks in a background thread while mission() runs as plain blocking code,
and switches to LAND if the mission fails after arming.
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class MissionError(RuntimeError):
    pass


class FlightNode(Node):
    def __init__(self, name):
        super().__init__(name)
        # Multiplies every wait. Gazebo with its GUI open can run at ~0.3x real time
        # on a laptop, so give it slack; set to 1.0 for headless runs.
        self.timeout_scale = self.declare_parameter('timeout_scale', 4.0).value

        self.state = State()
        self.pose = None   # geometry_msgs/Pose
        # Best-effort QoS matches any MAVROS publisher.
        self.create_subscription(State, '/mavros/state', self._on_state, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._on_pose,
                                 qos_profile_sensor_data)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.raw_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)

        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_cli = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.stream_cli = self.create_client(StreamRate, '/mavros/set_stream_rate')

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
        q = self.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def distance_to(self, x, y, z):
        return math.dist(self.position, (x, y, z))

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

    def wait_for(self, condition, timeout, what):
        self.get_logger().info(f'waiting for {what}...')
        deadline = time.time() + timeout * self.timeout_scale
        while not condition():
            if time.time() > deadline:
                raise MissionError(f'timed out waiting for {what}')
            time.sleep(0.1)

    def set_mode(self, mode):
        res = self.call(self.mode_cli, SetMode.Request(custom_mode=mode))
        if not res.mode_sent:
            raise MissionError(f'set_mode {mode} rejected')
        self.wait_for(lambda: self.state.mode == mode, 10.0, f'mode {mode}')

    def send_setpoint(self, x, y, z, yaw=None):
        """Position target in ENU. yaw=None keeps the current heading."""
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

    def rotation(self):
        """3x3 rotation matrix body (FLU) -> local ENU from the current attitude."""
        q = self.pose.orientation
        w, x, y, z = q.w, q.x, q.y, q.z
        return (
            (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
        )

    # --- flight phases ---
    def connect_and_arm(self):
        """Wait for MAVROS, switch to GUIDED and arm (retrying through SITL warm-up)."""
        self.wait_for(lambda: self.state.connected, 60.0, 'MAVROS connection to ArduPilot')
        # Ask ArduPilot to stream all telemetry at 10 Hz on this link.
        self.call(self.stream_cli, StreamRate.Request(stream_id=0, message_rate=10, on_off=True))
        self.wait_for(lambda: self.pose is not None, 30.0, 'local position')
        self.set_mode('GUIDED')

        # ArduPilot refuses to arm until its pre-arm checks pass (GPS lock, EKF ready).
        # In SITL that takes ~30 s after start, so keep retrying.
        deadline = time.time() + 120.0 * self.timeout_scale
        while not self.state.armed:
            if time.time() > deadline:
                raise MissionError('could not arm (check pre-arm messages in logs/sim_ardupilot.log)')
            self.call(self.arm_cli, CommandBool.Request(value=True))
            time.sleep(2.0)
        self.get_logger().info('armed')

    def takeoff(self, altitude):
        res = self.call(self.takeoff_cli, CommandTOL.Request(altitude=float(altitude)))
        if not res.success:
            raise MissionError('takeoff rejected')
        self.wait_for(lambda: self.position[2] > altitude - 0.2, 30.0, f'altitude {altitude} m')

    def land(self):
        self.set_mode('LAND')
        self.wait_for(lambda: not self.state.armed, 60.0, 'landing and disarm')
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
        node.mission()
    except (MissionError, KeyboardInterrupt) as e:
        node.get_logger().error(str(e) or type(e).__name__)
        if node.state.armed:
            # Never leave the drone hovering with nobody in charge.
            node.get_logger().warn('aborting: switching to LAND')
            try:
                node.call(node.mode_cli, SetMode.Request(custom_mode='LAND'))
            except Exception as land_err:  # e.g. context already shut down by Ctrl-C
                node.get_logger().error(f'could not switch to LAND: {land_err}')
        raise SystemExit(1)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
