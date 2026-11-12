"""Smoke test for the sim: GUIDED -> arm -> take off -> fly to one point -> land.

Talks to ArduPilot through MAVROS (ROS 2). Positions are in MAVROS's local ENU
frame (x = East, y = North, z = Up), in metres, relative to where the drone armed.

    ros2 run drone_mission hello_flight --ros-args -p altitude:=2.0 -p target_x:=3.0
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class HelloFlight(Node):
    def __init__(self):
        super().__init__('hello_flight')
        self.altitude = self.declare_parameter('altitude', 2.0).value
        self.target_x = self.declare_parameter('target_x', 3.0).value
        self.target_y = self.declare_parameter('target_y', 0.0).value
        self.tolerance = self.declare_parameter('tolerance', 0.3).value
        # Multiplies every wait. Gazebo with its GUI open can run at ~0.3x real time
        # on a laptop, so give it slack; set to 1.0 for headless runs.
        self.timeout_scale = self.declare_parameter('timeout_scale', 4.0).value

        self.state = State()
        self.pose = None
        # Best-effort QoS matches any MAVROS publisher.
        self.create_subscription(State, '/mavros/state', self._on_state, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._on_pose,
                                 qos_profile_sensor_data)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_cli = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.stream_cli = self.create_client(StreamRate, '/mavros/set_stream_rate')

    def _on_state(self, msg):
        self.state = msg

    def _on_pose(self, msg):
        self.pose = msg.pose.position

    # --- helpers (called from the main thread while the executor spins elsewhere) ---
    def call(self, client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service {client.srv_name} not available')
        future = client.call_async(request)
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                raise RuntimeError(f'service {client.srv_name} timed out')
            time.sleep(0.05)
        return future.result()

    def wait_for(self, condition, timeout, what):
        self.get_logger().info(f'waiting for {what}...')
        deadline = time.time() + timeout * self.timeout_scale
        while not condition():
            if time.time() > deadline:
                raise RuntimeError(f'timed out waiting for {what}')
            time.sleep(0.1)

    def set_mode(self, mode):
        res = self.call(self.mode_cli, SetMode.Request(custom_mode=mode))
        if not res.mode_sent:
            raise RuntimeError(f'set_mode {mode} rejected')
        self.wait_for(lambda: self.state.mode == mode, 10.0, f'mode {mode}')

    def distance_to(self, x, y, z):
        p = self.pose
        return math.dist((p.x, p.y, p.z), (x, y, z))

    def send_setpoint(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)

    def run(self):
        log = self.get_logger()
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
                raise RuntimeError('could not arm (check pre-arm messages in the SITL console)')
            self.call(self.arm_cli, CommandBool.Request(value=True))
            time.sleep(2.0)
        log.info('armed')

        res = self.call(self.takeoff_cli, CommandTOL.Request(altitude=float(self.altitude)))
        if not res.success:
            raise RuntimeError('takeoff rejected')
        self.wait_for(lambda: self.pose.z > self.altitude - 0.2, 30.0, f'altitude {self.altitude} m')

        start = (self.pose.x, self.pose.y)
        tx, ty, tz = start[0] + self.target_x, start[1] + self.target_y, self.altitude
        log.info(f'flying to ({tx:.1f}, {ty:.1f}, {tz:.1f})')
        deadline = time.time() + 60.0 * self.timeout_scale
        while self.distance_to(tx, ty, tz) > self.tolerance:
            if time.time() > deadline:
                raise RuntimeError('did not reach target')
            self.send_setpoint(tx, ty, tz)   # keep streaming setpoints at 10 Hz
            time.sleep(0.1)
        log.info(f'reached target, error {self.distance_to(tx, ty, tz):.2f} m; holding 3 s')
        for _ in range(30):
            self.send_setpoint(tx, ty, tz)
            time.sleep(0.1)

        self.set_mode('LAND')
        self.wait_for(lambda: not self.state.armed, 60.0, 'landing and disarm')
        log.info('landed and disarmed - sim works end to end')


def main():
    rclpy.init()
    node = HelloFlight()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        node.run()
    except RuntimeError as e:
        node.get_logger().error(str(e))
        if node.state.armed:
            # Never leave the drone hovering with nobody in charge.
            node.get_logger().warn('aborting: switching to LAND')
            try:
                node.call(node.mode_cli, SetMode.Request(custom_mode='LAND'))
            except RuntimeError as land_err:
                node.get_logger().error(f'could not switch to LAND: {land_err}')
        raise SystemExit(1)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
