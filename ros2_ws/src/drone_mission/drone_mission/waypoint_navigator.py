"""Fly an ordered list of waypoints, then land.

GUIDED -> arm -> take off -> for each waypoint: turn to face it, stream position setpoints at
10 Hz until the drone is within `tolerance` (arrived), keep hovering until it has settled within
`settle_tolerance` (ArduPilot overshoots a 3-4 m step by 10-20 cm and settles in 2-4 s), hold
`waypoint_hold` s, advance -> LAND.

Waypoints are (x, y, z) in metres, ENU axes (x = East, y = North, z = Up), with x and y
measured from where the drone took off and z as height above the takeoff spot. ROS 2
parameters can't hold a list of lists, so they're one flat list [x1, y1, z1, x2, y2, z2, ...]:

    ros2 run drone_mission waypoint_navigator
    ros2 run drone_mission waypoint_navigator --ros-args -p waypoints:="[5.0,0.0,2.0, 5.0,5.0,3.0]"
    ros2 run drone_mission waypoint_navigator --ros-args --params-file config/waypoints.yaml

Each run writes logs/waypoints_<time>.csv (10 Hz pose, estimate + Gazebo truth) and .json
(per-waypoint leg time and errors).
"""
import math
import time

from .flight_base import FlightNode, MissionError, run_node

MIN_ALTITUDE = 1.0   # m; refuse waypoints lower than this
DEFAULT_SQUARE = [4.0, 0.0, 2.0, 4.0, 4.0, 3.0, 0.0, 4.0, 3.0, 0.0, 0.0, 2.0]


class WaypointNavigator(FlightNode):
    def __init__(self, name='waypoint_navigator'):
        super().__init__(name)
        self.setup_waypoints(DEFAULT_SQUARE)

    def setup_waypoints(self, default_waypoints):
        flat = self.param('waypoints', default_waypoints)
        self.tolerance = self.param('tolerance', 0.3)               # m, 3D
        self.waypoint_hold = self.param('waypoint_hold', 2.0)       # s at each waypoint
        self.settle_tolerance = self.param('settle_tolerance', 0.10)  # m, before the hold counts
        self.settle_timeout = self.param('settle_timeout', 10.0)      # s; hold anyway after this
        self.takeoff_altitude = self.param('takeoff_altitude', 2.0)
        # Turn to face the next waypoint before flying to it (the real drone's camera looks
        # forward, so this keeps it pointed where it's going).
        self.face_travel = self.param('face_travel', True)
        self.leg_speed = self.param('min_expected_speed', 0.5)      # m/s, for leg timeouts

        if len(flat) == 0 or len(flat) % 3 != 0:
            raise MissionError(f'waypoints needs x,y,z triples; got {len(flat)} numbers')
        self.waypoints = [tuple(float(v) for v in flat[i:i + 3]) for i in range(0, len(flat), 3)]
        for i, (_, _, z) in enumerate(self.waypoints, 1):
            if z < MIN_ALTITUDE:
                raise MissionError(f'waypoint {i} altitude {z} m is below {MIN_ALTITUDE} m')

    def hold_position(self, x, y, z, yaw, seconds):
        end = time.time() + seconds
        while time.time() < end:
            self.send_setpoint(x, y, z, yaw)
            time.sleep(0.1)

    def turn_towards(self, x, y, z, yaw):
        """Hover in place and yaw to face the next waypoint before moving."""
        deadline = time.time() + 15.0 * self.timeout_scale
        while abs(math.remainder(yaw - self.yaw, 2 * math.pi)) > math.radians(10):
            if time.time() > deadline:
                self.get_logger().warning('yaw did not settle; flying anyway')
                return
            self.send_setpoint(x, y, z, yaw)
            time.sleep(0.1)

    def fly_to(self, index, wx, wy, wz):
        log = self.get_logger()
        x, y, z = self.to_local(wx, wy, wz)
        here = self.position
        dist = math.dist(here, (x, y, z))
        yaw = None
        if self.face_travel and math.hypot(x - here[0], y - here[1]) > 0.5:
            yaw = math.atan2(y - here[1], x - here[0])
            self.turn_towards(*here, yaw)
        yaw = self.yaw if yaw is None else yaw

        log.info(f'waypoint {index}/{len(self.waypoints)}: flying to '
                 f'({wx:.1f}, {wy:.1f}, {wz:.1f}), {dist:.1f} m away')
        start = time.time()
        deadline = start + (dist / self.leg_speed + 20.0) * self.timeout_scale
        while self.distance_to(x, y, z) > self.tolerance:
            if time.time() > deadline:
                raise MissionError(f'did not reach waypoint {index} '
                                   f'(still {self.distance_to(x, y, z):.2f} m away)')
            self.send_setpoint(x, y, z, yaw)   # ArduPilot needs a steady stream (~10 Hz)
            time.sleep(0.1)
        arrival_err = self.distance_to(x, y, z)
        leg_time = time.time() - start

        # Settled = inside settle_tolerance for 1 s straight (passing through the waypoint on the
        # way to an overshoot doesn't count).
        settle_start = time.time()
        settle_end = settle_start + self.settle_timeout * self.timeout_scale
        inside_since = None
        while time.time() < settle_end:
            if self.distance_to(x, y, z) <= self.settle_tolerance:
                inside_since = inside_since or time.time()
                if time.time() - inside_since >= 1.0:
                    break
            else:
                inside_since = None
            self.send_setpoint(x, y, z, yaw)
            time.sleep(0.1)
        settle_time = time.time() - settle_start
        self.hold_position(x, y, z, yaw, self.waypoint_hold)
        hold_err = self.distance_to(x, y, z)
        log.info(f'waypoint {index} reached in {leg_time:.1f} s, error {arrival_err:.2f} m; '
                 f'settled in {settle_time:.1f} s; after {self.waypoint_hold:.0f} s hold: {hold_err:.2f} m')
        return {'waypoint': index, 'x': wx, 'y': wy, 'z': wz, 'leg_time_s': round(leg_time, 2),
                'arrival_error_m': round(arrival_err, 3), 'settle_time_s': round(settle_time, 2),
                'hold_error_m': round(hold_err, 3),
                'hold_s': self.waypoint_hold,
                't_end': round(time.time() - self.rec_t0, 2)}   # run-log time

    def fly_waypoints(self):
        self.phase = 'waypoints'
        results = []
        self.summary['waypoints'] = results
        for i, (x, y, z) in enumerate(self.waypoints, 1):
            self.waypoint_index = i
            results.append(self.fly_to(i, x, y, z))
        self.waypoint_index = 0
        log = self.get_logger()
        log.info('waypoints done:')
        for r in results:
            log.info(f"  wp {r['waypoint']}: ({r['x']:.1f}, {r['y']:.1f}, {r['z']:.1f})  "
                     f"{r['leg_time_s']:5.1f} s  arrival {r['arrival_error_m']:.2f} m  "
                     f"hold {r['hold_error_m']:.2f} m")
        return results

    def mission(self):
        self.get_logger().info(f'{len(self.waypoints)} waypoints, tolerance {self.tolerance} m, '
                               f'hold {self.waypoint_hold} s')
        self.connect_and_arm()
        self.start_recording('waypoints')
        self.takeoff(self.takeoff_altitude)
        self.fly_waypoints()
        self.land()


def main():
    run_node(WaypointNavigator)


if __name__ == '__main__':
    main()
