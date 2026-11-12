"""Fly an ordered list of waypoints, then land.

GUIDED -> arm -> take off -> for each waypoint: stream position setpoints at 10 Hz until
the drone is within `tolerance`, hold `hold_time` s, advance -> LAND.

Waypoints are (x, y, z) in metres, ENU axes (x = East, y = North, z = Up), with x and y
measured from where the drone took off. (MAVROS's own local frame is anchored where
ArduPilot first got a position fix, which stays put across flights in one sim session,
so the node records the takeoff point and adds it on. z is height above the ground.) ROS 2 parameters can't hold a list of lists, so
they're given as one flat list [x1, y1, z1, x2, y2, z2, ...]:

    ros2 run drone_mission waypoint_navigator
    ros2 run drone_mission waypoint_navigator --ros-args -p waypoints:="[5.0,0.0,2.0, 5.0,5.0,3.0]"
    ros2 run drone_mission waypoint_navigator --ros-args --params-file config/waypoints.yaml

Each run writes a CSV of the drone's pose (10 Hz) and a per-waypoint summary to logs/.
"""
import csv
import math
import os
import threading
import time
from datetime import datetime

from .flight_base import FlightNode, MissionError, run_node

MIN_ALTITUDE = 1.0   # m; refuse waypoints lower than this


class WaypointNavigator(FlightNode):
    def __init__(self):
        super().__init__('waypoint_navigator')
        # Default: a 4 m square at 2-3 m altitude, ending back over the start.
        flat = self.declare_parameter(
            'waypoints', [4.0, 0.0, 2.0,  4.0, 4.0, 3.0,  0.0, 4.0, 3.0,  0.0, 0.0, 2.0]).value
        self.tolerance = self.declare_parameter('tolerance', 0.3).value          # m, 3D
        self.hold_time = self.declare_parameter('hold_time', 2.0).value          # s at each waypoint
        self.takeoff_altitude = self.declare_parameter('takeoff_altitude', 2.0).value
        # Turn to face the next waypoint before flying to it (the real drone's camera looks
        # forward, so this keeps it pointed where it's going).
        self.face_travel = self.declare_parameter('face_travel', True).value
        self.leg_speed = self.declare_parameter('min_expected_speed', 0.5).value  # m/s, for leg timeouts
        log_dir = self.declare_parameter('log_dir', '').value
        self.log_dir = log_dir or os.path.join(os.environ.get('DRONE_MISSION_ROOT', '.'), 'logs')

        if len(flat) == 0 or len(flat) % 3 != 0:
            raise MissionError(f'waypoints needs x,y,z triples; got {len(flat)} numbers')
        self.waypoints = [tuple(float(v) for v in flat[i:i + 3]) for i in range(0, len(flat), 3)]
        for i, (_, _, z) in enumerate(self.waypoints, 1):
            if z < MIN_ALTITUDE:
                raise MissionError(f'waypoint {i} altitude {z} m is below {MIN_ALTITUDE} m')

        self.origin = (0.0, 0.0, 0.0)   # local-frame position at arming; set in mission()
        self.current_wp = 0          # 0 = not flying a leg yet (index is 1-based in logs)
        self._logging = False

    # --- pose log for the experiment / plots ---
    def _log_loop(self, path):
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            # x, y, z relative to the takeoff point (same frame as the waypoints)
            w.writerow(['t', 'x', 'y', 'z', 'yaw', 'waypoint', 'mode', 'armed'])
            t0 = time.time()
            while self._logging:
                if self.pose is not None:
                    x, y, z = self.to_mission(*self.position)
                    w.writerow([f'{time.time() - t0:.2f}', f'{x:.3f}', f'{y:.3f}', f'{z:.3f}',
                                f'{self.yaw:.3f}', self.current_wp, self.state.mode,
                                int(self.state.armed)])
                time.sleep(0.1)

    def to_local(self, x, y, z):
        ox, oy, oz = self.origin
        return x + ox, y + oy, z + oz

    def to_mission(self, x, y, z):
        ox, oy, oz = self.origin
        return x - ox, y - oy, z - oz

    def hold(self, x, y, z, yaw, seconds):
        end = time.time() + seconds
        while time.time() < end:
            self.send_setpoint(x, y, z, yaw)
            time.sleep(0.1)

    def turn_towards(self, x, y, z, yaw):
        """Hover in place and yaw to face the next waypoint before moving."""
        deadline = time.time() + 15.0 * self.timeout_scale
        while abs(math.remainder(yaw - self.yaw, 2 * math.pi)) > math.radians(10):
            if time.time() > deadline:
                self.get_logger().warn('yaw did not settle; flying anyway')
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

        self.hold(x, y, z, yaw, self.hold_time)
        hold_err = self.distance_to(x, y, z)
        log.info(f'waypoint {index} reached in {leg_time:.1f} s, error {arrival_err:.2f} m '
                 f'(after {self.hold_time:.0f} s hold: {hold_err:.2f} m)')
        return {'waypoint': index, 'x': wx, 'y': wy, 'z': wz, 'leg_time_s': round(leg_time, 2),
                'arrival_error_m': round(arrival_err, 3), 'hold_error_m': round(hold_err, 3)}

    def mission(self):
        log = self.get_logger()
        log.info(f'{len(self.waypoints)} waypoints, tolerance {self.tolerance} m, '
                 f'hold {self.hold_time} s')
        self.connect_and_arm()
        # Ground level at the takeoff spot is the mission origin (see module docstring).
        self.origin = self.position

        os.makedirs(self.log_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        pose_csv = os.path.join(self.log_dir, f'waypoints_{stamp}_pose.csv')
        summary_csv = os.path.join(self.log_dir, f'waypoints_{stamp}_summary.csv')
        self._logging = True
        logger = threading.Thread(target=self._log_loop, args=(pose_csv,), daemon=True)
        logger.start()
        try:
            self.takeoff(self.takeoff_altitude)
            results = []
            for i, (x, y, z) in enumerate(self.waypoints, 1):
                self.current_wp = i
                results.append(self.fly_to(i, x, y, z))
            self.current_wp = 0
            self.land()
        finally:
            self._logging = False
            logger.join(timeout=1.0)

        with open(summary_csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
            w.writeheader()
            w.writerows(results)
        log.info('mission complete:')
        for r in results:
            log.info(f"  wp {r['waypoint']}: ({r['x']:.1f}, {r['y']:.1f}, {r['z']:.1f})  "
                     f"{r['leg_time_s']:5.1f} s  arrival {r['arrival_error_m']:.2f} m  "
                     f"hold {r['hold_error_m']:.2f} m")
        log.info(f'logs: {pose_csv}, {summary_csv}')


def main():
    run_node(WaypointNavigator)


if __name__ == '__main__':
    main()
