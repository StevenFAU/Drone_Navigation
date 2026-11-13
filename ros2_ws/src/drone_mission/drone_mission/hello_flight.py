"""Smoke test for the sim: GUIDED -> arm -> take off -> fly to one point -> land.

Talks to ArduPilot through MAVROS (ROS 2). Positions are in MAVROS's local ENU
frame (x = East, y = North, z = Up), in metres, relative to where the drone armed.

    ros2 run drone_mission hello_flight --ros-args -p altitude:=2.0 -p target_x:=3.0
"""
import time

from .flight_base import FlightNode, MissionError, run_node


class HelloFlight(FlightNode):
    def __init__(self):
        super().__init__('hello_flight')
        self.altitude = self.declare_parameter('altitude', 2.0).value
        self.target_x = self.declare_parameter('target_x', 3.0).value
        self.target_y = self.declare_parameter('target_y', 0.0).value
        self.tolerance = self.declare_parameter('tolerance', 0.3).value

    def mission(self):
        log = self.get_logger()
        self.connect_and_arm()
        self.takeoff(self.altitude)

        # target_x/target_y from here, in the mission frame (East/North, or forward/left with
        # heading_frame); altitude above the takeoff spot
        mx, my, _ = self.to_mission(*self.position)
        tx, ty, tz = self.to_local(mx + self.target_x, my + self.target_y, self.altitude)
        log.info(f'flying to ({tx:.1f}, {ty:.1f}, {tz:.1f})')
        deadline = time.time() + 60.0 * self.timeout_scale
        while self.distance_to(tx, ty, tz) > self.tolerance:
            if time.time() > deadline:
                raise MissionError('did not reach target')
            self.send_setpoint(tx, ty, tz)   # keep streaming setpoints at 10 Hz
            time.sleep(0.1)
        log.info(f'reached target, error {self.distance_to(tx, ty, tz):.2f} m; holding 3 s')
        for _ in range(30):
            self.send_setpoint(tx, ty, tz)
            time.sleep(0.1)

        self.land()
        log.info('sim works end to end')


def main():
    run_node(HelloFlight)


if __name__ == '__main__':
    main()
