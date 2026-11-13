"""The full course-project mission in one run:

    take off -> fly the waypoints -> search for the ArUco marker -> approach ->
    hold 1 m in front of it for 5 s -> land

Waypoints, tolerances and marker settings are the same parameters as waypoint_navigator and
marker_approach (see those modules); config/mission.yaml holds the defaults used for the demo.

    ros2 run drone_mission mission
    ros2 run drone_mission mission --ros-args --params-file ros2_ws/src/drone_mission/config/mission.yaml

Writes logs/mission_<time>.csv (10 Hz: phase, waypoint, estimate + Gazebo truth, marker errors,
commands) and .json (per-waypoint results, search, hold statistics, success).
"""
from .flight_base import FlightNode, run_node
from .marker_approach import MarkerApproach
from .waypoint_navigator import WaypointNavigator

# Default route for the sim world (marker 6 m North of the start, facing it). Mission frame:
# metres from takeoff, x = East, y = North, z = height. A loop to the South-West that ends 1 m
# East of the start heading East, with the marker ~100 deg to the left and out of view, so the
# search phase always runs.
MISSION_WAYPOINTS = [0.0, -3.0, 2.0,
                     -3.0, -3.0, 2.5,
                     -3.0, 0.0, 2.0,
                     1.0, 0.0, 1.5]


class Mission(WaypointNavigator, MarkerApproach):
    def __init__(self):
        FlightNode.__init__(self, 'mission')
        self.param('takeoff_altitude', 2.0)
        self.setup_waypoints(MISSION_WAYPOINTS)
        self.setup_marker()

    def mission(self):
        log = self.get_logger()
        log.info(f'mission: {len(self.waypoints)} waypoints, then marker {self.marker_id} '
                 f'at {self.target_range} m for {self.hold_time} s')
        self.connect_and_arm()
        self.start_recording('mission')
        self.takeoff(self.takeoff_altitude)
        self.fly_waypoints()
        self.search()
        self.approach_and_hold()
        self.land()
        log.info('mission complete')


def main():
    run_node(Mission)


if __name__ == '__main__':
    main()
