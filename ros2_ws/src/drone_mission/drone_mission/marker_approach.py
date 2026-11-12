"""Find the ArUco marker, fly to 1 m in front of it, hold there for 5 s, then land.

Phases:
  search    yaw slowly in place until the forward camera sees the marker (max one full turn)
  approach  proportional control, streamed as velocity setpoints at 10 Hz:
              yaw rate    <- bearing to the marker      (keep it centred in the image)
              climb rate  <- height difference          (camera level with the marker)
              horizontal  <- offset from the spot `target_range` m out along the marker's
                             normal (closes range and lines up square to the marker)
  hold      same controller; the hold timer runs while range, lateral offset, height and
            bearing are all inside tolerance with a fresh detection, and restarts otherwise
  land      after `hold_time` s continuously in tolerance

    ros2 run drone_mission marker_approach
    ros2 run drone_mission marker_approach --ros-args -p target_range:=1.5 -p hold_time:=5.0

Writes logs/marker_<time>.csv (10 Hz: phase, errors, commands, pose) and prints a summary.
"""
import csv
import math
import os
import time
from datetime import datetime

from .flight_base import FlightNode, MissionError, run_node
from .marker_tracker import MarkerTracker


def clamp(v, limit):
    return max(-limit, min(limit, v))


class MarkerApproach(FlightNode):
    def __init__(self, name='marker_approach'):
        super().__init__(name)
        p = self.declare_parameter
        self.marker_id = p('marker_id', 0).value                  # -1 = any id
        self.marker_size = p('marker_size', 0.20).value            # m, black border included
        self.takeoff_altitude = p('takeoff_altitude', 1.5).value   # m (sim marker is 1.5 m up)
        self.target_range = max(0.5, p('target_range', 1.0).value)  # m in front of the marker
        self.hold_time = p('hold_time', 5.0).value                 # s, continuously in tolerance
        # Tolerances for "holding"
        self.range_tol = p('range_tol', 0.10).value                # m
        self.lateral_tol = p('lateral_tol', 0.15).value            # m
        self.height_tol = p('height_tol', 0.10).value              # m
        self.bearing_tol = math.radians(p('bearing_tol_deg', 5.0).value)
        # Controller gains and limits
        self.k_pos = p('k_pos', 0.5).value          # (m/s) per m
        self.k_z = p('k_z', 0.8).value
        self.k_yaw = p('k_yaw', 1.0).value          # (rad/s) per rad
        self.max_speed = p('max_speed', 0.5).value  # m/s horizontal
        self.max_climb = p('max_climb', 0.3).value  # m/s
        self.max_yaw_rate = math.radians(p('max_yaw_rate_deg', 30.0).value)
        self.search_rate = math.radians(p('search_rate_deg', 20.0).value)
        self.lost_timeout = p('lost_timeout', 10.0).value    # s without detection before giving up
        self.approach_timeout = p('approach_timeout', 90.0).value
        log_dir = p('log_dir', '').value
        self.log_dir = log_dir or os.path.join(os.environ.get('DRONE_MISSION_ROOT', '.'), 'logs')

        self.tracker = MarkerTracker(self, marker_id=self.marker_id, marker_size=self.marker_size)
        self.phase = 'init'
        self._csv = None
        self._t0 = time.time()
        self._hold_alt = None

    # --- logging ---
    def _open_log(self):
        os.makedirs(self.log_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(self.log_dir, f'marker_{stamp}.csv')
        self._file = open(self.log_path, 'w', newline='')
        self._csv = csv.writer(self._file)
        self._csv.writerow(['t', 'phase', 'seen', 'range', 'lateral', 'height', 'bearing_deg',
                            'raw_range', 'vx', 'vy', 'vz', 'yaw_rate_deg', 'x', 'y', 'z', 'yaw_deg'])
        self._t0 = time.time()

    def _log(self, err, cmd):
        if self._csv is None:
            return
        x, y, z = self.position
        e = err or {}
        f = lambda k: f"{e[k]:.3f}" if k in e else ''
        self._csv.writerow([
            f'{time.time() - self._t0:.2f}', self.phase, int(self.tracker.seen_within(0.5)),
            f('range'), f('lateral'), f('height'),
            f"{math.degrees(e['bearing']):.1f}" if 'bearing' in e else '',
            f'{self.tracker.raw_range:.3f}' if self.tracker.raw_range else '',
            f'{cmd[0]:.3f}', f'{cmd[1]:.3f}', f'{cmd[2]:.3f}', f'{math.degrees(cmd[3]):.1f}',
            f'{x:.3f}', f'{y:.3f}', f'{z:.3f}', f'{math.degrees(self.yaw):.1f}'])

    def command(self, vx, vy, vz, yaw_rate, err=None):
        self.send_velocity(vx, vy, vz, yaw_rate)
        self._log(err, (vx, vy, vz, yaw_rate))

    def hold_altitude_rate(self):
        return clamp(self.k_z * (self._hold_alt - self.position[2]), self.max_climb)

    # --- phases ---
    def search(self):
        """Yaw in place until the marker is seen; give up after one full turn."""
        log = self.get_logger()
        self.phase = 'search'
        self._hold_alt = self.position[2]
        if self.tracker.seen_within(1.0):
            log.info('marker already in view')
            return
        log.info(f'searching: yawing at {math.degrees(self.search_rate):.0f} deg/s')
        turned, last_yaw = 0.0, self.yaw
        deadline = time.time() + (2 * math.pi / self.search_rate + 15.0) * self.timeout_scale
        while not self.tracker.seen_within(0.5):
            if turned > 2 * math.pi + 0.2 or time.time() > deadline:
                raise MissionError('marker not found after a full turn')
            self.command(0.0, 0.0, self.hold_altitude_rate(), self.search_rate)
            time.sleep(0.1)
            turned += abs(math.remainder(self.yaw - last_yaw, 2 * math.pi))
            last_yaw = self.yaw
        log.info(f'marker found after turning {math.degrees(turned):.0f} deg')

    def controller(self, err):
        """Velocity command (ENU) from the errors; see module docstring."""
        n = self.tracker.normal
        left = (-n[1], n[0])
        # The marker's facing direction is only good to a few degrees far away, which shows up
        # as tens of cm of fake sideways offset at 4-6 m. Fade the sideways correction in:
        # full strength within 2 m.
        lat = err['lateral'] * min(1.0, 2.0 / max(err['range'], 0.1))
        # Horizontal error toward the target spot, in ENU: move back along the normal by
        # range_err and sideways by lateral.
        ex = -err['range_err'] * n[0] - lat * left[0]
        ey = -err['range_err'] * n[1] - lat * left[1]
        vx, vy = self.k_pos * ex, self.k_pos * ey
        speed = math.hypot(vx, vy)
        if speed > self.max_speed:
            vx, vy = vx * self.max_speed / speed, vy * self.max_speed / speed
        # Don't translate much while the marker is far off-centre (it could leave the image).
        scale = max(0.0, math.cos(min(abs(err['bearing']) * 2, math.pi / 2)))
        vz = clamp(-self.k_z * err['height'], self.max_climb)
        yaw_rate = clamp(self.k_yaw * err['bearing'], self.max_yaw_rate)
        return vx * scale, vy * scale, vz, yaw_rate

    def in_tolerance(self, err):
        return (abs(err['range_err']) < self.range_tol and abs(err['lateral']) < self.lateral_tol
                and abs(err['height']) < self.height_tol and abs(err['bearing']) < self.bearing_tol)

    def approach_and_hold(self):
        log = self.get_logger()
        self.phase = 'approach'
        err = self.tracker.errors(self.target_range)
        log.info(f"approaching: marker {err['range']:.2f} m out, {err['lateral']:+.2f} m to the side, "
                 f"{err['height']:+.2f} m height diff, bearing {math.degrees(err['bearing']):+.0f} deg")
        start = time.time()
        deadline = start + self.approach_timeout * self.timeout_scale
        hold_start = None
        hold_errs = []
        last_report = 0.0
        while True:
            now = time.time()
            if now > deadline:
                raise MissionError('approach timed out')
            if not self.tracker.seen_within(self.lost_timeout * self.timeout_scale):
                raise MissionError(f'marker lost for {self.lost_timeout:.0f} s')

            err = self.tracker.errors(self.target_range)
            fresh = self.tracker.seen_within(0.5)
            if fresh:
                cmd = self.controller(err)
            else:
                # Lost for a moment: stop translating, keep facing where it was.
                cmd = (0.0, 0.0, 0.0, clamp(self.k_yaw * err['bearing'], self.max_yaw_rate))

            if fresh and self.in_tolerance(err):
                if hold_start is None:
                    hold_start = now
                    self.phase = 'hold'
                    log.info(f'in position after {now - start:.1f} s; holding {self.hold_time:.0f} s')
                hold_errs.append(err)
                if now - hold_start >= self.hold_time:
                    break
            elif hold_start is not None:
                log.info(f'drifted out of tolerance after {now - hold_start:.1f} s; restarting hold '
                         f"(range {err['range']:.2f} m, lateral {err['lateral']:+.2f} m, "
                         f"height {err['height']:+.2f} m, bearing {math.degrees(err['bearing']):+.1f} deg)")
                hold_start, hold_errs = None, []
                self.phase = 'approach'

            if hold_start is None and now - last_report > 3.0:
                last_report = now
                log.info(f"  range {err['range']:.2f} m  lateral {err['lateral']:+.2f} m  "
                         f"height {err['height']:+.2f} m  bearing {math.degrees(err['bearing']):+.1f} deg")
            self.command(*cmd, err=err)
            time.sleep(0.1)

        # Summary over the successful hold
        def stats(key, f=lambda v: v):
            vals = [f(e[key]) for e in hold_errs]
            return sum(vals) / len(vals), max(vals, key=abs)
        rm, rx = stats('range')
        lm, lx = stats('lateral')
        hm, hx = stats('height')
        bm, bx = stats('bearing', math.degrees)
        log.info(f'held {self.hold_time:.0f} s at {self.target_range:.2f} m '
                 f'(total approach {time.time() - start:.1f} s):')
        log.info(f'  range   mean {rm:.3f} m   worst {rx:.3f} m')
        log.info(f'  lateral mean {lm:+.3f} m   worst {lx:+.3f} m')
        log.info(f'  height  mean {hm:+.3f} m   worst {hx:+.3f} m')
        log.info(f'  bearing mean {bm:+.2f} deg worst {bx:+.2f} deg')

    def mission(self):
        log = self.get_logger()
        log.info(f'marker id {self.marker_id}, {self.marker_size} m; target {self.target_range} m, '
                 f'hold {self.hold_time} s')
        self.connect_and_arm()
        self._open_log()
        try:
            self.takeoff(self.takeoff_altitude)
            self.search()
            self.approach_and_hold()
            self.phase = 'land'
            self.land()
        finally:
            self._file.close()
        log.info(f'log: {self.log_path}')


def main():
    run_node(MarkerApproach)


if __name__ == '__main__':
    main()
