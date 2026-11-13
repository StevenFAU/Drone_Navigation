"""Find the ArUco marker, fly to 1 m in front of it, hold there for 5 s, then land.

Phases:
  search    yaw slowly in place until the forward camera sees the marker (max one full turn)
  approach  proportional control, streamed as velocity setpoints at 10 Hz:
              yaw rate    <- bearing to the marker      (keep it centred in the image)
              climb rate  <- height difference          (camera level with the marker)
              horizontal  <- offset from the spot `target_range` m out along the marker's
                             normal (closes range and lines up square to the marker)
            Within `fine_radius` of the hold spot it switches to position targets instead: the
            spot `target_range` out along the marker's normal, facing the marker, recomputed
            from the marker estimate every frame. ArduPilot's own position controller then
            rejects wind gusts, which the slow outer P loop alone could not (see README).
  hold      same control. The hold starts once every error is inside `entry_fraction` of
            its tolerance (settled), then counts while they stay inside the full tolerance with
            a fresh detection; leaving it restarts the hold.
  land      after `hold_time` s

    ros2 run drone_mission marker_approach
    ros2 run drone_mission marker_approach --ros-args -p target_range:=1.5 -p hold_time:=5.0

Writes logs/marker_<time>.csv (10 Hz: phase, errors, commands, estimate + Gazebo truth) and
.json (hold statistics).
"""
import math
import time

from .flight_base import FlightNode, MissionError, run_node
from .marker_tracker import MarkerTracker


def clamp(v, limit):
    return max(-limit, min(limit, v))


class MarkerApproach(FlightNode):
    def __init__(self, name='marker_approach'):
        super().__init__(name)
        self.setup_marker()

    def setup_marker(self):
        p = self.param
        self.marker_id = p('marker_id', 0)                    # -1 = any id
        self.marker_size = p('marker_size', 0.20)              # m, black border included
        self.takeoff_altitude = p('takeoff_altitude', 1.5)     # m (sim marker is 1.5 m up)
        self.target_range = max(0.5, p('target_range', 1.0))   # m in front of the marker
        self.hold_time = p('hold_time', 5.0)                   # s, continuously in tolerance
        # Tolerances while holding, and the fraction of them needed to start the hold
        self.range_tol = p('range_tol', 0.10)                  # m
        self.lateral_tol = p('lateral_tol', 0.15)              # m
        self.height_tol = p('height_tol', 0.10)                # m
        self.bearing_tol = math.radians(p('bearing_tol_deg', 5.0))
        self.entry_fraction = p('entry_fraction', 0.5)
        # Controller gains and limits
        self.k_pos = p('k_pos', 0.5)          # (m/s) per m
        self.k_z = p('k_z', 0.8)
        self.k_yaw = p('k_yaw', 1.0)          # (rad/s) per rad
        self.max_speed = p('max_speed', 0.5)  # m/s horizontal
        self.max_climb = p('max_climb', 0.3)  # m/s
        self.max_yaw_rate = math.radians(p('max_yaw_rate_deg', 30.0))
        self.search_rate = math.radians(p('search_rate_deg', 20.0))   # + = counter-clockwise
        self.lost_timeout = p('lost_timeout', 10.0)    # s without detection before giving up
        self.approach_timeout = p('approach_timeout', 90.0)
        # Switch to position targets within this distance (m) of the hold spot; back out at 2x.
        # 0 = proportional velocity control only.
        self.fine_radius = p('fine_radius', 0.3)
        # Camera (defaults match the sim; the real drone's driver may use other topics)
        image_topic = p('image_topic', '/camera/image')
        info_topic = p('camera_info_topic', '/camera/camera_info')
        cam_offset = p('camera_offset', [0.15, 0.0, -0.02])   # m, body FLU
        reliable = p('image_reliable', True)   # False if the camera driver is best-effort only
        self.cam_offset = cam_offset
        self.tracker = MarkerTracker(self, marker_id=self.marker_id, marker_size=self.marker_size,
                                     cam_offset=cam_offset, image_topic=image_topic,
                                     info_topic=info_topic, reliable=reliable)
        self._hold_alt = None

    def record(self, err):
        if err is None:
            self.rec.update(seen=int(self.tracker.seen_within(0.5)))
            return
        mx, my, mz = self.to_mission(*self.tracker.position)
        self.rec.update(marker_x=mx, marker_y=my, marker_z=mz)
        self.rec.update(range=err['range'], lateral=err['lateral'], height=err['height'],
                        bearing_deg=math.degrees(err['bearing']),
                        seen=int(self.tracker.seen_within(0.5)))

    def command(self, vx, vy, vz, yaw_rate, err=None):
        self.send_velocity(vx, vy, vz, yaw_rate)
        self.record(err)

    def steer(self, cmd, err):
        """cmd = (vx, vy, vz, yaw_rate) for velocity control, or None for a position target."""
        if cmd is None:
            x, y, z, yaw = self.hold_target()
            self.send_setpoint(x, y, z, yaw)
            tx, ty, _ = self.to_mission(x, y, z)
            self.rec.update(target_x=tx, target_y=ty)
            self.rec.update(ctrl='pos', cmd_vx=None, cmd_vy=None, cmd_vz=None, cmd_yaw_rate_deg=None)
            self.record(err)
        else:
            self.command(*cmd, err=err)
            self.rec['ctrl'] = 'vel'

    def hold_altitude_rate(self):
        return clamp(self.k_z * (self._hold_alt - self.position[2]), self.max_climb)

    # --- phases ---
    def search(self):
        """Yaw in place until the marker is seen; give up after one full turn."""
        log = self.get_logger()
        self.phase = 'search'
        self._hold_alt = self.position[2]
        t0 = time.time()
        if self.tracker.seen_within(1.0):
            log.info('marker already in view')
            self.summary['search'] = {'turned_deg': 0.0, 'time_s': 0.0}
            return
        log.info(f'searching: yawing at {math.degrees(self.search_rate):.0f} deg/s')
        turned, last_yaw = 0.0, self.yaw
        deadline = time.time() + (2 * math.pi / abs(self.search_rate) + 15.0) * self.timeout_scale
        while not self.tracker.seen_within(0.5):
            if turned > 2 * math.pi + 0.2 or time.time() > deadline:
                raise MissionError('marker not found after a full turn')
            self.command(0.0, 0.0, self.hold_altitude_rate(), self.search_rate)
            time.sleep(0.1)
            turned += abs(math.remainder(self.yaw - last_yaw, 2 * math.pi))
            last_yaw = self.yaw
        log.info(f'marker found after turning {math.degrees(turned):.0f} deg')
        self.summary['search'] = {'turned_deg': round(math.degrees(turned), 1),
                                  'time_s': round(time.time() - t0, 1)}

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

    def hold_target(self):
        """Drone position + yaw (local ENU) that puts the camera `target_range` out along the
        marker's normal, level with it. Yaw points at the marker centre from where the camera
        is now (not just along the normal), so a small sideways offset doesn't show up as a
        bearing error."""
        m, n = self.tracker.position, self.tracker.normal
        cam = self.tracker.camera_position()
        yaw = math.atan2(m[1] - cam[1], m[0] - cam[0])
        fx, _, fz = self.cam_offset   # camera forward / up offset from the drone centre
        x = m[0] + n[0] * self.target_range - math.cos(yaw) * fx
        y = m[1] + n[1] * self.target_range - math.sin(yaw) * fx
        return x, y, m[2] - fz, yaw

    def in_tolerance(self, err, fraction=1.0):
        return (abs(err['range_err']) < self.range_tol * fraction
                and abs(err['lateral']) < self.lateral_tol * fraction
                and abs(err['height']) < self.height_tol * fraction
                and abs(err['bearing']) < self.bearing_tol * fraction)

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
        restarts = 0
        fine = False
        last_report = 0.0
        while True:
            now = time.time()
            if now > deadline:
                raise MissionError('approach timed out')
            if not self.tracker.seen_within(self.lost_timeout * self.timeout_scale):
                raise MissionError(f'marker lost for {self.lost_timeout:.0f} s')

            err = self.tracker.errors(self.target_range)
            fresh = self.tracker.seen_within(0.5) and all(math.isfinite(v) for v in err.values())
            off = math.hypot(err['range_err'], err['lateral'])
            if self.fine_radius > 0:
                if not fine and fresh and off < self.fine_radius and abs(err['bearing']) < math.radians(15):
                    fine = True
                    log.info(f'within {self.fine_radius:.1f} m: holding with position targets')
                elif fine and off > 2 * self.fine_radius:
                    fine = False
                    log.info('drifted out: back to velocity control')
            if fine:
                cmd = None   # position target (keeps the last one while the marker is briefly lost)
            elif fresh:
                cmd = self.controller(err)
            else:
                # Lost for a moment: stop translating, keep facing where it was.
                cmd = (0.0, 0.0, 0.0, clamp(self.k_yaw * err['bearing'], self.max_yaw_rate))

            if hold_start is None:
                if fresh and self.in_tolerance(err, self.entry_fraction):
                    hold_start = now
                    self.phase = 'hold'
                    log.info(f'settled after {now - start:.1f} s; holding {self.hold_time:.0f} s')
            elif fresh and self.in_tolerance(err):
                pass
            else:
                restarts += 1
                log.info(f'left tolerance after {now - hold_start:.1f} s; restarting hold '
                         f"(range {err['range']:.2f} m, lateral {err['lateral']:+.2f} m, "
                         f"height {err['height']:+.2f} m, bearing {math.degrees(err['bearing']):+.1f} deg)")
                hold_start, hold_errs = None, []
                self.phase = 'approach'
            if hold_start is not None:
                hold_errs.append(err)
                if now - hold_start >= self.hold_time:
                    self.steer(cmd, err)
                    break

            if hold_start is None and now - last_report > 3.0:
                last_report = now
                log.info(f"  range {err['range']:.2f} m  lateral {err['lateral']:+.2f} m  "
                         f"height {err['height']:+.2f} m  bearing {math.degrees(err['bearing']):+.1f} deg")
            self.steer(cmd, err)
            time.sleep(0.1)

        # Summary over the successful hold (from the drone's own estimate; the experiment
        # tools recompute it from Gazebo's true pose)
        def stats(key, f=lambda v: v):
            vals = [f(e[key]) for e in hold_errs]
            return sum(vals) / len(vals), max(vals, key=abs)
        rm, rx = stats('range')
        lm, lx = stats('lateral')
        hm, hx = stats('height')
        bm, bx = stats('bearing', math.degrees)
        approach_s = hold_start - start
        log.info(f'held {self.hold_time:.0f} s at {self.target_range:.2f} m '
                 f'(settled after {approach_s:.1f} s):')
        log.info(f'  range   mean {rm:.3f} m   worst {rx:.3f} m')
        log.info(f'  lateral mean {lm:+.3f} m   worst {lx:+.3f} m')
        log.info(f'  height  mean {hm:+.3f} m   worst {hx:+.3f} m')
        log.info(f'  bearing mean {bm:+.2f} deg worst {bx:+.2f} deg')
        self.summary['hold'] = {
            'target_range': self.target_range, 'hold_time': self.hold_time,
            'fine_radius': self.fine_radius,
            'approach_s': round(approach_s, 1), 'restarts': restarts,
            'range_mean': round(rm, 4), 'range_worst': round(rx, 4),
            'lateral_mean': round(lm, 4), 'lateral_worst': round(lx, 4),
            'height_mean': round(hm, 4), 'height_worst': round(hx, 4),
            'bearing_mean_deg': round(bm, 3), 'bearing_worst_deg': round(bx, 3)}

    def mission(self):
        log = self.get_logger()
        log.info(f'marker id {self.marker_id}, {self.marker_size} m; target {self.target_range} m, '
                 f'hold {self.hold_time} s')
        self.connect_and_arm()
        self.start_recording('marker')
        self.takeoff(self.takeoff_altitude)
        self.search()
        self.approach_and_hold()
        self.land()


def main():
    run_node(MarkerApproach)


if __name__ == '__main__':
    main()
