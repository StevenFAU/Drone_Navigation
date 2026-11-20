#!/usr/bin/env python3
"""Read-only, runs on the Pi (mission container): stream RC channels 1-8 at 5 Hz plus FC status texts.
Lines: 'RC <age_s> c1 c2 ... c8' (age = seconds since the last RC message; channels from the ELRS
receiver's RC_CHANNELS_OVERRIDE, or /mavros/rc/in if that has data) and 'TEXT <severity> <text>'.
"""
import struct
import time

import rclpy
from mavros_msgs.msg import Mavlink, RCIn, StatusText, State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class Stream(Node):
    def __init__(self):
        super().__init__('rc_stream')
        self.ch, self.t = [0] * 8, 0.0
        self.mode, self.armed = '-', False
        q = qos_profile_sensor_data
        self.create_subscription(Mavlink, '/uas1/mavlink_source', self.on_mav, q)
        self.create_subscription(RCIn, '/mavros/rc/in', self.on_rc, q)
        self.create_subscription(StatusText, '/mavros/statustext/recv', self.on_text, 10)
        self.create_subscription(State, '/mavros/state', self.on_state, q)
        self.create_timer(0.2, self.tick)

    def on_mav(self, m):
        if m.msgid == 70:
            raw = b''.join(struct.pack('<Q', w) for w in m.payload64) + bytes(16)
            self.ch, self.t = list(struct.unpack_from('<8H', raw, 0)), time.time()

    def on_rc(self, m):
        if m.channels:
            self.ch, self.t = list(m.channels[:8]), time.time()

    def on_text(self, m):
        print(f'TEXT {m.severity} {m.text}', flush=True)

    def on_state(self, m):
        self.mode, self.armed = m.mode, m.armed

    def tick(self):
        age = time.time() - self.t if self.t else 999.0
        print(f'RC {age:.1f} ' + ' '.join(str(c) for c in self.ch) + f' {self.mode} {int(self.armed)}', flush=True)


def main():
    rclpy.init()
    rclpy.spin(Stream())


if __name__ == '__main__':
    main()
