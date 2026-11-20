#!/usr/bin/env python3
"""Read-only: print RC channels 5-8 and flight mode from MAVROS; one EVENT line per change.
RC comes from /mavros/rc/in, or (ELRS receiver in MAVLink mode, where /mavros/rc/in stays empty)
from the receiver's RC_CHANNELS_OVERRIDE messages seen on /uas1/mavlink_source."""
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import Mavlink, RCIn, State


def ekf_set(v):
    return 'set 1 (GPS)' if v < 1300 else ('set 2 (flow)' if v <= 1700 else 'set 3 (flow)')


class Watch(Node):
    def __init__(self):
        super().__init__('rc_watch')
        self.ch, self.st, self.last = None, None, {}
        self.create_subscription(RCIn, '/mavros/rc/in', self.on_rc, qos_profile_sensor_data)
        self.create_subscription(Mavlink, '/uas1/mavlink_source', self.on_mavlink, qos_profile_sensor_data)
        self.create_subscription(State, '/mavros/state', self.on_state, qos_profile_sensor_data)
        self.create_timer(1.0, self.tick)

    def chs(self):
        c = self.ch or []
        g = lambda i: c[i] if len(c) > i else '-'
        return f'ch5 {g(4)}  ch6 {g(5)}  ch7 {g(6)}'

    def on_rc(self, m):
        if m.channels:
            self.ch = list(m.channels)
            self.changed()

    def on_mavlink(self, m):
        if m.msgid != 70:          # RC_CHANNELS_OVERRIDE: u16 chan1..8 first
            return
        raw = b''.join(struct.pack('<Q', w) for w in m.payload64) + bytes(16)
        self.ch = list(struct.unpack_from('<8H', raw, 0))
        self.changed()

    def on_state(self, m):
        self.st = m
        self.changed()

    def changed(self):
        cur = {}
        if self.ch and len(self.ch) >= 7:
            cur['ekf'] = ekf_set(self.ch[5])
        for i, v in enumerate((self.ch or [])[4:12], start=5):
            cur[f'ch{i}'] = 'low' if v < 1300 else ('mid' if v <= 1700 else 'high')
        if self.st:
            cur['mode'] = self.st.mode
            cur['armed'] = self.st.armed
        for k in [k for k in cur if k.startswith('ch')] + ['ekf', 'mode', 'armed']:
            if k in cur and self.last.get(k) != cur[k]:
                mode = self.st.mode if self.st else '-'
                print(f'EVENT {k}={cur[k]}  | {self.chs()}  mode {mode}', flush=True)
        self.last.update(cur)

    def tick(self):
        if not self.ch:
            print('waiting for RC data (radio off?)', flush=True)


def main():
    rclpy.init()
    rclpy.spin(Watch())


if __name__ == '__main__':
    main()
