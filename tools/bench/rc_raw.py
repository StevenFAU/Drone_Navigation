#!/usr/bin/env python3
"""Read-only: print RC channel values from raw MAVLink (RC_CHANNELS 65 / RC_CHANNELS_OVERRIDE 70).
An ELRS receiver in MAVLink mode sends RC as overrides, which ArduPilot's RC_CHANNELS reports
with chancount 0, so /mavros/rc/in stays empty. Run in the mission container:
    python3 /logs/rc_raw.py [seconds]
"""
import struct
import sys
import time

import rclpy
from mavros_msgs.msg import Mavlink
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def payload(msg):
    raw = b''.join(struct.pack('<Q', w) for w in msg.payload64)
    return raw + bytes(64)


class Raw(Node):
    def __init__(self):
        super().__init__('rc_raw')
        self.last = {}
        self.create_subscription(Mavlink, '/uas1/mavlink_source', self.on_msg, qos_profile_sensor_data)

    def on_msg(self, m):
        p = payload(m)
        if m.msgid == 65:      # RC_CHANNELS: u32 time, u16 chan1..18, u8 chancount, u8 rssi
            ch = struct.unpack_from('<18H', p, 4)
            self.last['RC_CHANNELS'] = (m.sysid, ch[:8], p[40])
        elif m.msgid == 70:    # RC_CHANNELS_OVERRIDE: u16 chan1..8, u8 target sys, u8 target comp
            ch = struct.unpack_from('<8H', p, 0)
            self.last['OVERRIDE'] = (m.sysid, ch, None)


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 1e9
    rclpy.init()
    n = Raw()
    end, nxt = time.time() + secs, 0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() >= nxt:
            nxt = time.time() + 1.0
            for k, (sysid, ch, cnt) in n.last.items():
                print(f'{k} from sys {sysid}: ' + ' '.join(f'ch{i + 1}={v}' for i, v in enumerate(ch))
                      + (f'  chancount={cnt}' if cnt is not None else ''), flush=True)


if __name__ == '__main__':
    main()
