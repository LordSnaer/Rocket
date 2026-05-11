#!/usr/bin/env python3
"""Reset both rplidars before the driver starts.

Sends the rplidar STOP command (stops scanning) and pulls DTR high
(A2M12 motor-off) on every CP2102 USB-UART matching the rplidars.
This recovers lidars from a prior bad shutdown — motor still spinning,
device firmware wedged — without needing sudo / USB unbind.

If the device is held by a leftover rplidar_node, that process is killed
first and the open is retried. Idempotent and safe to run when nothing
is plugged in (just prints "no devices").
"""
import glob
import subprocess
import time

import serial

LIDAR_GLOB = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_*'
RPLIDAR_STOP = b'\xA5\x25'


def reset_one(path):
    for attempt in range(2):
        try:
            with serial.Serial(path, 256000, timeout=0.2) as s:
                s.dtr = True            # A2M12: DTR high stops the motor
                s.write(RPLIDAR_STOP)   # stop scanning
                time.sleep(0.1)
            print(f'[reset_lidars] {path}: ok')
            return
        except (serial.SerialException, OSError) as e:
            msg = str(e).lower()
            if attempt == 0 and ('busy' in msg or 'in use' in msg):
                print(f'[reset_lidars] {path}: busy — killing leftover rplidar_node and retrying')
                subprocess.run(['pkill', '-9', '-f', 'rplidar_node'], check=False)
                time.sleep(0.5)
                continue
            print(f'[reset_lidars] {path}: {e}')
            return


def main():
    devices = sorted(glob.glob(LIDAR_GLOB))
    if not devices:
        print('[reset_lidars] no rplidar devices found — skipping')
        return
    for d in devices:
        reset_one(d)


if __name__ == '__main__':
    main()
