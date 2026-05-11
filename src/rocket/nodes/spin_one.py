#!/usr/bin/env python3
"""Send velocity to ONE ODrive node only. Usage: python3 spin_one.py <node_id> <turns_per_sec>"""
import sys, struct, time
import can

CAN_INTERFACE = 'can0'
CMD_SET_AXIS_STATE = 0x07
CMD_SET_INPUT_VEL = 0x0D
CMD_CLEAR_ERRORS = 0x18
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

node_id = int(sys.argv[1])
vel = float(sys.argv[2])

bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan')

def send(cmd_id, data):
    arb = (node_id << 5) | cmd_id
    bus.send(can.Message(arbitration_id=arb, data=data, is_extended_id=False))

print(f'Arming node {node_id} only...')
send(CMD_CLEAR_ERRORS, b'')
send(CMD_SET_AXIS_STATE, struct.pack('<I', AXIS_STATE_CLOSED_LOOP_CONTROL) + b'\x00\x00\x00\x00')
time.sleep(0.5)

print(f'Commanding node {node_id} to {vel} turns/sec for 3 seconds. Watch which wheel moves.')
end = time.time() + 3.0
while time.time() < end:
    send(CMD_SET_INPUT_VEL, struct.pack('<ff', vel, 0.0))
    time.sleep(0.05)

print('Stopping.')
send(CMD_SET_INPUT_VEL, struct.pack('<ff', 0.0, 0.0))
send(CMD_SET_AXIS_STATE, struct.pack('<I', 1) + b'\x00\x00\x00\x00')  # idle
bus.shutdown()
# how to use
#python3 ~/rocket_ws/src/rocket/nodes/spin_one.py 58 2   # should be FR only
#python3 ~/rocket_ws/src/rocket/nodes/spin_one.py 59 2   # should be RR only
#python3 ~/rocket_ws/src/rocket/nodes/spin_one.py 60 2   # should be RL only
#python3 ~/rocket_ws/src/rocket/nodes/spin_one.py 61 2   # should be FL only
