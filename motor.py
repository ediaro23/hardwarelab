import os
import can

os.system('sudo ifconfig can0 down')
os.system('sudo ip link set can0 type can bitrate 1000000')
os.system('sudo ifconfig can0 up')

can0 = can.interface.Bus(channel='can0', bustype= 'socketcan')
msg = can.Message(arbitration_id=0x141, data=[0xA1, 0x00, 0x00, 0x00, 0x9C, 0xFF, 0x00, 0x00])
can0.send(msg)
