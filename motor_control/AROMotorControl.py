

import os
import can
import time
import logging

WRITEID = 0x140
READID  = 0x240

"""
PCANBus class is a wrapper for the python-can library. It is used to send and receive messages from the CANbus.
The class is initialized with the can0 channel and a baudrate of 1Mbit/s. The class also has a buffer which is used to store incoming messages.
"""
class PCANBus(object):
    def __init__(self):
        logging.info("Initializing CANbus")

        self.bus = can.Bus(channel="can0", bustype="socketcan", baudrate=1000000)
        self.buffer = can.BufferedReader()
        self.notifier = can.Notifier(self.bus, [self._get_message, self.buffer])

    def send_message(self, message):
        try:
            self.bus.send(message)
            return True
        except can.CanError:
            logging.error("message not sent!")
            return False

    def read_input(self):
        return self.buffer.get_message()

    def flush_buffer(self):
        msg = self.buffer.get_message()
        while (msg is not None):
            msg = self.buffer.get_message()

    def cleanup(self):
        self.notifier.stop()
        self.bus.shutdown()

    def disable_update(self):
        pass
    
    def _get_message(self, msg):
        return msg
    

"""
AROMotorControl class is used to control the motors.
This class provides all the methods to implement a closed loop control of the motors.
"""
class AROMotorControl():
    
    def __init__(self):
        # os.system('sudo ifconfig can0 down')
        # os.system('sudo ip link set can0 type can bitrate 1000000')
        # os.system("sudo ifconfig can0 txqueuelen 100000")
        # os.system('sudo ifconfig can0 up')
        self.pcan = PCANBus()
        self.current_command_last_msg = (0, 0, 0, 0)
    
    def _sendAndReceive(self, aribitration_id, data):
        msg = can.Message(arbitration_id=aribitration_id, data=data, is_extended_id=False)
        self.pcan.send_message(msg)
        msg = self.pcan.read_input()
        return msg
    
    def _bytestointeger(self, msg, range_ind=(6,8), signed=True, byteorder="little"):
        return int.from_bytes(msg.data[range_ind[0]:range_ind[1]], byteorder=byteorder, signed=signed)
    
    def readPosition(self, motorid=1):
        start = time.time()
        wid = WRITEID + motorid
        dataw=[0x92,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        msg = self._sendAndReceive(wid, dataw)
        value  = (int.from_bytes(msg.data[4:8], byteorder='little', signed=True) / 100) % 360
        return value
    
    def readPositionContinuous(self, frequency=100):
        start = time.time()
        wid1 = WRITEID + 1
        wid2 = WRITEID + 2
        dataw=[0x92,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        while True:
                msg = self._sendAndReceive(wid1, dataw)
                angle1  = (int.from_bytes(msg.data[4:8], byteorder='little', signed=True) / 100) % 360
                msg = self._sendAndReceive(wid2, dataw)
                angle2  = (int.from_bytes(msg.data[4:8], byteorder='little', signed=True) / 100) % 360
                yield (angle1, angle2)
                frequency = frequency if frequency > 0 else 0.001
                time.sleep(1/frequency)
        return value
    
    def readPID(self, motorid=1):
        wid = WRITEID + motorid
        rid = READID + motorid
        dataw=[0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        msg = self._sendAndReceive(wid, dataw)
        kpCurrent, kiCurrent = msg.data[2], msg.data[3]
        KpVel, KiVel = msg.data[4], msg.data[5]
        KpPos, KiPos = msg.data[6], msg.data[7]
        return kpCurrent, kiCurrent, KpVel, KiVel, KpPos, KiPos

    def setPIDInROM(self, motorid=1, KpCurrent=0, KiCurrent=0, KpVel=0, KiVel=0, KpPos=0, KiPos=0):
        try:
            KpCurrent = int(KpCurrent)
            KiCurrent = int(KiCurrent)
            KpVel = int(KpVel)
            KiVel = int(KiVel)
            KpPos = int(KpPos)
            KiPos = int(KiPos)
        except ValueError:
            print("Cant convert given gains into integers")
            return False  

        wid = WRITEID + motorid
        rid = READID + motorid
        dataw = [0x31,
                0x00,
                KpCurrent.to_bytes(1, "little")[0],
                KiCurrent.to_bytes(1, "little")[0],
                KpVel.to_bytes(1, "little")[0],
                KiVel.to_bytes(1, "little")[0],
                KpPos.to_bytes(1, "little")[0],
                KiPos.to_bytes(1, "little")[0]]
        msg = self._sendAndReceive(wid, dataw)
        return True


    def setPIDInRAM(self, motorid=1, KpCurrent=0, KiCurrent=0, KpVel=0, KiVel=0, KpPos=0, KiPos=0):
        try:
            KpCurrent = int(KpCurrent)
            KiCurrent = int(KiCurrent)
            KpVel = int(KpVel)
            KiVel = int(KiVel)
            KpPos = int(KpPos)
            KiPos = int(KiPos)
        except ValueError:
            print("Cant convert given gains into integers")
            return False  
        
        wid = WRITEID + motorid
        rid = READID + motorid
        dataw = [0x31,
                0x00,
                KpCurrent.to_bytes(1, "little")[0],
                KiCurrent.to_bytes(1, "little")[0],
                KpVel.to_bytes(1, "little")[0],
                KiVel.to_bytes(1, "little")[0],
                KpPos.to_bytes(1, "little")[0],
                KiPos.to_bytes(1, "little")[0]]
        msg = self._sendAndReceive(wid, dataw)
        return True
    
    def positionControl(self, motorid=1, rotation_dir=0, speed_limit=0, position=0, run_async=True, error_threshold=0.1):
        wid = WRITEID + motorid
        rid = READID + motorid
        rotation_dir_bytes = rotation_dir.to_bytes(1, "little")
        speed_limit_bytes = speed_limit.to_bytes(2, "little")
        position_bytes = position.to_bytes(2, "little", signed=True)
        dataw = [0xA6,
                rotation_dir_bytes[0],
                speed_limit_bytes[0],
                speed_limit_bytes[1],
                position_bytes[0],
                position_bytes[1],
                0x00,
                0x00]
        if not run_async:
            current_position = self.readPosition(motorid=motorid)
            while abs(current_position - position) > error_threshold:
                msg = self._sendAndReceive(wid, dataw)
                motor_temp, torque, speed, current_position = msg.data[1], int.from_bytes(msg.data[2:4], "little"), int.from_bytes(msg.data[4:6], "little"), int.from_bytes(msg.data[6:8], "little", signed=True)
                time.sleep(0.01)
        else:
            msg = self._sendAndReceive(wid, dataw)
            motor_temp, torque, speed, current_position = msg.data[1], int.from_bytes(msg.data[2:4], "little"), int.from_bytes(msg.data[4:6], "little"), int.from_bytes(msg.data[6:8], "little", signed=True)
        return motor_temp, torque, speed, current_position
        
    def setZero(self, motor_id=1):
        wid1 = WRITEID + motor_id
        rid1 = READID + motor_id
        dataw = [0x64,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        msg = can.Message(arbitration_id=wid1, data=dataw, is_extended_id=False)
        self.pcan.send_message(msg)
        msg = self.pcan.read_input()
        motor_offset = int.from_bytes(msg.data[4:8], "little")
        print(f"motor {motor_id} has been reset. offset: {motor_offset}. restart required.")
        return True
    
    def applyCurrentToMotor(self, motorid=1, current=0.18):
        current = max(min(current, 0.5), -0.5)
        current = int(current * 100)
        start = time.time()
        torque_bytes = current.to_bytes(2, "little", signed=True)
        #print(f"{torque_bytes[0]} {torque_bytes[1]}")
        wid1 = WRITEID + motorid
        rid1 = READID + motorid
        dataw = [0xA1,0x00,0x00,0x00,torque_bytes[0],torque_bytes[1],0x00,0x00]
        msg = can.Message(arbitration_id=wid1, data=dataw, is_extended_id=False)
        self.pcan.send_message(msg)
        msg = self.pcan.read_input()
        try:
            motor_temp = msg.data[1]
            current = int.from_bytes(msg.data[2:4], "little")
            speed = int.from_bytes(msg.data[4:6], "little")
            angle = int.from_bytes(msg.data[6:8], "little", signed=True) % 360
            self.current_command_last_msg = (motor_temp, current, speed, angle)
        except:
            print("no message in the buffer")
            return self.current_command_last_msg
        return motor_temp, current, speed, angle
    
  




