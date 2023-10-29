

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
        print(os.name)
        os.system('sudo ifconfig can0 down')
        os.system('sudo ip link set can0 type can bitrate 1000000')
        os.system("sudo ifconfig can0 txqueuelen 100000")
        os.system('sudo ifconfig can0 up')
        self.pcan = PCANBus()
    
    def _sendAndReceive(self, aribitration_id, data):
        msg = can.Message(arbitration_id=aribitration_id, data=data, is_extended_id=False)
        self.pcan.send_message(msg)
        msg = self.pcan.read_input()
        return msg

    
    def _bytestointeger(self, msg, range_ind=(6,8), signed=True, byteorder="little"):
        return int.from_bytes(msg.data[range_ind[0]:range_ind[1]], byteorder=byteorder, signed=signed)
    
    def readAngle(self, motorid=1,duration=10, frequency=100):
        start = time.time()
        wid = WRITEID + motorid
        dataw=[0x92,0x00,0x00,0x00,0x00,0x00,0x00,0x00]

        if duration == -1:
            msg = self._sendAndReceive(wid, dataw)
            value  = int.from_bytes(msg.data[4:8], byteorder='little', signed=True)

        while(time.time() - start < duration):
                msg = self._sendAndReceive(wid, dataw)
                value  = int.from_bytes(msg.data[4:8], byteorder='little', signed=True)
                time.sleep(1/frequency)
        return value
    
    def readPID(self, motorid=1, duration=10):
        wid = WRITEID + motorid
        rid = READID + motorid
        dataw=[0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        msg = self._sendAndReceive(wid, dataw)
        kpCurrent, kiCurrent = msg.data[2], msg.data[3]
        KpVel, KiVel = msg.data[4], msg.data[5]
        KpPos, KiPos = msg.data[6], msg.data[7]
        data = {
            "current": (kpCurrent, kiCurrent),
            "velocity": (KpVel, KiVel),
            "position": (KpPos, KiPos)
        }
        return data

    def setPIDInRAM(self, motorid=1, data={}):
        wid = WRITEID + motorid
        rid = READID + motorid
        KpCurrent = KiCurrent = KpVel = KiVel = KpPos = KiPos = 0
        try:
            KpCurrent, KiCurrent = data["current"]
            KpVel, KiVel = data["velocity"]
            KpPos, KiPos = data["position"]
        except:
            print("invalid data. data should be a dictionary with keys: current, velocity, position and values: (Kp, Ki)")
            return
        dataw = [0x31,
                0x00,
                int(KpCurrent).to_bytes("little", 1),
                int(KiCurrent).to_bytes("little", 1),
                int(KpVel).to_bytes("little", 1),
                int(KiVel).to_bytes("little", 1),
                int(KpPos).to_bytes("little", 1),
                int(KiPos).to_bytes("little", 1)]
        msg = self._sendAndReceive(wid, dataw)
        return msg

    def setPIDInROM(self, motorid=1, data={}):
        wid = WRITEID + motorid
        rid = READID + motorid
        KpCurrent = KiCurrent = KpVel = KiVel = KpPos = KiPos = 0
        try:
            KpCurrent, KiCurrent = data["current"]
            KpVel, KiVel = data["velocity"]
            KpPos, KiPos = data["position"]
        except:
            print("invalid data. data should be a dictionary with keys: current, velocity, position and values: (Kp, Ki)")
            return
        dataw = [0x31,
                0x00,
                int(KpCurrent).to_bytes("little", 1),
                int(KiCurrent).to_bytes("little", 1),
                int(KpVel).to_bytes("little", 1),
                int(KiVel).to_bytes("little", 1),
                int(KpPos).to_bytes("little", 1),
                int(KiPos).to_bytes("little", 1)]
        msg = self._sendAndReceive(wid, dataw)
        return msg


    def applyTorqueToBothMotors(self, torque=0, duration=5):
        start = time.time()
        torque_bytes = torque.to_bytes(2, "little")
        print(f"{torque_bytes[0]} {torque_bytes[1]}")
        wid1 = WRITEID + 1
        rid1 = READID + 1
        wid2 = WRITEID + 2
        rid2 = READID + 2
        dataw = [0xA1,0x01,0x00,0x00,torque_bytes[0],torque_bytes[1],0x00,0x00]
        msg = can.Message(arbitration_id=wid1, data=dataw, is_extended_id=False)
        self.pcan.send_message(msg)
        msg = can.Message(arbitration_id=wid2, data=dataw, is_extended_id=False)
        self.pcan.send_message(msg)
        while (time.time() - start < duration):
            msg = self.pcan.read_input()
            time.sleep(0.01)
        dataw = [0xA1,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        msg = can.Message(arbitration_id=wid1, data=dataw, is_extended_id=False)
        self.pcan.send_message(msg)
        msg = pcan.read_input()
        msg = can.Message(arbitration_id=wid2, data=dataw, is_extended_id=False)
        self.pcan.send_message(msg)
        msg = self.pcan.read_input()
        return msg

    def setZero(self, motor_id=1):
        wid1 = WRITEID + motor_id
        rid1 = READID + motor_id
        dataw = [0x64,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        msg = can.Message(arbitration_id=wid1, data=dataw, is_extended_id=False)
        self.pcan.send_message(msg)
        msg = self.pcan.read_input()
        motor_offset = int.from_bytes(msg.data[4:8], "little")
        print(f"motor {motor_id} has been reset. offset: {motor_offset}. restart required.")
        return
    
    def applyTorqueToMotor(self, motorid=1, torque=18):
        start = time.time()
        torque_bytes = torque.to_bytes(2, "little", signed=True)
        #print(f"{torque_bytes[0]} {torque_bytes[1]}")
        wid1 = WRITEID + motorid
        rid1 = READID + motorid
        dataw = [0xA1,0x00,0x00,0x00,torque_bytes[0],torque_bytes[1],0x00,0x00]
        msg = can.Message(arbitration_id=wid1, data=dataw, is_extended_id=False)
        self.pcan.send_message(msg)
        msg = self.pcan.read_input()
        motor_temp = msg.data[1]
        current = int.from_bytes(msg.data[2:4], "little")
        speed = int.from_bytes(msg.data[4:6], "little")
        angle = int.from_bytes(msg.data[6:8], "little", signed=True)
        return motor_temp, current, speed, angle
    
    
    





def readPosition(pcan , motorid=1,duration = 10):
        start = time.time()
        wid = WRITEID + motorid
        rid = READID + motorid
        dataw=[0x90,0x00,0x00,0x00,0x9C,0xFF,0x00,0x00]
        while(time.time() - start < duration):
                print("seding")
                msg = can.Message(arbitration_id=wid, data=dataw, is_extended_id=False)
                pcan.send_message(msg)
                msg = pcan.read_input()
                print ("low byte", msg.data[2])
                print ("high byte", msg.data[3])
                value  = valuefromdata(msg.data,2)
                print("value", value)
                print("value2", valuefromdata2(msg.data,2,3))
                print (pcan.read_input())
        return msg
        

        

def readAngle(pcan , motorid=1,duration = 100):
        start = time.time()
        wid = WRITEID + motorid
        rid = READID + motorid
        dataw=[0x94,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        while(time.time() - start < duration):
                msg = can.Message(arbitration_id=wid, data=dataw, is_extended_id=False)
                pcan.send_message(msg)
                msg = pcan.read_input()
                value  = int.from_bytes(msg.data[6:8], byteorder='little', signed=True)
                print("value", value * 0.01)
                time.sleep(0.1)
        return msg
        



def singleTurnPositionControl(pcan, motorid=2, duration=1):
    start = time.time()
    wid = WRITEID + motorid
    rid = READID + motorid
    dataw=[0xA6,0x00,0x11,0x00,0x00,0x00,0x00,0x00]
    msg = can.Message(arbitration_id=wid, data=dataw, is_extended_id=False)
    pcan.send_message(msg)
    msg = pcan.read_input()
    while (time.time() - start < duration):
        msg = pcan.read_input()
        time.sleep(0.1)
    dataw = [0xA1,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
    msg = can.Message(arbitration_id=wid, data=dataw, is_extended_id=False)
    pcan.send_message(msg)
    msg = pcan.read_input()
    return msg





