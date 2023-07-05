r"""
    Xsens Dot cmd server process. Run this server separately with your client to reduce sensor crashes.
"""
import time
import sys
sys.path.append("/Users/jammy/code/xplan/PIP")

import torch
from queue import Empty
from articulate.utils.imu948 import IMU948DotSet
from pygame.time import Clock
import socket
import numpy as np
# import keyboard
import articulate as art
from articulate.utils.print import *


imus_addr_mac = [
    'C09A717C-7830-BBF4-C294-3079E86778FC', #im948_A-V3.02
    '3670CCDC-D943-D007-608E-946A2C7F1667', #im948_B-V3.02
    '4C0E565D-5456-3743-21C0-35837B2E8044', #im948_C-V3.02
    '3165FEBF-F328-4820-B901-C1DF260F3BED', #im948_D-V3.02
    '4396544E-E3E6-4BB0-D0FF-DC860D1C387D', #im948_E-V3.02
    'EACA0B53-360C-6E61-1796-A29AE1D573A1', #im948_F-V3.02·
]

imus_addr = [
    'e2:62:8b:65:ac:fe', #A
    '88:b2:d0:3d:44:aa', #C
    '35:ec:33:e4:0a:59', #D
    'd1:31:63:f0:9c:96', #E 
    '2c:01:6e:17:62:3e', #H
    '5e:07:42:d7:e3:76', #I
]

addr = ('183.47.110.142', 8777)
ss = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
IMU948DotSet.set_buffer_len(5)
clock = Clock()

IMU948DotSet.sync_connect(imus_addr_mac)
IMU948DotSet.reset_xyz()
IMU948DotSet.start_streaming()

while True:
    if IMU948DotSet.is_started():
        try:
            clock.tick(0)
            b = bytearray()
            for i in range(len(imus_addr_mac)):
                bytes_data = IMU948DotSet.get_bytes_msg(i,timeout=1)
                b.append(i)
                b.append(len(bytes_data))
                b.extend(bytes_data)
            ss.sendto(b, addr)
            del b
            print('\rfps: ', clock.get_fps(), end='')

        except Empty:
            print('[warning] read IMU error: Buffer for sensor %d is empty' % i)
