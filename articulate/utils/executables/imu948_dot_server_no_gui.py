r"""
    Xsens Dot cmd server process. Run this server separately with your client to reduce sensor crashes.
"""
import time
import sys
sys.path.append("D:\\proj\\xplan\\PIP")

import torch
from queue import Empty
from articulate.utils.imu948 import IMU948DotSet
from pygame.time import Clock
import socket
import numpy as np
import keyboard
import articulate as art
from articulate.utils.print import *


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
IMU948DotSet.set_buffer_len(2)
clock = Clock()
is_hotkey_locked = True
fps = 60

helps = r'''
================= Hotkey =================
You need to unlock the hotkeys first.

l / shift + l       lock / unlock all hotkeys
h                   print this help
c / shift + c       connect / disconnect
s / shift + s       start / stop streaming
a / shift + a       reset / revert heading
r                   clear buffer
b                   print battery info
o                   power off
d                   print sensor angle
esc                 disconnect and quit
1 ~ 6               set fps to 10 ~ 60 (default 60)
'''
print(helps)

while True:
    if is_hotkey_locked:
        if keyboard.is_pressed('shift+l'):
            is_hotkey_locked = False
            print_green('hotkey unlocked')
            time.sleep(0.5)
    else:
        if keyboard.is_pressed('l'):
            is_hotkey_locked = True
            print_green('hotkey locked')
            time.sleep(0.5)
        if keyboard.is_pressed('h'):
            print(helps)
            time.sleep(0.5)
        if keyboard.is_pressed('shift+c'):
            IMU948DotSet.sync_disconnect()
            print_green('sensor disconnected')
        elif keyboard.is_pressed('c'):
            IMU948DotSet.sync_connect(imus_addr)
            print_green('sensor connected')
        if keyboard.is_pressed('o'):
            IMU948DotSet.sync_shutdown()
            print_green('sensor powered off')
        if keyboard.is_pressed('shift+s'):
            IMU948DotSet.stop_streaming()
            print_green('streaming stopped')
        elif keyboard.is_pressed('s'):
            IMU948DotSet.start_streaming()
            print_green('streaming started')
        if keyboard.is_pressed('shift+a'):
            IMU948DotSet.revert_heading_to_default()
            print_green('heading reverted')
        elif keyboard.is_pressed('a'):
            IMU948DotSet.reset_heading()
            print_green('heading reset')
        if keyboard.is_pressed('r'):
            IMU948DotSet.clear()
            print_green('buffer cleared')
        if keyboard.is_pressed('b'):
            IMU948DotSet.print_battery_info()
        if keyboard.is_pressed('esc'):
            IMU948DotSet.sync_disconnect()
            break
        if keyboard.is_pressed('1'):
            fps = 10
            print_green('fps set to 10')
            time.sleep(0.5)
        if keyboard.is_pressed('2'):
            fps = 20
            print_green('fps set to 20')
            time.sleep(0.5)
        if keyboard.is_pressed('3'):
            fps = 30
            print_green('fps set to 30')
            time.sleep(0.5)
        if keyboard.is_pressed('4'):
            fps = 40
            print_green('fps set to 40')
            time.sleep(0.5)
        if keyboard.is_pressed('5'):
            fps = 50
            print_green('fps set to 50')
            time.sleep(0.5)
        if keyboard.is_pressed('6'):
            fps = 60
            print_green('fps set to 60')
            time.sleep(0.5)

    clock.tick(fps)
    if IMU948DotSet.is_started():
        try:
            T, Q, A = [], [], []
            for i in range(len(imus_addr)):
                t, q, a = IMU948DotSet.get(i, timeout=1, preserve_last=True)
                T.append(t)
                Q.append(q)
                A.append(a)
            if not is_hotkey_locked and keyboard.is_pressed('d'):
                dq = ['%.1f' % art.math.radian_to_degree(art.math.angle_between(Q[0], q, art.math.RotationRepresentation.QUATERNION)) for q in Q]
                print('angle (deg):', dq)
            data = torch.cat((torch.tensor(T), torch.cat(Q), torch.cat(A)))
            data = data.numpy().astype(np.float32).tobytes()
            ss.sendto(data, addr)
            del data
        except Empty:
            print('[warning] read IMU error: Buffer for sensor %d is empty' % i)
