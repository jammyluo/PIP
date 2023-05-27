r"""
    Xsens Dot GUI server process. Run this server separately with your client to reduce sensor crashes.
"""
import sys
sys.path.append("D:\\proj\\xplan\\PIP")


import torch
from queue import Empty
from articulate.utils.imu948 import IMU948DotSet
from articulate.utils.bullet.bullet import Button, Slider
from articulate.utils.bullet.view_rotation_np import RotationViewer
from pygame.time import Clock
import socket
import numpy as np


imus_addr = [
    'e2:62:8b:65:ac:fe', #A
    '88:b2:d0:3d:44:aa', #C
    '35:ec:33:e4:0a:59', #D
    'd1:31:63:f0:9c:96', #E 
    '2c:01:6e:17:62:3e', #H
    '5e:07:42:d7:e3:76', #I
]
# straight (x = Forward, y = Left, z = Up).y
addr = ('192.168.3.56', 8777)
ss = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

IMU948DotSet.set_buffer_len(2)
viewer = RotationViewer(n=len(imus_addr))
viewer.connect()
clock = Clock()

connect_btn = Button('connect', viewer.physics_client)
disconnect_btn = Button('disconnect', viewer.physics_client)
shutdown_btn = Button('shutdown', viewer.physics_client)
start_streaming_btn = Button('start streaming', viewer.physics_client)
stop_streaming_btn = Button('stop streaming', viewer.physics_client)
reset_heading_btn = Button('reset heading', viewer.physics_client)
revert_heading_btn = Button('revert heading', viewer.physics_client)
clear_btn = Button('clear', viewer.physics_client)
battery_btn = Button('battery info', viewer.physics_client)
quit_btn = Button('quit', viewer.physics_client)
fps_slider = Slider('fps', (1, 60), 60, viewer.physics_client)

while True:
    if connect_btn.is_click():
        IMU948DotSet.sync_connect(imus_addr)
    if disconnect_btn.is_click():
        IMU948DotSet.sync_disconnect()
    if shutdown_btn.is_click():
        IMU948DotSet.sync_shutdown()
    if start_streaming_btn.is_click():
        IMU948DotSet.start_streaming()
    if stop_streaming_btn.is_click():
        IMU948DotSet.stop_streaming()
    if reset_heading_btn.is_click():
        IMU948DotSet.reset_heading()
    if revert_heading_btn.is_click():
        IMU948DotSet.revert_heading_to_default()
    if clear_btn.is_click():
        IMU948DotSet.clear()
    if battery_btn.is_click():
        IMU948DotSet.print_battery_info()
    if quit_btn.is_click():
        IMU948DotSet.sync_disconnect()
        viewer.disconnect()
        break

    if IMU948DotSet.is_started():
        clock.tick(fps_slider.get_int())
        try:
            T, Q, A = [], [], []
            for i in range(len(imus_addr)):
                t, q, a = IMU948DotSet.get(i, timeout=1, preserve_last=True)
                T.append(t)
                Q.append(q)
                A.append(a)
                viewer.update(q.numpy()[[1, 2, 3, 0]], i)
            data = torch.cat((torch.tensor(T), torch.cat(Q), torch.cat(A)))
            data = data.numpy().astype(np.float32).tobytes()
            ss.sendto(data, addr)
            del data
        except Empty:
            print('[warning] read IMU error: Buffer for sensor %d is empty' % i)
