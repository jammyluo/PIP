import numpy as np
import time
import socket
import torch
from pygame.time import Clock
from net import PIP
import articulate as art
import os
from config import *
from articulate.utils.imu948.imu948dc import *

IsBytes =  True
#IsBytes =  False

class IMUSet:
    def __init__(self):
        self.n_imus = 6
        self.cs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cs.bind(('0.0.0.0', 8777))
        self.cs.settimeout(3)
        self.last_data = None

    def get(self):
        try:
            if IsBytes :
                recv, server = self.cs.recvfrom(1024)
                pos = 0
                T, Q, A = [], [], []
                while pos < len(recv):
                    i = recv[pos]
                    pos += 1
                    l = recv[pos]
                    pos += 1
                    parsed = DeviceReportCharacteristic.from_bytes(recv[pos: pos+l]).payload
                    pos += l
                    T.append(parsed.timestamp.microseconds / 1e3)
                    Q.append(torch.tensor([parsed.quaternion.w, parsed.quaternion.x, parsed.quaternion.y, parsed.quaternion.z]))
                    A.append(torch.tensor([parsed.acceleration.x, parsed.acceleration.y, parsed.acceleration.z]))
                data = torch.cat((torch.tensor(T), torch.cat(Q), torch.cat(A)))
                data = data.numpy().astype(np.float32).tobytes()
                if len(recv)  < 6*23: # IMU数据缺失，用上次的数据
                    data = self.last_data
                    print('Miss IMU Data, recv:',len(recv))
                else :
                    self.last_data = data
            else: 
                data, server = self.cs.recvfrom(32 * self.n_imus)
            data = np.frombuffer(data, np.float32)
            t = torch.tensor(data[:self.n_imus])
            q = torch.tensor(data[self.n_imus:5 * self.n_imus]).view(self.n_imus, 4)
            a = torch.tensor(data[5 * self.n_imus:]).view(self.n_imus, 3)
            return t, q, a
        except socket.timeout:
            print('[warning] no imu data received for 3 seconds')

    def clear(self):
        while True:
            t1 = time.time()
            self.cs.recvfrom(int(32 * 6))
            self.cs.recvfrom(int(32 * 6))
            self.cs.recvfrom(int(32 * 6))
            t2 = time.time()
            if t2 - t1 > 2.5 / 60:
                break

def tpose_calibration(imu_set, conn):
    print('Used cached RMI? [y]/n    (If you choose no, put imu 1 straight (x = Forward, y = Left, z = Up).')
    if IsBytes :
        c = conn.recv(1024).decode('gbk')
    else :
        c = input('Used cached RMI? [y]/n    (If you choose no, put imu 1 straight (x = Forward, y = Left, z = Up).')

    if c == 'n' or c == 'N':
        imu_set.clear()
        RSI = art.math.quaternion_to_rotation_matrix(imu_set.get()[1][0]).view(3, 3).t()
        RMI = torch.tensor([[0, 1, 0], [0, 0, 1], [1, 0, 0.]]).mm(RSI)
        print('RMI Path %s', os.path.join(paths.temp_dir, 'RMI.pt') )
        torch.save(RMI, os.path.join(paths.temp_dir, 'RMI.pt'))
    else:
        RMI = torch.load(os.path.join(paths.temp_dir, 'RMI.pt'))
    print(RMI)
    print('Stand straight in T-pose and press enter. The calibration will begin in 3 seconds')
    if IsBytes :
        conn.recv(1024).decode('gbk')
    else :
        input('Stand straight in T-pose and press enter. The calibration will begin in 3 seconds')
        
    time.sleep(3)
    imu_set.clear()
    RIS = art.math.quaternion_to_rotation_matrix(imu_set.get()[1])
    RSB = RMI.matmul(RIS).transpose(1, 2).matmul(torch.eye(3))  # = (R_MI R_IS)^T R_MB = R_SB
    return RMI, RSB


def test_sensors(imu_set):
    from articulate.utils.bullet.view_rotation_np import RotationViewer
    clock = Clock()
    imu_set = IMUSet()
    with RotationViewer(6) as viewer:
        imu_set.clear()
        while True:
            clock.tick(63)
            t, q, a = imu_set.get()
            viewer.update_all(q[:, [1, 2, 3, 0]])
            print('time:', t, '\tacc:', a.norm(dim=1))


def task(conn, imu_set):
    RMI, RSB = tpose_calibration(imu_set,conn)
    net = PIP()
    clock = Clock()
    imu_set.clear()

    while True:
        clock.tick(63)
        tframe, q, a = imu_set.get()
        RMB = RMI.matmul(art.math.quaternion_to_rotation_matrix(q)).matmul(RSB)
        aM = a.mm(RMI.t())
        pose, tran, cj, grf = net.forward_frame(aM.view(1, 6, 3), RMB.view(1, 6, 3, 3), return_grf=True)
        pose = art.math.rotation_matrix_to_axis_angle(pose).view(-1, 72)
        tran = tran.view(-1, 3)
        # send motion to Unity
        s = ','.join(['%g' % v for v in pose.view(-1)]) + '#' + \
            ','.join(['%g' % v for v in tran.view(-1)]) + '#' + \
            ','.join(['%d' % v for v in cj]) + '#' + \
            (','.join(['%g' % v for v in grf.view(-1)]) if grf is not None else '') + '$'
        try:
            conn.send(s.encode('utf8'))
        except:
            print('Finish.\n')
            break
        print('\rfps: ', clock.get_fps(), end='')
        

if __name__ == '__main__':
    os.makedirs(paths.temp_dir, exist_ok=True)

    imu_set = IMUSet()

    is_executable = False
    server_for_unity = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_for_unity.bind(('0.0.0.0', 8888))
    server_for_unity.listen(1)
    print('Server start. ')
    conn, addr = server_for_unity.accept()
    task(conn,imu_set)
    while True:
        print('\nWaiting for unity3d to connect.')
        conn, addr = server_for_unity.accept()
        print('\nunity3d connect.')
        task(conn,imu_set)
        print('\rFinish.')
        # thread_task=threading.Thread(target=task, args=(conn, imu_set))
        # thread_task.setDaemon(True)
        # thread_task.start()

    

