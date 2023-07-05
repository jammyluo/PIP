r"""
    Wrapper for an IMU948 dot set.
"""


__all__ = ['IMU948DotSet']


import asyncio
import time
from .imu948dc import *
from queue import Queue
import torch
from pygame.time import Clock


_N = 10  # max 10 IMUs


class IMU948DotSet:
    # _lock = [threading.Lock() for _ in range(_N)]   # lists are thread-safe
    _SZ = 180    # max queue size
    _loop = None
    _buffer = [Queue(180) for _ in range(_N)]
    _is_connected = False
    _is_started = False
    _pending_event = None
    dots = []
    clock = Clock()

    @staticmethod
    def _on_device_report(message_id, message_bytes, sensor_id=-1):
        parsed = DeviceReportCharacteristic.from_bytes(message_bytes)
        if (parsed.TAG == 0x10):
            IMU948DotSet.dots[sensor_id].status = parsed.status
        if (parsed.TAG == 0x11):
            q = IMU948DotSet._buffer[sensor_id]
            if q.full():
                q.get()
            # q.put(parsed.payload)
            q.put(message_bytes)
            
        
        # print('\rreport fps: ', IMU948DotSet.clock.get_fps(), end='')
        

    @staticmethod
    async def _multiple_sensor(devices: list):
        # Please use xsens dot app to synchronize the sensors first.
        # do not use asyncio.gather() and run these in parallel, it has bugs
        from functools import partial

        # ascan_all(10)
        print('finding devices ...')
        for i, d in enumerate(devices):
            while True:
                try:
                    device = await afind_by_address(d, timeout=30)
                    if device is None:
                        print('\t[%d]' % i, 'device not detected')
                    else:
                        break
                except Exception as e:
                    print('\t[%d]' % i, e)
            IMU948DotSet.dots.append(Dot(device))
            print('\t[%d]' % i, device)

        print('connecting ...')
        for i, d in enumerate(IMU948DotSet.dots):
            while True:
                try:
                    await d.aconnect(timeout=8)
                    break
                except Exception as e:
                    print('\t[%d]' % i, e)
            print('\t[%d] connected' % i)

        print('keep_live ...')
        for i, d in enumerate(IMU948DotSet.dots):
            try:
                await d.akeep_live()
            except Exception as e:
                print('\t[%d]' % i, e)
            print('\t[%d] keep_live' % i)

        # print('stop_reporting ...')
        # for i, d in enumerate(IMU948DotSet.dots):
        #     try:
        #         await d.astop_reporting()
        #     except Exception as e:
        #         print('\t[%d]' % i, e)
        #     print('\t[%d] stop_reporting' % i)
        print('configuring the sensors ...')
        for i, d in enumerate(IMU948DotSet.dots):
            # 参数设置
            isCompassOn = 1 #使用磁场融合姿态
            barometerFilter = 2
            Cmd_ReportTag = 0x0021 # 功能订阅标识
            # Cmd_ReportTag = 0x0FFF # 功能订阅标识
            params = bytearray([0x00 for i in range(0,11)])
            params[0] = 0x12
            params[1] = 5       #静止状态加速度阀值
            params[2] = 255     #静止归零速度(单位cm/s) 0:不归零 255:立即归零
            params[3] = 0       #动态归零速度(单位cm/s) 0:不归零
            params[4] = ((barometerFilter&3)<<1) | (isCompassOn&1);   
            params[5] = 60      #数据主动上报的传输帧率[取值0-250HZ], 0表示0.5HZ
            params[6] = 2       #陀螺仪滤波系数[取值0-2],数值越大越平稳但实时性越差
            params[7] = 3       #加速计滤波系数[取值0-4],数值越大越平稳但实时性越差
            params[8] = 8       #磁力计滤波系数[取值0-9],数值越大越平稳但实时性越差
            params[9] = Cmd_ReportTag&0xff
            params[10] = (Cmd_ReportTag>>8)&0xff
            await d.aset_params(params)
            print('set_param:',i)

        print('start_notify ...')
        for i, d in enumerate(IMU948DotSet.dots):
            try:
                await d.adevice_report_start_notify(partial(IMU948DotSet._on_device_report, sensor_id=i))
            except Exception as e:
                print('\t[%d]' % i, e)
            print('\t[%d] start_notify' % i)
            
        # print('reading device infos ...')
        # for i, d in enumerate(IMU948DotSet.dots):
        #     await d.adevice_info_read()
        # await asyncio.sleep(1)
        # for i, d in enumerate(IMU948DotSet.dots):
        #     print('\t[%d] %s' % (i, d.status))

        print('sensors connected')
        IMU948DotSet._is_connected = True

        while True:
            if IMU948DotSet._pending_event == 0:   # close
                shutdown = False
                break
            elif IMU948DotSet._pending_event == 1:   # shutdown
                shutdown = True
                break
            elif IMU948DotSet._pending_event == 2:   # reset xyz
                print('reset_xyz ...')
                for i, d in enumerate(IMU948DotSet.dots):
                    try:
                        await d.areset_xyz()
                    except Exception as e:
                        print('\t[%d]' % i, e)
                    print('\t[%d] reset_xyz' % i)
            elif IMU948DotSet._pending_event == 3:   # revert heading
                print('revert heading ...')
                for i, d in enumerate(IMU948DotSet.dots):
                    await d.arevert_heading_to_default()
                print('\theading is reverted to default')
            elif IMU948DotSet._pending_event == 4:   # start streaming
                print('start streaming ...')
                for i, d in enumerate(IMU948DotSet.dots):
                    await d.astart_reporting()
                    await asyncio.sleep(0.2)
                IMU948DotSet._is_started = True
                print('\tstreaming started')
            elif IMU948DotSet._pending_event == 5:  # stop streaming
                print('stop streaming ...')
                for i, d in enumerate(IMU948DotSet.dots):
                    await d.astop_reporting()
                IMU948DotSet._is_started = False
                print('\tstreaming stopped')
            elif IMU948DotSet._pending_event == 6:  # print battery
                print('reading battery infos ...')
                for i, d in enumerate(IMU948DotSet.dots):
                    info = await d.abattery_read()
                    print('\t[%d] %d%%' % (i, info.battery_level))

            IMU948DotSet._pending_event = None
            await asyncio.sleep(1)

        print('disconnecting ...')
        for i, d in enumerate(IMU948DotSet.dots):
            await d.astop_reporting()
            await d.adevice_report_stop_notify()
            if shutdown:
                await d.apower_off()
                print('\t[%d] power off' % i)
            else:
                await d.adisconnect()
                print('\t[%d] disconnected' % i)

        IMU948DotSet._is_started = False
        IMU948DotSet._is_connected = False
        IMU948DotSet._pending_event = None

    @staticmethod
    def _run_in_new_thread(coro):
        r"""
        Similar to `asyncio.run()`, but create a new thread.
        """
        def start_loop(_loop):
            asyncio.set_event_loop(_loop)
            _loop.run_forever()

        if IMU948DotSet._loop is None:
            import threading
            IMU948DotSet._loop = asyncio.new_event_loop()
            thread = threading.Thread(target=start_loop, args=(IMU948DotSet._loop,))
            thread.setDaemon(True)
            thread.start()

        asyncio.run_coroutine_threadsafe(coro, IMU948DotSet._loop)

    @staticmethod
    def _wait_for_pending_event():
        while IMU948DotSet._pending_event is not None:
            time.sleep(0.3)

    @staticmethod
    def clear(i=-1):
        r"""
        Clear the cached measurements of the ith IMU. If i < 0, clear all IMUs.

        :param i: The index of the query sensor. If negative, clear all IMUs.
        """
        if i < 0:
            IMU948DotSet._buffer = [Queue(IMU948DotSet._SZ) for _ in range(_N)]  # max 10 IMUs
            
        else:
            IMU948DotSet._buffer[i] = Queue(IMU948DotSet._SZ)

    @staticmethod
    def is_started() -> bool:
        r"""
        Whether the sensors are started.
        """
        return IMU948DotSet._is_started

    @staticmethod
    def is_connected() -> bool:
        r"""
        Whether the sensors are connected.
        """
        return IMU948DotSet._is_connected

    @staticmethod
    def get_bytes_msg(i: int, timeout=None, preserve_last=False):
        r"""
        Get the next measurements of the ith IMU. May be blocked.

        :param i: The index of the query sensor.
        :param timeout: If non-negative, block at most timeout seconds and raise an Empty error.
        :param preserve_last: If True, do not delete the measurement from the buffer if it is the last one.
        :return: timestamp (seconds), quaternion (wxyz), and free acceleration (m/s^2 in the global inertial frame)
        """
        if preserve_last and IMU948DotSet._buffer[i].qsize() == 1:
            bytes_msg = IMU948DotSet._buffer[i].queue[0]
        else:
            bytes_msg = IMU948DotSet._buffer[i].get(block=True, timeout=timeout)
        return bytes_msg
    
    @staticmethod
    def get(i: int, timeout=None, preserve_last=False):
        r"""
        Get the next measurements of the ith IMU. May be blocked.

        :param i: The index of the query sensor.
        :param timeout: If non-negative, block at most timeout seconds and raise an Empty error.
        :param preserve_last: If True, do not delete the measurement from the buffer if it is the last one.
        :return: timestamp (seconds), quaternion (wxyz), and free acceleration (m/s^2 in the global inertial frame)
        """
        if preserve_last and IMU948DotSet._buffer[i].qsize() == 1:
            parsed = IMU948DotSet._buffer[i].queue[0]
        else:
            parsed = IMU948DotSet._buffer[i].get(block=True, timeout=timeout)
        t = parsed.timestamp.microseconds / 1e3
        q = torch.tensor([parsed.quaternion.w, parsed.quaternion.x, parsed.quaternion.y, parsed.quaternion.z])
        a = torch.tensor([parsed.acceleration.x, parsed.acceleration.y, parsed.acceleration.z])
        return t, q, a

    @staticmethod
    def async_connect(devices: list):
        r"""
        Connect to the sensors and start receiving the measurements.
        Only send the connecting command but will not be blocked.

        :param devices: List of Xsens dot addresses.
        """
        if not IMU948DotSet.is_connected():
            print('Remember: use xsens dot app to synchronize the sensors first.')
            IMU948DotSet._run_in_new_thread(IMU948DotSet._multiple_sensor(devices))
        else:
            print('[Warning] connect failed: IMU948DotSet is already connected.')

    @staticmethod
    def sync_connect(devices: list):
        r"""
        Connect to the sensors and start receiving the measurements. Block until finish.

        :param devices: List of Xsens dot addresses.
        """
        IMU948DotSet.async_connect(devices)
        while not IMU948DotSet.is_connected():
            time.sleep(1)

    @staticmethod
    def async_disconnect():
        r"""
        Stop reading and disconnect to the sensors.
        Only send the disconnecting command but will not be blocked.
        """
        if IMU948DotSet.is_connected():
            IMU948DotSet._pending_event = 0
        else:
            print('[Warning] disconnect failed: IMU948DotSet is not connected.')

    @staticmethod
    def sync_disconnect():
        r"""
        Stop reading and disconnect to the sensors. Block until finish.
        """
        IMU948DotSet.async_disconnect()
        while IMU948DotSet.is_connected():
            time.sleep(1)

    @staticmethod
    def async_shutdown():
        r"""
        Stop reading and shutdown the sensors.
        Only send the shutdown command but will not be blocked.
        """
        if IMU948DotSet.is_connected():
            IMU948DotSet._pending_event = 1
        else:
            print('[Warning] shutdown failed: IMU948DotSet is not connected.')

    @staticmethod
    def sync_shutdown():
        r"""
        Stop reading and shutdown the sensors. Block until finish.
        """
        IMU948DotSet.async_shutdown()
        while IMU948DotSet.is_connected():
            time.sleep(1)

    @staticmethod
    def reset_xyz():
        r"""
        Reset sensor xyz .
        """
        if IMU948DotSet.is_connected():
            IMU948DotSet._pending_event = 2
            IMU948DotSet._wait_for_pending_event()
        else:
            print('[Warning] reset xyz failed: IMU948DotSet is not started.')

    @staticmethod
    def revert_heading_to_default():
        r"""
        Revert sensor heading to default (yaw).
        """
        if IMU948DotSet.is_started():
            IMU948DotSet._pending_event = 3
            IMU948DotSet._wait_for_pending_event()
        else:
            print('[Warning] revert heading failed: IMU948DotSet is not started.')

    @staticmethod
    def start_streaming():
        r"""
        Start sensor streaming.
        """
        if not IMU948DotSet.is_connected():
            print('[Warning] start streaming failed: IMU948DotSet is not connected.')
        elif IMU948DotSet.is_started():
            print('[Warning] start streaming failed: IMU948DotSet is already started.')
        else:
            IMU948DotSet._pending_event = 4
            IMU948DotSet._wait_for_pending_event()

    @staticmethod
    def stop_streaming():
        r"""
        Stop sensor streaming.
        """
        if not IMU948DotSet.is_connected():
            print('[Warning] stop streaming failed: IMU948DotSet is not connected.')
        elif not IMU948DotSet.is_started():
            print('[Warning] stop streaming failed: IMU948DotSet is not started.')
        else:
            IMU948DotSet._pending_event = 5
            IMU948DotSet._wait_for_pending_event()

    @staticmethod
    def print_battery_info():
        r"""
        Print battery level infos.
        """
        if not IMU948DotSet.is_connected():
            print('[Warning] print battery info failed: IMU948DotSet is not connected.')
        else:
            IMU948DotSet._pending_event = 6
            IMU948DotSet._wait_for_pending_event()

    @staticmethod
    def set_buffer_len(n=180):
        r"""
        Set IMU buffer length. Cache the latest n measurements.

        :param n: Length of IMU buffer.
        """
        IMU948DotSet._SZ = n
        IMU948DotSet.clear()


# example
if __name__ == '__main__':
    # copy the following codes outside this package to run
    from articulate.utils.imu948 import IMU948DotSet
    from articulate.utils.bullet import RotationViewer
    from articulate.math import quaternion_to_rotation_matrix
    imus = [
        # 'D4:22:CD:00:36:80',
        # 'D4:22:CD:00:36:04',
        # 'D4:22:CD:00:32:3E',
        # 'D4:22:CD:00:35:4E',
        # 'D4:22:CD:00:36:03',
        # 'D4:22:CD:00:44:6E',
        # 'D4:22:CD:00:45:E6',
        '2c:01:6e:17:62:3e',
    ]
    IMU948DotSet.sync_connect(imus)
    # IMU948DotSet.reset_xyz()
    IMU948DotSet.start_streaming()
    with RotationViewer(len(imus)) as viewer:
        IMU948DotSet.clear()
        for _ in range(60 * 10):  # 10s
            for i in range(len(imus)):
                t, q, a = IMU948DotSet.get(i)
                viewer.update(quaternion_to_rotation_matrix(q).view(3, 3), i)
    IMU948DotSet.sync_disconnect()
