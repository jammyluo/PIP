__all__ = ['Acceleration', 'AngularVelocity', 
           'DeviceInfoCharacteristic', 'DeviceReportCharacteristic', 'Dot', 'EulerAngles',
           'FreeAcceleration', 'MagneticField', 'TAH', 'Quaternion', 'Timestamp', 
           'adevice_info_read', 'afind_by_address',
           'afind_dot_by_address', 'ais_dot', 'apower_off',  'ascan', 'ascan_all',
           'device_info_read', 'find_by_address',
           'find_dot_by_address', 'is_dot', 'power_off', 'scan', 'scan_all']


import struct  # for unpacking floating-point data
import asyncio
from typing import Any  # for async BLE IO
from bleak import BleakScanner, BleakClient  # for BLE communication

scaleAccel       = 0.00478515625      # 加速度 [-16g~+16g]    9.8*16/32768
scaleQuat        = 0.000030517578125  # 四元数 [-1~+1]         1/32768
scaleAngle       = 0.0054931640625    # 角度   [-180~+180]     180/32768
scaleAngleSpeed  = 0.06103515625      # 角速度 [-2000~+2000]    2000/32768
scaleMag         = 0.15106201171875   # 磁场 [-4950~+4950]   4950/32768
scaleTemperature = 0.01               # 温度
scaleAirPressure = 0.0002384185791    # 气压 [-2000~+2000]    2000/8388608
scaleHeight      = 0.0010728836       # 高度 [-9000~+9000]    9000/8388608

# returns a pretty-printed representation of an arbitrary class
def _pretty_print(obj: Any) -> str:
    return f"{obj.__class__.__name__}({', '.join('%s=%s' % item for item in vars(obj).items())})"


# a helper class that encapsulates a reader "cursor" that indexes a
# location in an array of bytes. Provides methods for reading multi-byte
# sequences (floats, ints, etc.) from the array, advancing the cursor as
# it reads. Multi-byte sequences are parsed according to XSens's binary
# spec (namely, little-endian with IEE754 floats)
class _ResponseReader:

    # initializes a reader that's pointing to the start of `data`
    def __init__(self, data):
        self.pos = 0
        self.data = data

    # returns number of remaining bytes that the reader can still read
    def remaining(self) -> int:
        return len(self.data) - self.pos

    # read `n` raw bytes from the reader's current position and advance
    # the current position by `n`
    def read_bytes(self, n: int) -> bytes:
        rv = self.data[self.pos:self.pos+n]
        self.pos += n
        return rv

    # read 1 byte as an signed int
    def read_s8(self) -> int:
        return int.from_bytes(self.read_bytes(1), "little", signed=True)

    # read 2 bytes as an signed little-endian int
    def read_s16(self) -> int:
        return int.from_bytes(self.read_bytes(2), "little", signed=True)
    
    # read 3 bytes as an signed little-endian int
    def read_s24(self) -> int:
        return int.from_bytes(self.read_bytes(3), "little", signed=True)
    
    # read 4 bytes as an signed little-endian int
    def read_s32(self) -> int:
        return int.from_bytes(self.read_bytes(4), "little", signed=True)

    # read 8 bytes as an signed little-endian int
    def read_s64(self) -> int:
        return int.from_bytes(self.read_bytes(8), "little", signed=True)
    
    # read 1 byte as an unsigned int
    def read_u8(self) -> int:
        return int.from_bytes(self.read_bytes(1), "little", signed=False)

    # read 2 bytes as an unsigned little-endian int
    def read_u16(self) -> int:
        return int.from_bytes(self.read_bytes(2), "little", signed=False)
    
    # read 3 bytes as an unsigned little-endian int
    def read_u24(self) -> int:
        return int.from_bytes(self.read_bytes(3), "little", signed=False)
    
    # read 4 bytes as an unsigned little-endian int
    def read_u32(self) -> int:
        return int.from_bytes(self.read_bytes(4), "little", signed=False)

    # read 8 bytes as an unsigned little-endian int
    def read_u64(self) -> int:
        return int.from_bytes(self.read_bytes(8), "little", signed=False)

    # read 4 bytes as a IEE754 floating point number
    def read_f32(self) -> float:
        return struct.unpack('f', self.read_bytes(4))[0]


# Configuration Service: Device Info Characteristic (sec 2.1, p8 in the BLE spec)
#
# read-only characteristic for top-level device information
# 只读， 获取设备属性和状态
class DeviceInfoCharacteristic:
    # returns a `DeviceInfoCharacteristic` read from a `_ResponseReader`
    @staticmethod
    def from_reader(reader: _ResponseReader):
        rv = DeviceInfoCharacteristic()
        rv.still_limit = reader.read_u8()               #字节1 惯导-静止状态加速度阀值 单位dm/s²
        rv.still_to_zero = reader.read_u8()             #字节2 惯导-静止归零速度(单位mm/s) 0:不归零 255:立即归零
        rv.move_to_zero = reader.read_u8()              #字节3 惯导-动态归零速度(单位mm/s) 0:不归零
        tmp = reader.read_u8()
        rv.compass = (tmp>>0) & 0x01                    #字节4 bit[0]: 1=已开启磁场 0=已关闭磁场
        rv.barometer_filter = (tmp>>1) & 0x03           #字节4 bit[1-2]: 气压计的滤波等级[取值0-3],数值越大越平稳但实时性越差
        rv.IMU = (tmp>>3) & 0x01                        #字节4 bit[3]: 1=传感器已开启  0=传感器已睡眠
        rv.auto_report = (tmp>>4) & 0x01                #字节4 bit[4]: 1=已开启传感器数据主动上报 0=已关闭传感器数据主动上报
        rv.FPS = reader.read_u8()                       #字节5 数据主动上报的传输帧率[取值0-250HZ], 0表示0.5HZ
        rv.gyro_filter = reader.read_u8()               #字节6 陀螺仪滤波系数[取值0-2],数值越大越平稳但实时性越差
        rv.acc_filter = reader.read_u8()                #字节7 加速计滤波系数[取值0-4],数值越大越平稳但实时性越差
        rv.compass_filter = reader.read_u8()            #字节8 磁力计滤波系数[取值0-9],数值越大越平稳但实时性越差
        rv.subscribe_tag = reader.read_u16()            #字节[10-9] 功能订阅标识
        rv.charged_state = reader.read_u8()             #字节11 充电状态指示 0=未接电源 1=充电中 2=已充满
        rv.battery_level = reader.read_u8()             #字节12 当前剩余电量[0-100%]
        rv.battery_voltage = reader.read_u16()          #字节[14-13] 电池的当前电压mv
        tmp =  reader.read_bytes(6)                   #字节[15-20] 15:16:17:18:19:20 MAC地址
        rv.Mac = f'{tmp[0]:2x}:{tmp[1]:2x}:{tmp[2]:2x}:{tmp[3]:2x}:{tmp[4]:2x}:{tmp[5]:2x}'
        rv.version = reader.read_bytes(6)               #字节[21-26] 固件版本 字符串
        rv.product_model = reader.read_bytes(6)         #字节[27-32] 产品型号 字符串

        return rv

    # returns a `DeviceInfoCharacteristic` parsed from bytes
    @staticmethod
    def from_bytes(bites):
        reader = _ResponseReader(bites)
        return DeviceInfoCharacteristic.from_reader(reader)
    
    def to_bytes(self):
        rv = bytearray()
        rv += self.TAG.to_bytes(1, "little")
        
        return rv
    
    def __repr__(self):
        return _pretty_print(self)

# the next bunch of classes are parsers etc. for the wide variety of measurement structs
# that the measurement service can emit

# measurement data (sec. 3.5 in BLE spec): timestamp: "Timestamp of the sensor in microseconds"
class Timestamp:
    SIZE = 4

    @staticmethod
    def from_reader(reader: _ResponseReader):
        assert reader.remaining() >= Timestamp.SIZE

        rv = Timestamp()
        rv.microseconds = reader.read_u32()

        return rv

    def __repr__(self):
        return _pretty_print(self)



# measurement data (sec. 3.5 in the BLE spec): Acceleration: "Calibrated acceleration in sensor coordinate, m/s^2"
# 加速度xyz 去掉了重力 使用时需*scaleAccel m/s
class Acceleration:
    SIZE = 6
    TYPEID = 0x0001
    # returns a `Acceleration` read from a `_ResponseReader`
    @staticmethod
    def from_reader(reader: _ResponseReader):
        assert reader.remaining() >= Acceleration.SIZE

        rv = Acceleration()
        rv.x = reader.read_s16() * scaleAccel
        rv.y = reader.read_s16() * scaleAccel
        rv.z = reader.read_s16() * scaleAccel

        return rv

    def __repr__(self):
        return _pretty_print(self)

# measurement data (sec. 3.5 in the BLE spec): free acceleration: "Acceleration in local earth coordinate 
# and the local gravity is deducted, m/s^2"
# 加速度xyz 包含了重力 使用时需*scaleAccel m/s
class FreeAcceleration:
    SIZE = 6
    TYPEID = 0x0002
    # returns a `FreeAcceleration` read from a `_ResponseReader`
    @staticmethod
    def from_reader(reader: _ResponseReader):
        assert reader.remaining() >= FreeAcceleration.SIZE

        rv = FreeAcceleration()
        rv.x = reader.read_s16() * scaleAccel
        rv.y = reader.read_s16() * scaleAccel
        rv.z = reader.read_s16() * scaleAccel

        return rv

    def __repr__(self):
        return _pretty_print(self)

# measurement data (sec. 3.5 in the BLE spec): Angular Velocity: "Rate of turn in sensor coordinate, dps(degree per second)"
# 角速度xyz 使用时需*scaleAngleSpeed °/s
class AngularVelocity:
    SIZE = 6
    TYPEID = 0x0004
    # returns an `AngularVelocity` read from a `_ResponseReader`
    @staticmethod
    def from_reader(reader: _ResponseReader):
        assert reader.remaining() >= AngularVelocity.SIZE

        rv = AngularVelocity()
        rv.x = reader.read_s16() * scaleAngleSpeed
        rv.y = reader.read_s16() * scaleAngleSpeed
        rv.z = reader.read_s16() * scaleAngleSpeed

        return rv

    def __repr__(self):
        return _pretty_print(self)


# measurement data (sec. 3.5 in the BLE spec): Magnetic Field: "Magnetic field in the sensor coordinate, a.u."
class MagneticField:
    SIZE = 6
    TYPEID = 0x0008
    # returns a `MagneticField` read from a `_ResponseReader`
    @staticmethod
    def from_reader(reader: _ResponseReader):
        assert reader.remaining() >= MagneticField.SIZE
        
        rv = MagneticField()
        rv.x = reader.read_s16() * scaleMag
        rv.y = reader.read_s16() * scaleMag
        rv.z = reader.read_s16() * scaleMag

        return rv

    def __repr__(self):
        return _pretty_print(self)

# 温度 气压 高度
class TAH:
    SIZE = 6
    TYPEID = 0x0010
    # returns a `TAH` read from a `_ResponseReader`
    @staticmethod
    def from_reader(reader: _ResponseReader):
        assert reader.remaining() >= TAH.SIZE
        
        rv = TAH()
        rv.temperature = reader.read_s16() * scaleTemperature
        rv.airPressure = reader.read_s24() * scaleAirPressure
        rv.height = reader.read_s24() * scaleHeight

        return rv

    def __repr__(self):
        return _pretty_print(self)
    
# measurement data (sec. 3.5 in BLE spec): quaternion: "The orientation expressed as a quaternion"
# 四元素 wxyz 使用时需*scaleQuat
class Quaternion:
    SIZE = 8
    TYPEID = 0x0020
    # returns a `Quaternion` read from a `_ResponseReader`
    @staticmethod
    def from_reader(reader: _ResponseReader):
        assert reader.remaining() >= Quaternion.SIZE

        rv = Quaternion()
        rv.w = reader.read_s16() * scaleQuat
        rv.x = reader.read_s16() * scaleQuat
        rv.y = reader.read_s16() * scaleQuat
        rv.z = reader.read_s16() * scaleQuat

        return rv

    def __repr__(self):
        return _pretty_print(self)


# measurement data (sec. 3.5 in BLE spec): euler angles: "The orientation expressed as Euler angles, degree"
# 欧拉角xyz 使用时需*scaleAngle
class EulerAngles:
    SIZE = 6
    TYPEID = 0x0040
    # returns a `EulerAngles` read from a `_ResponseReader`
    @staticmethod
    def from_reader(reader: _ResponseReader):
        assert reader.remaining() >= EulerAngles.SIZE

        rv = EulerAngles()
        rv.x = reader.read_s16() * scaleAngle
        rv.y = reader.read_s16() * scaleAngle
        rv.z = reader.read_s16() * scaleAngle

        return rv

    def __repr__(self):
        return _pretty_print(self)

class DevicePayloadCharacteristic:
    @staticmethod
    def from_reader(reader: _ResponseReader):
        rv = DevicePayloadCharacteristic()
        rv.typeid = reader.read_u16()                   #字节[2-1]，功能订阅标识
        rv.timestamp = Timestamp.from_reader(reader)    #字节[6-3]，模块开机后时间戳（单位ms）
        if (rv.typeid & Acceleration.TYPEID) != 0: 
            rv.acceleration = Acceleration.from_reader(reader)
        if (rv.typeid & FreeAcceleration.TYPEID) != 0:
            rv.free_acceleration = FreeAcceleration.from_reader(reader)
        if (rv.typeid & AngularVelocity.TYPEID) != 0:
            rv.angular_velocity = AngularVelocity.from_reader(reader)
        if (rv.typeid & MagneticField.TYPEID) != 0:
            rv.magnetic_field = MagneticField.from_reader(reader)
        if (rv.typeid & TAH.TYPEID) != 0:
            rv.TAH = TAH.from_reader(reader)
        if (rv.typeid & Quaternion.TYPEID) != 0:
            rv.quaternion = Quaternion.from_reader(reader)
        return rv

    # returns a `DevicePayloadCharacteristic` parsed from bytes
    @staticmethod
    def from_bytes(bites):
        reader = _ResponseReader(bites)
        return DevicePayloadCharacteristic.from_reader(reader)

    def __repr__(self):
        return _pretty_print(self)
    
class DeviceReportCharacteristic:
    UUID = 0x0007
    @staticmethod
    def from_reader(reader: _ResponseReader):
        rv = DeviceReportCharacteristic()
        rv.size = reader.remaining()
        rv.TAG = reader.read_u8()                       #字节[0] 标签
        if (rv.TAG == 0x11) :
            rv.payload = DevicePayloadCharacteristic.from_reader(reader)
        if (rv.TAG == 0x10):
            rv.status = DeviceInfoCharacteristic.from_reader(reader)

        return rv

    # returns a `DeviceReportCharacteristic` parsed from bytes
    @staticmethod
    def from_bytes(bites):
        reader = _ResponseReader(bites)
        return DeviceReportCharacteristic.from_reader(reader)

    def __repr__(self):
        return _pretty_print(self)
    

# a class for connecting to, and using, an XSens DOT
#
# This class provides:
#
# - Methods for connecting to the underlying DOT device through the Bluetooth Low-Energy
#   (BLE) connection (`Dot.connect`, `Dot.disconnect`, `with`, and `async with`)
#
# - Low-level methods for reading, writing, and receiving notifications from the low-level
#   BLE characteristic the DOT exposes
#
# - High-level methods that use the low-level methods (e.g. turn the DOT on, identify it)
#
# This class acts as a lifetime wrapper around the underlying BLE connection, so
# you should use it in something like a `with` or `async with` block. Using the async
# API (methods prefixed with `a`) and `async with` block is better. The underlying BLE
# implementation is asynchronous. The synchronous API (other methods and plain `with`
# blocks) is more convenient, but has to hop into an asynchronous event loop until the
# entire method call is complete. The asynchronous equivalents have the opportunity to
# cooperatively yield so that other events can be processed while waiting for the response.
# It is practically a necessity to use the async API if handling a large amount of DOTs
# from one thread (otherwise, you will experience head-of-line blocking and have a harder
# time handling the side-effects of notications).
class Dot:

    # init/enter/exit: connect/disconnect to the DOT

    # init a new `Dot` instance
    #
    # initializes the underlying connection client, but does not connect to
    # the DOT. Use `.connect`/`.disconnect`, or (better) a context manager
    # (`with Dot(ble) as dot`), or (better again) an async context manager
    # (`async with Dot(ble) as dot`) to setup/teardown the connection
    def __init__(self, ble_device):
        self.client = BleakClient(ble_device)

    # automatically called when entering `async with` blocks
    async def __aenter__(self):
        await self.client.__aenter__()
        return self

    # automatically called when exiting `async with` blocks
    async def __aexit__(self, exc_type, value, traceback):
        await self.client.__aexit__(exc_type, value, traceback)

    # automatically called when entering (synchronous) `with` blocks
    def __enter__(self):
        asyncio.get_event_loop().run_until_complete(self.__aenter__())
        return self

    # automatically called when exiting (synchronous) `with` blocks
    def __exit__(self, exc_type, value, traceback):
        asyncio.get_event_loop().run_until_complete(self.__aexit__(exc_type, value, traceback))

    # (dis)connection methods

    # asynchronously establishes a connection to the DOT
    async def aconnect(self, timeout=10):
        return await self.client.connect(timeout=timeout)

    # synchronously establishes a connection to the DOT
    def connect(self, timeout=10):
        return asyncio.get_event_loop().run_until_complete(self.aconnect(timeout))

    # asynchronously terminates the connection to the DOT
    async def adisconnect(self):
        await self.client.write_gatt_char(0x0005, bytes([0x26]))
        return await self.client.disconnect()

    # synchronously terminates the connection to the DOT
    def disconnect(self):
        return asyncio.get_event_loop().run_until_complete(self.adisconnect())

    # low-level characteristic accessors

    # asynchronously reads the "Device Info Characteristic" (sec. 2.1 in the BLE spec)
    async def adevice_info_read(self):
        await self.client.write_gatt_char(0x0005, bytes([0x10]))
        await asyncio.sleep(0.2)    

    # synchronously reads the "Device Info Characteristic" (sec. 2.1 in the BLE spec)
    def device_info_read(self):
        return asyncio.get_event_loop().run_until_complete(self.adevice_info_read())

    # asynchronously enable notifications from the "Device Report Characteristic" (sec.
    # 2.3 in the BLE spec)
    #
    # once notifications are enabled, `callback` will be called with two arguments:
    # a message ID and the raw message bytes (which can be parsed using
    # `DeviceReportCharacteristic.parse`). Notifications arrive from the DOT whenever
    # a significant event happens (e.g. a button press). See the BLE spec for which
    # events trigger from which actions.
    async def adevice_report_start_notify(self, callback):
        await self.client.start_notify(DeviceReportCharacteristic.UUID, callback)

    # synchronously enable notifications from the "Device Report Characteristic" (sec.
    # 2.3 in the BLE spec)
    #
    # once notifications are enabled, `callback` will be called with two arguments:
    # a message ID and the raw message bytes (which can be parsed using
    # `DeviceReportCharacteristic.parse`). Notifications arrive from the DOT whenever
    # a significant event happens (e.g. a button press). See the BLE spec for which
    # events trigger from which actions.
    def device_report_start_notify(self, callback):
        asyncio.get_event_loop().run_until_complete(self.adevice_report_start_notify(callback))

    # asynchronously disable notifications from the "Device Report Characteristic" (sec.
    # 2.3 in the BLE spec)
    #
    # this disables notifications that were enabled by the `device_report_start_notify`
    # method. After this action completes, the `callback` in the enable call will no longer
    # be called
    async def adevice_report_stop_notify(self):
        await self.client.stop_notify(DeviceReportCharacteristic.UUID)

    # synchronously disable notifications from the "Device Report Characteristic" (sec.
    # 2.3 in the BLE spec)
    #
    # this disables notifications that were enabled by the `device_report_start_notify`
    # method. After this action completes, the `callback` in the enable call will no longer
    # be called
    def device_report_stop_notify(self):
        asyncio.get_event_loop().run_until_complete(self.adevice_report_stop_notify())


    # high-level operations

    # asynchronously requests that the DOT powers itself off
    async def apower_off(self):
        print("apower_off TODO")

    # synchronously requests that the DOT powers itself off
    def power_off(self):
        asyncio.get_event_loop().run_until_complete(self.apower_off())

    # asynchronously keep live the measurement (BLE spec sec. 3.1)
    # enable BLE notification to get the measurement data for real-time streaming before calling this
    async def akeep_live(self):
        # 保持连接 0x29
        await self.client.write_gatt_char(0x0005, bytes([0x29]))
        await asyncio.sleep(0.2)
        # 尝试采用蓝牙高速通信特性 0x46
        await self.client.write_gatt_char(0x0005, bytes([0x46]))
        # await asyncio.sleep(0.2)         

    # synchronously keep live the measurement (BLE spec sec. 3.1)
    # enable BLE notification to get the measurement data for real-time streaming before calling this
    def keep_live(self):
        asyncio.get_event_loop().run_until_complete(self.akeep_live())

    # asynchronously keep live the measurement (BLE spec sec. 3.1)
    # enable BLE notification to get the measurement data for real-time streaming before calling this
    async def aset_params(self, params):
        await self.client.write_gatt_char(0x0005, params)
        # await asyncio.sleep(0.2)    

    # synchronously keep live the measurement (BLE spec sec. 3.1)
    # enable BLE notification to get the measurement data for real-time streaming before calling this
    def set_params(self, params):
        asyncio.get_event_loop().run_until_complete(self.aset_params(), params)

    # asynchronously start the measurement (BLE spec sec. 3.1)
    # enable BLE notification to get the measurement data for real-time streaming before calling this
    async def astart_reporting(self):
        # 开始主动上报
        await self.client.write_gatt_char(0x0005, bytes([0x19]))
        # await asyncio.sleep(0.2) 

    # synchronously start the measurement (BLE spec sec. 3.1)
    # enable BLE notification to get the measurement data for real-time streaming before calling this
    def start_reporting(self,):
        asyncio.get_event_loop().run_until_complete(self.astart_reporting())

    async def astop_reporting(self):
        # 关闭主动上报
        await self.client.write_gatt_char(0x0005, bytes([0x18]))
        # await asyncio.sleep(0.2)    

    def stop_reporting(self,):
        asyncio.get_event_loop().run_until_complete(self.astop_reporting())

    async def areset_accel(self):
        # 关闭主动上报
        await self.client.write_gatt_char(0x0005, bytes([0x17]))

    def reset_accel(self,):
        asyncio.get_event_loop().run_until_complete(self.areset_accel())

    async def areset_xyz(self):
        #xyz坐标清零
        await self.client.write_gatt_char(0x0005, bytes([0x05]))
        await asyncio.sleep(0.2)
        await self.client.write_gatt_char(0x0005, bytes([0x06]))
        await asyncio.sleep(0.2)

    def reset_xyz(self,):
        asyncio.get_event_loop().run_until_complete(self.areset_xyz())

    async def areset_z_coordinate(self):
        #恢复默认Z轴角及坐标系
        await self.client.write_gatt_char(0x0005, bytes([0x08]))

    def reset_z_coordinate(self,):
        asyncio.get_event_loop().run_until_complete(self.areset_z_coordinate())


# asynchronously returns `True` if the provided `bleak.backends.device.BLEDevice`
# is believed to be an XSens DOT sensor
async def ais_dot(bledevice):
    if bledevice.name and "IM" in bledevice.name:
        return True
    return False


# synchronously returns `True` if the provided `bleak.backends.device.BLEDevice` is an
# XSens DOT
def is_dot(bledevice):
    return asyncio.get_event_loop().run_until_complete(ais_dot(bledevice))


# asynchronously returns a list of all (not just DOT) BLE devices that
# the host's bluetooth adaptor can see. Each element in the list is an
# instance of `bleak.backends.device.BLEDevice`
async def ascan_all(timeout=5):
    return await BleakScanner.discover(timeout=timeout)


# synchronously returns a list of all (not just DOT) BLE devices that the
# host's bluetooth adaptor can see. Each element in the list is an instance
# of `bleak.backends.device.BLEDevice`
def scan_all(timeout=5):
    return asyncio.get_event_loop().run_until_complete(ascan_all(timeout=timeout))


# asynchronously returns a list of all XSens DOTs that the host's bluetooth
# adaptor can see. Each element in the list is an instance of
# `bleak.backends.device.BLEDevice`
async def ascan(timeout=5):
    return [d for d in await ascan_all(timeout=timeout) if await ais_dot(d)]


# synchronously returns a list of all XSens DOTs that the host's bluetooth
# adaptor can see. Each element in the list is an instance of
# `bleak.backends.device.BLEDevice`
def scan(timeout=5):
    return asyncio.get_event_loop().run_until_complete(ascan(timeout=timeout))


# asynchronously returns a BLE device with the given identifier/address
#
# returns `None` if the device cannot be found (e.g. no connection, wrong
# address)
async def afind_by_address(device_identifier, timeout=5):
    return await BleakScanner.find_device_by_address(device_identifier, timeout=timeout)


# synchronously returns a BLE device with the given identifier/address
#
# returns `None` if the device cannot be found (e.g. no connection, wrong
# address)
def find_by_address(device_identifier, timeout=5):
    return asyncio.get_event_loop().run_until_complete(afind_by_address(device_identifier, timeout=timeout))


# asynchronously returns a BLE device with the given identifier/address if the
# device appears to be an XSens DOT
#
# effectively, the same as `afind_by_address` but with the extra stipulation that
# the given device must be a DOT
async def afind_dot_by_address(device_identifier, timeout=5):
    dev = await afind_by_address(device_identifier, timeout=timeout)

    if dev is None:
        return None  # device cannot be found
    elif not await ais_dot(dev):
        return None  # device exists but is not a DOT
    else:
        return dev


# synchronously returns a BLE device with the given identifier/address, if the device
# appears to be an XSens DOT
#
# effectively, the same as `find_by_address`, but with the extra stipulation that
# the given device must be a DOT
def find_dot_by_address(device_identifier, timeout=5):
    return asyncio.get_event_loop().run_until_complete(afind_dot_by_address(device_identifier, timeout=timeout))


# low-level characteristic accessors (free functions)


# asynchronously returns the "Device Info Characteristic" for the given DOT device
#
# see: sec 2.1 Device Info Characteristic in DOT BLE spec
async def adevice_info_read(bledevice):
    async with Dot(bledevice) as dot:
        return await dot.adevice_info_read()


# synchronously returns the "Device Info Characteristic" for the given DOT device
#
# see: sec 2.1: Device Info Characteristic in DOT BLE spec
def device_info_read(bledevice):
    return asyncio.get_event_loop().run_until_complete(adevice_info_read(bledevice))

async def apower_off(bledevice):
    async with Dot(bledevice) as dot:
        await dot.apower_off()

def power_off(bledevice):
    asyncio.get_event_loop().run_until_complete(apower_off(bledevice))



# example
if __name__ == '__main__':
    def on_device_report(message_id, message_bytes, sensor_id=-1):
        parsed = DeviceReportCharacteristic.from_bytes(message_bytes)
        print(sensor_id, parsed)

    async def scan():
        print('finding devices ...')
        all_devices = await ascan_all()
        print(all_devices)

    async def single_sensor():
        print('connecting ...')
        dot = await afind_by_address('2c:01:6e:17:62:3e', 30)

        async with Dot(dot) as device:
            print('connected')
            await device.adevice_report_start_notify(on_device_report)
            await device.akeep_live()
            await device.astop_reporting()
            
            # 参数设置
            isCompassOn = 0 #使用磁场融合姿态
            barometerFilter = 2
            Cmd_ReportTag = 0x0FFF # 功能订阅标识
            params = bytearray([0x00 for i in range(0,11)])
            params[0] = 0x12
            params[1] = 5       #静止状态加速度阀值
            params[2] = 254     #静止归零速度(单位cm/s) 0:不归零 255:立即归零
            params[3] = 0       #动态归零速度(单位cm/s) 0:不归零
            params[4] = ((barometerFilter&3)<<1) | (isCompassOn&1);   
            params[5] = 60      #数据主动上报的传输帧率[取值0-250HZ], 0表示0.5HZ
            params[6] = 1       #陀螺仪滤波系数[取值0-2],数值越大越平稳但实时性越差
            params[7] = 3       #加速计滤波系数[取值0-4],数值越大越平稳但实时性越差
            params[8] = 5       #磁力计滤波系数[取值0-9],数值越大越平稳但实时性越差
            params[9] = Cmd_ReportTag&0xff
            params[10] = (Cmd_ReportTag>>8)&0xff
            await device.aset_params(params)
            await device.adevice_info_read()
            # await device.astart_reporting()

            print('start')
            await asyncio.sleep(1)
            await device.adisconnect()
            print('stop')
            
        print('Not connected')

    async def multiple_sensor():
        # Please use xsens dot app to synchronize the sensors first.
        from functools import partial

        devices = [
            'D4:CA:6E:F1:99:84',
            'D4:CA:6E:F1:99:A6',
            'D4:CA:6E:F1:9A:30',
            'D4:CA:6E:F1:99:82',
            'D4:22:CD:00:01:27',
            # 'D4:22:CD:00:03:0C',
            # 'D4:22:CD:00:00:8B',
            # 'D4:22:CD:00:06:5F',
            # 'D4:CA:6E:F0:C7:22',
            # 'D4:22:CD:00:02:87',
        ]

        print('finding devices ...')
        bledevices = await asyncio.gather(*[afind_by_address(d) for d in devices])
        dots = [Dot(d) for d in bledevices]

        print('connecting ...')
        await asyncio.gather(*[d.aconnect() for d in dots])

        print('reading battery infos ...', end=' ')
        battery_infos = await asyncio.gather(*[d.abattery_read() for d in dots])
        print(' '.join(['%d%%' % info.battery_level for info in battery_infos]))

        print('start the notifications ...', end=' ')
        await asyncio.gather(*[d.adevice_report_start_notify(partial(on_device_report, sensor_id=i)) for i, d in enumerate(dots)])

        print('starting and reset heading ...')
        await asyncio.gather(*[d.astart_streaming(payload_mode=3) for d in dots])
        await asyncio.gather(*[d.areset_heading() for d in dots])
        await asyncio.sleep(2)

        print('stopping ...')
        await asyncio.gather(*[d.astop_streaming() for d in dots])
        await asyncio.gather(*[d.amedium_payload_stop_notify() for d in dots])
        await asyncio.gather(*[d.adevice_report_stop_notify() for d in dots])
        await asyncio.gather(*[d.adisconnect() for d in dots])

    def run_in_new_thread(coro):
        r"""
        Similar to `asyncio.run()`, but create a new thread.
        """
        def start_loop(_loop):
            asyncio.set_event_loop(_loop)
            _loop.run_forever()

        import threading
        loop = asyncio.new_event_loop()
        thread = threading.Thread(target=start_loop, args=(loop,))
        thread.setDaemon(True)
        thread.start()
        asyncio.run_coroutine_threadsafe(coro, loop)

    # asyncio.run(single_sensor())
    # asyncio.run(multiple_sensor())
    run_in_new_thread(scan())
    # run_in_new_thread(single_sensor())
    # run_in_new_thread(multiple_sensor())

    import time
    while True:
        time.sleep(20)  # main thread blocking
