'''
typedef struct __attribute__((__packed__)) serial_mbot_encoders_t {
    int64_t utime;
    int64_t ticks[3]; // no units
    int32_t delta_ticks[3]; // no units
    int32_t delta_time; // [usec]
} serial_mbot_encoders_t;

typedef struct __attribute__((__packed__)) serial_pose2D_t {
    int64_t utime;
    float x;
    float y;
    float theta;
} serial_pose2D_t;

typedef struct __attribute__((__packed__)) serial_mbot_imu_t {
    int64_t utime;
    float gyro[3];
    float accel[3];
    float mag[3];
    float angles_rpy[3]; // roll (x), pitch (y), yaw, (z)
    float angles_quat[4]; // quaternion
    float temp;
} serial_mbot_imu_t;

typedef struct __attribute__((__packed__)) serial_twist2D_t {
    int64_t utime;
    float vx;
    float vy;
    float wz;
} serial_twist2D_t;

typedef struct __attribute__((__packed__)) serial_mbot_motor_vel_t {
    int64_t utime;
    float velocity[3]; // [rad/s]
} serial_mbot_motor_vel_t;

typedef struct __attribute__((__packed__)) serial_mbot_motor_pwm_t {
    int64_t utime;
    float pwm[3]; // [-1.0..1.0]
} serial_mbot_motor_pwm_t;

typedef struct __attribute__((__packed__)) packets_wrapper {
    serial_mbot_encoders_t encoders;
    serial_pose2D_t odom;
    serial_mbot_imu_t imu;
    serial_twist2D_t mbot_vel;
    serial_mbot_motor_vel_t motor_vel;
    serial_mbot_motor_pwm_t motor_pwm;
} packets_wrapper_t;
'''

import struct
from typing import List

SYNC_FLAG = 0xFF
VERSION_FLAG = 0xFE

MBOT_TIMESYNC = 201
MBOT_ODOMETRY = 210
MBOT_ODOMETRY_RESET = 211
MBOT_VEL_CMD = 214
MBOT_IMU = 220
MBOT_ENCODERS = 221
MBOT_ENCODERS_RESET = 222
MBOT_MOTOR_PWM_CMD = 230
MBOT_MOTOR_VEL_CMD = 231
MBOT_MOTOR_VEL = 232
MBOT_MOTOR_PWM = 233
MBOT_VEL = 234

class SerialMbotEncoders:
    byte_size = 8 + 8 * 3 + 4 * 3 + 4
    def __init__(self):
        self.utime: int = 0
        self.ticks: List[int] = [0, 0, 0]
        self.delta_ticks: List[int] = [0, 0, 0]
        self.delta_time: int = 0

    def decode(self, data: bytes):
        unpacked_data = struct.unpack('q3q3ii', data)
        self.utime = unpacked_data[0]
        self.ticks = list(unpacked_data[1:4])
        self.delta_ticks = list(unpacked_data[4:7])
        self.delta_time = unpacked_data[7]

    def encode(self) -> bytes:
        return struct.pack('q3q3ii', self.utime, *self.ticks, *self.delta_ticks, self.delta_time)

class SerialPose2D:
    byte_size = 8 + 4 * 3
    def __init__(self):
        self.utime: int = 0
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0
    
    def decode(self, data: bytes):
        self.utime, self.x, self.y, self.theta = struct.unpack('qfff', data)

    def encode(self) -> bytes:
        return struct.pack('qfff', self.utime, self.x, self.y, self.theta)

class SerialMbotImu:
    byte_size = 8 + 3 * 4 + 3 * 4 + 3 * 4 + 3 * 4 + 4 * 4 + 4
    def __init__(self):
        self.utime: int = 0
        self.gyro: List[float] = [0.0, 0.0, 0.0]
        self.accel: List[float] = [0.0, 0.0, 0.0]
        self.mag: List[float] = [0.0, 0.0, 0.0]
        self.angles_rpy: List[float] = [0.0, 0.0, 0.0]
        self.angles_quat: List[float] = [0.0, 0.0, 0.0, 0.0]
        self.temp: float = 0.0

    def decode(self, data: bytes):
        unpacked_data = struct.unpack('q3f3f3f3f4ff', data)
        self.utime = unpacked_data[0]
        self.gyro = list(unpacked_data[1:4])
        self.accel = list(unpacked_data[4:7])
        self.mag = list(unpacked_data[7:10])
        self.angles_rpy = list(unpacked_data[10:13])
        self.angles_quat = list(unpacked_data[13:17])
        self.temp = unpacked_data[17]

    def encode(self) -> bytes:
        return struct.pack('q3f3f3f3f4ff', self.utime, *self.gyro, *self.accel, *self.mag, *self.angles_rpy, *self.angles_quat, self.temp)

class SerialTwist2D:
    byte_size = 8 + 4 * 3
    def __init__(self):
        self.utime: int = 0
        self.vx: float = 0.0
        self.vy: float = 0.0
        self.wz: float = 0.0

    def decode(self, data: bytes):
        self.utime, self.vx, self.vy, self.wz = struct.unpack('qfff', data)

    def encode(self) -> bytes:
        return struct.pack('qfff', self.utime, self.vx, self.vy, self.wz)


class SerialMbotMotorVel:
    byte_size = 8 + 4 * 3
    def __init__(self):
        self.utime: int = 0
        self.velocity: List[float] = [0.0, 0.0, 0.0]
    
    def decode(self, data: bytes):
        unpacked_data = struct.unpack('q3f', data)
        self.utime = unpacked_data[0]
        self.velocity = list(unpacked_data[1:])

    def encode(self) -> bytes:
        return struct.pack('q3f', self.utime, *self.velocity)

class SerialMbotMotorPwm:
    byte_size = 8 + 4 * 3
    def __init__(self):
        self.utime: int = 0
        self.pwm: List[float] = [0.0, 0.0, 0.0]
    
    def decode(self, data: bytes):
        unpacked_data = struct.unpack('q3f', data)
        self.utime = unpacked_data[0]
        self.pwm = list(unpacked_data[1:])

    def encode(self) -> bytes:
        return struct.pack('q3f', self.utime, *self.pwm)

class PacketsWrapper:
    byte_size = SerialMbotEncoders.byte_size + SerialPose2D.byte_size + SerialMbotImu.byte_size + SerialTwist2D.byte_size + SerialMbotMotorVel.byte_size + SerialMbotMotorPwm.byte_size

    def __init__(self):
        self.encoders: SerialMbotEncoders = SerialMbotEncoders()
        self.odom: SerialPose2D = SerialPose2D()
        self.imu: SerialMbotImu = SerialMbotImu()
        self.mbot_vel: SerialTwist2D = SerialTwist2D()
        self.motor_vel: SerialMbotMotorVel = SerialMbotMotorVel()
        self.motor_pwm: SerialMbotMotorPwm = SerialMbotMotorPwm()

    def decode(self, data: bytes):
        offset = 0
        self.encoders.decode(data[offset:offset + SerialMbotEncoders.byte_size])
        offset += SerialMbotEncoders.byte_size

        self.odom.decode(data[offset:offset + SerialPose2D.byte_size])
        offset += SerialPose2D.byte_size

        self.imu.decode(data[offset:offset + SerialMbotImu.byte_size])
        offset += SerialMbotImu.byte_size

        self.mbot_vel.decode(data[offset:offset + SerialTwist2D.byte_size])
        offset += SerialTwist2D.byte_size

        self.motor_vel.decode(data[offset:offset + SerialMbotMotorVel.byte_size])
        offset += SerialMbotMotorVel.byte_size

        self.motor_pwm.decode(data[offset:])

    def encode(self) -> bytes:
        return self.encoders.encode() + self.odom.encode() + self.imu.encode() + self.mbot_vel.encode() + self.motor_vel.encode() + self.motor_pwm.encode()

'''    
uint8_t checksum(uint8_t* addends, int len) {
    int sum = 0;
    for (int i = 0; i < len; i++) {
        sum += addends[i];
    }
    return 255 - ((sum) % 256);
}
'''

def checksum(addends: bytes) -> int:
    return 255 - (sum(addends) % 256)

def mac_bytes_to_str(mac: bytes) -> str:
    return ':'.join([f'{b:02x}' for b in mac])

def encode_vel_cmd(msg: SerialTwist2D) -> bytes:
    len = msg.byte_size
    topic = 214
    out = bytes([SYNC_FLAG, VERSION_FLAG])
    len_bytes = struct.pack('<H', len)
    cs1 = checksum(len_bytes)
    out += len_bytes
    out += bytes([cs1])
    topic_bytes = struct.pack('<H', topic)
    msg_bytes = msg.encode()
    cs2 = checksum(topic_bytes + msg_bytes)
    out += topic_bytes
    out += msg_bytes
    out += bytes([cs2])
    return out

def encode_cmd_packet(cmd: bytes, mac: str) -> bytes:
    out = bytes([SYNC_FLAG])
    len_bytes = struct.pack('<H', len(cmd))
    mac_bytes = bytes.fromhex(mac.replace(':', ''))
    out += len_bytes
    out += mac_bytes
    out += cmd
    return out