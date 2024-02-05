#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <Python.h>

class Serial {
public:
    Serial(const std::string& port, unsigned int baudrate);
    void read(uint8_t* buffer, size_t size);
    void write(const uint8_t* buffer, size_t size);

    static void init();
    struct PortInfo {
        std::string device;
        std::string name;
        std::string description;
        std::string hwid;
    };
    static std::vector<PortInfo> listPorts();
private:
    int serial;
};

template <typename PackedType>
void serialize(const PackedType& src, uint8_t* dest);

template <typename PackedType>
void deserialize(const uint8_t* src, PackedType& dest);

#pragma pack(push, 1)
class MbotEncoders{
public:
    int64_t utime;
    int64_t ticks[3]; // no units
    int32_t delta_ticks[3]; // no units
    int32_t delta_time; // [usec]
};

class Pose2D{
public:
    int64_t utime;
    float x;
    float y;
    float theta;
};

class MbotImu{
public:
    int64_t utime;
    float gyro[3];
    float accel[3];
    float mag[3];
    float angles_rpy[3]; // roll (x), pitch (y), yaw, (z)
    float angles_quat[4]; // quaternion
    float temp;
};

class Twist2D{
public:
    int64_t utime;
    float vx;
    float vy;
    float wz;
};

class MbotMotorVel{
public:
    int64_t utime;
    float velocity[3]; // [rad/s]
};

class MbotMotorPwm{
public:
    int64_t utime;
    float pwm[3]; // [-1.0..1.0]
};

class MbotPacket{
public:
    MbotEncoders encoders;
    Pose2D odom;
    MbotImu imu;
    Twist2D mbot_vel;
    MbotMotorVel motor_vel;
    MbotMotorPwm motor_pwm;
};
#pragma pack(pop)