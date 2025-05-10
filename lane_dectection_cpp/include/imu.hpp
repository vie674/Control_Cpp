#ifndef IMU_HPP
#define IMU_HPP

#include <cstdint>

#define IMU_RANDOM 1
#define I2C_DEV "/dev/i2c-1"
#define BNO055_ADDR 0x29

#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_EULER_H_LSB_ADDR 0x1A
#define BNO055_LIN_ACCEL_DATA_X_LSB_ADDR 0x28

struct ImuData {
    float yaw;
    float pitch;
    float roll;
    float ax;
    float ay;
    float accel_signed;
};

bool imu_init(int &file);
bool imu_read_data(int file, ImuData &data);
void imu_reader_random(const bool isRandom);
void imu_reader_bno055();


#endif