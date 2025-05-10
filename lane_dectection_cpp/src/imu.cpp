#include "imu.hpp"
#include "share.hpp"
#include "debug.hpp"
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <thread>
#include <iostream>
#include <cmath>
#include <chrono>


class SimpleKalman {
    public:
        SimpleKalman(float q = 0.008, float r = 0.8) : x(0.0), p(1.0), q(q), r(r) {}
        float update(float measurement) {
            p += q;
            float k = p / (p + r);
            x += k * (measurement - x);
            p *= (1 - k);
            return x;
        }
    private:
        float x, p, q, r;
    };
 
SimpleKalman kf_ax, kf_ay;

static float convertToFloat(uint8_t lsb, uint8_t msb, float scale) {
    int16_t raw = (msb << 8) | lsb;
    return static_cast<float>(raw) / scale;
}

bool imu_init(int &file) {
    file = open(I2C_DEV, O_RDWR);
    if (file < 0) return false;
    if (ioctl(file, I2C_SLAVE, BNO055_ADDR) < 0) return false;

    uint8_t mode_cmd[2] = { BNO055_OPR_MODE_ADDR, 0x0C };
    write(file, mode_cmd, 2);
    usleep(20000);
    return true;
}

bool imu_read_data(int file, ImuData &data) {
    uint8_t reg;
    uint8_t buffer[6];

    // Euler angles
    reg = BNO055_EULER_H_LSB_ADDR;
    if (write(file, &reg, 1) != 1 || read(file, buffer, 6) != 6) return false;
    data.yaw = convertToFloat(buffer[0], buffer[1], 16.0f);
    data.pitch = convertToFloat(buffer[2], buffer[3], 16.0f);
    data.roll = convertToFloat(buffer[4], buffer[5], 16.0f);
    if (data.yaw > 180.0f) data.yaw -= 360.0f;

    // Linear acceleration
    reg = BNO055_LIN_ACCEL_DATA_X_LSB_ADDR;
    if (write(file, &reg, 1) != 1 || read(file, buffer, 6) != 6) return false;
    data.ax = kf_ax.update(convertToFloat(buffer[0], buffer[1], 100.0f));
    data.ay = kf_ay.update(convertToFloat(buffer[2], buffer[3], 100.0f));

    float yaw_rad = data.yaw * M_PI / 180.0f;
    data.accel_signed = data.ax * cos(yaw_rad) + data.ay * sin(yaw_rad);
    if (std::abs(data.accel_signed) < 0.05f) data.accel_signed = 0.0f;

    return true;
}

void imu_reader_random(const bool isRandom) {
    while (!stop_flag) {
        if (isRandom != 0) {
            float yaw = rand() % 360 - 180;
            float pitch = rand() % 180 - 90;
            float roll = rand() % 360 - 180;
            float ax = (rand() % 200 - 100) / 100.0f;
            float ay = (rand() % 200 - 100) / 100.0f;
            float yaw_rad = yaw * M_PI / 180.0f;
            float a_signed = ax * cos(yaw_rad) + ay * sin(yaw_rad);
            if (std::abs(a_signed) < 0.05f) a_signed = 0.0f;

            {
                std::lock_guard<std::mutex> lock(mtx);
                imu_data[0] = yaw;
                imu_data[1] = pitch;
                imu_data[2] = roll;
                imu_data[3] = ax;
                imu_data[4] = ay;
                imu_data[5] = a_signed;
                imu_ready = true;
            }
            cv_imu.notify_all();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void imu_reader_bno055() {
    int file;
    if (!imu_init(file)) {
        std::cerr << "Failed to initialize IMU in thread" << std::endl;
        return;
    }

    ImuData data;
    while (!stop_flag) {
        if (imu_read_data(file, data)) {
            {
                std::lock_guard<std::mutex> lock(mtx);
                imu_data[0] = data.yaw;
                imu_data[1] = data.pitch;
                imu_data[2] = data.roll;
                imu_data[3] = data.ax;
                imu_data[4] = data.ay;
                imu_data[5] = data.accel_signed;
                imu_ready = true;
            }
            cv_imu.notify_all();
        } else {
            std::cerr << "Failed to read IMU data" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    close(file);
}