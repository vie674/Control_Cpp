#include "imu.hpp"
#include "share.hpp"
#include "debug.hpp"
#include <thread>
#include <iostream>
#include <cmath>
#include <chrono>

void imu_reader(const bool isRandom) {
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

