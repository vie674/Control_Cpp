#ifndef SHARE_H
#define SHARE_H

#include <atomic>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <thread>
#include <opencv2/opencv.hpp>

extern cv::Mat shared_frame;
extern float encoder_data;
extern std::vector<float> imu_data;
extern std::atomic<int> control_signal;

extern std::mutex mtx;
extern std::condition_variable cv_encoder, cv_imu;
extern bool encoder_ready, imu_ready;
extern std::atomic<bool> stop_flag;


#endif