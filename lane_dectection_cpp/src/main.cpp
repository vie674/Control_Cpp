#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <condition_variable>
#include <pthread.h>  // Dùng pthread để set CPU affinity
#include "image.hpp"
#include "debug.hpp"
#include "controller.hpp"
#include "imu.hpp"
#include "serial.hpp"
#include "socket.hpp"

// Shared resources
cv::Mat shared_frame;
float encoder_data = 0.0f;
std::vector<float> imu_data(6, 0.0f);
std::atomic<int> control_signal{0};

std::mutex mtx;
std::condition_variable cv_encoder, cv_imu;
bool encoder_ready = false, imu_ready = false;
std::atomic<bool> stop_flag{false};

void bindingToCore(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);  // Gán vào core_id

    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
        std::cerr << "[STEERING] Failed to bind thread to CPU core " << core_id << "\n";
    } 
    else {
        cpu_set_t checkset;
        CPU_ZERO(&checkset);
        pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &checkset);
        if (CPU_ISSET(core_id, &checkset)) {
            std::cout << "[STEERING] Successfully bound to CPU core " << core_id << "\n";
        } else {
            std::cerr << "[STEERING] Binding verification failed\n";
        }
    }
}

void steering_processor() {
    bindingToCore(1);

    UndistortData undistortData = setupUndistort(cameraMatrix, distCoeffs, cv::Size(640, 480));
    std::vector<int> prevLx, prevRx;
    std::vector<cv::Point2f> pts1, pts2;
    cv::Mat leftImage, rightImage, mask, warped;
    int splitAngle = 90;

    double fps = 0;     
    double lastTick = cv::getTickCount();
    /*
    if (!initializeSerial(serial_port, portName)) {
    std::cerr << "[ERROR] Không thể khởi tạo cổng serial trong encoder_reader_serial." << std::endl;
    return;
    }
    */
    mpcInit(10.0, 10.0, 5.0);
    Eigen::VectorXd x(4);
    Eigen::VectorXd v_k(6);

    x << 0.0, 0.0, 0.0, 0.0;
    while (!stop_flag) {
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(mtx);
            if (shared_frame.empty()) continue;
            shared_frame.copyTo(frame);
        }   

        int local_signal = control_signal.load();
        
        // Preprocess frame and split into left/right
        preprocessAndSplitFrame(frame, undistortData, splitAngle, leftImage, rightImage, pts1, pts2, mask, warped);

        // Detect lane lines
        std::vector<int> lx, rx;
        detectLaneLines(mask, leftImage, rightImage, prevLx, prevRx, lx, rx);
        std::cout << "FF" << lx[0] - rx[0] <<"FF";
        // Compute control parameters
        vehicleState states = computeControlParam(lx, rx, warped, splitAngle);

        for (int i = 0; i < 6; i++) {
            v_k(i) = 0 ;
        }
        
        x << states.offset, states.offset/0.03, states.angle_y *3.14 /180, (states.angle_y/0.03)*3.14 /180;


        // Calculate steering control and update split angle
        int delta = stanleyControl(states.offset, states.angle_y, 0.3, 1.4);
        //std::cout <<"--"<< states.offset << "--" << states.angle_y <<"--" << delta; 
        //int delta = mpcControl(x, v_k);
        int steeringAngle = 80 + delta;
        splitAngle = updateSplitAngle(steeringAngle);
        sendToSerial (serial_port, 4900, steeringAngle);

        std::cout << "[STEERING] Góc đánh lái = " << steeringAngle << std::endl;

        showFPS(lastTick, fps);
        // Visualize and show results
        auto result = visualize(frame, warped, warped, pts1, pts2, delta, states);
        cv::imshow("Lane Detection", result);
        cv::imshow("Mask", mask);
        cv::imshow("left",leftImage);
        cv::imshow("right,",  rightImage);
        if (cv::waitKey(10) == 27) break;
    }

    std::cout << "[STEERING] Thread exited." << std::endl;
    sendToSerial (serial_port, 0, 80);
}


int main() {

    if (!initializeSerial(serial_port, portName)) {
    std::cerr << "[ERROR] Không thể khởi tạo cổng serial trong encoder_reader_serial." << std::endl;
    return 1;
    }

    std::thread t1(image_reader, IMAGE_READ_FROM_CAM); // Truyền đối số IMAGE_READ_FROM_VIDEO vào image_reader
    std::thread t2(steering_processor);  // steering_processor không có đối sốh
    std::thread t3(encoder_reader_serial) ;  // Truyền serial_port và SERIAL_READ vào encoder_reader
    std::thread t4(imu_reader_bno055);  // Truyền IMU_RANDOM vào imu_reader
    std::thread t5(signal_receiver, 8888);  // Truyền cổng vào signal_receiver
    std::thread t6(server_uploader, "192.168.1.113", 8890);  // Truyền địa chỉ IP và cổng vào server_uploader

    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
    t6.join();

    return 0;
}
