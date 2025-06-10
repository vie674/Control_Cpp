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
float shared_steer = 0.0f;
std::vector<float> imu_data(7, 0.0f);
std::atomic<int> control_signal{10};


std::mutex ImageMtx;
std::mutex ImuMtx;
std::mutex EncMtx;
std::mutex SteerMtx;

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

    mpcInit(1000.0, 50.0, 5.0);
    Eigen::VectorXd x(4);
    Eigen::VectorXd v_k(10);

    float desired_velocity = 0.3;
    x << 0.0, 0.0, 0.0, 0.0;
    vehicleState states;
    vehicleState priv_state {
        .curvature = {0.0f, 10.0f},
        .angle_y = 0.0f
    };

    while (!stop_flag) {
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(ImageMtx);
            if (shared_frame.empty()) continue;
            shared_frame.copyTo(frame);
        }

        int local_signal = control_signal.load();

        if (local_signal == 0) {
            desired_velocity = 0.0f;
            //sendSpeedPWM(serial_port, 0);
        }
        else if (local_signal == 5) {
            desired_velocity = 0.15f;
            //sendSpeedPWM(serial_port, 4500);
        }
        else if (local_signal == 10)
        {
            desired_velocity = 0.3f;
            //sendSpeedPWM(serial_port, 6660);
        }

        sendSpeedPID(serial_port, static_cast<int> (desired_velocity*100));

        // Preprocess frame and split into left/right
        preprocessAndSplitFrame(frame, undistortData, splitAngle, leftImage, rightImage, pts1, pts2, mask, warped);

        // Detect lane lines
        std::vector<int> lx, rx;
        detectLaneLines(mask, leftImage, rightImage, prevLx, prevRx, lx, rx);
        std::cout << " Pixel per lane " << lx[0] - rx[0] <<" ";
        // Compute control parameters
        std::cout << local_signal << std::endl;
        states = computeControlParam(lx, rx, warped, splitAngle);
        std::cout << "Offset " << states.offset << " " << "Angle " << states.angle_y; ;
        for (int i = 0; i < 10; i++) {
            //std::cout << " Curvature " << i << " " << states.curvature[i] << " ";
            v_k(i) = states.curvature[i] * desired_velocity;
        }
        

        x(0) = states.offset;
        x(1) =0;// (states.offset - priv_state.offset) / 0.071;
        x(2) = states.angle_y *3.14 /180;
        x(3) =0;// (states.angle_y - priv_state.angle_y) *3.14 /180 /0.071;

        priv_state = states;
        // Calculate steering control and update split angle
        //int delta = stanleyControl(states.offset, states.angle_y, desired_velocity, 1.75);
        //std::cout <<"--"<< states.offset << "--" << states.angle_y <<"--" << delta; 
        int delta = mpcControl(x, v_k);

        int steeringAngle = 80 + delta;
        //splitAngle = updateSplitAngle(steeringAngle);
        {
            std::lock_guard<std::mutex> lock(SteerMtx);
            shared_steer = delta;
        }
        sendSteering(serial_port, steeringAngle);
        //sendSpeedPID(serial_port, 30);

        std::cout << "[STEERING] Góc đánh lái = " << steeringAngle << std::endl;
        showFPS(lastTick, fps);
        // Visualize and show results
        //auto result = visualize(frame, warped, warped, pts1, pts2, delta, states);
        //cv::imshow("Lane Detection", result);
        //cv::imshow("Mask", mask);
        //cv::imshow("left",leftImage);
        //cv::imshow("right,",  rightImage);
        if (cv::waitKey(10) == 27) break;
    }

    std::cout << "[STEERING] Thread exited." << std::endl;
    sendToSerial (serial_port, 0, 80);
}


int main(int argc, char* argv[]) {
   
    // // Trạng thái ban đầu x0 từ tham số dòng lệnh
    // if (argc != 5) {
    //     std::cerr << "Error: You must provide 4 parameters for x0." << std::endl;
    //     return -1;
    // }

    

    
    // // Khởi tạo các tham số và trạng thái
    // float Q1Coff = 10.0f;
    // float Q2Coff = 1.2f;
    // float RCoff = 1;

    
    // Eigen::VectorXd x0(4);
    // x0 << std::atof(argv[1]),  // x0(1)
    //       std::atof(argv[2]),  // x0(2)
    //       std::atof(argv[3]) *3.14/180,  // x0(3)
    //       std::atof(argv[4]);  // x0(4)

    // // Khởi tạo mpc với các giá trị cố định
    // mpcInit(Q1Coff, Q2Coff, RCoff);

    // // Vector điều khiển v_k (ví dụ)
    // Eigen::VectorXd v_k(10);
    // v_k << 0, 0, 0, 0,0,0,0,0,0,0; // [delta, a, v_x, v_y] ví dụ [0, 0, 0, 0]

    // // Điều khiển MPC
    // float u_cmd = mpcControl(x0, v_k);
    // std::cout << "MPC control command (u_cmd): " << u_cmd << " degrees" << std::endl;

    if (!initializeSerial(serial_port, portName)) {
        std::cerr << "[ERROR] Không thể khởi tạo cổng serial trong encoder_reader_serial." << std::endl;
        return 1;
        }
        

    sendToSerial (serial_port, 0, 80);
    std::thread t1(image_reader, IMAGE_READ_FROM_CAM);// Truyền đối số IMAGE_READ_FROM_VIDEO vào image_reader
    std::thread t2(steering_processor);  // steering_processor không có đối sốh
    std::thread t3(encoder_reader_serial) ;  // Truyền serial_port và SERIAL_READ vào encoder_reader
    std::thread t4(imu_reader_bno055);  // Truyền IMU_RANDOM vào imu_reader
    std::thread t5(signal_receiver, 8888);  // Truyền cổng vào signal_receiver
    std::thread t6(server_uploader, "192.168.1.111", 8890);  // Truyền địa chỉ IP và cổng vào server_uploader
    std::thread t7(save_data_cal_tire);
    
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
    //t6.join();
    t7.join();
    return 0;
}
