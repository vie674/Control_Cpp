// socket.cpp
#include "socket.hpp"
#include "share.hpp"
#include "debug.hpp"
#include <iostream>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <cstdint> 
#include <ctime>   
#include <iomanip>      // for std::put_time
#include <sstream>  
#include <fstream>  

void signal_receiver(int port) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in servAddr{};
    servAddr.sin_family = AF_INET;
    servAddr.sin_port = htons(port);
    servAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    bind(sock, (sockaddr*)&servAddr, sizeof(servAddr));
    listen(sock, 1);

    std::cout << "[SIGNAL RECEIVER] Waiting for TCP connection on port 8888..." << std::endl;
    sock = accept(sock, nullptr, nullptr);
    std::cout << "[SIGNAL RECEIVER] Client connected." << std::endl;

    char buffer[1024];
    while (!stop_flag) {
        ssize_t len = recv(sock, buffer, sizeof(buffer) - 1, 0);
        if (len > 0) {
            buffer[len] = '\0';
            int val = std::stoi(buffer);
            if (val >= 0 && val <= 10) {
                control_signal.store(val);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    close(sock);
}


void server_uploader(std::string serverIpAddr, int port) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in server{};
    server.sin_family = AF_INET;
    server.sin_port = htons(port);
    inet_pton(AF_INET, serverIpAddr.c_str(), &server.sin_addr);

    if (connect(sock, (sockaddr*)&server, sizeof(server)) < 0) {
        std::cerr << "[ERROR] Cannot connect to TCP server" << std::endl;
        return;
    }

    while (!stop_flag) {
        cv::Mat local_frame;
        float local_encoder;
        std::vector<float> local_imu(7);

        {
            std::scoped_lock lock(ImageMtx, ImuMtx, EncMtx);
            if (shared_frame.empty()) continue;

            shared_frame.copyTo(local_frame);
            local_encoder = encoder_data;
            std::copy(imu_data.begin(), imu_data.end(), local_imu.begin());
        }

        std::vector<uchar> buf;
        cv::imencode(".jpg", local_frame, buf);

        int32_t size = buf.size();
        send(sock, &size, sizeof(size), 0);
        send(sock, buf.data(), size, 0);

        std::string sensor_data = "ENC:" + std::to_string(local_encoder) +
                                  ",YAW:" + std::to_string(local_imu[0]) +
                                  ",PIT:" + std::to_string(local_imu[1]) +
                                  ",ROL:" + std::to_string(local_imu[2]) +
                                  ",AX:" + std::to_string(local_imu[3]) +
                                  ",AY:" + std::to_string(local_imu[4]) +
                                  ",ASIG:" + std::to_string(local_imu[5]);

        int32_t text_len = sensor_data.size();
        send(sock, &text_len, sizeof(text_len), 0);
        send(sock, sensor_data.c_str(), text_len, 0);
        
        #ifdef DEBUG_TERMINAL_SOCKET
            std::cout << "[UPLOAD] " << sensor_data << std::endl;
        #endif
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    close(sock);
}

void save_data() {
    std::ofstream outfile("output.txt");

    if (!outfile.is_open()) {
        std::cerr << "[ERROR] Cannot open output.txt for writing" << std::endl;
        return;
    }

    auto start_time = std::chrono::steady_clock::now();  // mốc bắt đầu

    while (!stop_flag) {
        float local_encoder;
        std::vector<float> local_imu(7);

        {
            std::scoped_lock lock(ImuMtx, EncMtx);
            local_encoder = encoder_data;
            std::copy(imu_data.begin(), imu_data.end(), local_imu.begin());
        }

        // Tính thời gian đã trôi qua kể từ lúc bắt đầu (đơn vị giây)
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - start_time;

        std::ostringstream timestamp;
        timestamp << std::fixed << std::setprecision(6) << elapsed.count();

        std::string sensor_data = timestamp.str() + " - " +
                                  "ENC:" + std::to_string(local_encoder) +
                                  ",YAW:" + std::to_string(local_imu[0]) +
                                  ",PIT:" + std::to_string(local_imu[1]) +
                                  ",ROL:" + std::to_string(local_imu[2]) +
                                  ",AX:" + std::to_string(local_imu[3]) +
                                  ",AY:" + std::to_string(local_imu[4]) +
                                  ",ASIG:" + std::to_string(local_imu[5]);

        outfile << sensor_data << std::endl;

        #ifdef DEBUG_TERMINAL_SOCKET
            std::cout << "[LOG] " << sensor_data << std::endl;
        #endif

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    outfile.close();
}

void save_data_cal_tire() {
    std::ofstream outfile("tire_calibration.csv");

    if (!outfile.is_open()) {
        std::cerr << "[ERROR] Cannot open tire_calibration.csv for writing" << std::endl;
        return;
    }

    // Ghi dòng tiêu đề
    outfile << "time,vx,ay,yaw_rate,steering_angle\n";

    auto start_time = std::chrono::steady_clock::now();

    while (!stop_flag) {
        float local_encoder = 0.0f;     // vx
        float local_ay = 0.0f;          // ay [m/s²]
        float local_yawrate = 0.0f;     // yaw_rate [rad/s]
        float local_steering = 0.0f;    // steering angle [rad]

        {
            std::scoped_lock lock(EncMtx, ImuMtx, SteerMtx);
            local_encoder = encoder_data;
            local_ay = imu_data[4];
            local_yawrate = imu_data[6];
            local_steering = shared_steer;
        }

        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - start_time;

        outfile << std::fixed << std::setprecision(6)
                << elapsed.count() << ","
                << local_encoder << ","
                << local_ay << ","
                << local_yawrate << ","
                << local_steering << "\n";

        #ifdef DEBUG_TERMINAL_SOCKET
            std::cout << "[CAL_TIRE_CSV] time=" << elapsed.count()
                      << " vx=" << local_encoder
                      << " ay=" << local_ay
                      << " yaw_rate=" << local_yawrate
                      << " δ=" << local_steering << std::endl;
        #endif

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    outfile.close();
}
