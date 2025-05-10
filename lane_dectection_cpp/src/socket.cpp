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
            if (val >= 1 && val <= 10) {
                control_signal.store(val);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
        std::vector<float> local_imu(6);

        {
            std::lock_guard<std::mutex> lock(mtx);
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