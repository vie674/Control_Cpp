#include <opencv2/opencv.hpp>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <vector>

#define SERVER_IP "192.168.1.109"  // 🛑 Đặt IP của máy Windows
#define SERVER_PORT 8888
#define MAX_PACKET_SIZE 65000

int main() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in servAddr{};
    servAddr.sin_family = AF_INET;
    servAddr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &servAddr.sin_addr);

    cv::VideoCapture cap(2);
    if (!cap.isOpened()) {
        std::cerr << "Không mở được video.\n";
        return -1;
    }

    while (true) {
        cv::Mat frame;
        cap >> frame;
        
        std::vector<uchar> buf;
        cv::imencode(".jpg", frame, buf, {cv::IMWRITE_JPEG_QUALITY, 50});

        if (buf.size() > MAX_PACKET_SIZE) {
            std::cerr << "Frame quá lớn! Hãy giảm độ phân giải hoặc tăng nén.\n";
            continue;
        }

        sendto(sock, buf.data(), buf.size(), 0,
               (sockaddr*)&servAddr, sizeof(servAddr));
        cv::imshow ("anh", frame);
        if (cv::waitKey(1) == 27) break;
    }

    close(sock);
    return 0;
}
