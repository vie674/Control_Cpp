#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    std::string filename = "output_video.mp4"; // Đặt tên file ở đây
    cv::VideoCapture cap(filename);

    if (!cap.isOpened()) {
        std::cerr << "Không thể mở file video: " << filename << std::endl;
        return 1;
    }

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cv::imshow("Video Playback", frame);
        if (cv::waitKey(30) == 27) break;  // Nhấn ESC để thoát
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
