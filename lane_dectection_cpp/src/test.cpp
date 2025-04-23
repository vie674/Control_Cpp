#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    cv::Mat img = cv::imread("test.jpg");
    if (img.empty()) {
        std::cout << "Không thể đọc ảnh\n";
        return -1;
    }
    cv::imshow("Hình ảnh", img);
    cv::waitKey(0);
    return 0;
}
