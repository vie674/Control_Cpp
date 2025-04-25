#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // === Thông số nội tại của camera ===
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
        262.08953333143063, 0.0,               330.77574325128484,
        0.0,               263.57901348164575, 250.50298224489268,
        0.0,               0.0,                1.0);

    // === Hệ số méo ===
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) <<
        -0.27166331922859776, 0.09924985737514846,
        -0.0002707688044880526, 0.0006724194580262318,
        -0.01935517123682299);

    // === Mở video ===
    cv::VideoCapture cap("thap.mp4");  // hoặc cv::VideoCapture(0) để mở webcam
    if (!cap.isOpened()) {
        std::cerr << "Không thể mở video hoặc webcam!" << std::endl;
        return -1;
    }

    cv::Mat frame, undistorted;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        // === Chỉnh méo ===
        cv::undistort(frame, undistorted, cameraMatrix, distCoeffs);

        // === Hiển thị ===
        cv::imshow("Frame Gốc", frame);
        cv::imshow("Frame Sau Khi Chỉnh Méo", undistorted);

        // Nhấn 'q' để thoát
        if (cv::waitKey(10) == 'q') break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
