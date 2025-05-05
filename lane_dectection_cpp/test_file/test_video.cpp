#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

// Function to split the image by angle
void splitImageByAngle(const cv::Mat& inputImage, cv::Mat& outputLeft, cv::Mat& outputRight, double alpha_deg) {
    int width = inputImage.cols;
    int height = inputImage.rows;

    // Origin point (320, 480) - center of the image (can be adjusted based on the application)
    cv::Point origin(320, 480);  // Thay đổi nếu cần thiết

    // Convert alpha angle from degrees to radians
    double alpha = alpha_deg * CV_PI / 180.0;

    // Create a line based on the angle alpha from the origin
    double slope = std::tan(alpha);

    // Create masks to separate left and right parts of the image
    cv::Mat maskLeft = cv::Mat::zeros(height, width, CV_8UC1);
    cv::Mat maskRight = cv::Mat::zeros(height, width, CV_8UC1);

    // Nếu góc là 90 độ (hoặc gần 90 độ), thì tạo một đường thẳng đứng chia đôi ảnh
    if (alpha_deg == 90) {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (x < width / 2) {
                    maskLeft.at<uchar>(y, x) = 255;  // Phần bên trái
                } else {
                    maskRight.at<uchar>(y, x) = 255;  // Phần bên phải
                }
            }
        }
    } else {
        // Xử lý các góc khác, sử dụng toán học như bình thường
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                // Calculate the y-coordinate on the line at x
                int lineY = origin.y + static_cast<int>(slope * (x - origin.x));

                if (y < lineY) {
                    maskLeft.at<uchar>(y, x) = 255; // Phần bên trái
                } else {
                    maskRight.at<uchar>(y, x) = 255; // Phần bên phải
                }
            }
        }
    }

    // Áp dụng mặt nạ vào ảnh để chia thành 2 phần
    inputImage.copyTo(outputLeft, maskLeft);
    inputImage.copyTo(outputRight, maskRight);
}


// Main function to read video and apply the splitting
int main() {
    // Open video file
    cv::VideoCapture vidcap("thap.mp4");
    if (!vidcap.isOpened()) {
        std::cerr << "Failed to open video" << std::endl;
        return -1;
    }

    double fps = vidcap.get(cv::CAP_PROP_FPS);
    cv::Mat frame;

    // Loop through each frame
    while (vidcap.read(frame)) {
        cv::Mat leftImage, rightImage;

        // Split the frame at a given angle (e.g., 45 degrees)
        splitImageByAngle(frame, leftImage, rightImage, 90);

        // Display the left and right parts of the frame
        cv::imshow("Left Image", leftImage);
        cv::imshow("Right Image", rightImage);

        // Wait for 1 ms and exit if 'q' is pressed
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    vidcap.release();
    cv::destroyAllWindows();
    return 0;
}
