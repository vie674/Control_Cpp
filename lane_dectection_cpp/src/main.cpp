#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include "utils.hpp"


int main() {
    // 1. Open video file
    cv::VideoCapture vidcap("thap.mp4");
    if (!vidcap.isOpened()) {
        std::cerr << "Failed to open video" << std::endl;
        return -1;
    }
    
    double fps = 0;
    double lastTick = cv::getTickCount();

    // Camera intrinsic parameters (calibration data)
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
        262.08953333143063, 0.0, 330.77574325128484,
        0.0, 263.57901348164575, 250.50298224489268,
        0.0, 0.0, 1.0);

    // Camera distortion coefficients
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) <<
        -0.27166331922859776, 0.09924985737514846,
        -0.0002707688044880526, 0.0006724194580262318,
        -0.01935517123682299);

    UndistortData undistortData = setupUndistort(cameraMatrix, distCoeffs, cv::Size(640, 480));

    cv::Mat frame, warped, mask, overlay, leftImage, rightImage;
    std::vector<int> prevLx, prevRx;
    int del = 90;
    
    while (true) {
        vidcap >> frame;
        if (frame.empty()) break;
        
        // Undistort frame
        frame = undistortFrame(frame, undistortData);
        cv::resize(frame, frame, cv::Size(640, 480));
        // Apply perspective transform to the frame
        std::vector<cv::Point2f> pts1, pts2;
        warped = perspectiveTransform(frame, pts1, pts2);

        // Apply HSV color selection to get lane mask
        mask = HSVColorSelection(warped);
        leftImage = cv::Mat::zeros(640, 480, CV_8UC1);
        rightImage = cv::Mat::zeros(640, 480, CV_8UC1);
        // Split image by angle
        splitImageByAngle(mask, leftImage, rightImage, del);  // Use alpha = 90 degrees

        // Resize images if necessary
        cv::resize(leftImage, leftImage, cv::Size(640, 480));
        cv::resize(rightImage, rightImage, cv::Size(640, 480));

        // Get histograms for left and right regions
        auto hist_left = getHistogram(leftImage);
        auto hist_right = getHistogram(rightImage);

        // Get the base positions for both left and right histograms
        int left_base = getBase(hist_left, 0, hist_left.cols);
        int right_base = getBase(hist_right, 0, hist_right.cols);  // Corrected this line

        // Detect lane points based on base positions
        auto lx = detectLanePoints(mask, left_base);
        auto rx = detectLanePoints(mask, right_base);

        // If lane points are not found, use previous values
        if (lx.empty()) lx = prevLx;
        if (rx.empty()) rx = prevRx;

        prevLx = lx;
        prevRx = rx;

        overlay = warped.clone();
        vehicleState states;

        // Handle left and right lanes based on detected points
        if (lx.size() <= 1 && rx.size() >= 3) {
            states = handleRightLaneOnly(rx, overlay);
        }
        else if (lx.size() >= 3 && rx.size() <= 1) {
            states = handleLeftLaneOnly(lx, overlay);
        }
        else {
            states = handleBothLanes(lx, rx, overlay);
        }

        // Calculate steering angle using Stanley controller
        int delta = stanleyControl(states.offset, states.angle_y, 0.4, 0.79);
        int steeringAngle = 80 + delta;
        del = steeringAngle + 10;
        // Visualize results
        auto result = visualize(frame, warped, overlay, pts1, pts2, delta, states);
        showFPS(lastTick, fps);
        cv::imshow("Left", leftImage);
        cv::imshow("Right", rightImage);
        cv::imshow("Lane Detection", result);
        cv::imshow("Mask", mask);

        if (cv::waitKey(10) == 27) break;  // Exit if 'Esc' is pressed
    }

    vidcap.release();
    cv::destroyAllWindows();
    return 0;
}
