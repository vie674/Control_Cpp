// File: main.cpp
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include "utils.hpp"



int main() {
   // 1. Mở camera
   cv::VideoCapture vidcap("output_video.mp4");
   if (!vidcap.isOpened()) {
       std::cerr << "Failed to open camera" << std::endl;
       return -1;
   }
   
   // 2. Mở cổng serial
   //std::string portName = "/dev/ttyACM0";
   //LibSerial::SerialPort ser;
   //initializeSerial(ser, portName);
    
    //cv::Mat cameraMatrix, distCoeffs;
    cv::Mat cameraMatrix = loadMatrix("data/cameraMatrix.txt", 3, 3,' ');         
    cv::Mat distCoeffs   = loadMatrix("data/cameraDistortion.txt", 1, 5,' '); 
    
    cv::Mat frame, warped, mask, overlay;
    std::vector<int> prevLx, prevRx;

    while (true) {
        vidcap >> frame;
        if (frame.empty()) break;

        //frame = undistortImage(frame, cameraMatrix, distCoeffs);
        cv::resize(frame, frame, cv::Size(640, 480));

        std::vector<cv::Point2f> pts1, pts2;
        warped = perspectiveTransform(frame, pts1, pts2);

        mask = HSVColorSelection(warped);
        auto hist = getHistogram(mask);
        int midpoint = hist.cols / 2;
        int left_base = getBase(hist, 0, midpoint);
        int right_base = getBase(hist, midpoint, hist.cols) + midpoint;

        auto lx = detectLanePoints(mask, left_base);
        auto rx = detectLanePoints(mask, right_base);
        if (lx.empty()) lx = prevLx;
        if (rx.empty()) rx = prevRx;
        prevLx = lx;
        prevRx = rx;
        
        overlay = warped.clone();
        vehicleState states;

        if (lx.size() <= 1 && rx.size() >= 3) {
            states = handleRightLaneOnly(rx, overlay);
        }
        else if (lx.size() >= 3 && rx.size() <= 1) {
            states = handleLeftLaneOnly(lx, overlay);
        }
        else if (lx.size() >= 2 && rx.size() >=2) {
            states = handleBothLanes(lx, rx, overlay);
        }
        else {
            states.offset = 0;
            states.angle_y = 0;
            states.curvature = 0;
        }

        int delta = stanleyControl(states.offset, states.angle_y, 0.4, 0.79);
        int steeringAngle = 80 + delta;
        
        //sendToSerial(ser, 4700, steeringAngle);

        auto result = visualize(frame, warped, overlay, pts1, pts2, delta, states);
        cv::imshow("Lane Detection", result);
        cv::imshow("Tranformation", warped);
        cv::imshow("Mask", mask);
        cv::imshow("Overlay", overlay);
        if (cv::waitKey(10) == 27) break;
    }

    vidcap.release();
    cv::destroyAllWindows();
    return 0;
}
