// File: main.cpp
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include "utils.hpp"
//#include <serial/serial.h>


int main() {
   // 1. Mở camera
   cv::VideoCapture vidcap(0);
   if (!vidcap.isOpened()) {
       std::cerr << "Failed to open camera" << std::endl;
       return -1;
   }
   /*
   // 2. Mở cổng serial
   serial::Serial ser;
   try {
       ser.setPort("/dev/ttyACM0");
       ser.setBaudrate(115200);
       ser.setTimeout(serial::Timeout::simpleTimeout(1000));
       ser.open();
   } catch (const std::exception& e) {
       std::cerr << "Failed to open serial port: " << e.what() << std::endl;
       return -1;
   }
   /*
    cv::Mat cameraMatrix, distCoeffs;
    cameraMatrix = loadMatrix("cameraMatrix.txt");
    distCoeffs = loadMatrix("cameraDistortion.txt");
    */
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

        if (lx.size() <= 1 && rx.size() > 3) {
            states = handleRightLaneOnly(rx, overlay);
        }
        else if (lx.size() > 3 && rx.size() <= 1) {
            states = handleLeftLaneOnly(lx, overlay);
        }
        else if (lx.size() >= 2 && rx.size() >=2) {
            states = handleBothLanes(lx, rx, overlay);
        }
        else {
            break;
        }

        int delta = stanleyControl(states.offset, states.angle_y, 0.4, 0.79);
        int steeringAngle = 80 + delta;
        
        sendToSerial(ser, 4900, 80 + denta);

        auto result = visualize(frame, warped, overlay, pts1, pts2, denta, offset, angle_y);
        cv::imshow("Lane Detection", result);

        if (cv::waitKey(10) == 27) break;
    }

    vidcap.release();
    cv::destroyAllWindows();
    return 0;
}
