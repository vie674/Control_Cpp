// File: utils.cpp
#include "utils.hpp"
#include <fstream>
#include <iostream>
#include <numeric>
#include <cmath>
//#include <serial/serial.h>

struct vehicleState {
    float angle_y = 0.0f;
    float offset = 0.0f;
    float curvature = 0.0f;
};

cv::Mat loadMatrix(const std::string& path) {
    std::ifstream file(path);
    std::vector<double> values;
    double val;
    while (file >> val) values.push_back(val);
    int size = static_cast<int>(std::sqrt(values.size()));
    return cv::Mat(values).reshape(1, size);
}
//?
cv::Mat undistortImage(const cv::Mat& image, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
    cv::Mat newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, image.size(), 1);
    cv::Mat map1, map2, undistorted;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), newCameraMatrix, image.size(), CV_16SC2, map1, map2);
    cv::remap(image, undistorted, map1, map2, cv::INTER_LINEAR);
    return undistorted;
}
//có thể xem lại
cv::Mat HSVColorSelection(const cv::Mat& image) {
    cv::Mat hsv, white_mask, yellow_mask, combined, gray;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, 200), cv::Scalar(180, 50, 255), white_mask);
    cv::inRange(hsv, cv::Scalar(10, 100, 100), cv::Scalar(25, 255, 255), yellow_mask);
    cv::bitwise_or(white_mask, yellow_mask, combined);
    cv::bitwise_and(image, image, gray, combined);
    cv::cvtColor(gray, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(7, 7), 0);
    return gray;
}

//oki
cv::Mat perspectiveTransform(const cv::Mat& frame, std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2) {
    int height = frame.rows, width = frame.cols;
    pts1 = { {0, height}, {float(width), float(height)}, {float(width * 0.8), float(height * 0.65)}, {float(width * 0.25), float(height * 0.65)} };
    pts2 = { {0, float(height)}, {float(width), float(height)}, {float(width), 0}, {0, 0} };
    auto M = cv::getPerspectiveTransform(pts1, pts2);
    cv::Mat warped;
    cv::warpPerspective(frame, warped, M, frame.size());
    return warped;
}

cv::Mat getHistogram(const cv::Mat& mask) {
    return cv::Mat(1, mask.cols, CV_32F, cv::Scalar(0)).clone();
}

int getBase(const cv::Mat& hist, int start, int end) {
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(hist.colRange(start, end), &minVal, &maxVal, &minLoc, &maxLoc);
    return maxLoc.x;
}

std::vector<int> detectLanePoints(const cv::Mat& mask, int base) {
    std::vector<int> points;
    int y = mask.rows - 8;
    while (y > mask.rows * 0.55) {
        cv::Rect roi(base - 30, y - 35, 60, 35);
        cv::Mat img = mask(roi);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(img, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        for (auto& cnt : contours) {
            auto M = cv::moments(cnt);
            if (M.m00 > 50) {
                int cx = int(M.m10 / M.m00);
                points.push_back(base - 30 + cx);
                base = base - 30 + cx;
            }
        }
        y -= 35;
    }
    return points;
}
//? giong ham fit polomial
std::vector<double> fitLaneCurve(const std::vector<int>& points) {
    std::vector<cv::Point2f> pts;
    for (size_t i = 0; i < points.size(); ++i)
        pts.emplace_back(points[i], 470 - i * 35);
    if (pts.size() < 2) return {};
    cv::Mat A(pts.size(), 3, CV_64F), Y(pts.size(), 1, CV_64F);
    for (size_t i = 0; i < pts.size(); ++i) {
        A.at<double>(i, 0) = pts[i].y * pts[i].y;
        A.at<double>(i, 1) = pts[i].y;
        A.at<double>(i, 2) = 1;
        Y.at<double>(i, 0) = pts[i].x;
    }
    cv::Mat coeffs;
    cv::solve(A, Y, coeffs, cv::DECOMP_SVD);
    return { coeffs.at<double>(0), coeffs.at<double>(1), coeffs.at<double>(2) };
}

void drawPolynomial(cv::Mat& image, const std::vector<double>& coeffs, const cv::Scalar& color) {
    for (int y = 0; y < image.rows - 1; y += 5) {
        int x1 = coeffs[0]*y*y + coeffs[1]*y + coeffs[2];
        int y1 = y;
        int x2 = coeffs[0]*(y+5)*(y+5) + coeffs[1]*(y+5) + coeffs[2];
        int y2 = y + 5;
        cv::line(image, {x1, y1}, {x2, y2}, color, 2);
    }
}

float computeCurvature(const cv::Vec3f& fit, float y_eval, float scale_factor = 0.00062857f) {
    float dy = 2 * fit[0] * y_eval + fit[1];
    float curvature = std::pow(1 + dy * dy, 1.5f) / std::abs(2 * fit[0]);
    return curvature * scale_factor;
}

// Khớp đa thức bậc 2 theo chiều y
cv::Vec3f fitPolynomial(const std::vector<cv::Point>& pts) {
    if (pts.size() < 2) return cv::Vec3f(0, 0, 0);

    cv::Mat X(static_cast<int>(pts.size()), 3, CV_32F);
    cv::Mat Y(static_cast<int>(pts.size()), 1, CV_32F);

    for (size_t i = 0; i < pts.size(); ++i) {
        float y = static_cast<float>(pts[i].y);
        X.at<float>(static_cast<int>(i), 0) = y * y;
        X.at<float>(static_cast<int>(i), 1) = y;
        X.at<float>(static_cast<int>(i), 2) = 1;
        Y.at<float>(static_cast<int>(i), 0) = static_cast<float>(pts[i].x);
    }

    cv::Mat coeffs;
    cv::solve(X, Y, coeffs, cv::DECOMP_SVD);
    return cv::Vec3f(coeffs.at<float>(0), coeffs.at<float>(1), coeffs.at<float>(2));
}

// Tính góc nghiêng với trục Y (độ)
float calculateAngle(float slope) {
    return -std::atan(slope) * 180.0f / CV_PI;
}

// Xử lý khi chỉ có làn phải
vehicleState handleRightLaneOnly(const std::vector<int>& rx, cv::Mat& overlay) {
    vehicleState result;
    if (rx.size() < 2) return result;

    std::vector<cv::Point> rightPoints;
    for (size_t i = 0; i < rx.size(); ++i)
        rightPoints.emplace_back(rx[i], 470 - static_cast<int>(i) * 35);

    cv::Vec3f right_fit = fitPolynomial(rightPoints);
    float y_eval = 450;
    result.curvature = computeCurvature(right_fit, y_eval);
    float slope = 2 * right_fit[0] * y_eval + right_fit[1];
    result.angle_y = calculateAngle(slope);
    result.offset = -((530.0f / 2.0f) - std::abs(rx[0] - 320));

    // Vẽ trên hình ảnh
    drawPolynomial(overlay, right_fit, cv::Scalar(0, 255, 255));

    std::cout << "Right lane only:\n"
              << "Slope: " << slope << "\n"
              << "Angle wrt Y: " << result.angle_y << " deg\n"
              << "Curvature: " << result.curvature << " m\n"
              << "Offset: " << result.offset << " m\n";

    return result;
}



// Xử lý khi chỉ có làn trái
vehicleState handleLeftLaneOnly(const std::vector<int>& lx, cv::Mat& overlay) {
    vehicleState result;
    if (lx.size() < 2) return result;

    std::vector<cv::Point> leftPoints;
    for (size_t i = 0; i < lx.size(); ++i)
        leftPoints.emplace_back(lx[i], 470 - static_cast<int>(i) * 35);

    cv::Vec3f left_fit = fitPolynomial(leftPoints);
    float y_eval = 450;
    result.curvature = computeCurvature(left_fit, y_eval);
    float slope = 2 * left_fit[0] * y_eval + left_fit[1];
    result.angle_y = calculateAngle(slope);
    result.offset = ((530.0f / 2.0f) - std::abs(lx[0] - 320));

    // Vẽ trên hình ảnh
    drawPolynomial(overlay, left_fit, cv::Scalar(0, 255, 0));

    std::cout << "Left lane only:\n"
              << "Slope: " << slope << "\n"
              << "Angle wrt Y: " << result.angle_y << " deg\n"
              << "Curvature: " << result.curvature << " m\n"
              << "Offset: " << result.offset << " m\n";

    return result;
}



vehicleState handleBothLanes(const std::vector<int>& lx, const std::vector<int>& rx, cv::Mat& overlay) {
    int minLength = std::min(lx.size(), rx.size());
    if (minLength < 2) return;

    std::vector<cv::Point2f> midpoints;
    std::vector<cv::Point> leftPoints;
    std::vector<cv::Point> rightPoints;
    cv::Point2f previous_mid(320.0f, 472.0f);

    // Lấy vector midpoint và vẽ kết quả lên overlay
    for (int i = 0; i < minLength; ++i) {
        cv::Point left(lx[i], 470 - i * 35);
        cv::Point right(rx[i], 470 - i * 35);

        // Lưu các điểm bên trái và bên phải 
        leftPoints.push_back(left);
        rightPoints.push_back(right);

        // Vẽ đường nối giữa left và right
        cv::line(overlay, left, right, cv::Scalar(0, 255, 0), 1);

        // Tính midpoint
        cv::Point2f midpoint((left.x + right.x) / 2.0f, (left.y + right.y) / 2.0f);
        midpoints.push_back(midpoint);

        // Vẽ midpoint
        cv::circle(overlay, midpoint, 5, cv::Scalar(0, 0, 255), -1);

        // Vẽ đường nối các midpoint
        cv::line(overlay, previous_mid, midpoint, cv::Scalar(200, 100, 250), 1);
        previous_mid = midpoint;
    }

    // Tính toán các trạng thái của xe so với làn đường 
    vehicleState result;
    int carPosition = 320;
    int laneCenter = (lx[0] + rx[0]) / 2;

    // Tính vector offset
    result.offset = laneCenter - carPosition;

    float slope = 0;
    float intercept = 0;
    float angle_x = 0;
    float angle_y = 0;

    if (midpoints.size() == 2) {
        cv::Point2f mid1 = midpoints[0];
        cv::Point2f mid2 = midpoints[1];

        slope = (mid1.y - mid2.y) / (mid1.x - mid2.x);
        intercept = mid1.y - slope * mid1.x;
        angle_x = calculateAngle(slope);

        // Tính toán góc lệch
        if (angle_x >= 0) {
            result.angle_y = 90 - angle_x;
        }
        else {
            result.angle_y = - (90 - std::abs(angle_x));
        }
    }
    else if (midpoints.size() >= 3) {
        std::vector<cv::Point2f> midToFit;
        midToFit.pushback(midpoints[1]);
        midToFit.pushback(midpoints[2]);
        midToFit.pushback(midpoints[3]);

        cv::Vec4f lineParams;
        cv::fitLine(midToFit, lineParams, cv::DIST_L2, 0, 0.01, 0.01);

        float vx = lineParams[0];
        float vy = lineParams[1];
        float x0 = lineParams[2];
        float y0 = lineParams[3];

        slope = vy / vx;
        intercept = y0 - slope * x0;
        angle_x = calculateAngle(slope);

        // Tính toán góc lệch
        if (angle_x >= 0) {
            result.angle_y = 90 - angle_x;
        }
        else {
            result.angle_y = - (90 - std::abs(angle_x));
        }
    }

    cv::Vec3f left_fit = fitPolynomial(leftPoints);
    cv::Vec3f right_fit = fitPolynomial(rightPoints);

    y_eval = 450;
    float leftCurv = computeCurvature(right_fit, y_eval);
    float rightCurv = computeCurvature(left_fit, y_eval);

    result.curvature = (leftCurv + rightCurv) / 2;
    // In kết quả để debug
    std::cout << "[Both lanes]\n"
    << "Slope: " << slope << "\n"
    << "Angle wrt Y: " << result.angle_y << " deg\n"
    << "Curvature: " << result.curvature << "m\n"
    << "Offset: " << result.offset << " m\n";
    
}


double stanleyControl(double e, double psi, double v, double k) {
    delta = psi + std::atan(k * e / (v + 1e-6)) * 180.0 / CV_PI;
    if (delta > 28) {
        delta = 28.0f;
    }
    else if(delta < -28) {
        delta = -28.0f;
    }
}

void sendToSerial(serial::Serial& ser, int motor_speed, double servo_angle) {
    std::string msg = "M-" + std::to_string(motor_speed) + " S" + std::to_string(static_cast<int>(servo_angle)) + " ";
    ser.write(msg);
}


cv::Mat visualize(const cv::Mat& original, const cv::Mat& warped, const cv::Mat& overlay,
                  const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2,
                  double denta, vehicleState states) {
    cv::Mat invMatrix = cv::getPerspectiveTransform(pts2, pts1);
    cv::Mat lane_overlay;
    cv::warpPerspective(overlay, lane_overlay, invMatrix, original.size());
    cv::Mat result;
    cv::addWeighted(original, 1.0, lane_overlay, 0.5, 0.0, result);
    int end_x = static_cast<int>(320 + 100 * std::sin(denta * CV_PI / 180));
    int end_y = static_cast<int>(480 - 100 * std::cos(denta * CV_PI / 180));
    cv::line(result, {320, 480}, {end_x, end_y}, cv::Scalar(255, 0, 0), 2);
    cv::putText(result, "Offset: " + std::to_string(states.offset), {30, 70}, cv::FONT_HERSHEY_SIMPLEX, 1, {255, 255, 255}, 2);
    cv::putText(result, "Angle Y: " + std::to_string(states.angle_y), {30, 110}, cv::FONT_HERSHEY_SIMPLEX, 1, {255, 255, 255}, 2);
    cv::putText(result, "Steering: " + std::to_string(80 + denta), {30, 150}, cv::FONT_HERSHEY_SIMPLEX, 1, {255, 255, 255}, 2);
    return result;
}
