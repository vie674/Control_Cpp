// File: utils.cpp
#include "image.hpp"
#include "share.hpp"
#include "debug.hpp"
#include <iostream>
#include <numeric>
#include <cmath>
#include <thread>
#include <chrono>
#include <cmath>

bool right_deteted = 0;
bool left_detected = 0;

int N = 6;
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

/******************************** Fuctions for image processing **********************************************************************/
//?
UndistortData setupUndistort(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, cv::Size imageSize) {
    UndistortData data;
    cv::Mat newCameraMatrix = cv::getOptimalNewCameraMatrix(
        cameraMatrix, distCoeffs, imageSize, 0.4, imageSize, &data.roi
    );

    cv::initUndistortRectifyMap(
        cameraMatrix, distCoeffs, cv::Mat(), newCameraMatrix,
        imageSize, CV_16SC2, data.map1, data.map2
    );

    return data;
}

cv::Mat undistortFrame(const cv::Mat& frame, const UndistortData& data) {
    cv::Mat undistorted;
    cv::remap(frame, undistorted, data.map1, data.map2, cv::INTER_LINEAR);
    undistorted = undistorted(data.roi);  // Cắt theo ROI nếu cần
    return undistorted;
}
//có thể xem lại
cv::Mat HSVColorSelection(const cv::Mat& image) {
    cv::Mat hsv, white_mask, yellow_mask, combined, gray;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, 150), cv::Scalar(180, 50, 255), white_mask);
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

    // Tính toán các điểm cho phép biến đổi
    pts1 = { 
        {0.0f + 27.0f, float(height)}, 
        {float(width), float(height)}, 
        {float(width * 0.7), float(height * 0.70)}, 
        {float(width * 0.3 + 27.0f), float(height * 0.70)} 
    };

    pts2 = { 
        {0.0f, float(height)}, 
        {float(width), float(height)}, 
        {float(width), 0.0f}, 
        {0.0f, 0.0f} 
    };

    // Vẽ các điểm trên hình ảnh
    cv::circle(frame, pts1[0], 5, cv::Scalar(0, 0, 255), -1); // Red color
    cv::circle(frame, pts1[1], 5, cv::Scalar(0, 0, 255), -1); // Red color
    cv::circle(frame, pts1[2], 5, cv::Scalar(0, 0, 255), -1); // Red color
    cv::circle(frame, pts1[3], 5, cv::Scalar(0, 0, 255), -1); // Red color

    // Tính toán ma trận biến đổi hình học
    auto M = cv::getPerspectiveTransform(pts1, pts2);

    // Áp dụng biến đổi perspective lên ảnh
    cv::Mat warped;
    cv::warpPerspective(frame, warped, M, frame.size());

    return warped;
}

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
    if (std::abs(alpha_deg - 90) < 3) { // Kiểm tra xem góc có gần 90 độ hay không
        for (int y = 0.45 * height; y < height; ++y) {
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
        for (int y = 0.45 * height; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                // Tính toạ độ y trên đường phân chia tại x
                int lineY = origin.y + static_cast<int>(slope * (x - origin.x));
                if (alpha_deg > 90) {
                    if (y < lineY) {
                        maskLeft.at<uchar>(y, x) = 255; // Phần bên trái
                    } else {
                        maskRight.at<uchar>(y, x) = 255; // Phần bên phải
                    }
                }
                else {
                    if (y > lineY) {
                        maskLeft.at<uchar>(y, x) = 255; // Phần bên trái
                    } else {
                        maskRight.at<uchar>(y, x) = 255; // Phần bên phải
                    }
                }
            }
        }
    }

    // Áp dụng mặt nạ vào ảnh để chia thành 2 phần
    inputImage.copyTo(outputLeft, maskLeft);
    inputImage.copyTo(outputRight, maskRight);
}

cv::Mat getHistogram(const cv::Mat& mask) {
    cv::Mat histogram = cv::Mat::zeros(1, mask.cols, CV_32F);
    for (int col = 0; col < mask.cols; ++col) {
        for (int row = mask.rows / 2; row < mask.rows; ++row) {
            histogram.at<float>(0, col) += mask.at<uchar>(row, col);
        }
    }
    return histogram;
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
    while (y > mask.rows * 0.45) {
        // Đảm bảo ROI không vượt quá ảnh
        int roi_x = base - 30;
        int roi_y = y - 30; 
        int roi_width = 60;
        int roi_height = 30;

        roi_x = std::max(roi_x, 0);  
        roi_y = std::max(roi_y, 0);  
        roi_width = std::min(roi_width, mask.cols - roi_x);  
        roi_height = std::min(roi_height, mask.rows - roi_y);  

        cv::Rect roi(roi_x, roi_y, roi_width, roi_height);
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
        cv::rectangle (mask, {base - 30, y}, {base + 30, y - 30}, cv::Scalar(255, 0, 0), 2);
        y -= 30;
    }
    return points;
}

void drawPolynomial(cv::Mat& image, const cv::Vec3f& coeffs, const cv::Scalar& color, int step = 1) {
    for (int y = 0; y < image.rows - step; y += step) {
        int x1 = static_cast<int>(coeffs[0] * y * y + coeffs[1] * y + coeffs[2]);
        int y1 = y;
        int y2 = y + step;
        int x2 = static_cast<int>(coeffs[0] * y2 * y2 + coeffs[1] * y2 + coeffs[2]);

        // Chỉ vẽ nếu x nằm trong ảnh
        if (x1 >= 0 && x1 < image.cols && x2 >= 0 && x2 < image.cols) {
            cv::line(image, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
        }
    }
}


cv::Vec3f fitPolynomial(const std::vector<cv::Point2f>& pts) {
    if (pts.size() < 2) return cv::Vec3f(0, 0, 0);

    cv::Mat X(static_cast<int>(pts.size()), 3, CV_32F);
    cv::Mat Y(static_cast<int>(pts.size()), 1, CV_32F);

    for (size_t i = 0; i < pts.size(); ++i) {
        float y = pts[i].y;
        X.at<float>(i, 0) = y * y;
        X.at<float>(i, 1) = y;
        X.at<float>(i, 2) = 1;
        Y.at<float>(i, 0) = pts[i].x;
    }

    cv::Mat coeffs;
    cv::solve(X, Y, coeffs, cv::DECOMP_SVD);
    return cv::Vec3f(coeffs.at<float>(0), coeffs.at<float>(1), coeffs.at<float>(2));
}


// Xử lý khi chỉ có làn phải
vehicleState handleRightLaneOnly(const std::vector<int>& rx, cv::Mat& overlay, int& splitAngle) {
    vehicleState result;
    if (rx.size() < 2) return result;

    float slope = 0;
    float intercept = 0;
    float angle_x = 0;

    std::vector<cv::Point2f> rightPoints;
    for (size_t i = 0; i < rx.size(); ++i)
        rightPoints.emplace_back(rx[i], 470 - static_cast<int>(i) * 35);

    cv::Vec3f right_fit = fitPolynomial(rightPoints);

    std::vector<cv::Point2f> centerPoints;
    float lane_offset_pixel = (LANE_WIDTH / 2.0f) / PIXEL_TO_METER;

    for (size_t i = 0; i < rightPoints.size(); ++i) {
        float y = rightPoints[i].y;
        float x_right = right_fit[0] * y * y + right_fit[1] * y + right_fit[2];

        float slope = 2 * right_fit[0] * y + right_fit[1];
        float theta = std::atan(slope);  // hướng tiếp tuyến

        // Dịch sang trái bằng pháp tuyến θ - π/2
        float dx = std::cos(theta - CV_PI / 2);
        float dy = std::sin(theta - CV_PI / 2);

        float x_center = x_right + dx * lane_offset_pixel;
        float y_center = y + dy * lane_offset_pixel;

        centerPoints.emplace_back(x_center, y_center);
    }

    cv::Point2f mid1 = centerPoints[0];
    cv::Point2f mid2 = centerPoints[centerPoints.size() - 1];

    slope = (mid1.y - mid2.y) / (mid1.x - mid2.x);
    intercept = mid1.y - slope * mid1.x;
    angle_x = calculateAngle(slope);

    // Tính toán góc lệch
    if (angle_x >= 0) {
        splitAngle = 90 - angle_x;
    }
    else {
        splitAngle = - (90 - std::abs(angle_x));
    }

    splitAngle += 90;

    // Fit lại đường giữa làn từ tập điểm đã dịch
    cv::Vec3f center_fit = fitPolynomial(centerPoints);

    // Tính toán tại y_eval
    float y_eval = 468.0f;
    float slope_at_eval = 2 * center_fit[0] * y_eval + center_fit[1];
    float x_center_at_eval = center_fit[0] * y_eval * y_eval + center_fit[1] * y_eval + center_fit[2];

    result.angle_y = calculateAngle(slope_at_eval);
    result.curvature = computeMultipleCurvatures(center_fit, N, PIXEL_TO_METER);

    float rawOffset = -((PIXELS_PER_LANE / 2.0f) - std::abs(rx[0] - 320)) * PIXEL_TO_METER;
    result.offset = rawOffset - DISTANCE_FROM_BOTTOM_OF_IMAGE_TO_AXLE * sin(DEG2RAD(result.angle_y));

    #ifdef DEBUG_IMAGE_LANE
        drawPolynomial(overlay, center_fit, cv::Scalar(0, 255, 255));  // đường giữa
        drawPolynomial(overlay, right_fit, cv::Scalar(255, 0, 255));   // vạch phải
        for (const auto& pt : centerPoints)
            cv::circle(overlay, pt, 2, cv::Scalar(0, 255, 255), -1);   // điểm giữa làn
    #endif
    #ifdef DEBUG_TERMINAL_LANE
        std::cout << "...........................[Right lane only – Tangent-based Estimation]....................\n"
                  << "Slope: " << slope_at_eval << "\n"
                  << "Angle wrt Y: " << result.angle_y << " deg\n"
                  << "Offset: " << result.offset << " m\n";
    #endif

    return result;
}


vehicleState handleLeftLaneOnly(const std::vector<int>& lx, cv::Mat& overlay, int& splitAngle) {
    vehicleState result;
    if (lx.size() < 2) return result;

    float slope = 0;
    float intercept = 0;
    float angle_x = 0;

    std::vector<cv::Point2f> leftPoints;
    for (size_t i = 0; i < lx.size(); ++i)
        leftPoints.emplace_back(lx[i], 470 - static_cast<int>(i) * 30);

    cv::Vec3f left_fit = fitPolynomial(leftPoints);

    std::vector<cv::Point2f> centerPoints;
    float lane_offset_pixel = (LANE_WIDTH / 2.0f) / PIXEL_TO_METER;  

    for (size_t i = 0; i < leftPoints.size(); ++i) {
        float y = static_cast<float>(leftPoints[i].y);
        float x_left = left_fit[0] * y * y + left_fit[1] * y + left_fit[2];

        float slope = 2 * left_fit[0] * y + left_fit[1];
        float theta = std::atan(slope);  // hướng tiếp tuyến

        // Dịch vuông góc sang phải (góc vuông + π/2)
        float dx = std::cos(theta + CV_PI / 2);
        float dy = std::sin(theta + CV_PI / 2);

        float x_center = x_left + dx * lane_offset_pixel;
        float y_center = y + dy * lane_offset_pixel;

        centerPoints.emplace_back(x_center, y_center);
    }

    cv::Point2f mid1 = centerPoints[0];
    cv::Point2f mid2 = centerPoints[centerPoints.size() - 1];

    slope = (mid1.y - mid2.y) / (mid1.x - mid2.x);
    intercept = mid1.y - slope * mid1.x;
    angle_x = calculateAngle(slope);

    // Tính toán góc lệch
    if (angle_x >= 0) {
        splitAngle = 90 - angle_x;
    }
    else {
        splitAngle = - (90 - std::abs(angle_x));
    }

    splitAngle += 90;

    // Fit lại đường giữa làn từ tập điểm đã dịch
    cv::Vec3f center_fit = fitPolynomial(centerPoints);


    // Tính toán tại y_eval
    float y_eval = 468.0f;
    float slope_at_eval = 2 * center_fit[0] * y_eval + center_fit[1];
    float x_center_at_eval = center_fit[0] * y_eval * y_eval + center_fit[1] * y_eval + center_fit[2];

    result.angle_y = calculateAngle(slope_at_eval);
    result.curvature = computeMultipleCurvatures(center_fit, N, PIXEL_TO_METER);

    float rawOffset = ((PIXELS_PER_LANE / 2.0f) - std::abs(lx[0] - 320)) * PIXEL_TO_METER;
    result.offset = rawOffset - DISTANCE_FROM_BOTTOM_OF_IMAGE_TO_AXLE * sin(DEG2RAD(result.angle_y));

    #ifdef DEBUG_IMAGE_LANE
        drawPolynomial(overlay, center_fit, cv::Scalar(0, 255, 0));  // Tâm làn
        drawPolynomial(overlay, left_fit, cv::Scalar(0, 0, 255));    // Vạch trái
        for (const auto& pt : centerPoints)
            cv::circle(overlay, pt, 2, cv::Scalar(0, 255, 255), -1); // Các điểm center vàng
    #endif
    #ifdef DEBUG_TERMINAL_LANE
        std::cout << "...........................[Left lane only – Tangent-based Estimation]....................\n"
                  << "Slope: " << slope_at_eval << "\n"
                  << "Angle wrt Y: " << result.angle_y << " deg\n"
                  << "Offset: " << result.offset << " m\n";
    #endif

    return result;
}


vehicleState handleBothLanes(const std::vector<int>& lx, const std::vector<int>& rx, cv::Mat& overlay, int& splitAngle) {
    int minLength = std::min(lx.size(), rx.size());

    std::vector<cv::Point2f> midpoints;
    std::vector<cv::Point> leftPoints;
    std::vector<cv::Point> rightPoints;
    cv::Point2f previous_mid(320.0f, 472.0f);

    // Lấy vector midpoint và vẽ kết quả lên overlay
    for (int i = 0; i < minLength; ++i) {
        cv::Point left(lx[i], 470 - i * 30);
        cv::Point right(rx[i], 470 - i * 30);

        // Lưu các điểm bên trái và bên phải 
        leftPoints.push_back(left);
        rightPoints.push_back(right);


        // Tính midpoint
        cv::Point2f midpoint((left.x + right.x) / 2.0f, (left.y + right.y) / 2.0f);
        midpoints.push_back(midpoint);

        #ifdef DEBUG_IMAGE_LANE
            // Vẽ đường nối giữa left và right
            cv::line(overlay, left, right, cv::Scalar(0, 255, 0), 1);

            // Vẽ midpoint
            cv::circle(overlay, midpoint, 5, cv::Scalar(0, 0, 255), -1);

            // Vẽ đường nối các midpoint
            cv::line(overlay, previous_mid, midpoint, cv::Scalar(200, 100, 250), 1);
        #endif // DEBUG

        previous_mid = midpoint;
    }

    // Tính toán các trạng thái của xe so với làn đường 
    vehicleState result;
    float slope = 0;
    float intercept = 0;
    float angle_x = 0;

    if (midpoints.size() == 2) {
        cv::Point2f mid1 = midpoints[0];
        cv::Point2f mid2 = midpoints[1];

        slope = (mid1.y - mid2.y) / (mid1.x - mid2.x);
        intercept = mid1.y - slope * mid1.x;
        angle_x = calculateAngle(slope);

        // Tính toán góc lệch
        if (angle_x >= 0) {
            result.angle_y = 90 - angle_x;
            splitAngle = 90 - angle_x;
        }
        else {
            result.angle_y = - (90 - std::abs(angle_x));
            splitAngle =  - (90 - std::abs(angle_x));
        }
        result.curvature = std::vector<float>(N, 0.0f);
    }

    else if (midpoints.size() >= 3) {
        /*
        std::vector<cv::Point2f> midToFit;
        midToFit.push_back(midpoints[0]);
        midToFit.push_back(midpoints[1]);
        midToFit.push_back(midpoints[2]);

        cv::Vec4f lineParams;
        cv::fitLine(midToFit, lineParams, cv::DIST_L2, 0, 0.01, 0.01);

        float vx = lineParams[0];
        float vy = lineParams[1];
        float x0 = lineParams[2];
        float y0 = lineParams[3];

        slope = vy / vx;
        intercept = y0 - slope * x0;
        angle_x = calculateAngle(slope);
        */

        cv::Point2f mid1 = midpoints[0];
        cv::Point2f mid2 = midpoints[midpoints.size() - 1];

        slope = (mid1.y - mid2.y) / (mid1.x - mid2.x);
        intercept = mid1.y - slope * mid1.x;
        angle_x = calculateAngle(slope);

        // Tính toán góc lệch
        if (angle_x >= 0) {
            splitAngle = 90 - angle_x;
        }
        else {
            splitAngle = - (90 - std::abs(angle_x));
        }
        
        if (splitAngle > 10) {
            splitAngle +=10;
        }
        else if (splitAngle < -10) {
            splitAngle -=10;
        }
        /*
        // Tính toán góc lệch
        if (angle_x >= 0) {
            result.angle_y = 90 - angle_x;
        }
        else {
            result.angle_y = - (90 - std::abs(angle_x));
        }
        */
        float y_eval = 470.0f;

        // Calculate cuvarute
        cv::Vec3f midFit = fitPolynomial(midpoints);
        float slope_at_eval = 2 * midFit[0] * y_eval + midFit[1];
        float x_center_at_eval = midFit[0] * y_eval * y_eval + midFit[1] * y_eval + midFit[2];
        result.angle_y = calculateAngle(slope_at_eval);
        result.curvature = computeMultipleCurvatures(midFit, N, PIXEL_TO_METER);
        splitAngle += 90;
    }


    int carPosition = 320;
    int laneCenter = midpoints[0].x;
    // Tính vector offset
    float rawDistance = (laneCenter - carPosition) * PIXEL_TO_METER;
    result.offset = rawDistance - DISTANCE_FROM_BOTTOM_OF_IMAGE_TO_AXLE * sin(DEG2RAD(result.angle_y));

    #ifdef DEBUG_TERMINAL_LANE
        // In kết quả để debug
        std::cout << "...........................[Both lanes].......................\n"
        << "Slope: " << slope << "\n"
        << "Angle wrt Y: " << result.angle_y << " deg\n"
        << "Curvature: " << result.curvature << "m\n"
        << "Offset: " << result.offset << " m\n";
    #endif
    return result;
}

cv::Mat visualize(const cv::Mat& original, const cv::Mat& warped, const cv::Mat& overlay,
        const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2,
        double denta, vehicleState states) {
    #ifdef DEBUG_IMAGE_LANE
        cv::Mat invMatrix = cv::getPerspectiveTransform(pts2, pts1);
        cv::Mat lane_overlay;
        cv::warpPerspective(overlay, lane_overlay, invMatrix, original.size());
        cv::Mat result;
        cv::addWeighted(original, 1.0, lane_overlay, 0.5, 0.0, result);

        int end_x = static_cast<int>(320 + 100 * std::sin(denta * CV_PI / 180));
        int end_y = static_cast<int>(480 - 100 * std::cos(denta * CV_PI / 180));
        cv::line(result, {320, 480}, {end_x, end_y}, cv::Scalar(255, 0, 0), 2);
        cv::line(result, {320, 480}, {320, 440}, cv::Scalar(0, 0, 0), 2);

        cv::putText(result, "Offset: " + std::to_string(states.offset), {30, 70}, cv::FONT_HERSHEY_SIMPLEX, 1, {255, 255, 255}, 2);
        cv::putText(result, "Angle Y: " + std::to_string(states.angle_y), {30, 110}, cv::FONT_HERSHEY_SIMPLEX, 1, {255, 255, 255}, 2);
        cv::putText(result, "Steering: " + std::to_string(80 + denta), {30, 150}, cv::FONT_HERSHEY_SIMPLEX, 1, {255, 255, 255}, 2);
        return result;
    #else
        return original;
    #endif

}

/******************************Compute functions**********************************************/

float computeCurvatureRadius(const cv::Vec3f& fit, float y_eval, float scale_factor = 0.00062857f) {
    float dy = 2 * fit[0] * y_eval + fit[1];
    float curvature = std::pow(1 + dy * dy, 1.5f) / std::abs(2 * fit[0]);
    return curvature * scale_factor;
}

// Tính góc nghiêng với trục Y (độ)
float calculateAngle(float slope) {
    return -std::atan(slope) * 180.0f / CV_PI;
}

std::vector<float> computeMultipleCurvatures(const cv::Vec3f& coeffs, int M = 6, float scale_factor = PIXEL_TO_METER) {
    std::vector<float> curvatures;
    float a = coeffs[0];
    float b = coeffs[1];

    for (int i = 0; i < M; ++i) {
        float y_eval = 472.0f;
        float y = y_eval - i * 38.0f;  //38 diem anh -- 3cm
        float dy = 2.0f * a * y + b;
        float numerator = std::abs(2.0f * a);
        float denominator = std::pow(1.0f + dy * dy, 1.5f);

        float kappa_px = (denominator != 0.0f) ? (numerator / denominator) : 0.0f;
        float kappa_m = kappa_px / scale_factor;
        curvatures.push_back(kappa_m);
    }

    return curvatures;
}


void showFPS(double& lastTick, double& fps) {
    double currentTick = cv::getTickCount();
    double dt = (currentTick - lastTick) / cv::getTickFrequency();  // thời gian xử lý 1 frame
    lastTick = currentTick;

    if (dt > 0.0)
        fps = 1.0 / dt;
    std::cout << "FPS: " << fps;
}
/// @brief/////////////////////////////////////////////////////////

void preprocessAndSplitFrame(const cv::Mat& input,
    const UndistortData& undistortData,
    int splitAngle,
    cv::Mat& leftImage, cv::Mat& rightImage,
    std::vector<cv::Point2f>& pts1,
    std::vector<cv::Point2f>& pts2,
    cv::Mat& mask, cv::Mat& warped) {

    // B1: Undistort và resize ảnh
    cv::Mat undistorted = undistortFrame(input, undistortData);
    cv::resize(undistorted, undistorted, cv::Size(640, 480));
    undistorted.copyTo(input);
    
    // B2: Biến đổi phối cảnh
    warped = perspectiveTransform(undistorted, pts1, pts2);

    // B3: Chuyển ảnh sang không gian HSV và lọc màu
    mask = HSVColorSelection(warped);

    // B4: Khởi tạo ảnh trắng đen cho lane trái/phải
    leftImage = cv::Mat::zeros(640, 480, CV_8UC1);
    rightImage = cv::Mat::zeros(640, 480, CV_8UC1);

    // B5: Chia vùng trái/phải theo góc đánh lái
    splitImageByAngle(mask, leftImage, rightImage, splitAngle);
}

void detectLaneLines(const cv::Mat& mask,
    const cv::Mat& leftImage, const cv::Mat& rightImage,
    std::vector<int>& prevLx, std::vector<int>& prevRx,
    std::vector<int>& lx, std::vector<int>& rx) {

    auto hist_left = getHistogram(leftImage);
    auto hist_right = getHistogram(rightImage);

    int left_base = getBase(hist_left, 0, hist_left.cols);
    int right_base = getBase(hist_right, 0, hist_right.cols);

    lx = detectLanePoints(mask, left_base);
    rx = detectLanePoints(mask, right_base);

    if (lx.empty()) lx = prevLx;
    if (rx.empty()) rx = prevRx;

    if (lx.size() > 2) {left_detected = 1;}
    else {left_detected = 0;}
    if (rx.size() > 2) {right_deteted = 1;}    
    else {right_deteted = 0;}
    prevLx = lx;
    prevRx = rx;
}

vehicleState computeControlParam(const std::vector<int>& lx, const std::vector<int>& rx,
    cv::Mat& overlay, int& splitAngle) {

    vehicleState states;

    if (right_deteted == 1 && left_detected == 0) {
        states = handleRightLaneOnly(rx, overlay, splitAngle);
    } else if (left_detected == 1 && right_deteted == 0) {
        states = handleLeftLaneOnly(lx, overlay, splitAngle);
    } else if (right_deteted == 1 && left_detected == 1) {
        states = handleBothLanes(lx, rx, overlay, splitAngle);
    }
    return states;
}

int updateSplitAngle (int steerAngle) {
    if (steerAngle > 90) {
        steerAngle += 10;
    }
    else if (steerAngle <70)
    {
        steerAngle -= 10;
    } 

    return steerAngle + 10; //midServo = 80 => 90 - 80 = 10
}

// Thread for reading image
void image_reader(const bool isReadfromVideo) {
    cv::VideoCapture cap;
    cv::Mat frame;

    std::cout << "[IMAGE] Thread entered" << std::endl;

    if (isReadfromVideo) {
        cap.open("thap.mp4");
    } else {
        cap.open(0, cv::CAP_V4L2);
    }

    if (!cap.isOpened()) {
        std::cerr << "[ERROR] Cannot open video source" << std::endl;
        stop_flag = true;
        return;
    }

    const int delay_ms = isReadfromVideo ? (1000 / 30) : 0;

    while (!stop_flag) {
        cap >> frame;

        if (frame.empty()) {
            if (isReadfromVideo) {
                cap.set(cv::CAP_PROP_POS_FRAMES, 0); // tua lại nếu là video
                continue;
            } else {
                continue; // webcam lỗi thì bỏ qua
            }
        }

        {
            std::lock_guard<std::mutex> lock(mtx);
            frame.copyTo(shared_frame);
        }

        if (delay_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
    }

    std::cout << "[IMAGE] Thread exited." << std::endl;
}
