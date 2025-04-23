// File: utils.hpp
#ifndef UTILS_HPP
#define UTILS_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

cv::Mat loadMatrix(const std::string& path);
cv::Mat undistortImage(const cv::Mat& image, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
cv::Mat HSVColorSelection(const cv::Mat& image);
cv::Mat perspectiveTransform(const cv::Mat& frame, std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2);
cv::Mat getHistogram(const cv::Mat& mask);
int getBase(const cv::Mat& hist, int start, int end);
std::vector<int> detectLanePoints(const cv::Mat& mask, int base);
std::vector<double> fitLaneCurve(const std::vector<int>& points);
void drawPolynomial(cv::Mat& image, const std::vector<double>& coeffs, const cv::Scalar& color);
std::vector<cv::Point> getMidpoints(const std::vector<int>& lx, const std::vector<int>& rx);
double calculateAngle(const std::vector<cv::Point>& midpoints);
double computeOffset(const std::vector<int>& left, const std::vector<int>& right);
double stanleyControl(double e, double psi, double v, double k);
void sendToSerial(serial::Serial& ser, int motor_speed, double servo_angle);
cv::Mat visualize(const cv::Mat& original, const cv::Mat& warped, const cv::Mat& overlay,
                  const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2,
                  double denta, double offset, double angle_y);

#endif // UTILS_HPP