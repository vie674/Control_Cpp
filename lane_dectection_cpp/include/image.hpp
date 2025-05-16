// File: image.hpp
#ifndef IMAGE_HPP
#define IMAGE_HPP

#define IMAGE_READ_FROM_VIDEO 1
#define IMAGE_READ_FROM_CAM   0 
#define PIXELS_PER_LANE 335
#define PIXEL_TO_METER 0.29 / PIXELS_PER_LANE
#define DEG2RAD(deg) ((deg) * M_PI / 180.0f)
#define DISTANCE_FROM_BOTTOM_OF_IMAGE_TO_AXLE 0.15f
#define DISTANCE_FROM_FRONT_AXLE_TO_COM 0.12f + 0.15f
#define LANE_WIDTH 0.29f

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>


extern cv::Mat cameraMatrix;
extern cv::Mat distCoeffs;

// Cấu trúc lưu trạng thái xe
struct vehicleState {
    std::vector<float> curvature;
    float angle_y = 0.0f;
    float offset = 0.0f;
};

struct UndistortData {
    cv::Mat map1, map2;
    cv::Rect roi;
};

// Bỏ biến dạng ảnh sử dụng camera matrix và distortion coeffs
UndistortData setupUndistort(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, cv::Size imageSize) ;

cv::Mat undistortFrame(const cv::Mat& frame, const UndistortData& data);

// Chuyển đổi màu để phát hiện lane (white + yellow)
cv::Mat HSVColorSelection(const cv::Mat& image);

// Biến đổi phối cảnh ảnh
cv::Mat perspectiveTransform(const cv::Mat& frame, std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2);

//Chia anh de tinh histogram
void splitImageByAngle(const cv::Mat& inputImage, cv::Mat& outputLeft, cv::Mat& outputRight, double alpha_deg);

// Histogram (khởi tạo)
cv::Mat getHistogram(const cv::Mat& mask);

// Tìm base (điểm bắt đầu lane)
int getBase(const cv::Mat& hist, int start, int end);

// Phát hiện các điểm làn đường (dạng cây)
std::vector<int> detectLanePoints(const cv::Mat& mask, int base);

// Vẽ đa thức lên ảnh
void drawPolynomial(cv::Mat& image, const cv::Vec3f& coeffs, const cv::Scalar& color, int step) ;

// Fit đa thức bậc 2 từ tập điểm (y^2, y, 1)
cv::Vec3f fitPolynomial(const std::vector<cv::Point2f>& pts);

// Tính độ cong (radius of curvature)
float computeCurvatureRadius(const cv::Vec3f& fit, float y_eval, float scale_factor);
std::vector<float> computeMultipleCurvatures(const cv::Vec3f& coeffs, int M , float scale_factor );

// Tính góc lệch (từ slope)
float calculateAngle(float slope);

// Xử lý trường hợp chỉ có làn phải
vehicleState handleRightLaneOnly(const std::vector<int>& rx, cv::Mat& overlay);

// Xử lý trường hợp chỉ có làn trái
vehicleState handleLeftLaneOnly(const std::vector<int>& lx, cv::Mat& overlay);

// Xử lý khi có cả 2 làn
vehicleState handleBothLanes(const std::vector<int>& lx, const std::vector<int>& rx, cv::Mat& overlay);

// 
void preprocessAndSplitFrame(const cv::Mat& input, const UndistortData& undistortData,
                            int splitAngle, cv::Mat& leftImage, cv::Mat& rightImage,
                            std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2,
                            cv::Mat& mask, cv::Mat& warped);

// Compute 
void detectLaneLines(const cv::Mat& mask, const cv::Mat& leftImage, const cv::Mat& rightImage,
                    std::vector<int>& prevLx, std::vector<int>& prevRx, 
                    std::vector<int>& lx, std::vector<int>& rx);               

vehicleState computeControlParam(const std::vector<int>& lx, const std::vector<int>& rx,
                                cv::Mat& overlay, int& splitAngle);

int updateSplitAngle (int steerAngle);

void image_reader(const bool isReadfromVideo);

// Hiển thị ảnh kết quả với lane và hướng điều khiển
cv::Mat visualize(const cv::Mat& original, const cv::Mat& warped, const cv::Mat& overlay,
                  const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2,
                  double denta, vehicleState states);

// Show fps on terminal
void showFPS(double& lastTick, double& fps); 

#endif // IMAGE_HPP
