cmake_minimum_required(VERSION 3.10)
project(ObjectDetection)

# Chỉ rõ đường dẫn tới OpenCVConfig.cmake
set(OpenCV_DIR "C:/Users/hung/Music/opencv/build")

# Tìm OpenCV
find_package(OpenCV REQUIRED)

# Tạo file thực thi
add_executable(${PROJECT_NAME} src/test.cpp)

# Liên kết thư viện OpenCV
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

