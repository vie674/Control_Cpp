cmake_minimum_required(VERSION 3.10)
project(mpc_controller_example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ----------------------------
# Tự phát hiện OpenCV_DIR nếu chưa set
if(NOT OpenCV_DIR)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")
    endif()
endif()

# ----------------------------
# Find packages
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(OpenCV REQUIRED)

# ----------------------------
# Include directories
include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${OsqpEigen_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    include
    /usr/include/libserial  # Nếu dùng thư viện SerialPort
)

# ----------------------------
# Source files
add_executable(mpc_controller
    src/main.cpp
    src/serial.cpp
    src/image.cpp
    src/controller.cpp
    src/imu.cpp
    src/socket.cpp
)

# ----------------------------
# Link libraries
target_link_libraries(mpc_controller
    OsqpEigen::OsqpEigen
    ${OpenCV_LIBS}
    serial  # Nếu dùng libserial
)
