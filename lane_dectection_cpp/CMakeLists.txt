cmake_minimum_required(VERSION 3.10)
project(mpc_controller_example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include Eigen and OsqpEigen
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(OpenCV REQUIRED)


# Include directories
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OsqpEigen_INCLUDE_DIRS})
include_directories(include)
include_directories(/usr/include/libserial)  # hoặc nơi chứa SerialPort.h

# Add executable from your src directory
add_executable(mpc_controller
    src/main.cpp
    src/utils.cpp
    src/mpc.cpp
)

# Link to OsqpEigen and Eigen
target_link_libraries(mpc_controller
    OsqpEigen::OsqpEigen
    ${OpenCV_LIBS}
    serial  # Nếu dùng libserial
)
