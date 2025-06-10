#include "serial.hpp"
#include "share.hpp"
#include "debug.hpp"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>



std::string portName = "/dev/ttyACM0";
LibSerial::SerialPort serial_port;

bool initializeSerial(LibSerial::SerialPort& serial, const std::string& port_name) {
    try {
        serial.Open(port_name);
        serial.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        serial.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial.SetParity(LibSerial::Parity::PARITY_NONE);
        serial.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        return true;
    }
    catch (const LibSerial::OpenFailed&) {
        std::cerr << "Không thể mở cổng serial: " << port_name << std::endl;
        return false;
    }
}

void sendToSerial(LibSerial::SerialPort& serial_port, int motor_speed, int servo_angle) {
    std::string msg = "M+" + std::to_string(motor_speed) + " S" + std::to_string(servo_angle) + " ";
    serial_port.Write(msg);
}

void sendSpeedPWM(LibSerial::SerialPort& serial_port, int PWM_pulse) {
    std::string msg = "M+" + std::to_string(PWM_pulse) + " ";
    serial_port.Write(msg);
}

void sendSpeedPID(LibSerial::SerialPort& serial_port, int motor_speed) {
    std::string msg = "V" + std::to_string(motor_speed) + " ";
    serial_port.Write(msg);
}

void sendSteering(LibSerial::SerialPort& serial_port, int servo_angle) {
    std::string msg = "S" + std::to_string(servo_angle) + " ";
    serial_port.Write(msg);
}

std::string readFromSerial(LibSerial::SerialPort& serial_port) {
    std::string data;
    try {
        if (serial_port.IsDataAvailable()) {
            serial_port.ReadLine(data, '\n', 1);
            //std::cout << "Dữ liệu nhận được từ cổng serial: " << data <<"l"<< std::endl;   // đọc đến newline hoặc timeout sau 100ms   
            return data;
        }
    } catch (const LibSerial::ReadTimeout&) {
        // Không có dữ liệu trong thời gian chờ, trả về chuỗi rỗng
    } catch (const std::exception& e) {
        std::cerr << "[SERIAL READ ERROR] " << e.what() << std::endl;
    }
    return "";
}


void encoder_reader_serial() {
    /*
    if (!initializeSerial(serial_port, portName)) {
        std::cerr << "[ERROR] Không thể khởi tạo cổng serial trong encoder_reader_serial." << std::endl;
        return;
    }
    else {
        std::cout << "Ket noi serial thanh cong";
    }
    */  
    while (!stop_flag) {
        std::string line = readFromSerial(serial_port);  // Đọc dữ liệu từ serial port
        if (!line.empty()) {
            try {
                // Kiểm tra nếu dòng bắt đầu với 'E' (định dạng "E123 \n")
                if (line[0] == 'E') {
                    // Tìm vị trí dấu cách sau 'E'
                    size_t space_pos = line.find(' ');

                    if (space_pos != std::string::npos) {
                        // Cắt chuỗi từ vị trí sau 'E' tới dấu cách
                        std::string value_str = line.substr(1, space_pos - 1);
                        // Chuyển đổi chuỗi thành số nguyên (dùng std::stoi)
                        int value = std::stoi(value_str);
                        float mps = pulses_to_mps(value);
                        {
                            std::lock_guard<std::mutex> lock(EncMtx);
                            encoder_data = static_cast<float>(mps);  // Chuyển đổi int thành float
                            encoder_ready = true;
                        }
                        cv_encoder.notify_all();
                    }
                }
            } catch (const std::exception& e) {
                std::cerr << "[ENCODER PARSE ERROR] " << e.what() << " | Input: " << line << std::endl;
            }
            serial_port.FlushInputBuffer();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));  // Đợi 50ms
    }

    serial_port.Close();  // Đóng cổng khi thoát thread
}



void encoder_reader_random() {
    while (!stop_flag) {
        float encoder = rand() % 360;
        {
            std::lock_guard<std::mutex> lock(EncMtx);
            encoder_data = encoder;
            encoder_ready = true;
        }
        cv_encoder.notify_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

float pulses_to_mps(int pulses) {
    float radius = 0.065f / 2.0f;
    float gear_ratio = 13.0f / 38.0f;
    float pulses_per_rev = 11.0f * 4.0f * 19.0f;
    float time_interval = 0.01f;

    float wheel_rps = ((pulses / pulses_per_rev) / time_interval) * gear_ratio;
    float speed = wheel_rps * (2.0f * M_PI * radius);

    return speed;
}
