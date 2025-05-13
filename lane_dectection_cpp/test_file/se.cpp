#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <mutex>
#include <atomic>
#include <SerialStream.h>

std::mutex data_lock;
std::atomic<bool> stop_flag(false);  // Đổi tên biến terminate để tránh xung đột

// Dữ liệu chia sẻ giữa các luồng
struct SharedData {
    int encoder_value = 0;
    double speed_mps = 0.0;
} shared_data;

using namespace std;
using namespace LibSerial;

// Cấu hình cổng UART
const string UART_PORT = "/dev/ttyUSB0";  // Thay đổi tùy theo cổng STM32
const unsigned long UART_BAUDRATE = 115200;

// Tính vận tốc (m/s) từ số xung trong khoảng thời gian đo.
double pulses_to_mps(int pulses) {
    const double radius = 0.065 / 2;  // Bán kính bánh xe (m)
    const double gear_ratio = 13.0 / 38.0;  // Tỉ số truyền từ động cơ đến bánh xe
    const int pulses_per_rev = 11 * 4 * 19;  // Tổng số xung trên một vòng quay bánh xe
    const double time_interval = 0.01;  // Khoảng thời gian đo (s)

    // Tính số vòng quay trên giây của bánh xe (RPS - Revolutions per second)
    double wheel_rps = ((pulses / pulses_per_rev) / time_interval) * gear_ratio;

    // Tính vận tốc tuyến tính (m/s)
    double speed = wheel_rps * (2 * M_PI * radius);
    return speed;
}

// Hàm nhận tín hiệu từ STM32
void receive_from_stm32(SerialStream& serial) {
    while (!stop_flag) {
        try {
            if (serial.rdbuf()->in_avail()) {
                string raw_data;
                getline(serial, raw_data);
                cout << "[Receive] Raw data received: " << raw_data << endl;

                // Phân tích dữ liệu nếu có dạng "E%d "
                if (raw_data[0] == 'E') {
                    try {
                        int pulses = stoi(raw_data.substr(1));  // Lấy giá trị số xung
                        double speed = pulses_to_mps(pulses);  // Tính vận tốc

                        // Cập nhật dữ liệu chia sẻ
                        lock_guard<mutex> lock(data_lock);
                        shared_data.encoder_value = pulses;
                        shared_data.speed_mps = speed;

                        cout << "[Receive] Pulses: " << pulses << ", Speed: " << speed << " m/s" << endl;
                    } catch (const invalid_argument& e) {
                        cout << "[Receive] Error parsing pulses: " << raw_data << endl;
                    }
                }
            }
        } catch (const exception& e) {
            cout << "[Receive] Error: " << e.what() << endl;
        }
    }
}

int main() {
    try {
        // Mở kết nối UART
        SerialStream serial(UART_PORT);
        serial.SetBaudRate(UART_BAUDRATE);  // Sử dụng số nguyên cho BaudRate
        serial.SetCharacterSize(CharacterSize::CHAR_SIZE_8);  // Chỉnh sửa CharacterSize
        serial.SetParity(Parity::PARITY_NONE);  // Chỉnh sửa Parity
        serial.SetStopBits(StopBits::STOP_BITS_1);  // Chỉnh sửa StopBits
        serial.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);  // Chỉnh sửa FlowControl

        // Kiểm tra kết nối
        if (!serial.IsOpen()) {
            cerr << "Failed to open serial port!" << endl;
            return 1;
        }

        // Khởi tạo luồng nhận dữ liệu
        thread recv_thread(receive_from_stm32, ref(serial));

        // Hiển thị vận tốc trong vòng lặp chính
        while (!stop_flag) {
            {
                lock_guard<mutex> lock(data_lock);
                double speed = shared_data.speed_mps;
                cout << "[Main] Speed: " << speed << " m/s" << endl;
            }
            this_thread::sleep_for(chrono::milliseconds(500));  // Hiển thị vận tốc mỗi 0.5 giây
        }

        // Dừng luồng nhận và đóng cổng UART khi kết thúc
        stop_flag = true;
        recv_thread.join();
        serial.Close();
        cout << "\nProgram terminated." << endl;

    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
    }

    return 0;
}
