class EncoderReader {
private:
    std::string portName;
    LibSerial::SerialPort serial_port;

public:
    EncoderReader(const std::string& port) : portName(port) {}

    // Initializes the serial connection
    bool initializeSerial() {
        try {
            serial_port.Open(portName);
            serial_port.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            serial_port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            serial_port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            serial_port.SetParity(LibSerial::Parity::PARITY_NONE);
            serial_port.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            return true;
        }
        catch (const LibSerial::OpenFailed&) {
            std::cerr << "Unable to open serial port: " << portName << std::endl;
            return false;
        }
    }

    // Reads data from the serial port
    std::string readFromSerial() {
        std::string data;
        try {
            if (serial_port.IsDataAvailable()) {
                serial_port.ReadLine(data, '\n', 1);
                return data;
            }
        } catch (const LibSerial::ReadTimeout&) {
            // Handle timeout exception
        } catch (const std::exception& e) {
            std::cerr << "[SERIAL READ ERROR] " << e.what() << std::endl;
        }
        return "";
    }

    // Writes data to the serial port
    void sendToSerial(int motor_speed, int servo_angle) {
        std::string msg = "M-" + std::to_string(motor_speed) + " S" + std::to_string(servo_angle) + " ";
        serial_port.Write(msg);
    }

    // Encoder reading thread function
    void encoderReaderSerial() {
        if (!initializeSerial()) {
            std::cerr << "[ERROR] Failed to initialize serial port." << std::endl;
            return;
        }

        std::cout << "Serial port initialized successfully." << std::endl;

        while (!stop_flag) {
            std::string line = readFromSerial();  // Read data from serial port
            if (!line.empty()) {
                try {
                    // Check if the line starts with 'E' (e.g., "E123 \n")
                    if (line[0] == 'E') {
                        size_t space_pos = line.find(' ');
                        if (space_pos != std::string::npos) {
                            std::string value_str = line.substr(1, space_pos - 1);
                            int value = std::stoi(value_str);

                            // Lock shared resources for thread safety
                            {
                                std::lock_guard<std::mutex> lock(mtx);
                                encoder_data = static_cast<float>(value);  // Convert to float
                                encoder_ready = true;
                                std::cout << "Encoder data: " << encoder_data << std::endl;
                            }

                            cv_encoder.notify_all();
                        }
                    }
                } catch (const std::exception& e) {
                    std::cerr << "[ENCODER PARSE ERROR] " << e.what() << " | Input: " << line << std::endl;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(30));  // Delay for 30ms
        }

        serial_port.Close();  // Close the serial port
    }

    // Encoder reader with random data for testing
    void encoderReaderRandom() {
        while (!stop_flag) {
            float encoder = rand() % 360;
            {
                std::lock_guard<std::mutex> lock(mtx);
                encoder_data = encoder;
                encoder_ready = true;
            }
            cv_encoder.notify_all();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));  // Delay for 50ms
        }
    }

    // Stop the encoder reading process
    void stopReading() {
        stop_flag = true;
    }

    // Get the encoder data
    float getEncoderData() {
        std::lock_guard<std::mutex> lock(mtx);
        return encoder_data;
    }

    // Wait for the encoder data to be ready
    void waitForEncoderData() {
        std::unique_lock<std::mutex> lock(mtx);
        cv_encoder.wait(lock, [this]() { return encoder_ready; });
        encoder_ready = false;
    }
};