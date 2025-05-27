#ifndef SERIAL_H
#define SERIAL_H

#define SERIAL_READ 1
#define SERIAL_RANDOM 0

#include <libserial/SerialPort.h>
#include <string>

extern LibSerial::SerialPort serial_port;
extern std::string portName;

bool initializeSerial(LibSerial::SerialPort& serial, const std::string& port_name);

void sendToSerial(LibSerial::SerialPort& serial_port, int motor_speed, int servo_angle);

std::string readFromSerial(LibSerial::SerialPort& serial_port);

void sendSpeedPWM(LibSerial::SerialPort& serial_port, int PWM_pulse);

void sendSpeedPID(LibSerial::SerialPort& serial_port, int motor_speed);

void sendSteering(LibSerial::SerialPort& serial_port, int servo_angle);

void encoder_reader_serial();

void encoder_reader_random();

float pulses_to_mps(int pulses);

#endif // SERIAL_H
 