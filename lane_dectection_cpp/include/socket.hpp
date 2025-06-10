// socket.hpp
#ifndef SOCKET_H
#define SOCKET_H

#include <iostream>

void signal_receiver(int port);
void server_uploader(std::string serverIpAddr, int port);
void save_data();
void save_data_cal_tire();
#endif