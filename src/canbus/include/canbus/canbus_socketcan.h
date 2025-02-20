#pragma once
#ifndef _CANBUS_SOCKET_H_
#define _CANBUS_SOCKET_H_
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>  // 包含close函数
#include<thread>
#include"canbus_msg/msg/can_bus_msg.hpp"
#include <atomic>
// struct ControlCANFlags
// {
//     bool mode_enable = false;
//     bool steer_enable = false;
//     bool gear_enable = false;
//     bool throttle_enable = false;
//     bool brake_enable = false;
//     bool light_enable = false;
// };
// struct ControlCanMessage{
//     float control_steer; //角度
//     float control_steer_rate;
//     int control_throttle;
//     int control_brake;
//     int control_gear;
//     int left_light;
//     int right_light;

//     bool mode_enable;
//     bool steer_enable;
//     bool gear_enable;
//     bool throttle_enable;
//     bool brake_enable;
//     bool light_enable;
//     bool inter_by_brake;
// };

class canbus_socketcan
{
private:
    int s = -1;
    struct sockaddr_can addr;
    struct ifreq ifr;
	const double targetFrameRate = 200.0;                      // 目标帧率（帧/秒）
	const double targetFrameDuration = 1.0 / targetFrameRate; // 目标帧持续时间（秒）
	std::atomic<bool> running;
public:
    canbus_msg::msg::CanBusMsg tmp_flags;
	bool start_socketcan_flag = false;
    // ControlCanMessage tmp_flags;
    // ControlCANFlags output_ctrl_value;
	std::thread  send_thread;
	std::thread  recv_thread;
	int brake_flag = 0;
	double vehicle_speed = 0.0;
	// init data 
	unsigned int frame_ID_=0x20; 
	int brake_=0;
	int sw_angle_=0; 
	unsigned char gear_ask_=0xA;
	int speed_ask_=0;
    void send_data();
	void recv_data();
	void UpdateMessage(canbus_msg::msg::CanBusMsg &msg);
    void open_socketcan();
	void start_thread();
	void exit_socketcan();
	double Handleox34a(const can_frame &msg);
	double getvehicle_speed();
	void send_data_nobusy(int socket_fd, struct can_frame *frame); 
    canbus_socketcan();
    ~canbus_socketcan();
};
#endif