#pragma once
#ifndef _SOCKET_H_
#define _SOCKET_H_
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
#include <fcntl.h>
#include <cstring>
#include <cerrno>
#include <unistd.h>  // 包含close函数
#include<thread>
struct ControlQuantity
{
	double throttle;
	int  brake;
	double sw_angle;
	unsigned char eps_auto_model;
	unsigned char  acc_ask_enable;
	unsigned char  auto_model;
}; 
struct EnableAutoDrive
{
	unsigned char control_ask; //	unsigned char 0～255
	unsigned short  speed_ask;
	unsigned char gear_ask;	
	unsigned char autodriver_ask;
};
struct Car_Light_Control{
	unsigned char TurnlightL_ask;
	unsigned char TurnlightR_ask;
	unsigned char Horn_ask;
};
class socketcan
{
private:
    int s = -1;
    struct sockaddr_can addr;
    struct ifreq ifr;
	const double targetFrameRate = 200.0;                      // 目标帧率（帧/秒）
	const double targetFrameDuration = 1.0 / targetFrameRate; // 目标帧持续时间（秒）
	bool running;
public:
    struct can_frame frame;
	std::thread  send_thread;
	std::thread  recv_thread;
	bool startsocketcanflag = false;
	int brake_flag = 0;
	// init data 
	unsigned int frame_ID_ = 0x20; 
	int brake_ = 0;
	int sw_angle_= 0; 
	unsigned char gear_ask_ = 0xA;
	int speed_ask_ = 0;
    int send_init(unsigned int frame_ID, unsigned char* data);
    void send_data();
	void message_0x20(int brake,int sw_angle);
	void message_0x1c5(unsigned char gear_ask,int speed_ask);
	void meeeage_all(int brake,int sw_angle,unsigned char gear_ask,int speed_ask);
    void open_socketcan();
	void start_thread();
	void receive_data();
	void exit_socketcan();
	int getbrakeflag();
	void send_data_nobusy(int socket_fd, struct can_frame *fram);
    socketcan();
    ~socketcan();
};
#endif