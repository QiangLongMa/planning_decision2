#include"can/socketcan.h"
#include<thread>
#include<iostream>
#include <stdio.h>
#include <time.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <stdio.h>
#include <unistd.h>
//INIT 
socketcan::socketcan()
{
    open_socketcan();//openc can
    unsigned char data[8] = { 0,0,0,0,0,0,0,0};//0x20
	data[0]= 0;
	data[1]=0;
	data[4]=0x80;
	data[5]=0;
	data[6] = 0x64;
    int return_code0;
    for (size_t i = 0; i < 100; i++)
    {
        auto startTime = std::chrono::high_resolution_clock::now();
        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> frameDuration = endTime - startTime;
        double actualFrameRate = 1.0 / frameDuration.count();
        // 等待，以使每个循环迭代的持续时间达到目标帧持续时间
        std::this_thread::sleep_for(std::chrono::duration<double>(0.01)); 
        return_code0 = send_init(0x20, data);	//0x20
    }
	sleep(0.01);
	if (return_code0==0){
		printf("方向盘回正\n");
	} 
	data[1]= 0x1<<3;
	int return_code1 = send_init(0x20, data);	//0x20
	if (return_code1==0){
		printf("自动转向激活\n");
	} 
	sleep(0.01);
	//车速和档位进入自动驾驶模式
	unsigned char data1[8] = { 0,0,0,0,0,0,0,0};//0x1C5
	data1[0]=0xA;
	data1[1]=0x0;
	// data1[2]=0x0;
	// data1[2]|= 0x0B<<8;
    data1[2]= 0x0B;
	data1[3]=0xB8;
	data1[4]=0x2A;
	data1[5]=0x30;
	data1[6]=0x68;
	int return_code2 = send_init(0x1C5, data1);	//0x20
	if (return_code2==0){
		printf("车速和档位进入自动驾驶模式\n");
	} 
	sleep(0.01);
    start_thread();
}
socketcan::~socketcan(){

    running = false;
    //std::cout << "running: " << running << " startsocketcanflag: " << startsocketcanflag << std::endl;
    sleep(0.1);
    unsigned char data[8] = {0,0,0,0,0,0,0,0};//0x20
	data[0]= 0;
	data[1]= 0x1<<3;
	data[4]=0x80;//2048*16;
	data[5]=0;
	data[6] = 0x64;
    int return_code0;
    for (size_t i = 0; i < 150; i++)
    {
        auto startTime = std::chrono::high_resolution_clock::now();
        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> frameDuration = endTime - startTime;
        double actualFrameRate = 1.0 / frameDuration.count();
        // 等待，以使每个循环迭代的持续时间达到目标帧持续时间
        std::this_thread::sleep_for(std::chrono::duration<double>(0.01)); 
        return_code0 = send_init(0x20, data);	//0x20
    }
    // int return_code0 = send_init(0x20, data);	//0x20
	if (return_code0==0){
		printf("方向盘回正\n");
	} 
	sleep(0.01);
	data[1]=0;
	int return_code1 = send_init(0x20, data);	//0x20
	if (return_code1==0){
		printf("功能关闭，回到人工驾驶状态\n");
	} 
	sleep(0.01);
	unsigned char data1[8] = {0,0,0,0,0,0,0,0};//0x1C5
	data1[0]=0xA;
	data1[1]=0x0;
	data1[2]=0x0;
	data1[2]|= 0x0B<<8;
	data1[3]=0xB8;
	data1[4]=0x2A;
	data1[5]=0x30;
	data1[6]=0x68;
	int return_code2 = send_init(0x1C5, data1);	//0x1C5
	data1[0]=0xB;
	data1[6]=0x48;
	sleep(0.01);
	int return_code3 = send_init(0x1C5, data1);	//0x1C5
	if (return_code3==0){
		printf("自动驾驶模式退出\n");
	} 
	sleep(0.01);
    close(s);
    printf("退出socketcan\n");
}
void socketcan::exit_socketcan(){
    running = false;
    sleep(0.1);
    unsigned char data[8] = {0,0,0,0,0,0,0,0};//0x20
	data[0]= 0;
	data[1]= 0x1<<3;
	data[4]=0x80;//2048*16;
	data[5]=0;
	data[6] = 0x64;
    int return_code0;
    for (size_t i = 0; i < 150; i++)
    {
        auto startTime = std::chrono::high_resolution_clock::now();
        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> frameDuration = endTime - startTime;
        double actualFrameRate = 1.0 / frameDuration.count();
        // 等待，以使每个循环迭代的持续时间达到目标帧持续时间
        std::this_thread::sleep_for(std::chrono::duration<double>(0.01)); 
        return_code0 = send_init(0x20, data);	//0x20
    }
	// int return_code0 = send_init(0x20, data);	//0x20
	if (return_code0==0){
		printf("方向盘回正\n");
	} 
	sleep(0.01);
	data[1]=0;
	int return_code1 = send_init(0x20, data);	//0x20
	if (return_code1==0){
		printf("功能关闭，回到人工驾驶状态\n");
	} 
	sleep(0.01);
	unsigned char data1[8] = {0,0,0,0,0,0,0,0};//0x1C5
	data1[0]=0xA;
	data1[1]=0x0;
	// data1[2]=0x0;
	// data1[2]|= 0x0B<<8;
    data1[2]= 0x0B;
	data1[3]=0xB8;
	data1[4]=0x2A;
	data1[5]=0x30;
	data1[6]=0x68;
	int return_code2 = send_init(0x1C5, data1);	//0x1C5
	data1[0]=0xB;
	data1[6]=0x48;
	sleep(0.01);
	int return_code3 = send_init(0x1C5, data1);	//0x1C5
	if (return_code3==0){
		printf("自动驾驶模式退出\n");
	} 
	sleep(0.01);
    close(s);
    printf("退出socketcan\n");
}
void socketcan::open_socketcan(){
    // 创建socket
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        std::cout<<"Error while opening socket"<<std::endl;
        return;
    }
    // 指定can0接口
    strcpy(ifr.ifr_name, "can1");
    ioctl(s, SIOCGIFINDEX, &ifr);
    // 绑定socket
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return;
    }
    startsocketcanflag = true;
    std::cout<<"open socketcan succesed"<<std::endl;
}
int socketcan::send_init(unsigned int frame_ID, unsigned char* data)
{
    frame.can_id = frame_ID;  // 设置CAN ID
    frame.can_dlc = 8;     // 设置数据长度
    frame.data[0] = data[0];   // 数据字节1
    frame.data[1] = data[1];   // 数据字节2
    frame.data[2] = data[2]; 
    frame.data[3] = data[3]; 
    frame.data[4] = data[4]; 
    frame.data[5] = data[5]; 
    frame.data[6] = data[6]; 
    frame.data[7] = data[7];
    // 发送CAN数据帧
    if (send(s, &frame, sizeof(struct can_frame), 0) < 0) {
        //perror("Error in sending frame");
        return -1;       
    }
}

void socketcan::send_data() {  
    std::cout<<"running"<<std::endl;
    while (running) { 
        // if (!startsocketcanflag) {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //     continue;
        // }
        auto startTime = std::chrono::high_resolution_clock::now();
        frame.can_id = 0x20;  // 设置CAN ID
        frame.can_dlc = 8;     // 设置数据长度
        // message_0x20.acc_ask_enable=0x0;
        // message_0x20.sw_angle = 0;
        // message_0x20.eps_auto_model=0x1;
        // message_0x20.brake =0;
        short sw_angle = (sw_angle_ + 2048) * 16;//16
        short brake = brake_*10;
        frame.data[0] = brake;   // 数据字节1
        frame.data[1] = 0x0;   // 数据字节2
        frame.data[1]|= 0x09;//0x1<<3; 
        frame.data[2] = 0;
        frame.data[3] = 0;
        frame.data[4]= (sw_angle >> 8)&0xFF;
        frame.data[5]= sw_angle & 0xFF;
        frame.data[6]= 0x64; 
        frame.data[7]= 0;
        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> frameDuration = endTime - startTime;
        std::this_thread::sleep_for(std::chrono::duration<double>(targetFrameDuration - frameDuration.count())); 
        send_data_nobusy(s,  &frame);
        // if (send(s, &frame, sizeof(struct can_frame), 0) < 0) {
        //     //std::cout<<"Error in sending frame"<<std::endl;
        // }
        
        auto startTime2 = std::chrono::high_resolution_clock::now();

        frame.can_id = 0x1C5;  // 设置CAN ID
        frame.can_dlc=8;
        int speed_ask =speed_ask_*20;
        for (size_t i = 0; i < 8; i++)
        {
            frame.data[i]=0;
        }
        frame.data[0] = gear_ask_;   // 数据字节1
        frame.data[1] = (speed_ask>>3)&0xFF;   //数据字节
        frame.data[2] = (speed_ask<<5)&0xFF; 
        frame.data[3] =0xB8;
        frame.data[4]= 0x2A;
        frame.data[5]= 0x30;
        frame.data[6]= 0x68;
        // frame.data[6] =0x1<<5;
        // frame.data[6]|=0x1<<6;//车辆控制模式请求  ; 
        frame.data[7] = 0;
        // printf("十六进制  %X %X %X %X %X %X %X %X\n",(int)(frame.data[0]&0xff),(int)(frame.data[1]&0xff),
        //                 (int)(frame.data[2]&0xff),(int)(frame.data[3]&0xff),(int)(frame.data[4]&0xff),
        //                (int)(frame.data[5]&0xff),(int)(frame.data[6]&0xff),(int)(frame.data[7]&0xff));
        auto endTime2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> frameDuration2 = endTime2 - startTime2;
        // 等待，以使每个循环迭代的持续时间达到目标帧持续时间
        std::this_thread::sleep_for(std::chrono::duration<double>(targetFrameDuration - frameDuration2.count())); 
        send_data_nobusy(s,  &frame);
        // if (send(s, &frame, sizeof(struct can_frame), 0) < 0) {
        //         //std::cout<<"Error in sending frame"<<std::endl;
        //         // return -1;       
        // }
    }
}
void socketcan::message_0x20(int brake,int sw_angle){
    frame_ID_ =0x20;
    brake_ = brake;
    sw_angle_ =sw_angle;
}
void socketcan::message_0x1c5(unsigned char gear_ask,int speed_ask){

    frame_ID_ =0x1c5;
    gear_ask_ =gear_ask;
    speed_ask_ =speed_ask;
}
void socketcan::meeeage_all(int brake,int sw_angle,unsigned char gear_ask,int speed_ask)
{
    brake_ = brake;
    sw_angle_ =sw_angle;
    gear_ask_ =gear_ask;
    speed_ask_ =speed_ask;
}
void socketcan::start_thread() {
    // 创建线程
    running =true;
    send_thread = std::thread(&socketcan::send_data,this);
    recv_thread = std::thread(&socketcan::receive_data,this);
    send_thread.detach();  // 将线程放到后台执行，此处不阻塞 or can0read_thread.join(); 
    recv_thread.detach();
}

void socketcan::receive_data() {
    while (running){
        //std::cout<<"s: "<<s<<std::endl;
        //std::cout << "running: " << running << " startsocketcanflag: " << startsocketcanflag << std::endl;

        // 使用 read_fds 监视可读事件
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(s, &read_fds);  // 监视套接字 s

        struct timeval timeout;
        timeout.tv_sec = 0;      // 设置超时时间
        timeout.tv_usec = 500000; // 0.5 秒

        int ret = select(s + 1, &read_fds, NULL, NULL, &timeout);  // 只监视读事件

        if (ret < 0) {
            //std::cerr << "select() failed: " << strerror(errno) << std::endl;
            continue;
        } else if (ret == 0) {
            // 超时，继续下一个循环
            //std::cout << "Timeout, no data available" << std::endl;
            continue;
        }

        if (FD_ISSET(s, &read_fds)) {
            struct can_frame receive_frame;
            int nbytes = read(s, &receive_frame, sizeof(struct can_frame));
            if (nbytes < 0) {
                //std::cerr << "Error reading CAN frame: " << strerror(errno) << std::endl;
                continue; // 错误时跳过此轮
            }

            // 仅处理 ID 为 0x451 的 CAN 帧
            if (receive_frame.can_id == 0x451) {
                std::cout << "Received CAN frame with ID 0x451:" << std::endl;

                // 确保使用 receive_frame 来访问数据
                uint8_t second_byte = receive_frame.data[1];
                if ((second_byte & (1 << 1)) == 1) {
                    brake_flag = 1;
                } else {
                    brake_flag = 0;
                }
                //std::cout << "brake_flag: " << brake_flag << std::endl;
            } else {
                // 可以在这里处理其他 ID 的帧
                //std::cout << "Received a frame with different ID: " << receive_frame.can_id << std::endl;
            }
        }
    }
        
}

void socketcan::send_data_nobusy(int socket_fd, struct can_frame *frame) {
    fd_set write_fds;
    FD_ZERO(&write_fds);
    FD_SET(s, &write_fds);

    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    int ret = select(s + 1, NULL, &write_fds, NULL, &timeout);
    
    if (ret > 0 && FD_ISSET(s, &write_fds)) {
        int nbytes = send(s, frame, sizeof(struct can_frame), 0);
        if (nbytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                //std::cout << "Send operation would block, try again later." << std::endl;
            } else {
                //perror("Error in sending frame");
            }
        } else {
            //std::cout << "Frame sent successfully" << std::endl;
        }
    } else if (ret == 0) {
        //std::cout << "Timeout: Socket is not ready to send" << std::endl;
    } else {
        //perror("select() error");
    }
}







int socketcan::getbrakeflag() {
    return brake_flag;
}








