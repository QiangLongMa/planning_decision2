#include"canbus/canbus_socketcan.h"
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
canbus_socketcan::canbus_socketcan()
{
    open_socketcan();//openc can
	sleep(0.01);
    start_thread();
}
canbus_socketcan::~canbus_socketcan(){
   exit_socketcan();
}
void canbus_socketcan::exit_socketcan(){
    running = false;
    sleep(0.1);
    if (send_thread.joinable()) {
        send_thread.join();

    }
    if (recv_thread.joinable()) {
        recv_thread.join();

    }
    if (s >= 0) {
        close(s);
        s = -1;
    }
    printf("退出socketcan\n");
}
void canbus_socketcan::open_socketcan(){
    // 创建socket
    std::cout<<"open_socketcan_s: "<<s<<std::endl;
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        std::cout<<"Error while opening socket"<<std::endl;
        return;
    }
    std::cout<<"open_socketcan_s: "<<s<<std::endl;
    // 指定can1接口
    strcpy(ifr.ifr_name, "can1");
    ioctl(s, SIOCGIFINDEX, &ifr);
    // 绑定socket
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cout<<"Error in socket bind"<<std::endl;
        return;
    }
    std::cout<<"open socketcan succesed"<<std::endl;
}
void canbus_socketcan::send_data() {  
    int cnt = 0;
    while (running) {
        // if (s < 0 ) {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //     continue;
        // }
        auto startTime = std::chrono::high_resolution_clock::now();
        canbus_msg::msg::CanBusMsg  ctrl_flags_= tmp_flags;
        if (cnt%4 == 0) {
            struct can_frame frame;
            frame.can_id = 0xE2;  // 设置CAN ID
            frame.can_dlc = 8;     // 设置数据长度
            for (size_t i = 0; i < 8; i++) {
                frame.data[i] = 0X00;
            }
            unsigned char enableByte = 0x00;
            if (ctrl_flags_.mode_enable) {
                enableByte |= 0x08;
                if(ctrl_flags_.steer_enable)
                    enableByte |= (1 << 6);
                if(ctrl_flags_.throttle_enable)
                    enableByte |= (1 << 5);
                if(ctrl_flags_.brake_enable)
                    enableByte |= (1 << 7);
                if(ctrl_flags_.gear_enable) {
                    enableByte |= (1 << 2);
                    switch (ctrl_flags_.control_gear)
                    {
                    case 3: //Gear::N:
                    {
                        enableByte |= 0x00;
                        break;
                    }
                    case 4: //Gear::D:
                    {
                        enableByte |= 0x01;
                        break;
                    }
                    case 2: //Gear::R:
                    {
                        enableByte |= 0x02;
                        break;
                    }
                    default:
                        enableByte |= 0x00;
                    }
                }
            }
            frame.data[0] = enableByte;
            if(ctrl_flags_.control_throttle>=0 && ctrl_flags_.control_brake==0)
            {
                frame.data[1] = static_cast<unsigned char>(ctrl_flags_.control_throttle & 0xff);
                frame.data[2] = 0x00;
                frame.data[5] = 0x00;
            }   
            else
            {
                frame.data[1] = 0x00;
                frame.data[2] = static_cast<unsigned char>(ctrl_flags_.control_brake & 0xff);
                frame.data[5] = static_cast<unsigned char>(ctrl_flags_.control_brake & 0xff);
            }

            int tmpsteer = static_cast<int>(ctrl_flags_.control_steer);

            frame.data[4] = static_cast<unsigned char>((tmpsteer*10) >> 8);
            frame.data[3] = static_cast<unsigned char>((tmpsteer*10) & 0xff);
            frame.data[6] = 0xff;
            send_data_nobusy(s, &frame);
            // if (send(s, &frame, sizeof(struct can_frame), 0) < 0) {
            //         //std::cout<<"Error in sending oxE2 frame"<<std::endl;
            // }
        }
        if (cnt %20 == 0) {
            struct can_frame frame;
            //灯光控制 
            frame.can_id = 0xE0;  // 设置CAN ID
            frame.can_dlc = 8;     // 设置数据长度
            for (size_t i = 0; i < 8; i++){
                frame.data[i] = 0x00;
            }
            if (ctrl_flags_.left_light == 1) {
                frame.data[5] = 0x80;
            }
            if (ctrl_flags_.right_light == 1) {
                frame.data[6] = 0x80;
            }
            send_data_nobusy(s, &frame);
            // if (send(s, &frame, sizeof(struct can_frame), 0) < 0) {
            //     //std::cout<<"Error in sending 0xE0 frame"<<std::endl;
            // }
        }
        cnt ++ ;
        if (cnt >= 10000) cnt = 0;
        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> frameDuration = endTime - startTime;
        std::this_thread::sleep_for(std::chrono::duration<double>(targetFrameDuration - frameDuration.count())); 
    }

    std::cout<<"Send Thread Is Exiting"<<std::endl;
}

void canbus_socketcan::recv_data () {
    struct can_frame package;
    struct can_filter rfilter[1];  // 定义过滤器
    int sizeOfFrame = sizeof(struct can_frame);
    // 设置过滤器，只接收can_id为0x34a的数据帧
    rfilter[0].can_id = 0x34a;  // 设置ID为0x34a
    rfilter[0].can_mask = CAN_SFF_MASK;  // 使用标准帧格式的掩码
     // 设置过滤器
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) {
        std::cerr << "Error setting filter" << std::endl;
        return;
    }
    // 使用 read_fds 监视可读事件
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(s, &read_fds);  // 监视套接字 s

    struct timeval timeout;
    timeout.tv_sec = 0;      // 设置超时时间
    timeout.tv_usec = 5000; // 0.5 秒
    while (running) {   
        //std::cout<<"recv"<<std::endl;    
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
            int cnt = read(s, &package, sizeOfFrame);
            if (cnt > 0) { //处理速度信息
                if (package.can_id == 0x34a) {
                    vehicle_speed = Handleox34a(package);
                }
            } else {
                //std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }
        }
    }

    std::cout<<"Recv Thread Is Exiting"<<std::endl;
}

double canbus_socketcan::Handleox34a (const can_frame &msg) {
    const double speedFactor = 0.03125;
    int wheelSpeedRL = (((int)msg.data[0]) << 8) + msg.data[1];
    int wheelSpeedRR = (((int)msg.data[2]) << 8) + msg.data[3];
    return ((double)(wheelSpeedRL + wheelSpeedRR)) / 2.0 * speedFactor;
}

double canbus_socketcan::getvehicle_speed () {
    return vehicle_speed;
}

void canbus_socketcan::UpdateMessage (canbus_msg::msg::CanBusMsg &msg) {
    tmp_flags = msg;
}

void canbus_socketcan::start_thread () {
    // 创建线程
    running = true;
    send_thread = std::thread(&canbus_socketcan::send_data,this);
    recv_thread = std::thread(&canbus_socketcan::recv_data,this);
    // send_thread.detach();  // 将线程放到后台执行，此处不阻塞 or can0read_thread.join(); 
    // recv_thread.detach();
}

void canbus_socketcan::send_data_nobusy(int socket_fd, struct can_frame *frame) {
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







