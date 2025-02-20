#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include"std_msgs/msg/int32.hpp"
#include <Eigen/Eigen>
#include<Eigen/Dense>
#include<Eigen/Core>
#include "toolKits.h"
#include "localFuns.h"
#include "lqr.h"
#include "PID.h"
#include "stanley.h"
#include <fstream>
#include"Projectionpointcalculation.h"
#include"pure_pursuit.h"
#include"digital_filter_coefficients.h"
#include"digital_filter.h"
#include <iostream>
#include <fstream>
#include <cstdlib> // 包含 <cstdlib> 头文件以使用 std::abs()
#include "lon_controller.h"
#include <chrono>
#include"KineMPCControler.h"
#include"controlmath.h"
#include"canbus_msg/msg/can_bus_msg.hpp"
#define ts_ 0.01//0.01 car0.02 yijie
#define cutoff_freq 20   //20  ///8  car yijie  
class control_node : public rclcpp::Node{
    public:
        //构造函数,有一个参数为节点名称
        control_node(std::string name) : Node(name){
            RCLCPP_INFO(this->get_logger(), "%s start!", name.c_str());
            //创建话题订阅者，订阅GPS消息
            gps_subscribe = this->create_subscription<std_msgs::msg::Float64MultiArray>("gps", 5, std::bind(&control_node::gps_callback, this, std::placeholders::_1));
            //创建话题订阅者, 订阅者全局路径消息
            local_subscribe = this->create_subscription<std_msgs::msg::Float64MultiArray>("local_to_control_pub", 10, std::bind(&control_node::local_callback, this, std::placeholders::_1));
            
            //local_subscribe0 = this->create_subscription<std_msgs::msg::Float64MultiArray>("local_publisher", 10, std::bind(&control_node::local_callback0, this, std::placeholders::_1));
            
            sub_global=this->create_subscription<std_msgs::msg::Float64MultiArray>("global_local_topic", 10, std::bind(&control_node::global_callback, this, std::placeholders::_1));
            
            //创建相机目标检测订阅者
            sub_cam = this->create_subscription<std_msgs::msg::Float64MultiArray>("cam_pub", 10, std::bind(&control_node::cam_callback, this, std::placeholders::_1));
            
            
            sub_lidar = this->create_subscription<std_msgs::msg::Float64MultiArray>("pub_control_obses", 1, std::bind(&control_node::lidar_callback, this, std::placeholders::_1));

            pub_can = this->create_publisher<canbus_msg::msg::CanBusMsg>("/control_pub",1);

            sub_DecelerateFlag = this->create_subscription<std_msgs::msg::Int32>("pub_DecelerateFlag", 5, std::bind(&control_node::subDecelerateFlag, this, std::placeholders::_1));
            sub_brake_flag = this->create_subscription<std_msgs::msg::Int32>("/brake_flag", 5, std::bind(&control_node::brakeflag_callback, this, std::placeholders::_1));
           // sub_global= this->create_publisher<std_msgs::msg::Float64MultiArray>("global_local_topic",10);
            pub_speed = this->create_publisher<std_msgs::msg::Float32>("/speed_topic",1);
            pub_error = this->create_publisher<std_msgs::msg::Float32>("/error_topic",1);
            pub_steer_angle = this->create_publisher<std_msgs::msg::Float32MultiArray>("/steer_angle_topic",1);

            //创建话题订阅者, 订阅者速度与档位消息
            speed_gears_subscribe = this->create_subscription<std_msgs::msg::Int64MultiArray>("speed_gears_pub", 10, std::bind(&control_node::speed_gears_callback, this, std::placeholders::_1));
            //初始化can
            //can_set();
            //定时发送can报文
            oendTime = std::chrono::steady_clock::now();
            sTime = std::chrono::steady_clock::now();
            // timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&control_node::timer_can_callback, this));  
            timer0 = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&control_node::timer_can_callback0, this));     
            // control_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("control_pub", 10);         
            ofs.open(fileName);
            double kp = 50, ki = 0, kd = 0;
            pid.setPID(kp,ki,kd);   
            InitializeFilters(); 
        }

        void gps_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
            auto start = std::chrono::high_resolution_clock::now();  
            if(msg->data.size()!=0){            
                // // GPS Data
                gpsx = msg->data[0]; 
                gpsy = msg->data[1];
                gpsD = msg->data[2];
                gpsS = msg->data[3];
                gpsA = msg->data[7];
                car.resize(5,1);
                tool::getCarPosition(gpsx, gpsy, gpsD, gpsS, car);
            }
            auto end = std::chrono::high_resolution_clock::now();  
            std::chrono::duration<double> duration = end - start;
            //std::cout << "control运行时间: " << duration.count() << " 秒" << std::endl;
        } 
        
        void timer_can_callback0(){           
            if(run_stop_flag){
                //没有局部路径规划 
                if((interpolationoptTrajxy.array() != 0.0).any() == 0||interpolationoptTrajxy.cols()== 1) {
                    std::cout<<"optTrajxy in None!!!"<<std::endl;
                    newSpeedRef=0;    
                    return;
                }
                /**********************激光雷达未启动时****************************/
                if ((obs_lidar.array()!=0.0).any()==0){
                    obs_lidar.setOnes(2,1);
                    obs_lidar*=1000;
                }
                Eigen::VectorXd::Index minCol;
                double min_long_dis = obs_lidar.row(0).minCoeff(&minCol);
                int mindex=0;
                // close Car 
                tool::frentPoint car_frent;
                if (car.size()!=0){
                    carfindClosestPoint(car,globalPath,mindex);
                    cartesianToFrenet(car,globalPath,car_frent,mindex);
                    index =mindex;
                }else{
                    return;
                }
                if (first_vehicle_start_flag) {  //车辆首次启动的操作 
                    current_a = vehicle_start_acc ;
                    if (car(2) > speed_threshold) {
                        first_vehicle_start_flag = false;
                    }
                } else {
                    //判断是否由静止转为起步  上一步标志为 true 当前帧为false 并且车辆静止 这仅仅是启动条件 
                    //启动后 由replan_vehicle_start_flag标志符进行控制 
                    std::cout<<"Control: "<<std::endl;
                    std::cout<<"previous_DecelerateFlag: "<<previous_DecelerateFlag<<" "<<"DecelerateFlag: "<<DecelerateFlag<<" "
                             <<"car_speed: "<<car(2)<<" "<<"replan_vehicle_start_flag: "<<replan_vehicle_start_flag<<std::endl;                             
                    if ((previous_DecelerateFlag && !DecelerateFlag && car(2) < 0.1) ||
                        replan_vehicle_start_flag) {
                        current_a = vehicle_start_acc ;
                        replan_vehicle_start_flag = true;
                        if (car(2) > speed_threshold) {
                            replan_vehicle_start_flag = false;
                        } 
                    } else {
                        /******************PID************************/
                        double a;
                        std::tuple<double, double, double, double, double> vehicle_state;
                        vehicle_state = std::make_tuple(car(0), car(1), car(3), car(2), gpsA);
                        loncontroller_.Init(closestIndex, vehicle_state, localpath_time);
                        loncontroller_.ComputeControlCommand(interpolationoptTrajxy, a);
                        current_a = a;                                            
                    }
                }
                previous_DecelerateFlag = DecelerateFlag;
                std_msgs::msg::Float32 speed_msg;
                speed_msg.data = car(2);
                pub_speed->publish(speed_msg);
                /***********************转角**************************/
                sw = CalculateSteerAngle(index);       
                if((index > (maxIndex - 40))){
                    /****************************End Stop*******************************/
                    SendBusCan(sw, -2, 1);
                    std::cout << "End-brake!!!" << std::endl;
                    std::cout << "car.v: " << car(2) << std::endl;
                    if(gpsS < 0.03){
                        SendBusCan(sw, -2, 1);
                        run_stop_flag = false;
                        std::cout << "End-Stop!!!" << std::endl;
                    }
                }else{
                    /*****************Driving*******************/
                    std_msgs::msg::Float32MultiArray msg;
                    //接近局部路径的终点 车辆正在减速 3m的距离 
                    if (closestIndex > interpolationoptTrajxy.cols() - 2) { //也就是2m的距离
                        SendBusCan(sw, -2, 1);
                    } else {
                        SendBusCan(sw, current_a, 2);
                    }
                    // if(closestIndex>=static_cast<int>(optTrajxy.cols()-6)&&DecelerateFlag){//3m的距离 
                    //     SendCan(0,1,sw,1,1);//速度为0 刹车 
                    //     std::cout << "stop car!!!!" << std::endl; 
                }                             
            }
            else{
                SendBusCan(0.0, 0, 1);
            }                 
        }
        
        void cam_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
            /************************障碍物信息获取与处理*************************/
            Eigen::VectorXd r;
            int cols = 0;
            if(msg->data.size() == 0){//无可识别障碍物
                cols = 1;
                obs_cam.setOnes(5, cols);
                obs_cam*=100;
            }else{ //可识别障碍物
                cols = msg->data.size()/2;
                r.setOnes(cols);
                obs_cam.setZero(5, cols);
                Eigen::Map<Eigen::MatrixXd> obs_cam_pos(msg->data.data(), 2, cols);
                obs_cam.row(0) = obs_cam_pos.row(0);
                obs_cam.row(1) = obs_cam_pos.row(1);
                obs_cam.row(4) = r;
            }            
        }

        void lidar_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
            /**获取激光雷达传递过来的信息*/  //min_x max_x min_y max_y thea 
            Eigen::VectorXd r;
            int cols =0;
            if (msg->data.size() == 0)
            {
                obs_lidar.setOnes(2,1); //没有接收到消息 激光雷达的储存队列为1
                obs_lidar*=1000; 
            }
            else{
                cols =msg->data.size()/2;
                obs_lidar.resize(2,cols);
                Eigen::Map<Eigen::MatrixXd> lidar(msg->data.data(),2,cols);
                obs_lidar.row(0)=lidar.row(0);//s
                obs_lidar.row(1)=lidar.row(1);//l
            }
        }
       
        void global_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
            if(msg->data.size() != 0) {      
                int cols = msg->data.back();
                msg->data.pop_back();
                globalPath.resize(7, cols);
                Eigen::Map<Eigen::MatrixXd> globalPathReshaped(msg->data.data(), 7, cols);
                maxIndex = cols;
                globalPath= std::move(globalPathReshaped);
                std::cout<<"control golbal size: "<<globalPath.cols()<<std::endl;
            }
        }

        void local_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
            if(msg->data.size() == 1 && msg->data[0] < 0)
            {
                /***********************制动***************************/
                optTrajxy.resize(1,1);
                Eigen::Map<Eigen::MatrixXd> optTrajxyReshaped(msg->data.data(), 1, 1);
                optTrajxy= optTrajxyReshaped;
                //std::cout << "No optimal local trajs generation"<< std::endl;
            }else{
                localpath_time = (int)msg->data.back();//获取车辆时间
                msg->data.pop_back();
               // maxIndex = (int)msg->data.back();//全局路径的长度 
                msg->data.pop_back();
                int cols = msg->data.size()/9;
                optTrajxy.resize(9, cols);
                Eigen::Map<Eigen::MatrixXd> optTrajxyReshaped(msg->data.data(), 9, cols);
                optTrajxy = std::move(optTrajxyReshaped);
                //control_math::InterpolateUsingS(optTrajxy, interpolationoptTrajxy);
                interpolationoptTrajxy = optTrajxy;
                //线性插值 使用s
            } 
        }

        void local_callback0(const std_msgs::msg::Float64MultiArray::SharedPtr msg){ 
            if(msg->data.size() == 1 && msg->data[0] < 0){
                /***********************制动***************************/
                optTrajxy0.resize(1,1);
                Eigen::Map<Eigen::MatrixXd> optTrajxyReshaped(msg->data.data(), 1, 1);
                optTrajxy0= optTrajxyReshaped;
                // std::cout << "No optimal local trajs generation 000"<< std::endl;
            }else{
                int cols = msg->data.size()/9;
                optTrajxy0.resize(9, cols);
                Eigen::Map<Eigen::MatrixXd> optTrajxyReshaped(msg->data.data(), 9, cols);
                optTrajxy0= optTrajxyReshaped;
            } 
            // std::cout << "control node_optTrajxy0 : " << std::endl;
            // std::cout << optTrajxy0.transpose() << std::endl;     
        }

        void speed_gears_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg){                 
            int gear_flag = msg->data[1];
            run_stop_flag = (bool)msg->data[2];
            if(run_stop_flag == true){ //run==true 启动
                speedRef = msg->data[0];
                switch (gear_flag){ //获得档位索引P = 0, N = 1, D = 2, R = 3;
                    case 0:{
                        gearsRef = 0xB;
                        break;
                    }
                    case 1:{
                        gearsRef = 0xA;
                        break;
                    }
                    case 2:{
                        gearsRef = 0x1;
                        break;
                    }
                    case 3:{
                        gearsRef = 0x9;
                        break;
                    }            
                    default:
                        break;
                }
            }
            std::cout << "speedRef:" << speedRef << ", "<< "gearsRef:" << gearsRef << ", "<< "flag:" << run_stop_flag << std::endl;
        }

        void subDecelerateFlag(const std_msgs::msg::Int32::SharedPtr msg){
            if (msg->data==0) {
                DecelerateFlag = false;
            } else {
                DecelerateFlag = true;
            }           
        }

        void SendBusCan(double control_steer, double acc,
            int control_gear) {
            canbus_msg::msg::CanBusMsg msg;
            msg.control_steer = control_steer; //方向盘角度
            msg.control_steer_rate = 2; //转角速率
            if (acc >=0) { 
                msg.control_brake = 0;
                msg.control_throttle = static_cast<int>(12 * acc);
                msg.throttle_enable = 1;
                msg.brake_enable = 0;
            } else { //a < 0
                msg.control_throttle = 0;
                msg.control_brake = static_cast<int>(15 * acc);
                msg.throttle_enable = 0;
                msg.brake_enable = 1;
            }
            msg.control_gear = control_gear; //档位 gear  0 p     1  n     2 d    3   r
            msg.left_light = 0; 
            msg.right_light = 0;
            msg.mode_enable = 1;
            msg.steer_enable = 1;
            msg.gear_enable = 1;
            msg.light_enable = 0;
            pub_can->publish(msg);
        }
        
        void carfindClosestPoint(const Eigen::VectorXd& realPosition, const Eigen::MatrixXd& globalPath, int& minIndex){
            static int startIndex = 0;
            static int end = globalPath.cols();
            double distance, d_min = std::numeric_limits<double>::max();
            int i = startIndex;
            double dx , dy;
            for (; i < end; i++) {
                dx = globalPath(0, i) - realPosition(0);
                dy = globalPath(1, i) - realPosition(1);
                distance = dx * dx + dy * dy;
                if (distance < d_min) {
                    minIndex = i;
                    d_min = distance;
                    startIndex = i;
                }
            }
            startIndex = startIndex - 20 < 0 ? 0 : startIndex;
            end = startIndex + 100;
            end = end - globalPath.cols() < 0 ? end : globalPath.cols();
        }
        
        void cartesianToFrenet(const Eigen::VectorXd& CAR, const Eigen::MatrixXd& path, tool::frentPoint& carFrent, int& mindex) 
        {
            //findClosestPoint(car, path, minIndex);
            const double dx = CAR(0) - path(0,mindex);
            const double dy = CAR(1) - path(1,mindex);

            const double cos_theta_r = std::cos(path(3,mindex));
            const double sin_theta_r = std::sin(path(3,mindex));

            const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
            // carFrentPoint.d = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
            carFrent.d = std::copysign(cross_rd_nd, cross_rd_nd);
            carFrent.s = path(6, mindex);
        }

        void InitializeFilters(){
             // Low pass filter
            std::vector<double> den(3, 0.0);
            std::vector<double> num(3, 0.0);
            common::LpfCoefficients(
                ts_, cutoff_freq , &den, &num);
            digital_filter_.set_coefficients(den, num);
        }
    
        double CalculateSteerAngle(const int &minindex) {
            int pre_index = (minindex + 8) > (globalPath.cols()-1) ? 
                            (globalPath.cols()-1) : (minindex + 8);//在全局路径向前四个点 也就是4m的距离
            prepoint = 0;//弯道时预瞄距离变短两个点的育苗 1m
            if (std::abs(globalPath(4, minindex)) > 0.03 || std::abs(globalPath(4, pre_index)) > 0.03){ //弯道 
                Q <<100,  0,  0,
                    0, 100,  0,
                    0,  0, 10;
                R << 0, 0, 0, 4;
            } else {
                Q <<20,  0,  0,
                    0, 20,  0,
                    0,  0, 10;
                R << 0, 0, 0, 4;
            }
            //弯道  m
            local::findClosestPoint(car, interpolationoptTrajxy, closestIndex);
            double preview_time = 0.8; //
            double preview_distance = preview_time * gpsS;

            //int pre_closestIndex = closestIndex ;//control_math::findcloseindex(interpolationoptTrajxy, closestIndex, preview_distance);
            // x y v thea dk 投点计算 
            std::array<double,5> Projection_point_message = 
                                Projectionpoint::GetProjectionpoint(closestIndex,interpolationoptTrajxy,car(0),car(1));
            double dx = car(0) - Projection_point_message[0];
            double dy = car(1) - Projection_point_message[1];
            double theta = Projection_point_message[3];
            float error = dy * std::cos(theta) - dx * std::sin(theta);
            std_msgs::msg::Float32 error_msg;
            error_msg.data = error;
            pub_error->publish(error_msg);
            double deltaX =  dx;
            double deltaY =  dy;
            double deltaYAW = tool::normalizeAngle(tool::d2r(car(3)) - theta);
            /***************LQR***********/
            /*Q中元素的大小表示状态量的重要程度，越大表示越重要，控制的快速性会越好，但会增大输入（能量消耗增多），导致超调、震荡可能会比较明显；
            控制权重矩阵R中元素的大小表示对能量消耗的关注度，越大表示对该输入的能量消耗越敏感，会减小输入，有效降低超调、震荡，控制过程柔和，但快速性较差；
            QR的选取是两种约束互相妥协的过程，重点关注调参*/
            //在轨迹跟踪中，前一项优化目标表示跟踪过程路径偏差的累积大小，第二项优化目标表示跟踪过程控制能量的损耗
            ref_delta = atan2(car(4) * interpolationoptTrajxy(4, closestIndex), 1);                     
            LQR.stateSpaceMatrix(A, B, car, cps, ref_delta, theta);
            //u = LQR.calcU(car, interpolationoptTrajxy, pre_closestIndex, A, B, Q, R)(1);
            u=LQR.calcU(car, interpolationoptTrajxy, Projection_point_message, A, B, Q, R)(1);
            // std::cout<<"lqr_u: " <<  u <<std::endl;
            // std::cout<<"lqr_ref_delta: " << ref_delta <<std::endl;

            u = -(u+ref_delta);//-kx+kl
            // Q2 << 10      , 0    , 0,
            //         0      , 10    , 0,
            //         0      , 0    , 12;
            // R2 << 5;   
            // u = LQR.lqrComputeCommand(theta, deltaX, deltaY, deltaYAW, 
            //                         optTrajxy(4, pre_closestIndex), 3.0,car(4),0.01,Q2,R2);          
            // std::cout<<"lqr: " << K * u <<std::endl;
            /***************方向盘限幅*****************/
            /**********MPC************/
            // std::tuple<double, double, double, double> vehicle_state = {car(0), car(1), tool::normalizeAngle(tool::d2r(car(3))), car(2)};
            // std::tuple<double, double, double, double> reference_point = {interpolationoptTrajxy(0, closestIndex), interpolationoptTrajxy(1, closestIndex),
            //                                                                 interpolationoptTrajxy(3, closestIndex), interpolationoptTrajxy(4, closestIndex)};
            // std::vector<double> cmd;
            // KineMPCController mpc;
            // if (mpc.ComputeControlCommand(vehicle_state, reference_point, cmd)){
            //     if (!cmd.empty()) {
            //         std::cout<<"cmd.at(0): " << cmd.at(0)<<std::endl;
            //         u = -(cmd.at(0) + ref_delta);
            //         std::cout<<"mpc: " << K * u <<std::endl;
            //     }
            // }
            //弯道 
            // if (std::abs(globalPath(4, index)) > 0.03 || std::abs(globalPath(4, pre_index)) > 0.03){
            //     sw = K * u;
            // }else{
            //     sw = K * u;
            // }
            double steerangle = K * u;
            if (steerangle > swth) {
                steerangle = swth;
            } else if (steerangle < -swth) {
                steerangle = -swth;
            } 
            double filter_steer_angle = digital_filter_.Filter(steerangle);
            std_msgs::msg::Float32MultiArray steer_angle_msg;
            steer_angle_msg.data.emplace_back(steerangle);
            steer_angle_msg.data.emplace_back(filter_steer_angle);
            pub_steer_angle->publish(steer_angle_msg);
            return filter_steer_angle;
        }

        void brakeflag_callback(const std_msgs::msg::Int32::SharedPtr msg){
            int brake_flag = msg->data;

        }


    private:
        // 声明gps话题订阅者
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gps_subscribe;
        // 声明local_trajs话题订阅者
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr local_subscribe;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr local_subscribe0;
        // 声明cam话题订阅者
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_cam;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_lidar;
        // 声明speed_gears话题订阅者
        rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr speed_gears_subscribe;
        //声明control话题发布者
        rclcpp::Publisher<canbus_msg::msg::CanBusMsg>::SharedPtr pub_can;

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_global;

        // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr control_publisher;
        
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_DecelerateFlag;

        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_brake_flag;


        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_speed;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_error;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_steer_angle;

        // 声名Can定时器指针
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer0;

        Eigen::VectorXd car;
        Eigen::MatrixXd optTrajxy;//局部路径
        Eigen::MatrixXd interpolationoptTrajxy;//局部路径
        Eigen::MatrixXd optTrajxy0;//局部路径
        Eigen::MatrixXd optTrajsd;
        Eigen::MatrixXd obs_cam;
        Eigen::MatrixXd obs_lidar;
        Eigen::MatrixXd globalPath;//全局路径


        tool::controlParams cps;
        tool::frentPoint carFrentPoint;
        int closestIndex = 0;

        double gpsx;
        double gpsy;
        double gpsD;
        double gpsS;
        double gpsA;
        //int  index = -1;//最近点下标
        bool in_flag = false; //文本提示输出的标志位
        /*************From HMI**********************/
        bool run_stop_flag = false; //启动停止标志位
        double speedRef = 0; //设定速度
        unsigned short gearsRef = 0xA;
        double realSpeed = 0;//渐加速变量，最大为设定值
        double newSpeedRef=0;
        int prepoint = 0;//预瞄点的数量 

        /****************LQR**********/
        lqr LQR;
        Eigen::Matrix3d  Q;
        Eigen::Matrix2d  R;
        Eigen::MatrixXd Q2 = Eigen::MatrixXd::Zero(3,3);  
        Eigen::MatrixXd R2 = Eigen::MatrixXd::Zero(1,1);
       
        double ref_delta  = 0;
        Eigen::Matrix<double, 3, 3>  A;
        Eigen::Matrix<double, 3, 2>  B;

        Eigen::Matrix<double, 3, 3> Q_new;
        Eigen::Matrix<double, 1, 1> R_new;
        double u = 0.0; //
        double K = 13; //
        /*****************************/

        /***********stanley****************/
        double delta = 0.0;

        /***********PID*************/
        PID pid;
        /*********pure pursuit*****/

        double pre_distance = 2 ;

        /********Can*********/
        int ret, ret2, ret3, ret4, ret5;
        float vel;
       
        bool exit_flag = true;

        /*************Cam Obs AEB******************/
        double disOfObsMax = 3;  //(单位：m)
        double disOfObsMin = 1;
        bool initstart = true;
        int index = 0;
        int maxIndex = 0;
        double sw=0;
        bool accflag=false;

        /******************************************/
        using time_point = std::chrono::steady_clock::time_point;
        using time_ms = std::chrono::milliseconds;
        time_point t1,t2,tk1,tk2,ntk1,ntk2,oendTime, sTime;
        time_ms ms,dtk,dntk;
        bool KF=true;
        bool NKF=true;

        /*******************/
        // double ts_=0.002; //采样时间 
        // int cutoff_freq=10;  //car yijie   截至频率

        /**********Out File**********/
        std::string fileName = "path1.txt";
        std::ofstream ofs;  //创建输入流对象 

        bool DecelerateFlag = false;
        double current_a;//车辆当前的加速度 
        loncontroller loncontroller_;
        double localpath_time;
        double swth = 410;
        double Constant_deceleration = -0.5; 
        DigitalFilter digital_filter_;

        /**************MPC**************/


        /********起步速度控制*********/
        bool first_vehicle_start_flag = true; //车辆第一次起步控制
        bool replan_vehicle_start_flag = false; //车辆静止后再次起步 
        double vehicle_start_acc = 1;//起步加速度控制 1 m/s^s
        double speed_threshold = 1.38;//起步速度阈值 5km/h  
        bool previous_DecelerateFlag = false;

        /************canbus*************/
};





int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<control_node>("control_node");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}