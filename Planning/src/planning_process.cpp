#include "planning_process.h"

PlanningProcess::PlanningProcess()
    : Node("planning_process")
{
    // 初始化智能指针对象frame_
    RCLCPP_INFO(this->get_logger(), "planning_process created");

    // 创建服务器端
    global_path_service_ = this->create_service<ros2_path_interfaces::srv::PathInterfaces>("global_path_srv",
                                                                                           std::bind(&PlanningProcess::global_path_callback, this, std::placeholders::_1, std::placeholders::_2));

    // gps数据订阅
    gps_subscribe_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("gps", 10, std::bind(&PlanningProcess::gps_callback, this, std::placeholders::_1));

    // 地图订阅
    global_path_subscribe_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("global_local_topic", 10, std::bind(&PlanningProcess::global_callback, this, std::placeholders::_1));

    // 订阅雷达数据
    sub_LiDAR_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("lidar_pub", 10, std::bind(&PlanningProcess::lidar_callback, this, std::placeholders::_1));

    // 订阅相机数据
    sub_cam_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("cam_pub", 10, std::bind(&PlanningProcess::cam_callback, this, std::placeholders::_1));

    // 订阅速度和挡位信息
    speed_gears_subscribe_ = this->create_subscription<std_msgs::msg::Int64MultiArray>("speed_gears", 10, std::bind(&PlanningProcess::speed_gears_callback, this, std::placeholders::_1));

    // 发布局部路径到hmi
    local_to_hmi_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("local_publisher ", 10);

    // 发布路径到控制节点
    local_to_control_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("local_to_control", 10);

    // 可视化car
    sub_car_local_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("car_local_points", 10);

    // 发布到rviz
    pub_rviz_local_ = this->create_publisher<visualization_msgs::msg::Marker>("pub_rviz_local", 10);

    // 发布障碍物信息到hmi
    pub_hmi_obses_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("pub_hmi_obses", 10);

    // 发布障碍物信息到控制节点
    pub_control_obses_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("pub_control_obses", 10);

    // 发布障碍物信息到全局路径
    pub_global_obses_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("global_obses", 10);

    // 发布减速标志
    pub_control_DecelerateFlag_ = this->create_publisher<std_msgs::msg::Int32>("pub_DecelerateFlag", 10);

    // 发布停止线标志
    pub_stop_line_ = this->create_publisher<std_msgs::msg::Float64>("pub_stop_line", 1);

    // 局部路径生成主函数
    timer_getlocalpath_ = this->create_wall_timer(std::chrono::milliseconds(Slowdowntimethreshold), std::bind(&PlanningProcess::get_local_path, this));
}

bool PlanningProcess::process()
{

    frame_ = std::make_shared<Frame>(Eigen::MatrixXd::Random(5, 2), Eigen::VectorXd::Random(5));
    // frame_->obs_lidar_ << 1, 2,
    //     3, 4,
    //     5, 6,
    //     7, 8,
    //     9, 10;
    // frame_->car_ << 1, 2, 3, 4, 5;

    // cout 输出 frame
    std::cout << "frame_->obs_lidar_:" << std::endl;
    std::cout << frame_->obs_lidar_ << std::endl;
    std::cout << "frame_->car_:" << std::endl;
    std::cout << frame_->car_ << std::endl;

    return true;
}

// 相应全局地图服务的回调函数，用于人机交互
void PlanningProcess::global_path_callback(const std::shared_ptr<ros2_path_interfaces::srv::PathInterfaces::Request> request, const std::shared_ptr<ros2_path_interfaces::srv::PathInterfaces::Response> response)
{
    if (request->global_request > 0)
    {
        response->local_response = 1;
    }
}

// 给控制端发送障碍物信息
void PlanningProcess::SendControlObs(std::vector<obses_sd> &obses)
{
    std_msgs::msg::Float64MultiArray msg;
    // msg.data.resize(obses.size()*2);
    for (size_t i = 0; i < obses.size(); ++i)
    {
        msg.data.push_back(obses[i].centre_points.s);
        msg.data.push_back(obses[i].centre_points.l);
    }
    pub_control_obses_->publish(msg);
}
// 给交互界面发送障碍物信息
void PlanningProcess::SendHmiObs(std::vector<Eigen::VectorXd> &obses)
{
    std_msgs::msg::Float64MultiArray obs_msg;
    if (obses.size() > 0)
    {
        // obs_msg.data.resize(obses.size()*5);
        for (size_t i = 0; i < obses.size(); ++i)
        {
            obs_msg.data.push_back(obses[i](0));
            obs_msg.data.push_back(obses[i](1));
            obs_msg.data.push_back(obses[i](2));
            obs_msg.data.push_back(obses[i](3));
            obs_msg.data.push_back(obses[i](4));
        }
        pub_hmi_obses_->publish(obs_msg);
    }
    else
    {
        pub_hmi_obses_->publish(obs_msg);
    }
}
// 给全局路径发送障碍物信息
void PlanningProcess::SendGlobalObses(std::vector<Eigen::VectorXd> &obses)
{
    std_msgs::msg::Float64MultiArray obs_msg;
    if (obses.size() > 0)
    {
        // obs_msg.data.resize(obses.size()*5);
        for (size_t i = 0; i < obses.size(); ++i)
        {
            obs_msg.data.push_back(obses[i](0));
            obs_msg.data.push_back(obses[i](1));
            obs_msg.data.push_back(obses[i](2));
            obs_msg.data.push_back(obses[i](3));
            obs_msg.data.push_back(obses[i](4));
        }
        pub_hmi_obses_->publish(obs_msg);
    }
    else
    {
        pub_hmi_obses_->publish(obs_msg);
    }
    pub_global_obses_->publish(obs_msg);
}

// gps的回调函数，生成主车的gps坐标
void PlanningProcess::gps_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // 判断msg->data 是否是空的
    if (msg->data.empty())
    {
        // RCLCPP_INFO(this->get_logger(), "gps data is empty");
        car_.resize(1, 1);
    }
    // 生成主车的gps坐标
    {
        gpsx_ = msg->data[0];
        gpsy_ = msg->data[1];
        gpsD_ = msg->data[2];
        gpsS_ = msg->data[3];
        gpsA_ = msg->data[4];
        car_.resize(5, 1);
        tool::getCarPosition(gpsx_, gpsy_, gpsD_, gpsS_, car_);
    }
}

// 全局路径的回调函数，获取全局路径
void PlanningProcess::global_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 0)
    {
        if (!msg->data.empty())
        {
            int cols = msg->data.back();
            msg->data.pop_back();
            // 检查剩余数据量是否满足 7*cols
            if (msg->data.size() != static_cast<size_t>(7 * cols))
            {
                RCLCPP_ERROR(this->get_logger(), "globalPath data size mismatch: expected %d but got %zu", 7 * cols, msg->data.size());
                return;
            }
            // 直接将数据映射为一个 7 x cols 的矩阵，利用具体的 7 行模板参数
            globalPath = Eigen::Map<const Eigen::Matrix<double, 7, Eigen::Dynamic>>(msg->data.data(), 7, cols);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "globalPath data is empty");
        }
        // std::cout << "Local Node global path: " << std::endl;
        // std::cout << globalPath.transpose() << std::endl;
        // for (size_t i = 0; i < globalPath.cols(); ++i)
        // {
        //     outputFile << globalPath(4, i) << std::endl;
        // }
        // outputFile.close();
    }
}

// 雷达的回调函数，对雷达返回的数据做初步处理
void PlanningProcess::lidar_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    /**获取激光雷达传递过来的信息*/ // min_x min_y max_x  max_y thea  全局坐标系下
    if (msg->data.empty())
    {
        // 给雷达生成一个比较大的数
        obs_lidar_.setOnes(5, 1);
        obs_lidar_ *= 1000;
    }
    {
        int cols = msg->data.size() / 5; // 有多少个障碍物信息
        obs_lidar_.resize(5, cols);
        // 赋值，将雷达获取的到数据变成5行，clos列，给obs_lidar_
        // 直接将 msg->data 映射为一个 Eigen matrix，如果 msg->data 数据数量确实为 5*cols
        obs_lidar_ = Eigen::Map<const Eigen::Matrix<double, 5, Eigen::Dynamic>>(msg->data.data(), 5, cols);

        // 保证北斗和全局路径都存在
        if (car_.size() == 1 || !(globalPath.array() != 0.0).any())
        {
            return;
        }
        else
        {
            Eigen::MatrixXd T_obs_lidar(obs_lidar_.rows(), obs_lidar_.cols()); // 将雷达生成的障碍物膨胀，防止碰撞
            double Expansiondistance = 0.2;                                    // 10cm *2 一共是二十厘米
            // 对第0、1行统一减去 Expansiondistance
            obs_lidar_.block(0, 0, 2, obs_lidar_.cols()).array() -= Expansiondistance;
            // 对第2、3行统一加上 Expansiondistance
            obs_lidar_.block(2, 0, 2, obs_lidar_.cols()).array() += Expansiondistance;

            double thea = tool::d2r(car_(3));
            double costhea = std::cos(thea);
            double sinthea = std::sin(thea);
            double zhouju = 1.308;
            double term_x = car_(0) + zhouju * costhea; // 恢复到车轴中心坐标系下
            double term_y = car_(1) + zhouju * sinthea;

            // 构造二维旋转矩阵
            Eigen::Matrix2d R;
            R << costhea, -sinthea,
                sinthea, costhea;

            // 使用循环处理两个相似的 block（分别对应 obs_lidar_ 的第0~1行和第2~3行）
            for (int i = 0; i < 2; i++)
            {
                int start_row = i * 2;
                // 提取需要旋转的2行数据（X，Y部分）
                Eigen::Matrix2Xd src_block = obs_lidar_.block(start_row, 0, 2, obs_lidar_.cols());
                // 进行旋转变换并加上平移量
                Eigen::Matrix2Xd transformed = R * src_block;
                transformed.colwise() += Eigen::Vector2d(term_x, term_y);
                // 将变换后的数据赋值到 T_obs_lidar 对应 block 中
                T_obs_lidar.block(start_row, 0, 2, obs_lidar_.cols()) = transformed;
            }
            // 对第4行（其他数据）直接赋值
            T_obs_lidar.row(4) = obs_lidar_.row(4);

            obs::frentPoint FrentPoint_;
            obs::cartesianToFrenet(car_, globalPath, FrentPoint_, obs_car_globalpath_index); // 车辆在全局坐标系下的sd 编号
            obs::CalculateobsesSD(FrentPoint_, globalPath, T_obs_lidar, obses_limit_SD, LidarcoordinatesystemObsesLimit, obs_lidar_,
                                  GlobalcoordinatesystemObsesLimit, obs_car_globalpath_index);

            SendHmiObs(LidarcoordinatesystemObsesLimit);
            SendControlObs(obses_limit_SD);
            SendGlobalObses(GlobalcoordinatesystemObsesLimit);
        }
    }
}

// 相机的回调函数，对相机返回的数据做初步处理
void PlanningProcess::cam_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
}

// 速度挡位信息的回调函数，对速度挡位信息做初步处理
void PlanningProcess::speed_gears_callback(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
{
}

// 局部路径生成函数
bool PlanningProcess::get_local_path()
{
    // 如果 scenarioManager_ 不存在，则创建；否则更新数据
    if (scenario_manager_ == nullptr)
    {
        scenarioManager_ = std::make_unique<ScenarioManager>(car_, globalPath, obs_lidar);
    }
    else
    {
        scenarioManager_->UpdateData(car_, globalPath, obs_lidar);
    }
    state_ = scenarioManager->Update();
    // 根据senum class ScenarioState
    // {
    //     INIT,     // 第一次执行，初始化状态
    //     STRAIGHT, // 直行状态
    //     TURN,     // 转弯状态
    //     NEAR_STOP // 到达停止线附近
    // };
    // tate_状态执行不同的函数
    switch (state_)
    {
    case ScenarioState::INIT:
        // 创建一个FIrstRun类的智能指针
        first_run_ = std::make_unique<FirstRun>(car_, globalPtah, obs_lidar_);
        bool isFirstRunSuccessful = first_run_->Process();
        if (isFirstRunSuccessful)
        {
            scenarioManager_->ChangeFirstRun();
        }
        
        break;
    case ScenarioState::STRAIGHT:
        // 创建一个LaneFollow类的智能指针
        lane_follow_ = std::make_unique<LaneFollow>(car_, globalPtah, obs_lidar_);
        bool isLaneFollowSuccessful = lane_follow_->Process();
        break;
    case ScenarioState::TURN:
        // 创建一个ApproachingIntersection类的智能指针
        approaching_intersection_ = std::make_unique<ApproachingIntersection>(car_, globalPtah, obs_lidar_);
        bool isTurnSuccessful = approaching_intersection_->Process();
        break;

    case ScenarioState::NEAR_STOP:
        // 创建一个NearStop类的智能指针
        near_stop_ = std::make_unique<NearStop>(car_, globalPtah, obs_lidar_);
        bool isNearStopSuccessful = near_stop_->Process();
        break;
    }
    return false;
}
