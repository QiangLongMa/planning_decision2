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
    local_to_hmi_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("local_publisher", 10);

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
    // 打印一下msg->data的维度
    // std::cout << "gps data size: " << msg->data.size() << std::endl;

    // 判断msg->data 是否是空的
    if (msg->data.empty())
    {
        // RCLCPP_INFO(this->get_logger(), "gps data is empty");
        car_.resize(1, 1);
    }
    // 生成主车的gps坐标
    else
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
    else
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
            T_obs_lidar.resize(obs_lidar_.rows(), obs_lidar_.cols()); // 将雷达生成的障碍物膨胀，防止碰撞
            double Expansiondistance = 0.2;                           // 10cm *2 一共是二十厘米
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

            frentPoint FrentPoint_;
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

// 发布局部路径信息
void PlanningProcess::publish_localpath(Eigen::MatrixXd &localpath)
{
    /***********Only To Control Node*************/
    std_msgs::msg::Float64MultiArray local_trajs_msg;
    // 检查矩阵optTrajxy是否所有元素都等于零。如果所有元素都等于零，整个表达式将返回true；否则返回false
    if ((localpath.array() != 0.0).any() == 0)
    {
        local_trajs_msg.data.push_back(-1);
        local_to_control_publisher_->publish(local_trajs_msg);
        local_to_hmi_publisher_->publish(local_trajs_msg);
    }
    else
    {
        for (size_t i = 0; i < localpath.cols(); ++i)
        {
            localpath(8, i) = localpath(7, i) + heading_time_ + 0.1;
        }
        write_localpath(localpath);
        std::vector<double> localTrajReshape(&localpath(0), localpath.data() + localpath.size());
        local_trajs_msg.data = localTrajReshape;
        local_trajs_msg.data.push_back(globalPath.cols());
        local_trajs_msg.data.push_back(heading_time_);
        local_to_control_publisher_->publish(local_trajs_msg);
        // hmi 发送
        std_msgs::msg::Float64MultiArray local_trajs_msg2;
        std::vector<double> localTrajReshape2(&localpath(0), localpath.data() + localpath.size());
        local_trajs_msg2.data = localTrajReshape2;
        local_to_hmi_publisher_->publish(local_trajs_msg2);
    }
}

void PlanningProcess::write_localpath(Eigen::MatrixXd &path)
{
    // 获取当前路径
    char buffer[256];
    // 获取当前路径
    if (getcwd(buffer, sizeof(buffer)) != nullptr)
    {
        std::cout << "当前路径是: " << buffer << std::endl;
    }
    else
    {
        std::cerr << "获取当前路径失败" << std::endl;
    }
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm *tm_info = std::localtime(&now_time_t);

    // 格式化时间为字符串，格式为 yyyy-mm-dd_HH-MM-SS
    char time_buffer[256];
    std::strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d_%H-%M-%S", tm_info);
    // 检查local_path文件夹是否存在，如果不存在则创建

    std::string local_path_ = std::string(buffer) + "/local_path/";
    // 生成时间作为文件夹名称
    std::string folder_name = local_path_ + std::string(time_buffer);
    static int loopCount = 1;
    std::string fileName = folder_name + std::to_string(loopCount) + ".txt";
    std::ofstream outFile;
    outFile.open(fileName);
    for (size_t i = 0; i < path.cols(); ++i)
    {
        outFile << path(0, i) << " " << path(1, i) << " "
                << path(2, i) << " " << path(3, i) << " "
                << path(4, i) << " " << path(5, i) << " "
                << path(6, i) << " " << path(7, i) << std::endl;
    }
    loopCount = loopCount + 1;
}

/**
 * @brief 获取局部路径生成情况
 *
 * 该函数根据当前的状态生成局部路径，并发送给控制端
 *
 * @return true 生成路径成功
 * @return false 生成路径失败
 */
bool PlanningProcess::get_local_path()
{
    // 判断全局路径和是否为空
    if (globalPath.size() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "globalPath is empty");
        return false;
    }
    // 判断主车的gps坐标是否为空
    if (car_.size() == 1)
    {
        RCLCPP_WARN(this->get_logger(), "car_ is empty");
        return false;
    }
    // 如果 scenarioManager_ 不存在，则创建；否则更新数据
    heading_time_ = scenario_->Time();
    if (!scenario_manager_)
    {
        scenario_manager_ = std::make_unique<ScenarioManager>(car_, globalPath, obs_lidar_);
    }
    else
    {
        scenario_manager_->UpdateData(car_, globalPath, obs_lidar_);
    }
    state_ = scenario_manager_->Update();
    RCLCPP_INFO(this->get_logger(), "Current scenario state: %d", static_cast<int>(state_));
    // 输出car_变量
    indexinglobalpath = scenario_manager_->GetIndex();
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
    {
        // 创建 FirstRun 类的智能指针
        if (!scenario_ || (dynamic_cast<FirstRun *>(scenario_.get()) == nullptr))
        {
            scenario_ = std::make_unique<FirstRun>(car_, globalPath, obses_limit_SD, GlobalcoordinatesystemObsesLimit, gpsA_, indexinglobalpath);

        }
        else
        {
            scenario_->Updated(car_, obses_limit_SD, GlobalcoordinatesystemObsesLimit, gpsA_, indexinglobalpath);
        }  
            // 进行决策
        scenario_->MakeDecision();
        // 规划路径
        bool isFirstRunSuccessful = scenario_->Process();
        if (isFirstRunSuccessful)
        {
            scenario_manager_->ChangeFirstRun();
            // 获取路径
            optTrajxy = scenario_->getlocalpath();
            optTrajsd = scenario_->getlocalpathsd();
            // 发送给控制端
            publish_localpath(optTrajxy); // TODO: 补充发送控制端的逻辑
            write_localpath(optTrajxy);
            return true;
        }
        break;
    }
    case ScenarioState::STRAIGHT:
    {
        RCLCPP_INFO(this->get_logger(), "ScenarioState::STRAIGHT");
        // 创建 LaneFollow 类的智能指针

        if (!scenario_ || (dynamic_cast<LaneFollowScenario *>(scenario_.get()) == nullptr))
        {
            scenario_ = std::make_unique<LaneFollowScenario>(
                car_, globalPath, obses_limit_SD, GlobalcoordinatesystemObsesLimit, gpsA_, indexinglobalpath);
            RCLCPP_INFO(this->get_logger(), "Created new LaneFollowScenario");
            // 將optTrajxy赋予下一个规划的路径
            scenario_->setlocalpath(optTrajxy);
            scenario_->setlocalpath(optTrajsd);
        }
        else
        {
            scenario_->Updated(car_, obses_limit_SD, GlobalcoordinatesystemObsesLimit, gpsA_, indexinglobalpath);
            RCLCPP_INFO(this->get_logger(), "Reusing existing LaneFollowScenario");
        }

        // 进行决策
        scenario_->MakeDecision();
        bool isLaneFollowSuccessful = scenario_->Process();
        if (isLaneFollowSuccessful)
        {
            // 获取路径
            optTrajxy = scenario_->getlocalpath();
            optTrajsd = scenario_->getlocalpathsd();
            // 发送给控制端
            publish_localpath(optTrajxy);
            write_localpath(optTrajxy);
            RCLCPP_INFO(this->get_logger(), "publish in STRAIGHT");
            return true;
        }
        break;
    }
    case ScenarioState::TURN:
    {
        // 创建 ApproachingIntersection 类的智能指针
        if (!scenario_ || (dynamic_cast<ApproachingIntersection *>(scenario_.get()) == nullptr))
        {
            scenario_ = std::make_unique<ApproachingIntersection>(car_, globalPath, obses_limit_SD, GlobalcoordinatesystemObsesLimit, gpsA_, indexinglobalpath);
            scenario_->setlocalpath(optTrajxy);
            scenario_->setlocalpath(optTrajsd);
        }
        else
        {
            scenario_->Updated(car_, obses_limit_SD, GlobalcoordinatesystemObsesLimit, gpsA_, indexinglobalpath);
        }
        // 进行决策
        scenario_->MakeDecision();
        bool isTurnSuccessful = scenario_->Process();
        if (isTurnSuccessful)
        {
            // 获取路径
            optTrajxy = scenario_->getlocalpath();
            // 发送给控制端
            publish_localpath(optTrajxy);
            return true;
        }
        break;
    }
    case ScenarioState::NEAR_STOP:
    {
        // 创建 NearStop 类的智能指针
        if (!scenario_ || (dynamic_cast<NearStop *>(scenario_.get()) == nullptr))
        {
            scenario_ = std::make_unique<NearStop>(car_, globalPath, obses_limit_SD, GlobalcoordinatesystemObsesLimit, gpsA_, indexinglobalpath);
            scenario_->setlocalpath(optTrajxy);
            scenario_->setlocalpath(optTrajsd);
        }
        else
        {
            scenario_->Updated(car_, obses_limit_SD, GlobalcoordinatesystemObsesLimit, gpsA_, indexinglobalpath);
        }
        // 进行决策
        scenario_->MakeDecision();
        bool isNearStopSuccessful = scenario_->Process();
        if (isNearStopSuccessful)
        {
            // 获取路径
            optTrajxy = scenario_->getlocalpath();
            // 发送给控制端
            publish_localpath(optTrajxy); // TODO: 补充发送控制端的逻辑
            return true;
        }
        break;
    }
    default:
        break;
    }
    return false;
}
