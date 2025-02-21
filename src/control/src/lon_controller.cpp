#include "lon_controller.h"

loncontroller::loncontroller(/* args */)
{
}

loncontroller::~loncontroller()
{
}
double loncontroller::Time(){
  auto now = std::chrono::high_resolution_clock::now();
  auto nano_time_point = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
  auto epoch = nano_time_point.time_since_epoch();
  uint64_t now_nano = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
  return static_cast<double> (now_nano) / NANOS_PER_SECOND;
}

void loncontroller::QueryNearestPointByAbsoluteTime(double t, const Eigen::MatrixXd &localpath, 
                                                    std::array<double, 7> &point){
  double duration_t = t- header_time_;
  //std::cout<<"duration_t: "<<duration_t<<std::endl;

  size_t localpath_size = localpath.cols();
  double min_t = std::numeric_limits<double>::max();
  size_t index;
  for (size_t i = 0; i < localpath_size; ++i){
    double disntcne = std::abs(duration_t - localpath(7, i));
    if(disntcne < min_t){
      min_t = disntcne;
      index = i;
    }
  }
  //localpath x y v theta dk s a t 
  //point x y theta kappa s v a
  point.at(0) = localpath(0, index);
  point.at(1) = localpath(1, index);
  point.at(2) = localpath(3, index);
  point.at(3) = localpath(4, index);
  point.at(4) = localpath(5, index);
  point.at(5) = localpath(2, index);
  point.at(6) = localpath(6, index);

}

void loncontroller::Init(const size_t closeindex, std::tuple<double, double, double, double, double> vehicle_state,
                        double heading_time){
    /**
     * in control_conf.pb.txt, Line133-139
     * station_pid_conf {
     *   integrator_enable: false
     *   integrator_saturation_level: 0.3
     *   kp: 0.2
     *   ki: 0.0
     *   kd: 0.0
     * }
    */
   // 高速speed 3.0 m/s 
   station_pid_controller_.Init(0.2, 0.0, 0.0, false, 0.3);
   speed_pid_controller_.Init(0.5, 0.3, 0.0, true, 0.3);
   closeindex_ = closeindex;
   vehicle_state_ = vehicle_state;// x y theta v  a 
   std::get<2>(vehicle_state_) = NormalizeAngle(std::get<2>(vehicle_state_) / 180 * M_PI);
   header_time_ = heading_time;
}

bool loncontroller::ComputeControlCommand(const Eigen::MatrixXd &localpath, double &acceleration_cmd){
  if(localpath.cols() == 0){
      return false;
  }
  double preview_time = 10 * ts;
  //调用计算纵向误差函数，输入参数：
  ComputeLongitudinalErrors(localpath, preview_time, ts);
  //定义临时变量station_error_limit纵向位置误差限制
  double station_error_limit = 2.0;
  //定义临时变量station_error_limited为限幅后的纵向位置误差
  double station_error_limited = 0.0;
  if(FLAGS_enable_speed_station_preview){
    //基本概念 预览点：当前时间加上预览时间在轨迹上对应的点，车辆将要到达的纵向位置
    //基本概念 参考点：当前时间在轨迹上对应的点，车辆此刻应该到达的纵向位置
    //基本概念 匹配点：当前时间在轨迹上对应的点，车辆此刻应该到达的纵向位置
    //有两种位置误差
    //第一种, preview_station_error=预览点纵向位置 - 匹配点纵向位置
    //第二种, station_error=参考点纵向位置-匹配点纵向位置
    //std::cout<<"preview_station_error: "<<preview_station_error<<std::endl;
    station_error_limited =  Clamp(preview_station_error, -station_error_limit, station_error_limit);
  } else {
    station_error_limited =  Clamp(station_error, -station_error_limit, station_error_limit);
  }
  //非R档情况下，定义临时变量speed_offset速度偏差
	//速度偏差=位置控制器根据(限幅后位置误差，采样周期)计算出控制量即速度
  double speed_offset = station_pid_controller_.Control(station_error_limited, ts);
  //std::cout<<"speed_offset: "<<speed_offset<<std::endl;
  //定义一个临时变量速度控制器的输入speed_controller_input为0
  double speed_controller_input = 0.0;
  double speed_controller_input_limit = 1.0;//最大车辆速度偏差 
  double speed_controller_input_limited = 0.0;
  if(FLAGS_enable_speed_station_preview){
    //打开的话速度控制器的输入 = 位置控制器计算出的speed_offset + 当前时间向前加上预览时间在轨迹上的对应点的速度和当前车速的偏差
    speed_controller_input = speed_offset + preview_speed_error;
  } else {
    speed_controller_input = speed_offset + speed_error;
  }
  //std::cout<<"preview_speed_error: "<<preview_speed_error<<std::endl;
  //计算得到的速度控制器的输入再进行限幅
  speed_controller_input_limited = Clamp(speed_controller_input, -speed_controller_input_limit, speed_controller_input_limit);
	//定义临时变量acceleration_cmd_closeloop闭环加速度指令初始化为0
  double acceleration_cmd_closeloop = 0.0;
  acceleration_cmd_closeloop =
      speed_pid_controller_.Control(speed_controller_input_limited, ts);
  //std::cout<<"acceleration_cmd_closeloop: "<<acceleration_cmd_closeloop<<std::endl;
  //定义1个临时变量acceleration_cmd
	//总的加速度指令 = 闭环加速度指令 + 预览参考加速度 + 坡道补偿加速度(如果打开坡道补偿的话)
  //std::cout<<"preview_acceleration_reference: "<<preview_acceleration_reference<<std::endl;
  acceleration_cmd =  acceleration_cmd_closeloop + preview_acceleration_reference;

}
//该函数是计算纵向误差的
//输入参数规划发布轨迹信息TrajectoryAnalyzer类指针对象trajectory_analyzer
//输入参数preview_time,预览时间，在控制配置文件里面设置
//输入参数ts，采样时间，在控制配置文件里设置
//输入参数debug，SimpleLongitudinalDebug类指针对象debug用来存放计算得到纵向误差信息
//其他函数通过debug指针进行访问这些纵向误差
void loncontroller::ComputeLongitudinalErrors(const Eigen::MatrixXd &localpath,
                    const double preview_time, const double ts){
    // 分解车辆运动到Frenet坐标，就是分为纵向和横向
  // s: 纵向累积走过的距离沿着参考轨迹
  // s_dot: 纵向沿着参考轨迹的速度
  // d: 相对参考轨迹的横向距离
  // d_dot: 横向距离的变化率
  //matched:匹配点，在参考轨迹上距离当前车辆距离最近的点
  //初始化匹配点处的s,s',d,d'
  double s_matched = 0.0;
  double s_dot_matched = 0.0;
  double d_matched = 0.0;
  double d_dot_matched = 0.0;
  //匹配点为将车辆当前状态的x,y坐标代入去查找轨迹上的最近点
  //x y theta kappa s
  std::array<double ,5> matched_point;
  // for (size_t i = 0; i < localpath.cols(); ++i){
  //   std::cout<<"t: "<<localpath(7, i)<<" "<<"a: "<<localpath(6, i)<<" "<<localpath(2, i)<<std::endl;
    
  // }
  
  matched_point[0] = localpath(0, closeindex_);
  matched_point[1] = localpath(1, closeindex_);
  matched_point[2] = localpath(3, closeindex_);
  matched_point[3] = localpath(4, closeindex_);
  matched_point[4] = localpath(5, closeindex_);
  //轨迹信息将当前点x,y,theta,v以及参考点信息输入，输出当前点的s,d,s',d'
  //简而言之就是将大地坐标系转化为Frenet坐标
  //d是横向偏差，s是累积的弧长即纵向上走过的距离
  //函数参数最后几个都带&，熟悉的套路，引用变量传值，最后带&的几个变量都是待填充函数结果的变量
  ToTrajectoryFrame(std::get<0>(vehicle_state_), std::get<1>(vehicle_state_),
                    std::get<2>(vehicle_state_), std::get<3>(vehicle_state_),
                    matched_point, &s_matched, &s_dot_matched, &d_matched, &d_dot_matched);
  //定义临时变量当前控制时间=当前时间
  //定义临时变量预览控制时间=当前时间+预览时间
  //double preview_control_time = preview_time ;
  //参考点就是用当前时间去轨迹上查时间最近点
  double current_control_time = Time();
  double preview_control_time = current_control_time + preview_time;


  //x y theta kappa s v a 
  std::array<double ,7> reference_point;// 得到的是参考路径上的 信息 
  QueryNearestPointByAbsoluteTime(current_control_time, localpath, reference_point);
  //预览点就是去轨迹上查预览时间对应的点，就是当前时间向前看一段时间对应轨迹上的点
  std::array<double ,7> preview_point;
  QueryNearestPointByAbsoluteTime(preview_control_time, localpath, preview_point);
  //航向角误差 = 车辆当前状态航向角 - 匹配点的航向角
  //NormalizeAngle角度的规范化，就是将所有角度规范到-pi,pi
  double heading_error = NormalizeAngle(std::get<2>(vehicle_state_) - matched_point.at(2));
  //std::cout<<"vehicle_state_theta: "<<std::get<2>(vehicle_state_)<<std::endl;
  //std::cout<<"matched_point_theta: "<< matched_point.at(2)<<std::endl;
  //纵向速度 = 车辆速度 * cos(当前航向角 - 轨迹上距离最近点航向角)
  double lon_speed = std::get<3>(vehicle_state_) * std::cos(heading_error);
  //纵向加速度 = 车辆加速度 * cos(当前航向角 - 轨迹上距离最近点航向角)
  double lon_acceleration = std::get<4>(vehicle_state_) * std::cos(heading_error);
  //1-kd就是将大地坐标系转化到Frenet坐标纵向上引入的
  double one_minus_kappa_lat_error = 1 -  matched_point.at(3) * std::get<3>(vehicle_state_) * std::sin(heading_error);

  //std::cout<<"preview_point.at(4): "<<preview_point.at(4)<<std::endl;
  //std::cout<<"s_matched: "<<s_matched<<std::endl;

  preview_station_error = preview_point.at(4) - s_matched;
  preview_acceleration_reference =  preview_point.at(6);
  preview_speed_error = preview_point.at(5) - s_dot_matched;

  //std::cout<<"preview_point.at(5): "<<preview_point.at(5)<<std::endl;
  //std::cout<<"s_dot_matched: "<<s_dot_matched<<std::endl;
  //std::cout<<"lon_speed: "<<lon_speed<<std::endl;
  station_error = reference_point.at(4) - s_matched;
  speed_error = reference_point.at(5) - s_dot_matched;

}

void loncontroller::ToTrajectoryFrame(const double x, const double y,
                                      const double theta, const double v,
                                      std::array<double ,5> &ref_point,
                                      double *ptr_s, double *ptr_s_dot,double *ptr_d,
                                      double *ptr_d_dot){
    double dx = x - ref_point[0];
    double dy = y - ref_point[1];

    double cos_ref_theta = std::cos(ref_point[2]);
    double sin_ref_theta = std::sin(ref_point[2]);

    double cross_rd_nd = cos_ref_theta * dy - sin_ref_theta * dx;
    *ptr_d = cross_rd_nd; //距离匹配点的横向距离

    double dot_rd_nd = dx * cos_ref_theta + dy * sin_ref_theta;
    //std::cout<<"ref_point.at(4): "<<ref_point.at(4)<<std::endl;
    //std::cout<<"dot_rd_nd: "<<dot_rd_nd<<std::endl;
    *ptr_s = ref_point.at(4) + dot_rd_nd; //车辆在frenet坐标系下的s值 

    double delta_theta = theta - ref_point.at(2);
    double cos_delta_theta = std::cos(delta_theta);
    double sin_delta_theta = std::sin(delta_theta);

    *ptr_d_dot = v * sin_delta_theta;
    //x y theta kappa s
    double one_minus_kappa_r_d = 1 - ref_point.at(3) * (*ptr_d);
    if(one_minus_kappa_r_d <= 0.0){
        one_minus_kappa_r_d =0.01;
    }

    *ptr_s_dot = v * cos_delta_theta / one_minus_kappa_r_d;

}

double loncontroller::NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

double loncontroller::Clamp(const double value, double bound1, double bound2){
  if(value < bound1){
    return bound1;
  } else if (value > bound2){
    return bound2;
  }
  return value;
}

void loncontroller::Reset(){
  station_pid_controller_.Reset();
  speed_pid_controller_.Reset();
}
