#include"QP.h"

QP::QP(){}
QP::~QP(){}

void QP::CalculateHDMapBound(std::vector<Eigen::Vector4d> &frenet_path){

    //frenet_path.clear();
    // if(hdmap_bound_.size()!=0){
    //     hdmap_bound_.clear();
    // }
    hdmap_bound_.resize(frenet_path.size());
    for (size_t i = 0; i < frenet_path.size(); ++i){
        hdmap_bound_[i].first  =left_bound;
        hdmap_bound_[i].second =right_bound;
    }
}

/*****************************************************************************************
 * 
 * 路径规划的主流程 入口
 * 
 * ****************************************************************************************/

bool QP::Process(std::pair<std::array<double, 3>, std::array<double, 3>> inint_start_point,const std::vector<obses_sd> &obstacle,
                            std::vector<Eigen::Vector4d> &frenet_path,const Eigen::MatrixXd& globalPath,Eigen::MatrixXd &xy_list_qp,
                            bool changelaneflag,double leftbound,double rightbound){
    //inint_start_point S L
    /*default_task_config: {
    task_type: PIECEWISE_JERK_PATH_OPTIMIZER
    piecewise_jerk_path_optimizer_config {
        default_path_config {
        l_weight: 1.0
        dl_weight: 20.0
        ddl_weight: 1000.0
        dddl_weight: 50000.0
        }
        lane_change_path_config {
        l_weight: 1.0
        dl_weight: 5.0
        ddl_weight: 800.0
        dddl_weight: 30000.0
    }
  }
    }*/
    //std::array<double,5> w={0,std::fmax(inint_start_point.second[1] * inint_start_point.second[1],5.0),10,20000,0};//l l' l'' l'''
    std::array<double,5> w;
    if (changelaneflag) {//进行变道
        w = {1,std::fmax(inint_start_point.second[1] * inint_start_point.second[1],20.0),1000,50000,0};//l l' l'' l'''
    } else {
        w = {1,std::fmax(inint_start_point.second[1] * inint_start_point.second[1],5.0),8000,30000,0};//l l' l'' l'''
    }
    left_bound = leftbound;
    right_bound = rightbound;
    if (IsPointInRect(frenet_path, obstacle)) {
        std::cout<<"qp中路径点在障碍物中"<<std::endl;
        return false;
    }
    CalculateLLimit(frenet_path,obstacle);
    for(const auto & ll:l_limit){
        if(ll.first>ll.second){
            std::cout<<"first > second"<<std::endl;
            return false;
        }
    }
    for (size_t i = 0; i < frenet_path.size(); ++i){
        if (l_limit[i].first <=frenet_path[i](1) && l_limit[i].second >=frenet_path[i](1)) {
            continue;
        } else {
            std::cout<<"qp中路径点不在l的范围内"<<std::endl;
            l_limit[i].first = frenet_path[i](1) - 0.2;
            l_limit[i].second = frenet_path[i](1) + 0.2;
            //return false;
        }
    }
    
    // for(size_t i=0; i<l_limit.size();++i){
    //     std::cout<<"limit: "<<l_limit[i].first<<" "<<l_limit[i].second<<std::endl;
    // }
    // CalculateHDMapBound(frenet_path);//
    // MapStaticObstacleWithDecision(inint_start_point.first[0],inint_start_point.second[0],obstacle,frenet_path);
    // CalculateLLimit();// l_limit
    std::vector<double> opt_l;
    std::vector<double> opt_dl;
    std::vector<double> opt_ddl;
    std::array<double, 3> end_state = {0.0, 0.0, 0.0};
    end_state[0] = frenet_path[frenet_path.size()-1](1);
    //max_steer_angle 是车辆的最大转向角度
    //steer_ratio 是转向比，表示方向盘转动和车轮转动之间的比例关系
    //wheel_base 车辆的轴距
    //const double lat_acc_bound = std::tan(max_steer_angle)/steer_ratio/wheel_base;//横向加速度最大限制 暂定 
    const double lat_acc_bound = std::tan(35.0/180.0*M_PI)/wheel_base/16.5; //35/180*M_PI 车轮最大转角    
    std::vector<double> l_ref;//DP规划出来的参考l 
    std::vector<std::pair<double, double>> ddl_bounds;
    for (size_t i = 0; i < frenet_path.size(); ++i){
        double kappa = frenet_path[i](2);
        ddl_bounds.emplace_back(-100*lat_acc_bound - kappa, 100*lat_acc_bound - kappa);
        //ddl_bounds.emplace_back(-100*lat_acc_bound - kappa, 100*lat_acc_bound - kappa);
        //ddl_bounds.emplace_back(-10, 10);
        l_ref.push_back(frenet_path[i](1));
    }
    //std::cout<<"ddl_bounds: "<<-lat_acc_bound<<"-"<<lat_acc_bound<<std::endl;
    // std::cout<<"l_limit :"<<l_limit.size()<<std::endl;
    bool is_valid_path_reference=true;
    int max_iter = 4000;
    if(l_limit.size()==0||frenet_path.size()==0||globalPath.cols()==0){
        return false;
    }
    globalPath_ = std::move(globalPath); 
    bool res_opt = OptimizePath(inint_start_point.second,end_state,l_ref,frenet_path.size(),delta_s,
                                is_valid_path_reference,std::move(l_limit),std::move(ddl_bounds),w,max_iter,&opt_l, &opt_dl,&opt_ddl);
    if (res_opt) {
    //   for (size_t i = 0; i < frenet_path.size(); i += 4) {
    //     std::cout << "for s[" << static_cast<double>(i) * 0.5
    //            << "], l = " << opt_l[i] << ", dl = " << opt_dl[i]<<std::endl;
    //   }
    }
    else{
        return false;
    }
    // 得到离散的路标点,此时还缺少 (x,y)信息
    // 得到离散的路标点,此时还缺少 (x,y)信息
    if (res_opt){
        const auto &frenet_frame_path = ToPiecewiseJerkPath(std::move(opt_l), std::move(opt_dl), std::move(opt_ddl),delta_s, inint_start_point.first[0]);
        xy_list_qp.resize(3,frenet_frame_path.size());
        frenet_path.resize(frenet_frame_path.size());
        for (size_t i = 0; i < frenet_frame_path.size(); ++i){
            frenet_path[i](0) = frenet_frame_path[i].s;
            frenet_path[i](1) = frenet_frame_path[i].l;
            frenet_path[i](2) = frenet_frame_path[i].dl;
            frenet_path[i](3) = frenet_frame_path[i].ddl;
        }
        double path_l,path_dl,global_4,global_5;
        for (size_t i = 0; i < frenet_frame_path.size(); ++i){
            xy_list_qp(0,i) = frenet_frame_path[i].s;
            xy_list_qp(1,i) = frenet_frame_path[i].l;
            //曲率计算 公式推导 
            int col_index;
            int left =0;
            int right = globalPath_.cols()-1;
            while (left<=right) {
                if(left==right){
                if (left > 0 && (std::abs(frenet_frame_path[i].s -globalPath_(6,left)) < std::abs(frenet_frame_path[i].s -globalPath_(6,left-1))))
                    {
                        col_index = left;
                        break;
                    }
                    else
                    {
                        col_index= left-1;
                        break;
                    }
                }
                int mid =static_cast<int>(left +(right-left)/2);
                if (globalPath_(6,mid)==frenet_frame_path[i].s){
                    col_index =mid;
                    break;
                }
                else if (globalPath_(6,mid)<frenet_frame_path[i].s)
                {
                    left = mid+1;
                }else{
                    right = mid;       //nums[mid]>target,如果mid赋值right以后，（left=right）> target,所以下面的if判断，
                                        //分别是nums[left]、nums[left-1]和target的比较
                }
            }
            path_l = frenet_frame_path[i].l;
            path_dl = frenet_frame_path[i].dl;
            global_4 = globalPath_(4,col_index);
            global_5 = globalPath_(5,col_index);
            double thea = atan(path_dl/(1-global_4*path_l));//计算角度差值 
            double param = ((frenet_frame_path[i].ddl+(global_5*frenet_frame_path[i].l+global_4*
                            path_dl))*tan(thea))*cos(thea)*cos(thea);
            double ke = (param/(1-global_4*path_l)+global_4)*
                        cos(thea)/(1-global_4*path_l);
            xy_list_qp(2,i) = ke;
        }
        return true;
    }
    return true;
}

/***********************************************************************************
 * 
 * 借用QP，完成最优轨迹的求解
 * 输出：一串离散点 (s, l, l', l'')               
 * 
 ***********************************************************************************/
bool QP::OptimizePath(
    const std::array<double, 3>& init_state,
    const std::array<double, 3>& end_state,
    std::vector<double> path_reference_l_ref, const size_t path_reference_size,
    const double delta, const bool is_valid_path_reference, 
    const std::vector<std::pair<double, double>>& lat_boundaries,
    const std::vector<std::pair<double, double>>& ddl_bounds,
    const std::array<double, 5>& w, const int max_iter, std::vector<double>* x,
    std::vector<double>* dx, std::vector<double>* ddx){
    
    const size_t kNumKnots = lat_boundaries.size();//上下边界
    // std::cout<<"kNumKnots: "<<kNumKnots<<std::endl;

    PiecewiseJerkPathProblem piecewise_jerk_problem(kNumKnots, delta_s, init_state);
  // TODO(Hongyi): update end_state settings 终点权重 
    piecewise_jerk_problem.set_end_state_ref({1000.0, 0.0, 0.0}, end_state);

     // use path reference as a optimization cost function
     // dp 规划出来的线 
    if (is_valid_path_reference) {
        // for non-path-reference part
        // weight_x_ref is set to default value, where
        // l weight = weight_x_ + weight_x_ref_ = (1.0 + 0.0)
        std::vector<double> weight_x_ref_vec(kNumKnots, 0.0);
        // increase l weight for path reference part only
        //DP规划出来的参考线的权重 
        const double peak_value = 1000;//2000;//2000;//config_.piecewise_jerk_path_optimizer_config().path_reference_l_weight();
        const double peak_value_x = 0.5 * static_cast<double>(path_reference_size) * delta_s;

        for (size_t i = 0; i < path_reference_size; ++i) {
        // Gaussian weighting
        const double x = static_cast<double>(i) * delta_s;
        weight_x_ref_vec.at(i) = GaussianWeighting(x, peak_value, peak_value_x);
        }
        piecewise_jerk_problem.set_x_ref(std::move(weight_x_ref_vec),
                                     std::move(path_reference_l_ref));
    }
    piecewise_jerk_problem.set_weight_x(w[0]);
    piecewise_jerk_problem.set_weight_dx(w[1]);
    piecewise_jerk_problem.set_weight_ddx(w[2]);
    piecewise_jerk_problem.set_weight_dddx(w[3]);
    piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});

    auto start_time = std::chrono::system_clock::now();
    piecewise_jerk_problem.set_x_bounds(lat_boundaries);

    double FLAGS_lateral_derivative_bound_default=5;//5

    piecewise_jerk_problem.set_dx_bounds(-FLAGS_lateral_derivative_bound_default,
                                       FLAGS_lateral_derivative_bound_default);
    piecewise_jerk_problem.set_ddx_bounds(ddl_bounds);

    // Estimate lat_acc and jerk boundary from vehicle_params
    const double axis_distance = wheel_base;
    const double max_yaw_rate = max_steer_angle_rate/steer_ratio/2; //veh_param.max_steer_angle_rate() 最大偏航角速率

    // jerk_bound = max_yaw_rate / axis_distance / vehicle_speed

    const double jerk_bound = 100*EstimateJerkBoundary(3.6,
                                                 axis_distance, max_yaw_rate);
    //std::cout<<"jerk_bound: "<<jerk_bound<<std::endl;
    piecewise_jerk_problem.set_dddx_bound(jerk_bound);
    //求解！！！
    bool success = piecewise_jerk_problem.Optimize(max_iter);
    auto end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;
    std::cout<< "Path Optimizer used time: " << diff.count() * 1000 << " ms."<<std::endl;  

    if (!success) {
    std::cout<< "piecewise jerk path optimizer failed"<<std::endl;
    return false;
    } 
      // 做好输出 转接工作
    *x = piecewise_jerk_problem.opt_x();
    *dx = piecewise_jerk_problem.opt_dx();
    *ddx = piecewise_jerk_problem.opt_ddx();
    return true;
}

    //静态障碍物限制 l 
void QP::MapStaticObstacleWithDecision(double car_s,double car_l,const std::vector<obses_sd> &obstacle,std::vector<Eigen::Vector4d> &frenet_path){
    //  static_obstacle_bound_.clear();
    // if(static_obstacle_bound_.size()!=0){
    //     static_obstacle_bound_.clear();
    // }
    static_obstacle_bound_.resize(frenet_path.size(),std::make_pair(999999,-999999));//序列号 99999 -999999
    if (obstacle.size()==0){return;}
    for (size_t i = 0; i < obstacle.size(); ++i){
        //std::cout<<obstacle[i].centre_points.l<<" "<<std::abs(obstacle[i].centre_points.s-car_s)<<std::endl;
        if (std::abs(obstacle[i].centre_points.l-car_l)>=5||std::abs(obstacle[i].centre_points.s-car_s)>=30){continue;}

        //判断障碍物是在左边还是右边 
        double max_s = std::max(obstacle[i].point1.s,std::max(std::max(obstacle[i].point2.s,obstacle[i].point3.s),obstacle[i].point4.s));
        double min_s = std::min(obstacle[i].point1.s,std::min(std::min(obstacle[i].point2.s,obstacle[i].point3.s),obstacle[i].point4.s));
        double max_l = std::max(obstacle[i].point1.l,std::max(std::max(obstacle[i].point2.l,obstacle[i].point3.l),obstacle[i].point4.l));
        double min_l = std::min(obstacle[i].point1.l,std::min(std::min(obstacle[i].point2.l,obstacle[i].point3.l),obstacle[i].point4.l));
        if (obstacle[i].centre_points.l>=0){//车子在左边 右边绕行 
            for (double j = min_s; j <= max_s; j+=delta_s){
                // std::cout<<"obstacle[i].point1.s: "<<obstacle[i].point1.s<<" obstacle[i].point2.s: "<<obstacle[i].point2.s<<std::endl;
                double ld = min_l-car_wigth/2;//right边界
                int Serial_number = static_cast<int>((j-car_s)/delta_s);
                if(Serial_number<0||Serial_number>frenet_path.size()){continue;}//判断有没有超过边界 或者 在车子后面
                static_obstacle_bound_[Serial_number].first=std::min(ld,static_obstacle_bound_[Serial_number].first);
                static_obstacle_bound_[Serial_number].second=std::max(right_bound,static_obstacle_bound_[Serial_number].second);
            }
        }
        else if (obstacle[i].centre_points.l<0){//车子在右边  左边绕行 
            for (double j = min_s; j <= max_s; j+=delta_s){
                // std::cout<<"obstacle[i].point4.s: "<<obstacle[i].point4.s<<" obstacle[i].point3.s: "<< obstacle[i].point3.s<<std::endl;
                int Serial_number = static_cast<int>((j-car_s)/delta_s);
                double ld = max_l+car_wigth/2;//left边界
                if(Serial_number<0||Serial_number>frenet_path.size()){continue;}//判断有没有超过边界 或者 在车子后面
                //左边最小 右边最大 
                static_obstacle_bound_[Serial_number].first = std::min(left_bound,static_obstacle_bound_[Serial_number].first);
                static_obstacle_bound_[Serial_number].second = std::max(ld, static_obstacle_bound_[Serial_number].second);
            }
        }
    }
}

//l的限制 
void QP::CalculateLLimit(){
   // l_limit.clear();
    //    if(l_limit.size()!=0){
    //         l_limit.clear();
    //     }
    for (size_t i = 0; i < static_obstacle_bound_.size(); ++i){
        double left_min = std::min(static_obstacle_bound_[i].first,left_bound);
        double right_max = std::max(static_obstacle_bound_[i].second,right_bound);
        l_limit.emplace_back(std::make_pair(right_max,left_min)); //左边正 右边负 
    }
    // for (size_t i = 0; i < l_limit.size(); ++i)
    // {
    //     std::cout<<l_limit[i].first<<" "<<l_limit[i].second<<std::endl;
    // }
}

void QP::CalculateLLimit(std::vector<Eigen::Vector4d> &frenet_path,const std::vector<obses_sd> &obstacle){
    const constexpr double lateral_threshold = 5.0;
    const constexpr double longitudinal_threshold = 10.0;
    const constexpr double relax_margin = 0.2;
    l_limit.resize(frenet_path.size(),std::make_pair(right_bound,left_bound)); //先设定qp区间就是无障碍物时的区间 再一一和障碍物进行比较 这个是按照小-大
    if(obstacle.size()==0){//没有障碍物 
        return;
    }
    // for(const auto &obs:obstacle){
    //     std::cout<<"s1: "<<obs.point1.s<<" l1: "<<obs.point1.l<<" ";
    //     std::cout<<"s2: "<<obs.point2.s<<" l1: "<<obs.point2.l<<" ";
    //     std::cout<<"s3: "<<obs.point3.s<<" l1: "<<obs.point3.l<<" ";
    //     std::cout<<"s4: "<<obs.point4.s<<" l1: "<<obs.point4.l<<" "<<std::endl;
    // }
    // for(const auto &path:frenet_path){
    //     std::cout<<"local_path s: "<<path(0)<<" l: "<<path(1)<<std::endl;
    // }
    double path_s,path_l;
    for(size_t i = 0; i<frenet_path.size(); ++i){
        path_s = frenet_path[i](0);
        path_l = frenet_path[i](1);
        //横向距离太远以及纵向距离太远了都不考虑
        for(size_t j =0;j<obstacle.size();++j){
            double max_s = std::max({obstacle[j].point1.s,obstacle[j].point2.s,obstacle[j].point3.s,obstacle[j].point4.s}) + 1;//
            double min_s = std::min({obstacle[j].point1.s,obstacle[j].point2.s,obstacle[j].point3.s,obstacle[j].point4.s}) - 1;
            double max_l = std::max({obstacle[j].point1.l,obstacle[j].point2.l,obstacle[j].point3.l,obstacle[j].point4.l});
            double min_l = std::min({obstacle[j].point1.l,obstacle[j].point2.l,obstacle[j].point3.l,obstacle[j].point4.l});
            if(path_s > max_s || path_s < min_s){//大于最大s 小于最小s
                continue;
            }else{ //在最小s 和最大s之间 并且l也符合距离 
                //障碍物在左边 
                double comparisons;
                if(obstacle[j].centre_points.l>path_l && min_l > path_l){
                    //comparisons = min_l<0?min_l+car_wigth/2+relax_margin:min_l-car_wigth/2-relax_margin;
                    l_limit[i].second = std::min(min_l - car_wigth / 2, l_limit[i].second);//relax_margin为松弛度 
                    if ( l_limit[i].second <  path_l) {
                        l_limit[i].second  = path_l + relax_margin;
                    }
                } else if (obstacle[j].centre_points.l < path_l && max_l < path_l){//障碍物在右边 
                    l_limit[i].first =  std::max(max_l + car_wigth / 2, l_limit[i].first);
                    if (l_limit[i].first >  path_l) {
                        l_limit[i].first = path_l - relax_margin;
                    }
                } else {//未知处理 
                    continue;
                }
            }
            
        }
    }

}

double QP::GaussianWeighting(
    const double x, const double peak_weighting,
    const double peak_weighting_x) {
  // 方差
  double std = 1 / (std::sqrt(2 * M_PI) * peak_weighting);
  // 均值
  double u = peak_weighting_x * std;
  // 自变量
  double x_updated = x * std;
  //正态分布函数  Gussian(均值、方差、自变量)
  return Gaussian(u, std, x_updated);
}

// Gaussian
// 正态分布函数 Gussian(均值、方差、自变量)
inline double QP::Gaussian(const double u, const double std, const double x) {
  return (1.0 / std::sqrt(2 * M_PI * std * std)) *
         std::exp(-(x - u) * (x - u) / (2 * std * std));
}

inline double QP::EstimateJerkBoundary(
    const double vehicle_speed, const double axis_distance,
    const double max_yaw_rate) {
  return max_yaw_rate / axis_distance / vehicle_speed;
}

/*****************************************************************************************
* 这里使用jerk恒定曲线（即3次多项式），依据Δs、l、l'、l''，resolution插值得到离散的路标点
*****************************************************************************************/

std::vector<FrenetFramePoint> QP::ToPiecewiseJerkPath(
    const std::vector<double>& x, const std::vector<double>& dx,
    const std::vector<double>& ddx, const double delta_s,
    const double start_s){
    PiecewiseJerkTrajectory1d piecewise_jerk_traj(x.front(), dx.front(),
                                               ddx.front());
    // 从1开始，这一可以得到（n-1）个segment
    for (size_t i = 1; i < x.size(); ++i)
    {
        const auto dddl = (ddx[i]-ddx[i-1])/delta_s;
        piecewise_jerk_traj.AppendSegment(dddl, delta_s);
    }

    std::vector<FrenetFramePoint> frenet_frame_path;
    double accumulated_s = 0.0;
    int SIZE_=0;
    while (accumulated_s <= piecewise_jerk_traj.ParamLength()){
        double l = piecewise_jerk_traj.Evaluate(0, accumulated_s);
        double dl = piecewise_jerk_traj.Evaluate(1, accumulated_s);
        double ddl = piecewise_jerk_traj.Evaluate(2, accumulated_s);
        FrenetFramePoint frenet_frame_point;
        frenet_frame_point.l = l;// x[SIZE_];//l;
        frenet_frame_point.dl = dl;//dx[SIZE_];;
        frenet_frame_point.ddl = ddl;//ddx[SIZE_];;
        SIZE_+= 1;
        frenet_frame_point.s = accumulated_s + start_s; //实际的S 
        frenet_frame_path.push_back(std::move(frenet_frame_point));
        accumulated_s += delta_s;
    }
    return std::move(frenet_frame_path);
}

bool QP::IsPointInRect(std::vector<Eigen::Vector4d> &frenet_path, const std::vector<obses_sd> &obstacle) {
    if (obstacle.empty()) return false;

    for (const auto &p : frenet_path) {
        double path_s = p(0);
        double path_l = p(1);

        for (const auto &obs : obstacle) {
            Eigen::Vector2d vec1_p1(path_s - obs.point1.s, path_l - obs.point1.l);
            Eigen::Vector2d vec1_p2(path_s - obs.point2.s, path_l - obs.point2.l);
            Eigen::Vector2d vec1_p3(path_s - obs.point3.s, path_l - obs.point3.l);
            Eigen::Vector2d vec1_p4(path_s - obs.point4.s, path_l - obs.point4.l);

            Eigen::Vector2d vec12(obs.point2.s - obs.point1.s, obs.point2.l - obs.point1.l);
            Eigen::Vector2d vec23(obs.point3.s - obs.point2.s, obs.point3.l - obs.point2.l);
            Eigen::Vector2d vec34(obs.point4.s - obs.point3.s, obs.point4.l - obs.point3.l);
            Eigen::Vector2d vec41(obs.point1.s - obs.point4.s, obs.point1.l - obs.point4.l);

            if (PointInRect(vec12, vec23, vec34, vec41, vec1_p1, vec1_p2, vec1_p3, vec1_p4)) {
                return true;
            }
        }
    }
    return false;
}



//判断点是否在多边形的内部 ，如果在内部的 叉乘应该全小于0 
bool QP::PointInRect(Eigen::Vector2d &vec12, Eigen::Vector2d &vec23, Eigen::Vector2d &vec34, Eigen::Vector2d &vec41
                ,Eigen::Vector2d &vec1_p1, Eigen::Vector2d &vec1_p2, Eigen::Vector2d &vec1_p3, Eigen::Vector2d &vec1_p4){
  // p1 
  double o1 = vec12.x() * vec1_p1.y() - vec12.y() * vec1_p1.x();
  double o2 = vec23.x() * vec1_p2.y() - vec23.y() * vec1_p2.x();
  double o3 = vec34.x() * vec1_p3.y() - vec34.y() * vec1_p3.x();
  double o4 = vec41.x() * vec1_p4.y() - vec41.y() * vec1_p4.x();
  if(o1 < 0 && o2 < 0 && o3 < 0 && o4 < 0){
    return true;
  } else {
    return false;
  }
}

/*DEFINE_double(speed_lower_bound, -0.1, "The lowest speed allowed.");
DEFINE_double(speed_upper_bound, 40.0, "The highest speed allowed.");

DEFINE_double(longitudinal_acceleration_lower_bound, -4.5,
              "The lowest longitudinal acceleration allowed.");
DEFINE_double(longitudinal_acceleration_upper_bound, 4.0,
              "The highest longitudinal acceleration allowed.");
DEFINE_double(lateral_acceleration_bound, 4.0,
              "Bound of lateral acceleration; symmetric for left and right");

DEFINE_double(longitudinal_jerk_lower_bound, -4.0,
              "The lower bound of longitudinal jerk.");
DEFINE_double(longitudinal_jerk_upper_bound, 4.0,
              "The upper bound of longitudinal jerk.");
DEFINE_double(lateral_jerk_bound, 4.0,
              "Bound of lateral jerk; symmetric for left and right");

DEFINE_double(dl_bound, 0.10,
              "The bound for derivative l in s-l coordinate system.");
DEFINE_double(kappa_bound, 0.20, "The bound for trajectory curvature");
DEFINE_double(dkappa_bound, 0.02,
              "The bound for trajectory curvature change rate");*/
/* "/apollo/modules/planning/conf/planning_config.pb.txt",*/

