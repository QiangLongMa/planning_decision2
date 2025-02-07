#include"piecewise_jerk_speed_optimizer.h"

PiecewiseJerkSpeedOptimizer::PiecewiseJerkSpeedOptimizer(){}

bool PiecewiseJerkSpeedOptimizer::Process(double init_v,double init_a,Eigen::MatrixXd &localpath,
                                        std::vector<SpeedPoint> &speed_data, double total_length,double total_time,
                                        std::vector<SpeedLimit> &speed_limit,  std::vector<SpeedDkappa> &curise_speed_,
                                        std::vector<SpeedPoint> &output_speed_data,YAML::Node &config){
    if (speed_data.empty() || localpath.cols() == 0){
        std::cout<<"Empty path data"<<std::endl;
        return false;
    }
    reference_speed_data = std::move(speed_data);
    std::array<double,3 > init_s = {0.0 , init_v, init_a};
    double delta_t = 0.1;
    double total_length_ = total_length;
    double total_time_ = total_time;
    int num_of_knots = static_cast<int>(total_time_ / delta_t) + 1;
    // std::cout<<"total_length_: "<<total_length_<<std::endl;
    // std::cout<<"speed_data: "<<reference_speed_data.back().s<<std::endl;


    // if(total_length_ != reference_speed_data.back().s){
    //     std::cout<<"oooooooooooo"<<std::endl;
    //     std::cout<<"different s length"<<std::endl;
    //     return false;
    // }
    //加载yaml
    loadyaml(config);
    // 创建QP速度规划 对象
    PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t, init_s);
    piecewise_jerk_problem.set_weight_ddx(acc_weight);//加速度权重
    piecewise_jerk_problem.set_weight_dddx(jerk_weigth);//加加速度的权重 

    piecewise_jerk_problem.set_x_bounds(0.0,total_length_);
    piecewise_jerk_problem.set_dx_bounds(0.0,init_v);
    piecewise_jerk_problem.set_ddx_bounds(max_deceleration,max_acceleration);//加速度的范围
    piecewise_jerk_problem.set_dddx_bound(longitudinal_jerk_lower_bound,longitudinal_jerk_upper_bound);//加加速度的范围
    // Update SpeedBoundary and ref_s
    std::vector<double> x_ref;
    std::vector<double> penalty_dx;
    std::vector<std::pair<double, double>> s_dot_bounds;
    std::vector<SpeedLimit> speed_limit_ =  std::move(speed_limit);
    std::vector<double> curise_speed(num_of_knots);
    std::vector<SpeedPoint> OptimizeFalseData(num_of_knots); //优化失败的时候需要的每个点的数据
    std::cout<<" init_v; "<<init_v<<" init_a: "<<init_a<<std::endl;
    std::vector<std::vector<double>> vec(localpath.cols(), std::vector<double>(localpath.rows()));
    for (size_t i = 0; i < localpath.cols(); ++i){
      std::copy(localpath.col(i).data(), localpath.col(i).data() + localpath.rows(), vec[i].begin());
    }
    for (int i = 0; i < num_of_knots; ++i){
        double curr_t = i * delta_t;
        // get path_s // get curvature    // get v_upper_bound
        SpeedPoint sp;
        EvaluateByTime(curr_t,reference_speed_data,sp);
        const double path_s = sp.s;
        EvaluateByS(vec, path_s, sp);
        OptimizeFalseData[i] = sp;
        x_ref.emplace_back(path_s);
        penalty_dx.push_back(sp.dkappa*kappa_penalty_weight);//；曲率的限制 
        const double v_lower_bound = 0.0;
        double v_upper_bound = GetSpeedLimitByS(speed_limit_,path_s);
        s_dot_bounds.emplace_back(v_lower_bound, std::fmax(v_upper_bound , 0.0));
        double speed_ = GetCuriseSpeedByS(curise_speed_, path_s);
        curise_speed.push_back(speed_);
        //std::cout<<"path_s: "<<path_s<<" "<<"v_upper_bound: "<<v_upper_bound<<" "<<"curise_speed: "<<speed_<<std::endl;
        // std::cout << "x: "<<sp.x<<" "
        //           << "y: "<<sp.y<<" "
        //           << "v: "<<sp.v<<" "
        //           << "theta: "<<sp.theta<<" "
        //           << "dkappa: "<<sp.dkappa<<" "
        //           << "s: "<<sp.s<<" "
        //           << "a: "<<sp.a<<" "
        //           << "t: "<<sp.t<<" "<<std::endl;
    }
    //std::cout<<std::endl;
    piecewise_jerk_problem.set_x_ref(ref_s_weight,std::move(x_ref));//s的权重 目标s
    piecewise_jerk_problem.set_dx_ref(ref_v_weight, curise_speed);//待修改10km/h 参考速度
    piecewise_jerk_problem.set_penalty_dx(penalty_dx);//曲率
    piecewise_jerk_problem.set_dx_bounds(std::move(s_dot_bounds));//速度的上下限
    // Solve the problem 
    if (!piecewise_jerk_problem.Optimize()) {
        std::cout<<"Piecewise jerk speed optimizer failed!"<<std::endl;
        // std::cout<<"OptimizeFalseData: "<<OptimizeFalseData.size()<<std::endl;
        // size_t localpathrows = localpath.rows();
        // size_t dataSize = OptimizeFalseData.size();
        // localpath.resize(localpathrows, dataSize);
        // localpath.setZero();
        // for (size_t i = 0; i < dataSize; ++i){
        //   localpath(0, i) = OptimizeFalseData[i].x;
        //   localpath(1, i) = OptimizeFalseData[i].y;
        //   localpath(2, i) = OptimizeFalseData[i].v;
        //   localpath(3, i) = OptimizeFalseData[i].theta;
        //   localpath(4, i) = OptimizeFalseData[i].dkappa;
        //   localpath(5, i) = OptimizeFalseData[i].s;
        //   localpath(6, i) = OptimizeFalseData[i].a;
        //   localpath(7, i) = OptimizeFalseData[i].t;
        //   //localpath(8, i) = OptimizeFalseData[i].Absolute_time;
        // }
        //std::cout<<"localpath: "<<localpath.transpose()<<std::endl;
        // std::cout<<"localpathcols: "<<localpath.cols()<<std::endl;
      auto comp1=[](const SpeedPoint & sp,const double & s){
      return sp.s < s;
      };
      for (size_t i = 0; i < vec.size(); ++i) {
        double path_s = vec[i][5];
        auto it_lower = std::lower_bound(OptimizeFalseData.begin(), OptimizeFalseData.end(), path_s, comp1);
        if (it_lower == OptimizeFalseData.begin()){
          localpath(2, i) = it_lower->v;// v
          localpath(6, i) = it_lower->a;
          localpath(7, i) = it_lower->t;
        } else if (it_lower == OptimizeFalseData.end()){
          auto p = it_lower - 1;
          localpath(2, i) = p->v;// v
          localpath(6, i) = p->a;
          localpath(7, i) = p->t;
        } else {
          auto p0 = it_lower - 1;
          auto p1 = it_lower;
          localpath(2, i) = InterpolateUsingLinearApproximation(p0->s, p1->s, path_s, p0->v, p1->v);// v
          localpath(6, i) = InterpolateUsingLinearApproximation(p0->s, p1->s, path_s, p0->a, p1->a);
          localpath(7, i) = InterpolateUsingLinearApproximation(p0->s, p1->s, path_s, p0->t, p1->t);
        }
      }
      return false;
    }
     // Extract output
    const std::vector<double>& s = piecewise_jerk_problem.opt_x();
    const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
    const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();
    // for (int i = 0; i < num_of_knots; ++i) {
    // std::cout << "For t[" << i * delta_t << "], s = " << s[i] << ", v = " << ds[i]
    //        << ", a = " << dds[i]<<std::endl;
    // }
    output_speed_data.resize(num_of_knots);
    SpeedPoint point;
    for (int i = 0; i < num_of_knots; ++i){
        if (ds[i] <= 0.0) {
            break;
        }
        point.s = s[i];
        point.t = delta_t*i;
        point.v = ds[i];
        point.a = dds[i];
        output_speed_data[i]= point;
    }
    // size_t localpathrows = localpath.rows();
    // size_t dataSize = OptimizeFalseData.size();
    // if (output_speed_data.size() != OptimizeFalseData.size()) return false;
    // localpath.resize(localpathrows, dataSize);
    // localpath.setZero();
    // for (size_t i = 0; i < dataSize; ++i){
    //   localpath(0, i) = OptimizeFalseData[i].x;
    //   localpath(1, i) = OptimizeFalseData[i].y;
    //   localpath(2, i) = output_speed_data[i].v;
    //   localpath(3, i) = OptimizeFalseData[i].theta;
    //   localpath(4, i) = OptimizeFalseData[i].dkappa;
    //   localpath(5, i) = output_speed_data[i].s;
    //   localpath(6, i) = output_speed_data[i].a;
    //   localpath(7, i) = output_speed_data[i].t;
    //   //localpath(8, i) = OptimizeFalseData[i].Absolute_time;
    //   // std::cout<< "x: "<<localpath(0, i)<<" "
    //   //             << "y: "<<localpath(1, i)<<" "
    //   //             << "v: "<<localpath(2, i)<<" "
    //   //             << "theta: "<<localpath(3, i)<<" "
    //   //             << "dkappa: "<<localpath(4, i)<<" "
    //   //             << "s: "<<localpath(5, i)<<" "
    //   //             << "a: "<<localpath(6, i)<<" "
    //   //             << "t: "<<localpath(7, i)<<" "<<std::endl;
    // }
  //   for (size_t i = 0; i < localpath.cols(); ++i){
  //     std::cout<< "x: "<<localpath(0, i)<<" "
  //                   << "y: "<<localpath(1, i)<<" "
  //                   << "v: "<<localpath(2, i)<<" "
  //                   << "theta: "<<localpath(3, i)<<" "
  //                   << "dkappa: "<<localpath(4, i)<<" "
  //                   << "s: "<<localpath(5, i)<<" "
  //                   << "a: "<<localpath(6, i)<<" "
  //                   << "t: "<<localpath(7, i)<<" "<<std::endl;
  // }
    auto comp=[](const SpeedPoint & sp,const double & s){
      return sp.s < s;
    };
    for (size_t i = 0; i < vec.size(); ++i) {
      double path_s = vec[i][5];
      auto it_lower = std::lower_bound(output_speed_data.begin(), output_speed_data.end(), path_s, comp);
      if (it_lower == output_speed_data.begin()){
        localpath(2, i) = it_lower->v;// v
        localpath(6, i) = it_lower->a;
        localpath(7, i) = it_lower->t;
      } else if (it_lower == output_speed_data.end()){
        auto p = it_lower - 1;
        localpath(2, i) = p->v;// v
        localpath(6, i) = p->a;
        localpath(7, i) = p->t;
      } else {
        auto p0 = it_lower - 1;
        auto p1 = it_lower;
        localpath(2, i) = InterpolateUsingLinearApproximation(p0->s, p1->s, path_s, p0->v, p1->v);// v
        localpath(6, i) = InterpolateUsingLinearApproximation(p0->s, p1->s, path_s, p0->a, p1->a);//a
        localpath(7, i) = InterpolateUsingLinearApproximation(p0->s, p1->s, path_s, p0->t, p1->t);//t
      }
    }

    // SpeedPoint *speedpoint;
    // for (size_t i = 0; i < localpathsize; ++i){
    //   double min_distance = std::numeric_limits<double>::max();
    //   for (auto & sp:output_speed_data){
    //     double distance = std::abs(sp.s - localpath(5, i));
    //     if (distance < min_distance){
    //         min_distance = distance;
    //         speedpoint = &sp;
    //     }
    //   }
    //   localpath(2,i) = speedpoint->v;// v
    //   localpath(6,i) = speedpoint->a;
    //   localpath(7,i) = speedpoint->t;

    //   if (localpath(5,i)>output_speed_data.back().s){
    //       localpath(2,i) = output_speed_data.back().v;// v
    //       localpath(6,i) =output_speed_data.back().a;
    //   } else {
    //     auto it_lower = std::lower_bound(output_speed_data.begin(),output_speed_data.end(),localpath(5,i),comp);
    //     localpath(2,i) = it_lower->v;// v
    //     localpath(6,i) = it_lower->a;
    //   }
    // }
    return true;
}
void PiecewiseJerkSpeedOptimizer::loadyaml(YAML::Node &config){
    //YAML::Node config = YAML::LoadFile(yamllocate);
  if(config["piecewise_speed"]){
      YAML::Node lange_no_change = config["piecewise_speed"];
      acc_weight = lange_no_change["acc_weight"].as<double>();
      jerk_weigth = lange_no_change["jerk_weigth"].as<double>();
      max_deceleration = lange_no_change["max_deceleration"].as<double>();
      max_acceleration = lange_no_change["max_acceleration"].as<double>();
      longitudinal_jerk_lower_bound = lange_no_change["longitudinal_jerk_lower_bound"].as<double>();
      longitudinal_jerk_upper_bound = lange_no_change["longitudinal_jerk_upper_bound"].as<double>();
      ref_v_weight = lange_no_change["ref_v_weight"].as<double>();
      kappa_penalty_weight = lange_no_change["kappa_penalty_weight"].as<double>();
      ref_s_weight = lange_no_change["ref_s_weight"].as<double>();
  }
}


void PiecewiseJerkSpeedOptimizer::binarysearch(const std::vector<SpeedPoint> &point ,int &mindex,double s){
  if(s>=point.back().s){
    mindex = point.size()-1;
    return ;
  }  
  int left = 0;
  int right = point.size()-1;
  while (left<=right){
    if(left==right){
      if (left > 0 && (std::abs(s -point[left].s) < std::abs(s -point[left-1].s))){
        mindex = left;
        break;
      }
      else{
        mindex= left-1;
        break;
      }
    }
    int mid =static_cast<int>(left +(right-left)/2);
    if (point[mid].s==s){
      mindex =mid;
      break;
    }
    else if (point[mid].s<s)
    {
       left = mid+1;
    }else{
      right = mid;       //nums[mid]>target,如果mid赋值right以后，（left=right）> target,所以下面的if判断，
                            //分别是nums[left]、nums[left-1]和target的比较
    }
  }
} 