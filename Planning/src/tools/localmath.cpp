#include"localmath.h"
double NormalizeAngle(double angle) {
		angle = std::fmod(angle, 2 * M_PI);
		if (angle > M_PI)
			angle -= 2 * M_PI;
		else if (angle < -M_PI)
			angle += 2 * M_PI;
		return angle;
    }

double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t) {
  if (std::abs(t1 - t0) <= 1e-10) {
    return NormalizeAngle(a0);
  }
  const double a0_n = NormalizeAngle(a0);
  const double a1_n = NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const double r = (t - t0) / (t1 - t0);
  const double a = a0_n + d * r;
  return NormalizeAngle(a);
}

double lerp(const double &x0, const double t0, const double &x1, const double t1,
       const double t){
  if (std::abs(t1 - t0) <= 1.0e-6) {
    std::cout << "input time difference is too small"<<std::endl;;
    return x0;
  }
  const double r = (t - t0) / (t1 - t0);
  const double x = x0 + r * (x1 - x0);
  return x;     
}

double InterpolateUsingLinearApproximation(double s0,double s1,double s,double dkappa0,double dkappa1){
  double weight = (s - s0) / (s1 - s0);
  double dkappa = (1-weight)*dkappa0 + weight*dkappa1;
  return dkappa;
}

double lerp_v(double s0 ,double s1, double s, double v0, double v1){
  double weigth = (s - s0) / (s1 - s0);
  double v = (1- weigth)*v0 + weigth*v1;
  return v;
}


bool EvaluateByTime(const double t, std::vector<SpeedPoint> &speed_data, 
    SpeedPoint &speed_point){
  if(speed_data.size()<2){
    return false;
  }
  //确保时间是正确的
  // if(!(speed_data.front().t< t + 1e-6 && t - 1e6 < speed_data.back().t)){
  //   return false;
  // }
  //在speed_data容器中查找第一个不小于t的值 
  auto comp = [](const SpeedPoint& sp, const double t) {
    return sp.t < t;
  };
  auto it_lower = std::lower_bound(speed_data.begin(), speed_data.end(), t, comp);//自定义的比较方式
  if (it_lower == speed_data.end()) {
    speed_point = speed_data.back();
  } else if (it_lower == speed_data.begin()) {
    speed_point = speed_data.front();
  }
  else{
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    double t0 = p0.t;
    double t1 = p1.t;
    // 使用t，在p0和p1之间线性插值
    speed_point.s = lerp(p0.s,t0,p1.s,t1,t);
    speed_point.t = t;
    speed_point.v = lerp(p0.v,t0,p1.v,t1,t);
    speed_point.a = lerp(p0.a,t0,p1.a,t1,t);



  }
  return true;
}

void EvaluateByS(std::vector<std::vector<double>> &vec, const double s , SpeedPoint &speed_point){
  auto comp = [](const std::vector<double>& sp, const double s) {
    return sp[5] < s;
  };

  auto it_lower = std::lower_bound(vec.begin(), vec.end(), s, comp);//自定义的比较方式
  if (it_lower == vec.begin()) {
    speed_point.x = (*it_lower)[0];
    speed_point.y = (*it_lower)[1];
    speed_point.theta = NormalizeAngle((*it_lower)[3]);
    speed_point.dkappa = (*it_lower)[4];
  } else if (it_lower == vec.end()) {
    auto it = it_lower - 1 ;
    speed_point.x = (*it)[0];
    speed_point.y = (*it)[1];
    speed_point.theta = NormalizeAngle((*it)[3]);
    speed_point.dkappa = (*it)[4];
  } else {
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    speed_point.dkappa = InterpolateUsingLinearApproximation(p0[5], p1[5], s, p0[4], p1[4]);
    speed_point.x = InterpolateUsingLinearApproximation(p0[5], p1[5], s, p0[0], p1[0]);
    speed_point.y = InterpolateUsingLinearApproximation(p0[5], p1[5], s, p0[1], p1[1]);
    speed_point.theta = slerp(p0[3], s, p1[3], p0[5], p1[5]);
  }
}

//获取限速 
double GetSpeedLimitByS(std::vector<SpeedLimit> &speedlimit,double s){
  double v;
  auto compar_s= [](const SpeedLimit &speedlimit, const double &s){
    return speedlimit.s < s;
  };
  auto it_lower = std::lower_bound(speedlimit.begin(),speedlimit.end(),s,compar_s);
  if (it_lower == speedlimit.begin()) {
    v = it_lower->v;
    return v;
  }
  if (it_lower == speedlimit.end()){
    v = (it_lower - 1)->v;
    return v;
  } else {
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    v = InterpolateUsingLinearApproximation(p0.s, p1.s, s , p0.v , p1.v);
    return v;
  }
 
}
//获取当前的目标车速
double GetCuriseSpeedByS( std::vector<SpeedDkappa> &curise_speed,double s){
  double v;
  auto compar_s = [](const SpeedDkappa &curise_speed, const double &s){
    return curise_speed.s < s;
  };
  auto it_lower = std::lower_bound(curise_speed.begin(),curise_speed.end(),s,compar_s);
  if (it_lower == curise_speed.begin()) {
    v = it_lower->v;
    return v;
  }
  if (it_lower == curise_speed.end()){
    v = (it_lower - 1)->v;
    return v;
  } else {
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    v = InterpolateUsingLinearApproximation(p0.s, p1.s, s , p0.v , p1.v);
    return v;
  }
}


void GetsldlBys(std::vector<Eigen::Vector4d> &localpath, double path_s, double &start_point_s,
      double &start_point_l, double &start_point_dl, double &start_point_ddl){
  auto comp = [](const Eigen::Vector4d &Trajsd, const double &s){
        return Trajsd(0) < s;
    };
    auto it_lower = std::lower_bound(localpath.begin(), localpath.end(), path_s, comp);
   
    if(it_lower == localpath.begin()){
        start_point_s = path_s;
        start_point_l = (*it_lower)(1);
        start_point_dl =(*it_lower)(2);
    } else if (it_lower == localpath.end()) {
        auto &p = *(it_lower -1);
        start_point_s = path_s;
        start_point_l = p(1);
        start_point_dl = p(2);
    } else {
      auto &p1 = *(it_lower -1);
      auto &p2 = *(it_lower);
      start_point_l = InterpolateUsingLinearApproximation(p1(0),p2(0),path_s,p1(1),p2(1));
      start_point_dl = InterpolateUsingLinearApproximation(p1(0),p2(0),path_s,p1(2),p2(2));
      start_point_ddl = InterpolateUsingLinearApproximation(p1(0),p2(0),path_s,p1(3),p2(3));
      start_point_s = path_s;
    }
}

//计算L的二阶导数
double Caldll(double l, double dl, double ref_dkappa, double ref_kappa, double delta, double ptr_kappa){
  double a = -1 * (ref_dkappa * l + ref_kappa * dl) * std::tan(delta);
  double b = (1 - ref_kappa * l) / std::cos(delta) / std::cos(delta);
  double c = (ptr_kappa * (1 - ref_kappa * l)) / std::cos(delta) - ref_kappa;
  return a + b * c;

}



