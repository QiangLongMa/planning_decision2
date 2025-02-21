#include"local_dp_qp.h"
#include"toolKits.h"
#include<time.h>

//设置车辆位置和障碍物信息

local_dp_qp::local_dp_qp(){
  loadyaml();
}


void local_dp_qp::setPatam(double car_a,double car_v,double car_s,double car_l,double dl,double dll,const Eigen::MatrixXd& globalPath,int all_distances ,
                          int one_distances,int car_index,std::vector<obses_sd> obses_limit_SD, 
                          std::vector<Eigen::VectorXd> GlobalcoordinatesystemObsesLimit,double start_l,double end_l,
                          double delta_l,double target_v,double traget_l,Decisionflags Decisionflags_,double safetydistance_,
                          bool firstrunflag, bool ReducedDistanceFlag_,  double Reducedistanceeachtime_,  size_t Reducedistancesize_ ){
    //车辆位置 
    vehicle_position.s = car_s;
    vehicle_position.l = car_l;
    CAR_V = car_v;
    CAR_A = car_a;
    first_dl = dl;
    first_dll = dll;
    Start_l = start_l;
    End_l = end_l;
    Delta_l = delta_l;
    Target_v = target_v;
    Target_l = traget_l;
    ObstacleAvoidanceFlag_ = Decisionflags_.ObstacleAvoidanceFlag;//避障标志
    DecelerateFlag_        = Decisionflags_.DecelerateFlag;//减速标志
    DriveStraightLineFlag_ = Decisionflags_.DriveStraightLineFlag;//直线行使
    Overtakinginlaneflag_  = Decisionflags_.Overtakinginlaneflag;
    righttoleftlane_       = Decisionflags_.righttoleftlane;
    safetydistance = safetydistance_;//最近障碍物的s位置  基于车辆坐标系下的 
    FirstRunFlag = firstrunflag;//第一次运行标志
    ReducedDistanceFlag = ReducedDistanceFlag_;
    Reducedistanceeachtime = Reducedistanceeachtime_;
    Reducedistancesize = Reducedistancesize_;
    //加载yaml文件
    std::cout<<"Start_l: "<<Start_l<<"End_l: "<<End_l<<"Delta_l: "<<Delta_l<<"Target_v: "<<Target_v<<" traget_l: "<<traget_l<<std::endl;
    std::cout<<"all_distances: "<<all_distances<<"one_distances: "<<one_distances<<std::endl;
    std::cout<<"ObstacleAvoidanceFlag_: "<<ObstacleAvoidanceFlag_<<" "
             <<"DecelerateFlag_: "       <<DecelerateFlag_<<       " "
             <<"DriveStraightLineFlag_: "<<DriveStraightLineFlag_<<" "
             <<"Overtakinginlaneflag_: " <<Overtakinginlaneflag_<< " "
             <<"righttoleftlane_: "      <<righttoleftlane_<<std::endl;
   
    //总距离
    all_distance = all_distances;
    one_distance = one_distances;//单个距离 
    globalPath_  = std::move(globalPath);
    CAR_INDEX_ = car_index; //车辆位置编号  
    SIZE_ = 2*one_distances-1;
    getsamplepoints();
    Setpraents(sample_points_);
    //计算障碍物的sd和中心点
    obsesSD_= std::move(obses_limit_SD);
    obs_limits_distance = std::move(GlobalcoordinatesystemObsesLimit);//全局坐标系下在范围内的障碍物 
    // if(DriveStraightLineFlag_){
    //   obsesSD_.clear();
    //   obs_limits_distance.clear();
    // }
}

int local_dp_qp::GetoptTrajxy(Eigen::MatrixXd &optTrajxy,std::vector<Eigen::Vector4d> &frenet_path){
  CalculateCostTable();//计算代价 
  return FinalPath(optTrajxy,frenet_path);//获取路径  
}



//设置
void local_dp_qp::Setpraents(const std::vector<std::vector<std::vector<SlPoint>>>& sample_points) {
 // std::cout<<"sample_points_size: "<<sample_points.size()<<std::endl;
  cost_table_.resize(sample_points.size());
  for (size_t i = 0; i < sample_points.size(); ++i) {
    for (const auto& level : sample_points[i]) {
      std::vector<Node> level_points;
      for (const auto& p : level) 
      {
        level_points.emplace_back(Node(p));
      }
      cost_table_[i].emplace_back(level_points);//设置成node节点模式 
    }      
  }
  //std::cout<<"cost_table__size: "<<cost_table_.size()<<std::endl;
  //打印节点名
  // for (auto& level_points :cost_table_)
  // {
  //   for (auto& ll :level_points)
  //   {
  //     std::cout<<ll.sl_point.s<<" "<<ll.sl_point.l<<" ";
  //   }
  //   std::cout<<std::endl;
  // }
}

//计算代价 
void local_dp_qp::CalculateCostTable() {
  auto start = std::chrono::high_resolution_clock::now();
  for (auto &p : cost_table_) {
    p[0][0].cost = 0.0;
    p[0][0].dl = first_dl;
    p[0][0].ddl = first_dll;
    for (uint32_t s = 1; s < p.size(); ++s) {
      for (uint32_t l = 0; l < p[s].size(); ++l) {
        CalculateCostAt(s, l, p);
      }
    }
  }
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = end - start;
  std::cout << "AllCosttime: " << duration.count()<< " 秒" << std::endl;
}

void local_dp_qp::CalculateCostAt(const int32_t s, const int32_t l,  std::vector<std::vector<Node>> &single_cost)
{
  auto &pre_level = single_cost[s-1]; //前一层
  //auto &level_points  = cost_table_[s];//当前层
  double min_cost = std::numeric_limits<double>::max();
  quintic_polynomial_curve5d curve;//多项式 
  Eigen::VectorXd X_PARAM(6);//获取多项式的系数 
  //double timecost;
  for (auto& pre_point : pre_level) {
    curve.setparam(pre_point.sl_point.s, pre_point.sl_point.l, pre_point.dl, pre_point.ddl, 
                single_cost[s][l].sl_point.s,single_cost[s][l].sl_point.l,0.0,0.0);
    curve.Calcurver(X_PARAM);
    // if(DecelerateFlag_==false){//在进行减速时，所需的路径不进行障碍物的cost计算 只有当false时才可以 
    //   cost_obs = CalculateAllObstacleCost(curve,pre_point.sl_point, cost_table_[s][l].sl_point);//在直线行驶时，不进行障碍物的COST计算 
    // }else{
    //   cost_obs = CalculateAllObstacleCost(curve,pre_point.sl_point, cost_table_[s][l].sl_point);//在直线行驶时，不进行障碍物的COST计算 
    // }
    double cost_obs = CalculateAllObstacleCost(curve,pre_point.sl_point, single_cost[s][l].sl_point);
    //auto end = std::chrono::high_resolution_clock::now();
    double cost_path = CalculateReferenceLineCost(curve,pre_point.sl_point, single_cost[s][l].sl_point);
    double all_cost = cost_obs + cost_path;
    if(all_cost<min_cost){
      min_cost = all_cost;
      single_cost[s][l].pre_node = &pre_point;
      single_cost[s][l].curve = curve;
      single_cost[s][l].cost = min_cost;
    }
  }
}

int local_dp_qp::FinalPath(Eigen::MatrixXd &optTrajxy, std::vector<Eigen::Vector4d> &frenet_path){
  std::vector<Node> min_path;
  if (ReducedDistanceFlag) { //进行距离的缩减 
    std::vector<std::tuple<double, std::vector<Node>>> min_cost_list;
    for (size_t i = 0; i < cost_table_.size(); ++i){ //计算不同长度范围的cost
      auto & single_cost = cost_table_[i];
      double min_cost = std::numeric_limits<double>::max();
      Node * min_node=nullptr;
      int index; 
      std::vector<Node> path;
      for(int i = 0; i < single_cost[single_cost.size() - 1].size(); ++i){
        if (single_cost[single_cost.size()-1][i].cost < min_cost){
          min_cost = single_cost[single_cost.size()-1][i].cost;
          index = i;
        }
      }
      min_node = &single_cost[single_cost.size()-1][index]; //找到cost最小的局部路径 
      path.push_back(*min_node);
      while (min_node!=nullptr){
        min_node = min_node->pre_node;
        path.push_back(*min_node);
        if(min_node->pre_node==nullptr){
          break;
        }
      }
      min_cost_list.push_back(std::make_tuple(min_cost, path));
    }
    for (size_t i = 0; i < min_cost_list.size(); ++i){ //规划路径的总长度由变短 
      if (std::get<0>(min_cost_list[i]) < 1e100) {
        min_path = std::get<1>(min_cost_list[i]);
        break;
      } else { //下一组数据 
        if (i + 1 == min_cost_list.size()) { //如果是最后一组数据
          std::cout<<"多个路径均未找到"<<std::endl;
          return 0;
        } 
      }
    }
  } else { // 不进行距离的缩减 
   //先从最后一列中找到cost最小的点
    auto &single_cost = cost_table_.front();
    double min_cost = std::numeric_limits<double>::max();
    Node * min_node = nullptr;
    int index; 
    for(int i = 0; i < single_cost[single_cost.size() - 1].size(); ++i){
      if (single_cost[single_cost.size()-1][i].cost < min_cost){
        min_cost = single_cost[single_cost.size()-1][i].cost;
        index = i;
      }
    }
    min_node = &single_cost[single_cost.size()-1][index]; //找到cost最小的局部路径 
    min_path.push_back(*min_node);
    while (min_node!=nullptr){
      min_node = min_node->pre_node;
      min_path.push_back(*min_node);
      if(min_node->pre_node==nullptr){
        break;
      }
    }
    if(min_cost >= 1e100){
      std::cout<<"未找到局部路径"<<std::endl;
      return 0;
    }
  }
  //换算成从起点开始的排序 
  std::reverse(min_path.begin(),min_path.end());
  //最小cost的路已经发生碰撞 说明所有的路径均发生碰撞 
 
  // 生成min_cost_path
  if (frenet_path.size()!=0){frenet_path.clear();}
  frenet_path.reserve((min_path.size()-1)*(SIZE_+1));
  Eigen::Vector4d frenet_frame_point;
  for (size_t i = 1; i < min_path.size(); ++i){
    const auto prev_node = min_path[i-1];
    for (size_t j = 1; j <= SIZE_+1; ++j){
      double s_j_delta_s = prev_node.sl_point.s+j*delta_s; 
      double l = min_path[i].curve.Evaluate(0,s_j_delta_s);
      double dl = min_path[i].curve.Evaluate(1,s_j_delta_s);
      double dll = min_path[i].curve.Evaluate(2,s_j_delta_s);
      frenet_frame_point(0) =  s_j_delta_s;//s
      frenet_frame_point(1) = l;
      frenet_frame_point(2) = dl;
      frenet_frame_point(3) = dll;
      frenet_path.emplace_back(frenet_frame_point);
      //frenet_path[(SIZE_+1)*(i-1)+j]=frenet_frame_point;
    }
  }
  //在首位插入起始点的信息 
  frenet_path.insert(frenet_path.begin(), Eigen::Vector4d(min_path[0].sl_point.s,min_path[0].sl_point.l,min_path[0].dl,min_path[0].ddl));
  Eigen::MatrixXd xy_list;
  xy_list.resize(2,frenet_path.size());
  Eigen::Vector2d XY_;
  int INDEX=0;
  SlPoint SL;
  std::vector<Eigen::Vector3d> frenet_path_qp;
  Eigen::Vector3d qp_sl;
  frenet_path_qp.resize(frenet_path.size());
  optinglobalindex.resize(frenet_path.size());
  for (size_t i = 0; i < frenet_path.size(); ++i){
    SL.s = frenet_path[i](0);
    SL.l = frenet_path[i](1);
    qp_sl(0) = frenet_path[i](0);
    qp_sl(1) = frenet_path[i](1);
    qp_sl(2) = frenet_path[i](2);
    //std::cout<<"qp_sl: "<<qp_sl(0)<<" "<<qp_sl(1)<<" "<<qp_sl(2)<<std::endl;
    frenet_path_qp[i] = qp_sl;
    sdtofrenet(SL,INDEX,XY_);
    optinglobalindex[i]= INDEX;
    xy_list.col(i)=XY_;
  }
  // static int loopCount =1;
  // std::string fileName = "/home/mm/longshan_speedoptimisation/local/" + std::to_string(loopCount) + ".txt";
  // std::ofstream outFile;
  // outFile.open(fileName);
  // for(size_t i=0;i<xy_list.cols();++i){
  //     outFile <<xy_list(0,i)<<" "<<xy_list(1,i)<<std::endl;
  // }
  // outFile.close(); 
  // fileName = "/home/mm/longshan_speedoptimisation/local/obs" + std::to_string(loopCount) + ".txt";
  // outFile.open(fileName);
  // for(const auto &obs: obs_limits_distance){
  //     outFile <<obs(0)<<" "<<obs(1)<<" "
  //             <<obs(0)<<" "<<obs(3)<<" "
  //             <<obs(2)<<" "<<obs(3)<<" "
  //             <<obs(2)<<" "<<obs(1)<<std::endl;
  // }
  // outFile.close(); 
  // loopCount=loopCount+1;     

  bool Use_QP_FLAG =true;
  if (Use_QP_FLAG==true){
    QP qp_;
    std::pair<std::array<double, 3>, std::array<double, 3>> my_pair;
    my_pair.first = {vehicle_position.s,0,0};//s
    my_pair.second = {vehicle_position.l,first_dl,0};//l 
    bool changelaneflag;
    double leftbound ,rightbound;
    Eigen::MatrixXd xy_list_qp;
    try{
      if(Overtakinginlaneflag_||DriveStraightLineFlag_){
        changelaneflag = false;
      }else{
        changelaneflag = true; 
      }
      if(std::abs(vehicle_position.l)>2.3){
        leftbound = std::abs(vehicle_position.l);
        rightbound = -std::abs(vehicle_position.l);
      }else{
        leftbound = 2.3;
        rightbound = -2.3;
      }
      //std::cout<<"qp_.Process"<<std::endl;
      bool flag =qp_.Process(my_pair,std::move(obsesSD_),frenet_path,globalPath_,xy_list_qp,changelaneflag,leftbound,rightbound);
      if (!flag) {
        return 0;
      }
      std::cout<<"slqp_.flag"<< flag <<std::endl;
      // for (size_t i = 0; i < frenet_path_qp.size(); i++)
      // {
      //   std::cout<<frenet_path_qp[i](0)<<" "<<frenet_path_qp[i](1)<<" "<<frenet_path_qp[i](2)<<std::endl;
      // }
      
      if(flag){//有一个优化成功的，否则就按照cost最小的路径进行发布 
        Eigen::Vector2d xy_;
        Eigen::Vector3d xydk_;
        SlPoint sl;
        int mindex_qp = 0;
        Eigen::MatrixXd xy_qp;
        Eigen::MatrixXd xy_q_dk;
        size_t xy_list_qp_size = xy_list_qp.cols();
        xy_qp.resize(2,xy_list_qp_size);
        xy_q_dk.resize(3,xy_list_qp_size);
        for (size_t i = 0; i < xy_list_qp_size; ++i){
          sl.s = xy_list_qp(0,i);
          sl.l = xy_list_qp(1,i);
          sdtofrenet(sl, mindex_qp, xy_);
          xy_qp.col(i) = xy_;
          xydk_<< xy_(0), xy_(1), xy_list_qp(2, i);
          xy_q_dk.col(i) = xydk_;
        }
        referencePath(std::move(xy_q_dk),optTrajxy,frenet_path);// x y v thea dk 
        return 1;
      }
      else{
        referencePath(std::move(xy_list),10, optTrajxy, frenet_path);// x y v thea dk 
        return 1;
      }
    }
    catch(const std::exception& e){
      referencePath(std::move(xy_list),10,optTrajxy,frenet_path);// x y v thea dk 
    }
    return 1;
  }
  else{
    referencePath(std::move(xy_list),10,optTrajxy,frenet_path);// x y v thea dk 
  }
  return 1;
}

//将 sd转化为 xy 二分法  
void local_dp_qp::sdtofrenet(const SlPoint SL,int &index,Eigen::Vector3d &tesianXY){
  int left = 0;
  int right = globalPath_.cols()-1;
  while (left<=right)
  {
    if(left==right){
      if (left > 0 && (std::abs(SL.s -globalPath_(6, left)) < std::abs(SL.s -globalPath_(6, left-1))))
      {
          index = left;
          break;
      }
      else
      {
          index= left-1;
          break;
      }
    }
    int mid =static_cast<int>(left +(right-left)/2);
    if (globalPath_(6,mid)==SL.s){
      index =mid;
      break;
    }
    else if (globalPath_(6,mid) < SL.s)
    {
       left = mid+1;
    }else{
      right = mid;       //nums[mid]>target,如果mid赋值right以后，（left=right）> target,所以下面的if判断，
                         //分别是nums[left]、nums[left-1]和target的比较
    }
  }
  double cos_theta_r ;
  double sin_theta_r ;
  //寻找到最近的点 使用插值法计算xy  找到最近点 
  if (index==0||index==globalPath_.cols()-1){
    cos_theta_r = cos(globalPath_(3, index));
    sin_theta_r = sin(globalPath_(3, index));
    tesianXY(0) = globalPath_(0, index) - sin_theta_r * SL.l;
    tesianXY(1) = globalPath_(1, index) + cos_theta_r * SL.l;	
  }else{
    if (SL.s>=globalPath_(6,index)){
      double weigit = (SL.s-globalPath_(6,index))/(globalPath_(6,index+1)-globalPath_(6,index));
      cos_theta_r = cos(globalPath_(3, index));
      sin_theta_r = sin(globalPath_(3, index));
      double index_x  = globalPath_(0, index) - sin_theta_r * SL.l;
      double index_y  = globalPath_(1, index) + cos_theta_r * SL.l;

      cos_theta_r = cos(globalPath_(3, index+1));
      sin_theta_r = sin(globalPath_(3, index+1));
      double index_xadd  = globalPath_(0, index) - sin_theta_r * SL.l;
      double index_yadd  = globalPath_(1, index) + cos_theta_r * SL.l;
      tesianXY(0)  = index_x*(1-weigit)+index_xadd*weigit;
      tesianXY(1)  = index_y*(1-weigit)+index_yadd*weigit;
    }else{
      double weigit = (SL.s-globalPath_(6,index-1))/(globalPath_(6,index)-globalPath_(6,index-1));
      cos_theta_r = cos(globalPath_(3, index-1));
      sin_theta_r = sin(globalPath_(3, index-1));
      double index_x_  = globalPath_(0, index-1) - sin_theta_r * SL.l;
      double index_y_  = globalPath_(1, index-1) + cos_theta_r * SL.l;

      cos_theta_r = cos(globalPath_(3, index));
      sin_theta_r = sin(globalPath_(3, index));
      double index_x= globalPath_(0, index) - sin_theta_r * SL.l;
      double index_y  = globalPath_(1, index)+cos_theta_r * SL.l;

      tesianXY(0)  = index_x_ * (1-weigit) + index_x * weigit;
      tesianXY(1)  = index_y_ * (1-weigit) + index_y * weigit;
    }
  }
}

void local_dp_qp::sdtofrenet(const SlPoint SL,int &index,Eigen::Vector2d &tesianXY){
  int left = 0;
  int right = globalPath_.cols()-1;
  while (left<=right)
  {
    if(left==right){
      if (left > 0 && (std::abs(SL.s -globalPath_(6,left)) < std::abs(SL.s -globalPath_(6,left-1))))
      {
          index = left;
          break;
      }
      else
      {
          index= left-1;
          break;
      }
    }
    int mid =static_cast<int>(left +(right-left)/2);
    if (globalPath_(6,mid)==SL.s){
      index =mid;
      break;
    }
    else if (globalPath_(6,mid)<SL.s)
    {
       left = mid+1;
    }else{
      right = mid;       //nums[mid]>target,如果mid赋值right以后，（left=right）> target,所以下面的if判断，
                            //分别是nums[left]、nums[left-1]和target的比较
    }
  }
  double cos_theta_r ;
  double sin_theta_r ;
  //寻找到最近的点 使用插值法计算xy  找到最近点 
  if (index == 0 || index == globalPath_.cols() - 1){
    cos_theta_r = cos(globalPath_(3, index));
    sin_theta_r = sin(globalPath_(3, index));
    tesianXY(0) = globalPath_(0, index) - sin_theta_r * SL.l;
    tesianXY(1) = globalPath_(1, index) + cos_theta_r * SL.l;	
  }else{
    if (SL.s>=globalPath_(6,index)){
      double weigit = (SL.s-globalPath_(6,index))/(globalPath_(6,index+1)-globalPath_(6,index));
      cos_theta_r = cos(globalPath_(3, index));
      sin_theta_r = sin(globalPath_(3, index));
      double index_x  = globalPath_(0, index) - sin_theta_r * SL.l;
      double index_y  = globalPath_(1, index) + cos_theta_r * SL.l;

      cos_theta_r = cos(globalPath_(3, index+1));
      sin_theta_r = sin(globalPath_(3, index+1));
      double index_xadd  = globalPath_(0, index) - sin_theta_r * SL.l;
      double index_yadd  = globalPath_(1, index) +cos_theta_r * SL.l;
      tesianXY(0)  = index_x*(1-weigit)+index_xadd*weigit;
      tesianXY(1)  = index_y*(1-weigit)+index_yadd*weigit;
    }else{
      double weigit = (SL.s-globalPath_(6,index-1))/(globalPath_(6,index)-globalPath_(6,index-1));
      cos_theta_r = cos(globalPath_(3, index-1));
      sin_theta_r = sin(globalPath_(3, index-1));
      double index_x_  = globalPath_(0, index-1) - sin_theta_r * SL.l;
      double index_y_  = globalPath_(1, index-1) + cos_theta_r * SL.l;

      cos_theta_r = cos(globalPath_(3, index));
      sin_theta_r = sin(globalPath_(3, index));
      double index_x= globalPath_(0, index) - sin_theta_r * SL.l;
      double index_y  = globalPath_(1, index)+cos_theta_r * SL.l;

      tesianXY(0)  = index_x_*(1-weigit)+index_x*weigit;
      tesianXY(1)  = index_y_*(1-weigit)+index_y*weigit;
    }
  }
}

void local_dp_qp::referencePath(const Eigen::MatrixXd& xy, Eigen::MatrixXd& path,std::vector<Eigen::Vector4d> &frenetpath){
  path.resize(9, xy.cols());
  double dx, dy, ddx, ddy;
  path.setZero();
  path.row(0) = xy.row(0);//道路x
  path.row(1) = xy.row(1);//道路y
  double s;
  for (int i = 0; i < path.cols (); ++i) {
      path(2, i) = 10; //v
      if (i == 0) {
          dx = path(0, i + 1) - path(0, i);
          dy = path(1, i + 1) - path(1, i);
          ddx = path(0, 2) + path(0, 0) - 2 * path(0, 1);
          ddy = path(1, 2) + path(1, 0) - 2 * path(1, 1);
      }
      else if (i == path.cols() - 1) {
          dx = path(0, i) - path(0, i - 1);
          dy = path(1, i) - path(1, i - 1);
          ddx = path(0, i) + path(0, i - 2) - 2 * path(0, i - 1);
          ddy = path(1, i) + path(1, i - 2) - 2 * path(1, i - 1);
      }
      else {
          dx = path(0, i + 1) - path(0, i);
          dy = path(1, i + 1) - path(1, i);
          ddx = path(0, i + 1) + path(0, i - 1) - 2 * path(0, i);
          ddy = path(1, i + 1) + path(1, i - 1) - 2 * path(1, i);
      }
      // std::cout << "i: " << i << " dx: " << dx << " dy: " << dy << " ddx: " << ddx << " ddy: " << ddy << std::endl; 
      path(3, i) = atan2(dy, dx); // //道路yaw
      double kappa = (ddy * dx - ddx * dy) / pow((pow(dx, 2) + pow(dy, 2)), (1.5));
      path(4, i) =xy(2,i);// 道路y曲率k计算
      if(i==0){
        path(5, i)=0;
      }
      else{
        s += std::sqrt(std::pow(path(0, i )-path(0, i-1),2)+std::pow(path(1, i)-path(1, i-1),2));
        path(5, i) = s;
    }
  }
  try{
    std::cout<<"getspeeduseST"<<std::endl;
    getspeeduseST(path,frenetpath);
    if(Target_v == 0){//如果目标速度为0 则最后一点的速度和加速度一定是0
      path(2, path.cols()-1) = 0;
      path(6, path.cols()-1) = 0;
    }
    std::cout<<"getspeeduseST over"<<std::endl;
    // }else{
    //   getspeeduselinearinterpolation(path);
    // }
  }
  catch(const std::exception& e){
    getspeeduselinearinterpolation(path);
    if(Target_v == 0){//如果目标速度为0 则最后一点的速度和加速度一定是0
      path(2, path.cols()-1) = 0;
      path(6, path.cols()-1) = 0;
    }
  }
}

void local_dp_qp::referencePath(const Eigen::MatrixXd& xy, const double v, Eigen::MatrixXd& path,std::vector<Eigen::Vector4d> &frenetpath) {
  path.resize(9, xy.cols());
  double dx, dy, ddx, ddy;
  path.setZero();
  path.row(0) = xy.row(0);//道路x
  path.row(1) = xy.row(1);//道路y
  double s;
  for (int i = 0; i < path.cols (); i++) {
      path(2, i) = v; //v
      if (i == 0) {
          dx = path(0, i + 1) - path(0, i);
          dy = path(1, i + 1) - path(1, i);
          ddx = path(0, 2) + path(0, 0) - 2 * path(0, 1);
          ddy = path(1, 2) + path(1, 0) - 2 * path(1, 1);
      }
      else if (i == path.cols() - 1) {
          dx = path(0, i) - path(0, i - 1);
          dy = path(1, i) - path(1, i - 1);
          ddx = path(0, i) + path(0, i - 2) - 2 * path(0, i - 1);
          ddy = path(1, i) + path(1, i - 2) - 2 * path(1, i - 1);
      }
      else {
          dx = path(0, i + 1) - path(0, i);
          dy = path(1, i + 1) - path(1, i);
          ddx = path(0, i + 1) + path(0, i - 1) - 2 * path(0, i);
          ddy = path(1, i + 1) + path(1, i - 1) - 2 * path(1, i);
      }
      path(3, i) = atan2(dy, dx); // //道路yaw
      double kappa = (ddy * dx - ddx * dy) / pow((pow(dx, 2) + pow(dy, 2)), (1.5));
      path(4, i) = kappa;// 道路y曲率k计算
      if(i==0){
        path(5, i)=0;
      }
      else{
        s+= std::sqrt(std::pow(path(0, i )-path(0, i-1),2)+std::pow(path(1, i)-path(1, i-1),2));
        path(5, i)=s;
      }
  }
  try{
    getspeeduseST(path,frenetpath);
    if(Target_v == 0){//如果目标速度为0 则最后一点的速度和加速度一定是0
      path(2, path.cols()-1) = 0;
      path(6, path.cols()-1) = 0;
    }
  }
  catch(const std::exception& e){
    getspeeduselinearinterpolation(path);
    if(Target_v == 0){//如果目标速度为0 则最后一点的速度和加速度一定是0
      path(2, path.cols()-1) = 0;
      path(6, path.cols()-1) = 0;
    }
  }
}

inline double local_dp_qp::Sigmoid(const double x) { return 1.0/(1.0 + std::exp(-x));}

//障碍物的代价
double local_dp_qp::CalculateAllObstacleCost(quintic_polynomial_curve5d curver,SlPoint pre_point, SlPoint cur_point){
  std::vector<SlPoint> curver_points_list;
  Eigen::Vector3d XY_;
  int level_points_index;
  Eigen::MatrixXd car_Fpoint;
  double safedistance = vehicle_width_ * 1.8, s_cost = 0,l_cost = 0, last_point_cost;
  double obstacle_collision_distance ;
  constexpr double obstacle_collision_cost = 1e7;
  double l_min, l_max,l_wigth;
  constexpr double s_distance = 20; 
  SlPoint level_points;
  constexpr double kSafeDistance = 0.6;
  // std::vector<std::tuple<std::pair<double, double>,
	// 				std::pair<double, double>, std::pair<double, double>, std::pair<double, double>>> obstaclelist;
  // for (const auto &p : obsesSD_) {
  //   obstaclelist.emplace_back(std::make_tuple(std::make_pair(p.point1.s, p.point1.l), 
  //                                             std::make_pair(p.point2.s, p.point2.l),
  //                                             std::make_pair(p.point3.s, p.point3.l), 
  //                                             std::make_pair(p.point4.s, p.point4.l)));
  // }
  //与车辆位置的连线不进行障碍物的代价计算
  for (size_t i = 0; i <= SIZE_; ++i){
    if(i == 0){
      level_points.s = pre_point.s;
      level_points.l = pre_point.l;
    }else{
      level_points.s = pre_point.s + (i * delta_s);
      level_points.l = curver.Evaluate(0, level_points.s);
      //curver_points_list.emplace_back(std::move(level_points));
    }
     //轨迹上的点 将 sl 转为 x y THEA L w  
    sdtofrenet(level_points, level_points_index, XY_);//耗时操作 已修改  
    const double one_minus_kappa_r_d = 1 - globalPath_(4, level_points_index) * level_points.l;
    const double delta_theta = std::atan2(curver.Evaluate(1, level_points.s), one_minus_kappa_r_d);
    const double theta = tool::normalizeAngle(delta_theta + globalPath_(3, level_points_index));//局部路径的角度值 
    XY_(2) = theta;
    tool::get_car_fourpoint(vehicle_length_,vehicle_width_,XY_,car_Fpoint);//由车辆的中心点 计算局部路径上的每个店对应的车辆的四个顶点
    //右车辆的四个顶点 计算车辆四个顶点的SL；
    // std::vector<std::pair<double, double>> car_Fpoint_sl;
    // tool::get_car_fourpoint_sl(car_Fpoint, CAR_INDEX_, car_Fpoint_sl, globalPath_); //获取四个顶点的sl 
    // if (tool::HasOverlapUseSl(car_Fpoint_sl, obstaclelist)) {
    //   std::cout<<"sl线路上有碰撞"<<std::endl;
    //   return 1e100;//线路上有碰
    // }
    //计算车辆四个顶点的 s l 
    //与所有的障碍物进行一个判断 
    double CarAndObsDistanceThreshld = 2;
    for(size_t j = 0; j < obsesSD_.size(); ++j){
      auto &obsea_points = obsesSD_[j];
      double s_min = obsea_points.min_s;
      // l_min = std::min({obsea_points.point1.l,obsea_points.point2.l,obsea_points.point3.l,obsea_points.point4.l});
      // l_max = std::max({obsea_points.point1.l,obsea_points.point2.l,obsea_points.point3.l,obsea_points.point4.l});
      l_wigth = obsea_points.wigth;
      if(std::abs(s_min - level_points.s) > s_distance || 
        std::abs(obsea_points.centre_points.l-level_points.l) > (CarAndObsDistanceThreshld + l_wigth/2 + vehicle_width_/2)){ //纵向距离和横向距离的限制
        continue;
      } else { //在范围里面的 
          bool flag = tool::HasOverlap(car_Fpoint,obs_limits_distance[j],theta);
          if(flag){  //将轨迹上的点与 障碍物进行碰撞检测 有碰撞将返回较大的cost
            //std::cout<<"线路上有碰撞"<<std::endl;
            return 1e100;//线路上有碰撞
          }
          if(std::fabs(s_min - level_points.s) <= s_distance){
            if(Overtakinginlaneflag_ || DriveStraightLineFlag_){
              obstacle_collision_distance = vehicle_width_/2 + l_wigth / 2 + 1.0;
            }else if (ObstacleAvoidanceFlag_){
              if(l_wigth<1){ //障碍物较小 人
                obstacle_collision_distance = vehicle_width_/2 + l_wigth/2 + 1.0;
                }else{//车子 
                obstacle_collision_distance = vehicle_width_/2 + l_wigth/2 + 1.0;
              }
            }
            double delta_l = std::fabs(obsea_points.centre_points.l - level_points.l);
            l_cost += obstacle_collision_cost * Sigmoid(std::abs(obstacle_collision_distance - delta_l));
          } 
      }
    }
  }
  return (s_cost+l_cost);
}

//路径的代价
double local_dp_qp::CalculateReferenceLineCost(quintic_polynomial_curve5d curver, SlPoint pre_point, SlPoint cur_point){
  double path_cost;
  //0介导的损失  使用直线代替弧线距离
  //与车辆位置的连线不进行障碍物的代价计算 暂时 
  for (size_t i = 1; i <= SIZE_ + 1; ++i){
    float l = curver.Evaluate(0, pre_point.s + (i * delta_s));
    path_cost += (l - (Target_l)) * (l - (Target_l)) * path_l_cost;// 

    //判断是否已经偏离车道 点是否在车道外面 在车道外面增加 cost
    // float kBuff =1.5;
    // if(l>(left_boundary-kBuff)||l<(right_boundary+kBuff)){
    //   path_cost += l*l*1e13;
    // }
    const float dl = std::fabs(curver.Evaluate(1, pre_point.s+(i*delta_s))); //B. 侧方速度开销，path_dl_cost：4000
    path_cost += dl * dl * path_dl_cost; 

    const float ddl = std::fabs(curver.Evaluate(2, pre_point.s+(i*delta_s)));
    path_cost += ddl * ddl * path_ddl_cost;// C. 侧方加速度开销，path_ddl_cost：5

    // double last_point_cost = std::abs(l-targetl);
    // path_cost+=last_point_cost*1000;
  }

  return path_cost;
}

//加载文件
void local_dp_qp::loadyaml(){
  config = YAML::LoadFile(yamllocate);
  if(!ObstacleAvoidanceFlag_&&!righttoleftlane_){
    if(config["LANGE_NO_CHANGE"]){
      YAML::Node lange_no_change = config["LANGE_NO_CHANGE"];
      path_l_cost = lange_no_change["l_cost"].as<double>();
      path_dl_cost = lange_no_change["dl_cost"].as<double>();
      path_ddl_cost = lange_no_change["dll_cost"].as<double>();
    }
  }else if(ObstacleAvoidanceFlag_||righttoleftlane_){
    if(config["LANGE_CHANGE"]){
      YAML::Node lange_change = config["LANGE_CHANGE"];
      path_l_cost = lange_change["l_cost"].as<double>();
      path_dl_cost = lange_change["dl_cost"].as<double>();
      path_ddl_cost = lange_change["dll_cost"].as<double>();
    }
  }
  //std::cout <<"path_l_cost: "<<path_l_cost<<" path_dl_cost: "<<path_dl_cost<<" path_ddl_cost:"<<path_ddl_cost<<std::endl;
}

//获取限速 
void local_dp_qp::GetSpeedLimit(Eigen::MatrixXd &optTrajxy,std::vector<Eigen::Vector4d> &optTrajsd,std::vector<double> &speedlimit_){
  double limitspeedsingle = 2.77 * 1.2; //10km/h
  double distancethreshold = 0.5; //50cm
  double routelimitspeed = 1.38 * 1.2; //5*1.5 路口限速 
  double closrobslimitspeed = 1.38 * 1.2;  //接近障碍物限速 
  if (optTrajxy.cols() == 0){// local path messages 
    std::cout<<"local path is none in GetSpeedLimit"<<std::endl;
    return;
  }
  size_t optsize = optTrajxy.cols();
  speedlimit_.resize(optsize,limitspeedsingle);
  //车辆限速  obstacle 
  for (size_t i = 0; i < optsize; ++i){
    if(i == optsize-1){
      speedlimit_[optsize-1] = Target_v / 3.6 * 1.2;//最后一个点要给终点的速度
    }else{//其他的情况
      //1.判断当前的s的 l 的情况 首先计算距离障碍物横向距离的最小值 这里计算的是欧式距离 
      for (size_t j = 0; j < obsesSD_.size(); j++){
        if(std::abs(optTrajsd[i](0) - obsesSD_[j].centre_points.s) < 20){//20m的距离
          double min_s = std::min({obsesSD_[j].point1.s , obsesSD_[j].point2.s , obsesSD_[j].point3.s , obsesSD_[j].point4.s});
          double max_s = std::max({obsesSD_[j].point1.s , obsesSD_[j].point2.s , obsesSD_[j].point3.s , obsesSD_[j].point4.s});
          if(optTrajsd[i](0) >= (min_s - 10) && optTrajsd[i](0) <= (max_s + 10)){//在障碍物s内 障碍物扩大 
            Eigen::Vector3d car_pose(optTrajxy(0,i),optTrajxy(1,i),optTrajxy(3,i));
            Eigen::MatrixXd car_Fpoint;
            tool::get_car_fourpoint(vehicle_length_,vehicle_width_,car_pose,car_Fpoint);//由车辆的中心点 计算局部路径上的每个店对应的车辆的四个顶点
            //Eigen::Vector4d dd(obs_limits_distance[j](0),obs_limits_distance[j](1),obs_limits_distance[j](2),obs_limits_distance[j](3));
            double lat_min_diatcne = getmindistance(car_Fpoint,obsesSD_[j]);//求取 l 距离最小的值 
            if(lat_min_diatcne <= distancethreshold){
              speedlimit_[i] = closrobslimitspeed;//3.6km/h 
            }
          }
        }
      }
      // if(globalPath_(4,optinglobalindex[i])>=0.05){ //在路口
      //     speedlimit_[i] = std::min(routelimitspeed,speedlimit_[i]);// 5km/h
      //   }else{//不在路口
      //     speedlimit_[i] = std::min(limitspeedsingle,speedlimit_[i]);
      // }
    } 
  }

  // dkpappa speedlimit  5m
  for (size_t i = 0; i < optsize; ++i){
    if(std::abs(optTrajxy(4, i)) >= 0.03){
      size_t minindex = std::max(0, static_cast<int>(i - 10));
      for(size_t j = minindex; j <= i; ++j){
        speedlimit_[j] = std::min(routelimitspeed, speedlimit_[j]);
      }
      size_t maxindex = std::min(optsize , (i + 10));
      for(size_t k = i; k < maxindex; ++k){
        speedlimit_[k] = std::min(routelimitspeed, speedlimit_[k]);
      }
    }
  }
  // thansize = 0;
  // for (size_t i = 0; i < speedlimit_.size(); i++){
  //   std::cout<<"i: "<<i<<" dkpappa: "<<optTrajxy(4, i)<<" speedlimit: "<<speedlimit_[i]<<std::endl;
  //   if(speedlimit_[i] <= 3.0){
  //     thansize ++;
  //   }
  // }
  // std::cout<<"前后增加距离之后限制点的数量: "<<thansize<<std::endl;
  //终点速度是0的时候 要进行一些特殊的处理 
  if(Target_v == 0.0){
    if(speedlimit_.size() > 10){
      double speed = speedlimit_[speedlimit_.size()-10];
      double delta_speed = speed/10;//
      for (size_t i = speedlimit_.size()-10; i < speedlimit_.size(); ++i){
        speedlimit_[i] =std::min(speed - delta_speed * (i - speedlimit_.size() + 10), speedlimit_[i]);
      }
    }else{
      double speed = speedlimit_.front();
      double delta_speed = speed / speedlimit_.size();//
      for (size_t i = 0; i < speedlimit_.size(); ++i){
        speedlimit_[i] = std::min(speed - delta_speed * i, speedlimit_[i]);
      }
    }
  }
}

/****************计算车辆和障碍物最短的距离 sd******************/
double local_dp_qp::getmindistance(Eigen::MatrixXd &car_Fpoint,obses_sd &obs_sd){
  std::vector<std::pair<double,double>> points;
  for (size_t i = 0; i < car_Fpoint.cols(); ++i){
    const double x = car_Fpoint(0,i);
    const double y = car_Fpoint(1,i);
    size_t startindex = std::max(0, CAR_INDEX_-30);
    size_t endindex   = std::min(static_cast<size_t>(CAR_INDEX_+30), static_cast<size_t>(globalPath_.cols() - 1));
    size_t mindex;
    double mindistance=std::numeric_limits<double>::max();
    double dx, dy;
    for (size_t j = startindex; j < endindex; ++j){
      dx = x - globalPath_(0, j);
      dy = y - globalPath_(1, j);
      double distance = dx * dx + dy * dy;
      if(distance < mindistance){
        mindistance = distance;
        mindex = j;
      }
    }
    dx = x - globalPath_(0, mindex);
    dy = y - globalPath_(1, mindex);
    double cos_theta_r = std::cos(globalPath_(3, mindex));
    double sin_theta_r = std::sin(globalPath_(3, mindex));
    double ref_s = globalPath_(6, mindex);
    double path_s = dx * cos_theta_r + dy * sin_theta_r + ref_s;
    double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
    //std::pair<double,double> point;
    double d = std::copysign(cross_rd_nd, cross_rd_nd);
    double s = std::abs(path_s);
    points.emplace_back(std::make_pair(s, d));
  }
  double min_l = std::numeric_limits<double>::max();
  const std::array<double*, 4> obs_l = {&obs_sd.point1.l, &obs_sd.point2.l, 
                                           &obs_sd.point3.l, &obs_sd.point4.l};
  for (size_t i = 0; i < points.size(); ++i){
    double point_l = points[i].second;
    for (size_t j = 0; j < 4; ++j){
      double distance_l = std::abs(point_l - *obs_l[j]);
      if(distance_l < min_l){
        min_l = distance_l;
      }
    }
  }
  return min_l;
}

/****************计算车辆和障碍物最短的距离 sy******************/
double local_dp_qp::getmindistance(Eigen::MatrixXd &car_Fpoint,Eigen::Vector4d &obsxy){
  double distance = std::numeric_limits<double>::max();
  double x,y,dx,dy;
  std::array<double,8> list = {obsxy(0),obsxy(1),obsxy(0),obsxy(3),
                             obsxy(2),obsxy(3),obsxy(2),obsxy(1)};
  for (size_t i = 0; i < car_Fpoint.cols(); ++i){
    x= car_Fpoint(0,i);
    y= car_Fpoint(1,i);
    for (size_t j = 0; j < 4; ++j){
      dx = x - list[2 * j];
      dy = y - list[1 + 2 * j];
      double current_distance = dx * dx + dy * dy;
      if(current_distance < distance){
        distance = current_distance;
      }
    }
  }
  return distance;
}

/*****************线性插值获取速度*************/
void local_dp_qp::getspeeduselinearinterpolation(Eigen::MatrixXd &path){
 SpeedDecisions SpeedDecisions_;
  //首先需要先判断是否减速 
  if(DecelerateFlag_){  
    SpeedDecisions_.GetSpeed(vehicle_position.s,CAR_V,0,path,all_distance,true); //all_distance 
  }else if(ObstacleAvoidanceFlag_ || righttoleftlane_){
    SpeedDecisions_.GetSpeed(vehicle_position.s,CAR_V,Target_v,path,0,false);//
  }else if(Overtakinginlaneflag_ || DriveStraightLineFlag_){
    //在原车道行驶
    if(FirstRunFlag){
      SpeedDecisions_.GetSpeed(vehicle_position.s,CAR_V,Target_v,path,0,false);
    }else{
      SpeedDecisions_.GetSpeed(vehicle_position.s,CAR_V,Target_v,path,0,false);
    }
  }
}

/*****************ST获取速度*************/
void local_dp_qp::getspeeduseST(Eigen::MatrixXd &path,std::vector<Eigen::Vector4d> &frenetpath){
  speed_ speed;
  std::vector<double> speedlimit;
  //std::cout<<"GetSpeedLimit"<<std::endl;
  GetSpeedLimit(path, frenetpath, speedlimit);
  //std::cout<<"GetSpeedLimit over"<<std::endl;
  size_t limit_size = speedlimit.size();
  // 输出速度限制信息（仅用于调试目的）
  // std::cout << "path: "<<path.cols()<<std::endl;
  // std::cout << "frenetpath: "<<frenetpath.size()<<std::endl;
  // std::cout << "Speed Limits: "<<limit_size<<std::endl;
  //std::cout<<std::endl;
  std::vector<SpeedPoint> speeddata;
  std::vector<double> speeddkppa(limit_size);
  std::vector<SpeedDkappa> curise_speed(limit_size);
  for (size_t i = 0; i < limit_size; ++i){
    curise_speed[i].v = speedlimit[i] / 1.2; //巡航速度 也就是车辆正常情况下想要保持的速度 
    curise_speed[i].dkappa = path(4,i);
    curise_speed[i].s = path(5,i);
  }
  
  bool flag = speed.Search(all_distance,speeddata,CAR_V,CAR_A,curise_speed,speedlimit,speeddkppa,config);
  std::cout << "speed_dp_flag: "<<flag<<std::endl;
  // for (size_t i = 0; i < speeddata.size(); i++){
  //   std::cout<<"dpspeeddata t: "<<speeddata[i].t<<" v: "<<speeddata[i].v<<" a: "<<speeddata[i].a<<" s: "<<speeddata[i].s<<std::endl;
  // }
  if(flag){//speed dp 成功
    PiecewiseJerkSpeedOptimizer PiecewiseJerkSpeedOptimizer_; //QP
    std::vector<SpeedPoint> output_speed_data;
    std::vector<SpeedLimit> speedlimit_(limit_size); 
    for (size_t i = 0; i < limit_size; ++i){
      SpeedLimit limit(0.5 * i, speedlimit[i]);
      speedlimit_.emplace_back(limit);      
    }
    //由于纵向控制有问题，可能车辆实际的控制速度大于限速 
    if(speedlimit_.front().v < CAR_V){
       for(auto &limit : speedlimit_){
          limit.v = limit.v * 1.5;
       }
    }
    // std::cout << "Speed Limits: ";
    // for (size_t i = 0; i < limit_size; i++){
    //       std::cout<<speedlimit[i]<<" ";
    // }
    // std::cout<<std::endl;
    // double init_a;
    // if(CAR_V<0.2){//车辆速度很小 处于初始阶段
    //   init_a=0.1;
    // }else if(CAR_V>2){
    //   init_a=0.0;
    // }
    //这里需要车辆的加速度
    //std::cout<<path.transpose()<<std::endl;
    //std::cout<<"speeddata: "<<speeddata.size()<<" s" <<speeddata.back().s<<" t "<<speeddata.back().t<<std::endl;
    bool speed_qp_ = PiecewiseJerkSpeedOptimizer_.Process(CAR_V,CAR_A,path,speeddata,speeddata.back().s,speeddata.back().t,
                                                          speedlimit_,curise_speed,output_speed_data,config);
    
    std::cout<<"speed_qp_: "<<speed_qp_<<std::endl;
    //std::cout<<"path: "<<std::endl;
    //std::cout<<path.transpose()<<std::endl;

  }else{
    getspeeduselinearinterpolation(path);
  }
}

void local_dp_qp::getsamplepoints(){
  //清空之前的内容
  sample_points_.clear();
  cost_table_.clear();
  if (ReducedDistanceFlag) { //需要进行多个距离的计算 
    sample_points_.resize(Reducedistancesize); // 设置尺寸
    for (size_t i = 0; i < Reducedistancesize; i++){
      sample_points_[i].emplace_back(std::vector<SlPoint>{vehicle_position});//车辆位置 
      double alldistance = all_distance - i * Reducedistanceeachtime;//计算总长 
      for(double s = vehicle_position.s + one_distance; s <= vehicle_position.s + alldistance; s += one_distance) {
        std::vector<SlPoint> level_points;
        SlPoint sl;
        for (double l = Start_l; l >= End_l; l -= Delta_l) {
          sl.s = s;
          sl.l = l;
          level_points.emplace_back(sl);
        }
        sample_points_[i].emplace_back(level_points);
      }
    }
  } else { //不需要设置多个采样点 
    //设置采样点 
    sample_points_.resize(1);
    sample_points_[0].emplace_back(std::vector<SlPoint>{vehicle_position});//车辆位置 
    //首先进行当前车道的采样 
    for(double s = vehicle_position.s + one_distance; s <= vehicle_position.s + all_distance; s += one_distance) {
      std::vector<SlPoint> level_points;
      SlPoint sl;
      for (double l = Start_l; l >= End_l; l -= Delta_l) {
        sl.s = s;
        sl.l = l;
        level_points.emplace_back(sl);
      }
      sample_points_[0].emplace_back(level_points);
    }
  }
  
}


void local_dp_qp::findClosestPoint(const Eigen::VectorXd& car, const Eigen::MatrixXd& path, int& minIndex) {
		
  int startIndex = std::max(minIndex - 30, 0);
  int end = std::min(minIndex + 30, static_cast<int>(path.cols()-1));
  double distance, d_min = 999999;
  double dx, dy;
  for (int i = startIndex; i < end; i++) {
    dx = path(0, i) - car(0);
    dy = path(1, i) - car(1);
    distance = dx * dx + dy * dy;
    if (distance < d_min) {
      minIndex = i;
      d_min = distance;
    }
  }
	
}
void local_dp_qp::cartesianToFrenet(const Eigen::VectorXd& car, const Eigen::MatrixXd& path, frentPoint & carFrentPoint, int& minIndex) {
		
  findClosestPoint(car, path, minIndex);
  const double dx = car(0) - path(0,minIndex);
  const double dy = car(1) - path(1,minIndex);

  const double cos_theta_r = std::cos(path(3,minIndex));
  const double sin_theta_r = std::sin(path(3,minIndex));
  double ref_s = path(6, minIndex);
  double path_s = dx * cos_theta_r + dy * sin_theta_r + ref_s;
  const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
  // carFrentPoint.d = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
  carFrentPoint.d = std::copysign(cross_rd_nd, cross_rd_nd);
  carFrentPoint.s = std::abs(path_s);
}