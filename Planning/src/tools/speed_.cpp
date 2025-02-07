#include"speed_.h"

speed_::speed_(){}
/**************************************************************************** 
 * 速度的主流程：
 * 1、初始化CostTable，   InitCostTable()
 * 2、初始化限速查询表，   InitSpeedLimitLookUp()
 * 3、计算更新CostTable， CalculateTotalCost()
 * 4、回溯，得到SpeedProfile
 * **************************************************************************/
bool speed_::Search(double total_length_s_,std::vector<SpeedPoint> &speed_data,double init_point_v_,double init_point_a_,std::vector<SpeedDkappa> cruise_speed_,
                    std::vector<double>speed_limit_list_,std::vector<double> &speed_dkppa_,YAML::Node &config_){
  init_point_v = init_point_v_;// 初始的速度 
  init_point_a =  init_point_a_;
  car_cruise_speed =std::move(cruise_speed_);//巡航速度 参考的速度
  speed_limit_list = std::move(speed_limit_list_);
  speed_dkppa = std::move(speed_dkppa_);//速度曲率
  dp_st_cost.loadyaml(config_);
  // std::cout<<"InitCostTable"<<std::endl;
  InitCostTable(total_length_s_);
  // std::cout<<"InitSpeedLimitLookUp"<<std::endl;
  InitSpeedLimitLookUp();
  ////std::cout<<"CalculateTotalCost"<<std::endl;
  CalculateTotalCost();
  //std::cout<<"RetrieveSpeedProfile"<<std::endl;
  if(!RetrieveSpeedProfile(speed_data)){
    return false;
  }
  return true;
}

/*************************************************************************
* 初始化CostTable
* cost_table_ 初始化
* 1、t方向  和  s方向的离散化
* 2、两层for循环完成初始化
**************************************************************************/

void speed_::InitCostTable(double total_length_s_){
     // t方向的离散化，dimension_t_ = total_length_t_ 除以 unit_t_（时间分辨率）。
    //std::cout<<"total_length_s_: "<<total_length_s_<<std::endl; 
    int dimension_t_ = static_cast<int>(std::ceil(
                 total_length_t_ / static_cast<double>(unit_t_))) + 1;//
    total_s_ = total_length_s_;
    // s方向的离散化
    // total_length_s_ -   
    //             static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_;
    dimension_s_ = total_length_s_/sparse_length_s + 1;//加上起始点计算 
    // cost_table_为双层vector，外层是t，内层是s，每一个t对应不同的s值 
    //这行代码创建了一个二维的向量 cost_table_，其元素类型是 StGraphPoint 类型。
    //具体而言，它是一个大小为 行数是s，列数是t。的二维向量，每个元素都是 StGraphPoint 类型的对象。
    cost_table_ = std::vector<std::vector<StGraphPoint>>(dimension_t_, std::vector<StGraphPoint>(dimension_s_, StGraphPoint()));
    //std::cout<<"cost_table_: "<<cost_table_.size()<<" "<<cost_table_[dimension_t_-1].size()<<std::endl;

    double curr_t = 0.0;// 起点 t = 0
    for (size_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_){
      auto& cost_table_i = cost_table_[i];
      double curr_s = 0.0;
      for (uint32_t j = 0; j < dimension_s_; ++j, curr_s += sparse_length_s) { //待修改 
        //std::cout<<"curr_s: "<<curr_s<<std::endl;
        cost_table_i[j].Init(i, j, STPoint(curr_t, curr_s));//第几个t s和对应的时间和s //第几个t和s 以及对应的t和s的值
      }
    }
    // spatial_distance_by_index_为 cost_table_的第0列的s值，方便后续计算                  
    const std::vector<StGraphPoint>& cost_table_0 = cost_table_[0];
    spatial_distance_by_index_ = std::vector<double>(cost_table_0.size(), 0.0);
    for (uint32_t i = 0; i < cost_table_0.size(); ++i) {
        spatial_distance_by_index_[i] = cost_table_0[i].point().s;
        //std::cout<<"spatial_distance_by_index_[i]: "<< cost_table_0[i].point().s<<" ";   
    }
}

// 2 初始化限速查询
/*************************************************************************
*
* 初始化限速信息
* 这个限速来自于参考线上的限速，可能是地图上车道线限速或者其他的什么在固定位置的限速
**************************************************************************/
void speed_::InitSpeedLimitLookUp(){
    speed_limit_by_index_.clear();
    speed_limit_by_index_.resize(dimension_s_);
    for (size_t i = 0; i < dimension_s_; ++i){
      speed_limit_by_index_[i] = speed_limit_list[i]; //具体线路上的速度 是和s相对应的 
    }
    // const auto& speed_limit = st_graph_data_.speed_limit();
    // for (uint32_t i = 0; i < dimension_s_; ++i) {
    // speed_limit_by_index_[i] = speed_limit.GetSpeedLimitByS(cost_table_[0][i].point().s());
    // }
}

// 3 计算所有的cost，并更新CostTable
/*******************************************************************************
 * 
 * 动态规划的主体，两层for循环遍历cost_table_，t外循环，s内循环
 * 调用CalculateCostAt()计算total_cost
 * ****************************************************************************/
void speed_::CalculateTotalCost(){
  //首先计算的就是首列首行， 然后后面执行剪枝操作 
  // 需要通过函数GetRowRange()计算出s的范围[next_lowest_row,next_highest_row]
  // 剪枝操作，避免不必要的计算
  size_t next_highest_row =0;
  size_t next_lowest_row  = 0;
   // 外循环，每一列的index，即每一个t
  for (size_t c = 0; c < cost_table_.size(); ++c) {
    size_t highest_row = 0;
    size_t lowest_row  = cost_table_.back().size() - 1;//行数-1 
    //cout为每一列的点数 
    //std::cout<<"next_lowest_row: "<<next_lowest_row<<" next_highest_row: "<<next_highest_row<<std::endl;
    //int count = static_cast<int>(next_highest_row) - static_cast<int>(next_lowest_row) + 1;
    if (true){//count 确保 next_highest_row>=next_lowest_row
      // 内循环，每行的index，即每一个s /**********暂时先使用所有行进行处理 *********/
      for (size_t r = 0; r <= cost_table_.back().size() - 1; ++r) { 
        auto msg = std::make_shared<StGraphMessage>(c,r);//首先先计算首列的cost 0列 0行 
        CalculateCostAt(msg);
      }
    }
    // 下一轮循环的准备工作，更新范围 [next_lowest_row, next_highest_row]
    // for (size_t r = next_lowest_row; r <= next_highest_row; ++r) {
    //   const auto& cost_cr = cost_table_[c][r]; //首先是0行0列 就是车辆起始的位置
    //   if (cost_cr.total_cost() < std::numeric_limits<double>::infinity()) {
    //     size_t h_r = 0;
    //     size_t l_r = 0;
    //     GetRowRange(cost_cr, &h_r, &l_r);
    //     //由第c列、第 next_lowest_row~next_highest_row 行的节点可到达的最远的s
    //     highest_row = std::max(highest_row, h_r);
    //     //可到达的最近的s
    //     lowest_row  = std::min(lowest_row, l_r);
    //   }
    // }
    // next_highest_row = highest_row;
    // next_lowest_row  = lowest_row;
  }
}

/**************************************************************************
 * 
 * 找K的范围，依据max_acceleration_和max_deceleration_的剪枝操作
**************************************************************************/
void speed_::GetRowRange(const StGraphPoint& point,size_t* next_highest_row,size_t* next_lowest_row){
  double v0 = 0.0;
  double acc_coeff = 0.5;
  if (!point.pre_point()) {
    v0 = init_point_v;// 初始的速度 
  } else {
    v0 = point.GetOptimalSpeed();
  }

  const auto max_s_size = cost_table_.back().size() - 1;//行数-1 
  const double t_squared = unit_t_ * unit_t_;
  const double s_upper_bound = v0 * unit_t_ +
                               acc_coeff * max_acceleration_ * t_squared +
                               point.point().s; //z最大加速度下 车辆应该到的s

  const auto next_highest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), s_upper_bound);//第一个不小于 
  if (next_highest_itr == spatial_distance_by_index_.end()) {
    *next_highest_row = max_s_size;
  } else {
    *next_highest_row =
        std::distance(spatial_distance_by_index_.begin(), next_highest_itr);
  }

  const double s_lower_bound =
      std::fmax(0.0, v0 * unit_t_ + acc_coeff * max_deceleration_ * t_squared) +
      point.point().s;//保证不能向后退

  const auto next_lowest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), s_lower_bound);
  if (next_lowest_itr == spatial_distance_by_index_.end()) {
    *next_lowest_row = max_s_size;
  } else {
    *next_lowest_row = std::distance(spatial_distance_by_index_.begin(), next_lowest_itr);
  }
}


//障碍物的cost 
void speed_::GetObstacleCost(const StGraphPoint const_cr){

}

double speed_::GetPatialPotentialCost (const StGraphPoint const_cr){
  return (total_s_-const_cr.point().s) * spatial_potential_penalty;
}


void speed_::CalculateCostAt(const std::shared_ptr<StGraphMessage> &msg){

  const uint32_t c = msg->c;
  const uint32_t r = msg->r;
  auto & cost_cr = cost_table_[c][r];//对应的 i j 第多少个t 多少个s 而不是真实的t和s的值 
  // 1、计算 obstacle_cost，如果为无穷大，则停止，
  GetObstacleCost(cost_cr);//暂时不进行障碍物的cost计算 因为无法获取障碍物的速度和预测 

  //空间的cost  spatial_potential_cost 
  double spatial_potential_cost;
  cost_cr.SetSpatialPotentialCost(GetPatialPotentialCost(cost_cr));//设置空间成本 //当前位置距离终点的cost成本

  /********************对速度 加速度 加加速度的成本******************************/ 
  //double init_point_v;//设置初始速度 
  //对第0列进行处理 
  const auto & cost_init=cost_table_[0][0]; //& 表示引用，即cost_init是对cost_table_[0][0]的引用，这样可以避免复制大型对象
  // 第0列的特殊处理，设置起始点的TotalCost 为0。
  if(c==0){
    cost_cr.SetTotalCost(0.0);
    cost_cr.SetOptimalSpeed(init_point_v);
    return ;
  }
  speed_limit = speed_limit_by_index_[r]; //当前s的限速 
  double cruise_speed = car_cruise_speed[r].v;//当前s的期望的巡航速度 
  // std::cout<<"speed_limit: "<<speed_limit<<std::endl;  
  // std::cout<<"cruise_speed: "<<cruise_speed<<std::endl;  
  //要建模为恒定加速度公式的最小值 s
  const double min_s_consider_speed = sparse_length_s * dimension_t_;//确定速度的最小值 ？？？？ sparse_length_s 
  //第一列的特殊处理 
  if(c==1){
    //当前的的加速度 
    // v1 = v0 + a * t, v1^2 - v0^2 = 2 * a * s => a = 2 * (s/t - v)/t
    //std::cout<<"init_point_v: "<<init_point_v<<std::endl;
    const double acc = 2*(cost_cr.point().s/unit_t_-init_point_v)/unit_t_;
    //std::cout<<"acc: "<<acc<<std::endl;

    //std::cout<<"c: "<<c<<" r: "<<r<<"acc: "<<acc<<std::endl;
    //加速度、减速度超出范围，返回
    if (acc < max_deceleration_ ||acc > max_acceleration_){
      return;
    }
     // 若v1小于0，但s却大于min_s_consider_speed，倒车，返回
     /*******************????????????????????????????????????????****************/
    //std::cout<<"cost_cr.point().s: "<<cost_cr.point().s<<std::endl;
    if(init_point_v + acc*unit_t_<-kDoubleEpsilon){
      return;
    }
    // 当前点与起始点的连线与stboundary有重叠，返回 这个是和障碍物进行判断的，现阶段不进行处理 
    /********************************************************
     ******************************************/
    // 计算当前点的total_cost     //点间cost，从起点到[c=1,r]的各项cost，即[t0,s0]->[t1,sn]的cost
    cost_cr.SetTotalCost(
      cost_cr.obstacle_cost()+cost_cr.spatial_potential_cost()+
      cost_init.total_cost()+ CalculateEdgeCostForSecondCol(r, speed_limit, cruise_speed)); //计算 速度 加速度 加加速度的cost
    cost_cr.SetPrePoint(cost_init);   //前一个点为初始点
    cost_cr.SetOptimalSpeed(init_point_v + acc * unit_t_);
    return;
  }

  //第二列 
  static constexpr double kSpeedRangeBuffer = 0.20;
  // 由当前点推出能到达该点的前一列最小的s
  // 将当前点的pre_col缩小至 [r_low, r]
  const double pre_lowest_s = 
      cost_cr.point().s-
      speed_limit * (1 + kSpeedRangeBuffer) * unit_t_;
  //std::cout<<"pre_lowest_s "<<pre_lowest_s<<std::endl;
  //找到第一个不小于pre_lowest_s的元素的迭代器 
  const auto pre_lowest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), pre_lowest_s);
  uint32_t r_low = 0;   
  // 计算 pre_lowest_s 的 index
  //std::cout<<"44444444444"<<std::endl;
  if (pre_lowest_itr == spatial_distance_by_index_.end()) {   
    r_low = dimension_s_ - 1;
  } else {
    r_low = static_cast<uint32_t>(
        std::distance(spatial_distance_by_index_.begin(), pre_lowest_itr));
  }

  //std::cout<<"r_row "<<r_low<<std::endl;
  //std::cout<<"r "<<r<<std::endl;
  const uint32_t r_pre_size = r ;//在范围内点的个数 
  //std::cout<<"r_pre_size "<<r_pre_size<<std::endl;
  const auto& pre_col = cost_table_[c - 1];
  double curr_speed_limit = speed_limit;//需要后面计算得到
  if (c==2){
    // 对于前一列，遍历从r->r_low的点， 
    // 依据重新算得的cost，当前点的pre_point，也就是DP过程的状态转移方程
    for (uint32_t i = 0; i <= r_pre_size; ++i){
      uint32_t r_pre = r-i;
      //std::cout<<"r_pre "<<r_pre<<std::endl;
      //跳过无穷大和为空的点 说明没有计算 此路不通 
      if (std::isinf(pre_col[r_pre].total_cost()) ||
          pre_col[r_pre].pre_point() == nullptr) {
        continue;
      }
      const double curr_a =   //当前点的加速度
          2 *((cost_cr.point().s - pre_col[r_pre].point().s) / unit_t_ -pre_col[r_pre].GetOptimalSpeed())/unit_t_;
    //std::cout<<"c: "<<c<<" r: "<<r<<" curr_a: "<<curr_a<<std::endl;

      if (curr_a < max_deceleration_ || curr_a > max_acceleration_) {
        continue;
      }
      //倒车 不进行计算 
      if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ <
              -kDoubleEpsilon ||
          cost_cr.point().s < pre_col[r_pre].point().s) {
        continue;
      }
      curr_speed_limit = std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);
      cruise_speed = std::fmin(speed_limit_by_index_[r_pre],cruise_speed);
      const double cost = cost_cr.obstacle_cost() +
                          cost_cr.spatial_potential_cost() +
                          pre_col[r_pre].total_cost() +
                          CalculateEdgeCostForThirdCol(
                              r, r_pre, curr_speed_limit, cruise_speed);
      //找到最小的cost 
      //c, r）的原有代价值更小，则更新当前节点（c, r)的总代价值
      if (cost < cost_cr.total_cost()) {
        cost_cr.SetTotalCost(cost);
        cost_cr.SetPrePoint(pre_col[r_pre]);    //找到每一个当前点的pre_point
        cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() +
                                curr_a * unit_t_);
      }
    }
    return;
  }
  // 第3列，及以后列的处理
  // 依据重新算得的cost，当前点的pre_point，也就是DP过程的状态转移方程
  //std::cout<<"r_pre_size: "<<r_pre_size<<std::endl;
  for (uint32_t i = 0; i <= r_pre_size; ++i) {
    // std::cout<<"r: "<<r<<std::endl;
    // std::cout<<"i: "<<i<<std::endl;
    uint32_t r_pre = r - i;
    //std::cout<<"r_pre: "<<r_pre<<std::endl;
    if (std::isinf(pre_col[r_pre].total_cost()) ||
        pre_col[r_pre].pre_point() == nullptr) {
      continue;
    }
    const double curr_a =
        2 *
        ((cost_cr.point().s - pre_col[r_pre].point().s) / unit_t_ -
         pre_col[r_pre].GetOptimalSpeed()) /
        unit_t_;
    //std::cout<<"c: "<<c<<" r: "<<r<<" curr_a: "<<curr_a<<std::endl;

    if (curr_a > max_acceleration_ || curr_a < max_deceleration_) {
      continue;
    }
    if (pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_ < -kDoubleEpsilon ||
        cost_cr.point().s < pre_col[r_pre].point().s) {
     
      continue;
    }
    // if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
    //                             pre_col[r_pre])) {
    //   continue;
    // }
    // 前前一点的row
    uint32_t r_prepre = pre_col[r_pre].pre_point()->index_s();
    const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];
    // 跳过无穷大
    if (std::isinf(prepre_graph_point.total_cost())) {
      continue;
    }
    // 跳过为空的情况
    if (prepre_graph_point.pre_point()== nullptr) {
      continue;
    }
    const STPoint& triple_pre_point = prepre_graph_point.pre_point()->point();  //上上上个点
    const STPoint& prepre_point = prepre_graph_point.point();                   //上上个点
    const STPoint& pre_point = pre_col[r_pre].point();                          //上个点
    const STPoint& curr_point = cost_cr.point(); 
    //std::cout<<"r_pre: "<<r_pre<<std::endl;                               //当前点
    curr_speed_limit =
        std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);
    //std::cout<<"curr_speed_limit: "<<curr_speed_limit<<std::endl;                               //当前点
    cruise_speed = std::fmin(speed_limit_by_index_[r_pre],cruise_speed);
   // std::cout<<"cruise_speed: "<<cruise_speed<<std::endl;                               //当前点

    double cost = cost_cr.obstacle_cost() + cost_cr.spatial_potential_cost() +
                  pre_col[r_pre].total_cost() +
                  CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
                                    curr_point, curr_speed_limit, cruise_speed);
    //std::cout<<"cost: "<<cost<<std::endl;                               //当前点  
    if (cost < cost_cr.total_cost()) {
      cost_cr.SetTotalCost(cost);
      cost_cr.SetPrePoint(pre_col[r_pre]);
      cost_cr.SetOptimalSpeed(pre_col[r_pre].GetOptimalSpeed() + curr_a * unit_t_);
    }
  }
}


/*************************************************************************************
 * 
 * 回溯，得到最优 speed_data
 * 
 * **********************************************************************************/
bool speed_::RetrieveSpeedProfile(std::vector<SpeedPoint> &speed_data){
  
  double min_cost = std::numeric_limits<double>::max();
  StGraphPoint * best_end_pointss = nullptr;
  size_t minindex;
  std::vector<StGraphPoint> last_data_list;
  std::vector<StGraphPoint> sample_s;//相同的s
  // 从cost_table_最后一列找 min_cost
  // for (size_t i = 0; i < cost_table_.back().size(); i++){
  //   if(!std::isinf(cost_table_.back()[i].total_cost())&&cost_table_.back()[i].total_cost()<min_cost){
  //     best_end_point = &cost_table_.back()[i];
  //     min_cost = cost_table_.back()[i].total_cost();
  //   }
  // }
  // for(const auto & cur_point :cost_table_.back()){
  //   if(!std::isinf(cur_point.total_cost())&&cur_point.total_cost()<min_cost){
  //     best_end_point = &cur_point;
  //     min_cost = cur_point.total_cost();
  //   }
  // }
  // 遍历每一列的最后一个点，找正在的best_end_point，并更新min_cost
  // 这里不直接将最后一列的min_cost点作为最佳终点呢？
  // 因为采样时间是一个预估时间，在此之前的各列最后一个点可能已经到达终点 找到每一列中 cost最小的值
  // 最后一列的最后一行 就是S的终点，寻找到达终点最小的cost 
  // for(const auto& row : cost_table_){
  //     for(const auto& cur_point : row) 
  //   std::cout<<"t: "<<cur_point.point().t<<" s: "<<cur_point.point().s<<" cost: "<<cur_point.total_cost()<<std::endl;
  // }
  
  if(std::isinf(cost_table_.back().back().total_cost())){ //最后一行的最后一个为inf 没有到达终点 
    //需要判别有没有全为inf的列
    size_t IsNOInfNuM = 0;
    while (IsNOInfNuM < cost_table_.size()){
      size_t size = 0;
      for (auto & cur_point :cost_table_[IsNOInfNuM]) {
        if (std::isinf(cur_point.total_cost())) { //如果等于inf
          size ++;
        } 
      }
      if (size == cost_table_[IsNOInfNuM].size()) { //一列中全为inf
          break;
      } else {
        IsNOInfNuM ++;
      }
    }
    //std::cout<<"IsNOInfNuM: "<<IsNOInfNuM<<std::endl;
    size_t RealUsingCol = IsNOInfNuM - 1;
    size_t max_index = 0;
    for (size_t i = 0; i < cost_table_[RealUsingCol].size(); i++){
      if(!std::isinf(cost_table_[RealUsingCol][i].total_cost()) ){
        max_index = i ; //找到最大的index
      }
    }
    //std::cout<<"max_index: "<<max_index<<std::endl;
    for (size_t i = 0; i <= RealUsingCol;  i++) {
      if(!std::isinf(cost_table_[i][max_index].total_cost()) && i != 0 ) {
        sample_s.push_back(cost_table_[i][max_index]);
      }
    }
    //std::cout<<"sample_s: "<<sample_s.size()<<std::endl;
    if (!sample_s.empty()) {
      //从小到大进行排序 
      auto comp2 = [] (const StGraphPoint &p1, const StGraphPoint &p2){
          return p1.total_cost() < p2.total_cost();
      };
      std::sort(sample_s.begin(), sample_s.end(), comp2);
      // for (size_t i = 0; i < sample_s.size(); i++){
      //       std::cout<<"sample_t: "<<sample_s[i].point().t<<" sample_s: "<<sample_s[i].point().s
      //                 <<" sample_cost: "<<sample_s[i].total_cost()<<std::endl;
      // }
      best_end_pointss = &(sample_s.front());
      //std::cout<<"best_end_points: " << (best_end_pointss)->point().s<<" best_end_pointt:"<<(best_end_pointss)->point().t<<std::endl;
      //std::cout<<"sample_s—s: " << (sample_s.front()).point().s<<" sample_s-t:"<<(sample_s.front()).point().t<<std::endl;

      // min_cost = sample_s.front().total_cost();
    }
  } else {
    for(const auto& row : cost_table_){
      const StGraphPoint &cur_point = row.back();
      // std::cout<<"cur_points: "<<cur_point.point().s<<" cur_point:"<<cur_point.point().t<<std::endl;
      // std::cout<<"total_cost: "<<cur_point.total_cost()<<std::endl;
      if (!std::isinf(cur_point.total_cost()) && cur_point.total_cost() != 0){
        last_data_list.push_back(cur_point);
        // best_end_point = &cur_point;
        // min_cost = cur_point.total_cost();
      }
    }
    if (!last_data_list.empty()) {
      auto comp = [] (const StGraphPoint &p1, const StGraphPoint &p2){
          return p1.total_cost() < p2.total_cost();
      };
      std::sort(last_data_list.begin(), last_data_list.end(), comp);
      best_end_pointss = &(last_data_list.front());
      min_cost = last_data_list.front().total_cost();
    }
  }
  // std::cout<<"min_cost: "<<min_cost<<std::endl;
    // }
  if (best_end_pointss == nullptr){
   std::cout<<"Fail to find the best feasible trajectory"<<std::endl;
   return false;
  }

  std::cout<<"best_end_point—s: "<< (best_end_pointss)->point().s<<" best_end_point-t:"<<(best_end_pointss)->point().t<<std::endl;
  // std::cout<<"last_data_list: "<<last_data_list.size()<<std::endl;
  // for (size_t i = 0; i < cost_table_.size(); i++){

  //   for (size_t j = 0; j < cost_table_[i].size(); j++){
  //     std::cout<<"t: "<<cost_table_[i][j].point().t<<" "<<"s: "<<cost_table_[i][j].point().s<<" "<<"cost: "<<cost_table_[i][j].total_cost();
  //   }
  //   std::cout<<std::endl;

  
  // 回溯，得到最优的 speed_profile
  std::vector<SpeedPoint> speed_profile;
  const StGraphPoint* cur_point = best_end_pointss;
  while (cur_point!=nullptr){//
    SpeedPoint speed_point;
    speed_point.s = cur_point->point().s;
    speed_point.t = cur_point->point().t;
    speed_profile.push_back(speed_point);
    cur_point = cur_point->pre_point();
  }
  std::reverse(speed_profile.begin(), speed_profile.end());
  // for (size_t i = 0; i < speed_profile.size(); i++){
  //   std::cout<<"speed_profile_t: "<<speed_profile[i].t<<" speed_profile_s: "<<speed_profile[i].s<<std::endl;
  // }
  
  static double kEpsilon = std::numeric_limits<double>::epsilon();
  if (speed_profile.front().t > kEpsilon ||
      speed_profile.front().s > kEpsilon) {
      std::cout<<"Fail to retrieve speed profile"<<std::endl;
      return false;
  }

  // 计算每个点的速度 v
  for (size_t i = 0; i < speed_profile.size()-1; ++i){
    const double v = (speed_profile[i + 1].s - speed_profile[i].s)/
                    (speed_profile[i + 1].t - speed_profile[i].t + 1e-3);
    speed_profile[i].v = v;
  }
  speed_profile[speed_profile.size()-1].v = speed_profile[speed_profile.size()-2].v;

  for (size_t i = 0; i < speed_profile.size()-1; ++i){
    const double a = (speed_profile[i+1].v-speed_profile[i].v)/(speed_profile[i+1].t - speed_profile[i].t + 1e-3);
    speed_profile[i].a = a;
  }
  speed_profile[speed_profile.size()-1].a = speed_profile[speed_profile.size()-2].a;

  // for (size_t i = 0; i < speed_profile.size(); i++){
  //   std::cout<<"t: "<<speed_profile[i].t<<" "<<"s: "<<speed_profile[i].s<<" "<<"v: "<<speed_profile[i].v<<" "<<"a:"<<speed_profile[i].a<<std::endl;
  // }
  // for (size_t i = 0; i < car_cruise_speed.size(); i++)
  // {
  //   std::cout<<"car_cruise_speed: "<<car_cruise_speed[i].s<<" dkappa: "<<car_cruise_speed[i].dkappa<<std::endl;
  // }

  // std::sort(car_cruise_speed.begin(), car_cruise_speed.end(), [](const SpeedDkappa &a, const SpeedDkappa &b) {
  //   return a.s < b.s;  // 根据 sk.s 进行排序
  // });

  //计算每个点曲率
 
  for(size_t i = 0; i < speed_profile.size(); ++i){
    double profile_s = speed_profile[i].s;
    double min_distance = std::numeric_limits<double>::max();
    size_t index;
    for (size_t j = 0; j < car_cruise_speed.size(); ++j)
    {
      double distance = std::abs(profile_s - car_cruise_speed[j].s);
      if(distance<min_distance){
        min_distance = distance;
        index = j;
      }
    }
    speed_profile[i].dkappa = car_cruise_speed[index].dkappa;
  }

  // auto comp = [](const SpeedDkappa &sk,double s){
  //   return s<sk.s;
  // };
  // for (size_t i = 0; i < speed_profile.size(); ++i){   
  //   auto it_lower = std::lower_bound(car_cruise_speed.begin(),car_cruise_speed.end(),speed_profile[i].s,comp);
  //   if (it_lower==car_cruise_speed.end()){
  //     speed_profile[i].dkappa = car_cruise_speed.back().dkappa;
  //   }else{
  //     speed_profile[i].dkappa = (it_lower)->dkappa;
  //   }
  // }
  speed_data = std::move(speed_profile);
  return true;

}
/********************************************************************************
 * 
 *  计算EdgeCost，包含 速度、加速度、jerk三部分
 * *****************************************************************************/

double speed_::CalculateEdgeCost(
    const STPoint& first, const STPoint& second, const STPoint& third,
    const STPoint& forth, const double speed_limit, const double cruise_speed) {
  return dp_st_cost.GetSpeedCost(third, forth, speed_limit, cruise_speed) +
         dp_st_cost.GetAccelCostByThreePoints(second, third, forth) +
         dp_st_cost.GetJerkCostByFourPoints(first, second, third, forth);
}
/********************************************************************************
 *  边界情况，第二列的特殊处理
 * *****************************************************************************/
double speed_::CalculateEdgeCostForSecondCol(const uint32_t row, const double speed_limit, const double cruise_speed){
  double init_speed = init_point_v;//初始的速度 
  double init_acc = init_point_a;//初始的加速度 
  const STPoint& pre_point = cost_table_[0][0].point();//上一个点
  const STPoint& curr_point = cost_table_[1][row].point();//当前点 
  return dp_st_cost.GetSpeedCost(pre_point, curr_point, speed_limit,cruise_speed) +
         dp_st_cost.GetAccelCostByTwoPoints(init_speed, pre_point, curr_point) +
         dp_st_cost.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point,curr_point);
}
/********************************************************************************
 * 
 *  边界情况，第三列的特殊处理
 * *****************************************************************************/
double speed_::CalculateEdgeCostForThirdCol(
    const uint32_t curr_row, const uint32_t pre_row, const double speed_limit,
    const double cruise_speed) {
  double init_speed = init_point_v;
  const STPoint& first = cost_table_[0][0].point();
  const STPoint& second = cost_table_[1][pre_row].point();
  const STPoint& third = cost_table_[2][curr_row].point();
  return dp_st_cost.GetSpeedCost(second, third, speed_limit, cruise_speed) +
         dp_st_cost.GetAccelCostByThreePoints(first, second, third) +
         dp_st_cost.GetJerkCostByThreePoints(init_speed, first, second, third);
}

/********************StGraphPoint**************************/
StGraphPoint::StGraphPoint(){};

std::uint32_t StGraphPoint::index_s() const { return index_s_; }

std::uint32_t StGraphPoint::index_t() const { return index_t_; }

const STPoint& StGraphPoint::point() const { return point_; }

const StGraphPoint* StGraphPoint::pre_point() const { return pre_point_; }

double StGraphPoint::reference_cost() const { return reference_cost_; }

double StGraphPoint::obstacle_cost() const { return obstacle_cost_; }

double StGraphPoint::spatial_potential_cost() const {
  return spatial_potential_cost_;
}

double StGraphPoint::total_cost() const { return total_cost_; }


void StGraphPoint::Init(const std::uint32_t index_t,
                        const std::uint32_t index_s, const STPoint& st_point) {
  index_t_ = index_t;
  index_s_ = index_s;
  point_   = st_point;
}

void StGraphPoint::SetReferenceCost(const double reference_cost) {
  reference_cost_ = reference_cost;
}

void StGraphPoint::SetObstacleCost(const double obs_cost) {
  obstacle_cost_ = obs_cost;
}

void StGraphPoint::SetSpatialPotentialCost(
    const double spatial_potential_cost) {
  spatial_potential_cost_ = spatial_potential_cost;
}

void StGraphPoint::SetTotalCost(const double total_cost) {
  total_cost_ = total_cost;
}

void StGraphPoint::SetPrePoint(const StGraphPoint& pre_point) {
  pre_point_ = &pre_point;
}

double StGraphPoint::GetOptimalSpeed() const { return optimal_speed_; }

void StGraphPoint::SetOptimalSpeed(const double optimal_speed) {
  optimal_speed_ = optimal_speed;
}
