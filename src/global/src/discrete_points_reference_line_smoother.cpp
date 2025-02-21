#include"discrete_points_reference_line_smoother.h"

void DiscretePointsReferenceLineSmoother::SetAnchorPoints(const std::vector<AnchorPoint>& anchor_points){
    anchor_points_ = anchor_points;

}

bool DiscretePointsReferenceLineSmoother::Smooth(const Eigen::MatrixXd& raw_reference_line,Eigen::MatrixXd &smoothed_reference_line){
   
    std::vector<std::pair<double, double>> raw_point2d;
    std::vector<double> anchorpoints_lateralbound;

    for (const auto& anchor_point : anchor_points_) {
        raw_point2d.emplace_back(anchor_point.x,
                                anchor_point.y);
        anchorpoints_lateralbound.emplace_back(anchor_point.lateral_bound);
    }

      // fix front and back points to avoid end states deviate from the center of
    // road 初始设置为强约束 
    // for (size_t i = 0; i < raw_point2d.size(); i++)
    // {
    // std::cout<<"11raw_point2d.x: "<<raw_point2d[i].first<< " 11raw_point2d.y: "<< raw_point2d[i].second<<std::endl;
    // }
    anchorpoints_lateralbound.front() = 0.0;
    anchorpoints_lateralbound.back() = 0.0;

    NormalizePoints(&raw_point2d);//进行归一化处理 相对于起始点的偏移 
    // for (size_t i = 0; i < raw_point2d.size(); i++)
    // {
    // std::cout<<"raw_point2d.x: "<<raw_point2d[i].first<< " raw_point2d.y: "<< raw_point2d[i].second<<std::endl;
    // }
    std::vector<std::pair<double, double>> smoothed_point2d;
    bool status = FemPosSmooth(raw_point2d, anchorpoints_lateralbound,&smoothed_point2d); //xy  约束 新的xy
      if (!status) {
    std::cout << "discrete_points reference line smoother fails"<<std::endl;
    return false;
    }
    //  for (size_t i = 0; i < smoothed_point2d.size(); i++)
    // {
    // std::cout<<"smoothed_point2d.x: "<<smoothed_point2d[i].first<< " smoothed_point2d.y: "<< smoothed_point2d[i].second<<std::endl;
    // }
    DeNormalizePoints(&smoothed_point2d);
    Eigen::MatrixXd xy; xy.resize(2,smoothed_point2d.size());
    for (size_t i = 0; i < smoothed_point2d.size(); ++i){
        xy(0,i)= smoothed_point2d[i].first;
        xy(1,i)= smoothed_point2d[i].second;
    }
    referencePath(xy,smoothed_reference_line);
    return 1;

}
bool DiscretePointsReferenceLineSmoother::FemPosSmooth(const std::vector<std::pair<double, double>>& raw_point2d,const std::vector<double>& bounds,
                                                    std::vector<std::pair<double, double>>* ptr_smoothed_point2d){

    // box contraints on pos are used in fem pos smoother, thus shrink the
    // bounds by 1.0 / sqrt(2.0)
    std::vector<double> box_bounds = bounds;
    const double box_ratio = 1.0 / std::sqrt(2.0);
    for (auto& bound : box_bounds) {
        bound *= box_ratio;
    }

    std::vector<double> opt_x;
    std::vector<double> opt_y;
    bool status = Solve(raw_point2d, box_bounds, &opt_x, &opt_y);
    if (!status) {
        return false;
    }
    if (opt_x.size() < 2 || opt_y.size() < 2) {
        return false;
    }
    size_t point_size = opt_x.size();
    for (size_t i = 0; i < point_size; ++i) {
        ptr_smoothed_point2d->emplace_back(opt_x[i], opt_y[i]);
    }
    return true;
}

bool DiscretePointsReferenceLineSmoother::Solve(const std::vector<std::pair<double, double>>& raw_point2d,const std::vector<double>& bounds,
                std::vector<double> *opt_x,std::vector<double> *opt_y){
    if (opt_x == nullptr || opt_y == nullptr) {
        return false;
    }
    FemPosDeviationOsqpInterface solver;
    solver.set_weight_fem_pos_deviation(weight_fem_pos_deviation);
    solver.set_weight_path_length(weight_path_length);
    solver.set_weight_ref_deviation(weight_ref_deviation);

    solver.set_max_iter(max_iter);
    solver.set_time_limit(time_limit);
    solver.set_verbose(verbose);
    solver.set_scaled_termination(scaled_termination);
    solver.set_warm_start(warm_start);
    solver.set_ref_points(raw_point2d);
    solver.set_bounds_around_refs(bounds);         
    if (!solver.Solve()) {
        return false;
    }
    *opt_x = solver.opt_x();
    *opt_y = solver.opt_y();
    return true;
}


/*首先，它从输入参数 xy_points 中获取起始点的 x 和 y 值，并将它们存储在 zero_x_ 和 zero_y_ 中，作为归一化的参考点。
然后，使用 std::for_each 遍历 xy_points 中的每个坐标点。
对于每个坐标点，使用 Lambda 表达式进行操作，捕获了外部的 this 指针以及坐标点的引用。
在 Lambda 表达式中，将当前点的 x 和 y 值减去起始点的 x 和 y 值，以实现归一化。这样可以将所有点的坐标转换为相对于起始点的偏移量。
将归一化后的坐标重新存储到原始的 xy_points 中，覆盖原始的坐标值。
总的来说，这个函数的作用是将给定的一组坐标点按照起始点进行归一化，以便后续的处理能够更加简便和准确。*/
void DiscretePointsReferenceLineSmoother::NormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
    //起始点的x y值 
    zero_x_ = xy_points->front().first;
    zero_y_ = xy_points->front().second;
    std::for_each(xy_points->begin(), xy_points->end(),
                    [this](std::pair<double, double>& point) 
                    {
                        auto curr_x = point.first;
                        auto curr_y = point.second;
                        std::pair<double, double> xy(curr_x - zero_x_,
                                                    curr_y - zero_y_);

                        point = std::move(xy);
                    });
}

// add x add y 
void DiscretePointsReferenceLineSmoother::DeNormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
     std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x + zero_x_,
                                               curr_y + zero_y_);
                  point = std::move(xy);
                });
}
void DiscretePointsReferenceLineSmoother::referencePath(const Eigen::MatrixXd& xy, Eigen::MatrixXd &path) {
		path.resize(7, xy.cols());
		double dx, dy, ddx, ddy, dddx, dddy, a, b, c, d;
		path.setZero();
		path.row(0) = xy.row(0);//道路x
		path.row(1) = xy.row(1);//道路y
		for (int i = 0; i < path.cols(); i++) {
            path(2,i) = 10;//道路v
			if (i == 0) {
				dx = path(0, i + 1) - path(0, i);
				dy = path(1, i + 1) - path(1, i);
				ddx = path(0, i + 2) + path(0, i) - 2 * path(0, i + 1);
				ddy = path(1, i + 2) + path(1, i) - 2 * path(1, i + 1);
				dddx = path(0, 3) - 3 * path(0, 2) +  3 * path(0, 1) - path(0, 0);
				dddy = path(1, 3) - 3 * path(1, 2) +  3 * path(1, 1) - path(1, 0);
			}else if (i == 1) {
				dx = path(0, i + 1) - path(0, i);
				dy = path(1, i + 1) - path(1, i);
				ddx = path(0, i + 1) + path(0, i - 1) - 2 * path(0, i);
				ddy = path(1, i + 1) + path(1, i - 1) - 2 * path(1, i);
				dddx = path(0, i + 2) - 3 * path(0, i + 1) + 3 * path(0, i ) - path(0, i - 1) ;
				dddy = path(1, i + 2) - 3 * path(1, i + 1) + 3 * path(1, i ) - path(1, i - 1) ;
			}else if(i == path.cols() - 1){
				dx = path(0, i) - path(0, i - 1);
				dy = path(1, i) - path(1, i - 1);
				ddx = path(0, i) + path(0, i - 2) - 2 * path(0, i - 1);
				ddy = path(1, i) + path(1, i - 2) - 2 * path(1, i - 1);
				dddx = path(0, i) - 3 * path(0, i-1) +  3 * path(0, i-2) - path(0, i-3);
				dddy = path(1, i) - 3 * path(1, i-1) +  3 * path(1, i-2) - path(1, i-3);
			}else{
				dx = path(0, i + 1) - path(0, i);
				dy = path(1, i + 1) - path(1, i);
				ddx = path(0, i + 1) + path(0, i - 1) - 2 * path(0, i);
				ddy = path(1, i + 1) + path(1, i - 1) - 2 * path(1, i);
				dddx = path(0, i + 1) - 3 * path(0, i ) + 3 * path(0, i - 1) - path(0, i - 2) ;
				dddy = path(1, i + 1) - 3 * path(1, i ) + 3 * path(1, i - 1) - path(1, i - 2) ;
			}
			path(3, i) = atan2(dy, dx); // //道路yaw
			// 计算曲率:设曲线r(t) = (x(t), y(t)), 则曲率k = (x'y" - x"y') / ((x')^2 + (y') ^ 2) ^ (1.5).
			double kappa = (ddy * dx - ddx * dy) / pow((pow(dx, 2) + pow(dy, 2)), (1.5));
			//double kappa2 = (ddy * dx - ddx * dy) / pow((dx * dx + dy * dy), (3 / 2));
			path(4, i) = kappa;// 道路y曲率k计算
			a = dx * ddy - dy * ddx;
			b = dx * dddy - dy * dddx;	
			c = dx * ddx + dy * ddy;
			d = dx * dx + dy * dy;
			double dkappa = (b * d - 3.0 * a * c) / (d * d * d);
			path(5, i) = dkappa;
		}
        double s_distance=0;
        for (size_t i = 0; i < path.cols(); ++i)
        {
           if (i==0)
           {
            path(6, i) = 0;
           }
           else{
                s_distance+= std::sqrt(std::pow(path(0,i) - path(0,i-1), 2) + std::pow(path(1,i) - path(1,i-1), 2));
                path(6, i)  = s_distance;//s 
           }
           
        }
	}