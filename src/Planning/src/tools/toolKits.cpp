#include"toolKits.h"
/**
 * 1.与Ver4相比
 * 增加了由x,y,计算theta和kappa函数
 * 
*/
#define x0 629985.3117 // 园区西门口世界坐标，单位：m
#define y0 3481694.8199
namespace tool{
	
	double distance(const Eigen::Matrix<double, 20, 2>& finalKeyPoint, int i, int j) {
		return sqrt(pow(finalKeyPoint(i, 0) - finalKeyPoint(j, 0), 2) + pow(finalKeyPoint(i, 1) - finalKeyPoint(j, 1), 2));
	}

	int straight(Eigen::Vector3d start, Eigen::Vector3d end, double deltaT, Eigen::Matrix<double, 5, Eigen::Dynamic>&  path) {
		double T = sqrt(pow((end(0) - start(0)), 2) + pow((end(1) - start(1)), 2)) / (start(2));
		int t = T / deltaT;
		path.resize(5, t+1);
		path(0, 0) = start(0); //x
		path(1, 0) = start(1); //y
		path(2, 0) = start(2); //v
		path(3, 0) = atan2((end(1) - start(1)), (end(0) - start(0)));//theta
		path(4, 0) = 0;//kappa
		for (int i = 1; i < path.cols(); i++) {
			path(4, i) = path(4, i - 1);
			path(3, i) = path(3, i - 1);
			path(2, i) = path(2, i - 1);
			path(1, i) = path(1, i - 1) + path(2, i - 1) * sin(path(3, i - 1)) * deltaT;
			path(0, i) = path(0, i - 1) + path(2, i - 1) * cos(path(3, i - 1)) * deltaT;
		}
		return 0;
	}

	int arc(Eigen::Vector4d start, double R, double theta, double deltaT, Eigen::Matrix<double, 5, Eigen::Dynamic>& path) {
		double T = d2r(theta) * R / (start(2));
		int t = T / deltaT;
		path.resize(5, t);
		path(0, 0) = start(0); //x
		path(1, 0) = start(1); //y
		path(2, 0) = start(2); //v
		path(3, 0) = start(3); //theta
		path(4, 0) = 1 / R; //kappa
		for (int i = 1; i < path.cols(); i++) {
			path(4, i) = path(4, i - 1);
			path(2, i) = path(2, i - 1);
			path(3, i) = path(3, i - 1) + path(2, i - 1) * deltaT / R;
			path(1, i) = path(1, i - 1) + R * (sin(path(3, i)) - sin(path(3, i - 1)));
			path(0, i) = path(0, i - 1) + R * (cos(path(3, i - 1)) - cos(path(3, i)));
		}
		return 0;
	}

	int arc2(Eigen::Vector4d start, double R, double theta, double deltaT, Eigen::Matrix<double, 5, Eigen::Dynamic>& path) {
		double T = d2r(theta) * R / (start(2));
		int t = T / deltaT;
		path.resize(5, t);
		path(0, 0) = start(0); //x
		path(1, 0) = start(1); //y
		path(2, 0) = start(2); //v
		path(3, 0) = start(3); //theta
		path(4, 0) = 1 / R; //kappa
		for (int i = 1; i < path.cols(); i++) {
			path(4, i) = path(4, i - 1);
			path(2, i) = path(2, i - 1);
			path(3, i) = path(3, i - 1) + path(2, i - 1) * deltaT / R;
			path(1, i) = path(1, i - 1) - R * (sin(path(3, i)) - sin(path(3, i - 1)));
			path(0, i) = path(0, i - 1) + R * (cos(path(3, i - 1)) - cos(path(3, i)));
		}
		return 0;
	}

	int arc3(Eigen::Vector4d start, double R, double theta, double deltaT, Eigen::Matrix<double, 5, Eigen::Dynamic>& path) {
		double T = d2r(theta) * R / (start(2));
		int t = T / deltaT;
		path.resize(5, t);
		path(0, 0) = start(0); //x
		path(1, 0) = start(1); //y
		path(2, 0) = start(2); //v
		path(3, 0) = start(3); //theta
		path(4, 0) = 1 / R; //kappa
		for (int i = 1; i < path.cols(); i++) {
			path(4, i) = path(4, i - 1);
			path(2, i) = path(2, i - 1);
			path(3, i) = path(3, i - 1) + path(2, i - 1) * deltaT / R;
			path(1, i) = path(1, i - 1) - R * (sin(path(3, i)) - sin(path(3, i - 1)));
			path(0, i) = path(0, i - 1) + R * (cos(path(3, i - 1)) - cos(path(3, i)));
		}
		return 0;
	}

	int arc4(Eigen::Vector4d start, double R, double theta, double deltaT, Eigen::Matrix<double, 5, Eigen::Dynamic>& path) {
		double T = d2r(theta) * R / (start(2));
		int t = T / deltaT;
		path.resize(5, t);
		path(0, 0) = start(0); //x
		path(1, 0) = start(1); //y
		path(2, 0) = start(2); //v
		path(3, 0) = start(3); //theta
		path(4, 0) = 1 / R; //kappa
		for (int i = 1; i < path.cols(); i++) {
			path(4, i) = path(4, i - 1);
			path(2, i) = path(2, i - 1);
			path(3, i) = path(3, i - 1) + path(2, i - 1) * deltaT / R;
			path(1, i) = path(1, i - 1) + R * (sin(path(3, i)) - sin(path(3, i - 1)));
			path(0, i) = path(0, i - 1) + R * (cos(path(3, i - 1)) - cos(path(3, i)));
		}
		return 0;
	}

	int  arc5(Eigen::Vector4d start, double R, double theta, double deltaT, Eigen::Matrix<double, 5, Eigen::Dynamic>& path) {
		double T = d2r(theta) * R / (start(2));
		int t = T / deltaT;
		path.resize(5, t);
		path(0, 0) = start(0); //x
		path(1, 0) = start(1); //y
		path(2, 0) = start(2); //v
		path(3, 0) = start(3); //theta
		path(4, 0) = 1 / R; //kappa
		for (int i = 1; i < path.cols(); i++) {
			path(4, i) = path(4, i - 1);
			path(2, i) = path(2, i - 1);
			path(3, i) = path(3, i - 1) + path(2, i - 1) * deltaT / R;
			path(1, i) = path(1, i - 1) + R * (sin(path(3, i)) - sin(path(3, i - 1)));
			path(0, i) = path(0, i - 1) - R * (cos(path(3, i - 1)) - cos(path(3, i)));
		}
		return 0;
	}

	int arc6(Eigen::Vector4d start, double R, double theta, double deltaT, Eigen::Matrix<double, 5, Eigen::Dynamic>& path) {
		double T = d2r(theta) * R / (start(2));
		int t = T / deltaT;
		path.resize(5, t);
		path(0, 0) = start(0); //x
		path(1, 0) = start(1); //y
		path(2, 0) = start(2); //v
		path(3, 0) = start(3); //theta
		path(4, 0) = 1 / R; //kappa
		for (int i = 1; i < path.cols(); i++) {
			path(4, i) = path(4, i - 1);
			path(2, i) = path(2, i - 1);
			path(3, i) = path(3, i - 1) + path(2, i - 1) * deltaT / R;
			// path(1, i) = path(1, i - 1) + R * (cos(path(3, i - 1)) - cos(path(3, i)));
			// path(0, i) = path(0, i - 1) - R * (sin(path(3, i)) - sin(path(3, i - 1)));
			path(1, i) = path(1, i - 1) + R * (sin(path(3, i)) - sin(path(3, i - 1)));
			path(0, i) = path(0, i - 1) + R * (cos(path(3, i - 1)) - cos(path(3, i)));
		}
		return 0;
	}

	int arc7(Eigen::Vector4d start, double R, double theta, double deltaT, Eigen::Matrix<double, 5, Eigen::Dynamic>& path) {
		double T = d2r(theta) * R / (start(2));
		int t = T / deltaT;
		path.resize(5, t);
		path(0, 0) = start(0); //x
		path(1, 0) = start(1); //y
		path(2, 0) = start(2); //v
		path(3, 0) = start(3); //theta
		path(4, 0) = 1 / R; //kappa
		for (int i = 1; i < path.cols(); i++) {
			path(4, i) = path(4, i - 1);
			path(2, i) = path(2, i - 1);
			path(3, i) = path(3, i - 1) + path(2, i - 1) * deltaT / R;
			path(1, i) = path(1, i - 1) + R * (sin(path(3, i)) - sin(path(3, i - 1)));
			path(0, i) = path(0, i - 1) + R * (cos(path(3, i - 1)) - cos(path(3, i)));
		}
		return 0;
	}

	int arc8(Eigen::Vector4d start, double R, double theta, double deltaT, Eigen::Matrix<double, 5, Eigen::Dynamic>& path) {
		double T = d2r(theta) * R / (start(2));
		int t = T / deltaT;
		path.resize(5, t);
		path(0, 0) = start(0); //x
		path(1, 0) = start(1); //y
		path(2, 0) = start(2); //v
		path(3, 0) = start(3); //theta
		path(4, 0) = 1 / R; //kappa
		for (int i = 1; i < path.cols(); i++) {
			path(4, i) = path(4, i - 1);
			path(2, i) = path(2, i - 1);
			path(3, i) = path(3, i - 1) + path(2, i - 1) * deltaT / R;
			path(1, i) = path(1, i - 1) + R * (sin(path(3, i)) - sin(path(3, i - 1)));
			path(0, i) = path(0, i - 1) - R * (cos(path(3, i - 1)) - cos(path(3, i)));
		}
		return 0;
	}

	/**/
	double lineEq(const cv::Point2d& start, const cv::Point2d& end, const cv::Point2d& inflect) {
		double endx = end.x;
		double endy = end.y;
		double startx = start.x;
		double starty = start.y;
		double inflectx = inflect.x;
		double inflecty = inflect.y;
		return (endy - starty) * (inflectx - (endx + startx) / 2) - (endx - startx) * (inflecty - (endy + starty) / 2);
	}

	int realPosition2imagePosition(const double& x, const double& y, Eigen::Matrix<double, 1, 2>& imagePosition) {
		imagePosition(0, 0) = (x) * (558.0 / 319.3) + 45.0;
		imagePosition(0, 1) = (-1)*(y) * (200.0 / 113.0) + 470;
		// std::cout << "imagePosition:";
		// std::cout << imagePosition << std::endl;
		return 0;
	}

	int imagePosition2realPosition(const double& x, const double& y, Eigen::Matrix<double, 1, 2>& realPosition) {
		realPosition(0, 0) = (x - 46.0) * (319.3 / 558.0);
		realPosition(0, 1) = (-1) * (y - 470.0) * (113.0 / 200.0);
		// std::cout << "realPosition:";
		// std::cout << realPosition << std::endl;
		return 0;
	}

	int referencePath(const Eigen::MatrixXd& xyv, std::vector< Eigen::Matrix<double, 6, Eigen::Dynamic> >& reTrajt  ) {
		Eigen::MatrixXd path;
		path.resize(6, xyv.cols());
		double dx, dy, ddx, ddy, dddx, dddy, a, b, c, d;
		path.setZero();
		path.row(0) = xyv.row(0);//道路x
		path.row(1) = xyv.row(1);//道路y
		path.row(2) = xyv.row(2);//道路v
		for (int i = 0; i < path.cols(); i++) {
			if (i == 0) {
				dx = path(0, i + 1) - path(0, i);
				dy = path(1, i + 1) - path(1, i);
				ddx = path(0, i + 2) + path(0, i) - 2 * path(0, i + 1);
				ddy = path(1, i + 2) + path(1, i) - 2 * path(1, i + 1);
				dddx = path(0, 3) - 3 * path(0, 2) +  3 * path(0, 1) - path(0, 0);
				dddy = path(1, 3) - 3 * path(1, 2) +  3 * path(1, 1) - path(1, 0);
			}
			else if (i == path.cols() - 1) {
				dx = path(0, i) - path(0, i - 1);
				dy = path(1, i) - path(1, i - 1);
				ddx = path(0, i) + path(0, i - 2) - 2 * path(0, i - 1);
				ddy = path(1, i) + path(1, i - 2) - 2 * path(1, i - 1);
				dddx = path(0, i) - 3 * path(0, i-1) +  3 * path(0, i-2) - path(0, i-3);
				dddy = path(1, i) - 3 * path(1, i-1) +  3 * path(1, i-2) - path(1, i-3);
			}
			else {
				dx = path(0, i + 1) - path(0, i);
				dy = path(1, i + 1) - path(1, i);
				ddx = path(0, i + 1) + path(0, i - 1) - 2 * path(0, i);
				ddy = path(1, i + 1) + path(1, i - 1) - 2 * path(1, i);
				dddx = path(0, i + 2) - 3 * path(0, i + 1) + 3 * path(0, i) - path(0, i - 1) ;
				dddy = path(1, i + 2) - 3 * path(1, i + 1) + 3 * path(1, i) - path(1, i - 1) ;
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
		reTrajt.push_back(path);
		return 0;
	}

	double normalizeAngle(double angle) {
		angle = std::fmod(angle, 2 * M_PI);
		if (angle > M_PI)
			angle -= 2 * M_PI;
		else if (angle < -M_PI)
			angle += 2 * M_PI;
		return angle;
    }
/**/
	int deleteRepeatedData(const Eigen::Matrix<double, 5, Eigen::Dynamic>& subPath, Eigen::Matrix<double, 5, Eigen::Dynamic>& pathTemp, int& j) {
		//int j = 0;
		for (int i = 0; i < subPath.cols(); i++) {
			bool flag = true;
			for (int k = 0; k < pathTemp.cols(); k++) {
				if (pathTemp(0, k) == subPath(0, i) && pathTemp(1, k) == subPath(1, i)) {
					flag = false;
					break;
				}
				else {
					flag = true;
				}
			}
			if (flag) {
				pathTemp(0, j) = subPath(0, i);
				pathTemp(1, j) = subPath(1, i);
				pathTemp(2, j) = subPath(2, i);
				pathTemp(3, j) = subPath(3, i);
				pathTemp(4, j) = subPath(4, i);
				j += 1;
			}
		}
		return j;
	}

	void findClosestPoint(const Eigen::VectorXd& car, const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& path, int& minIndex) {
		// static int startIndex = 0;
		// static int end = path.cols();
		int startIndex = 0;
		int end = path.cols();
		double distance, d_min = 999999;
		int i = startIndex;
		for (; i < end; i++) {
			distance = std::pow(path(0,i) - car(0),2) + std::pow(path(1,i) - car(1),2);
			if (distance < d_min) {
				minIndex = i;
				d_min = distance;
				startIndex = i;
			}
		}
		// startIndex = startIndex - 10 < 0 ? 0 : startIndex;
		// end = startIndex + 30;
		// end = end - path.cols() < 0 ? end : path.cols();
	}

	void findClosestPointArray(const Eigen::VectorXd& car, const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& path, int& minIndex) {
		Eigen::MatrixXd distance;
		Eigen::MatrixXd::Index minRowIndex, minColIndex;
		distance.resize(1, path.cols());
        distance = abs(path.row(0).array() - car(0)) +  abs(path.row(1).array() - car(1)); 
		distance.minCoeff(&minRowIndex, &minColIndex);
		minIndex = minColIndex;
	}

	int binarySerach(const Eigen::MatrixXd set, double  value) {
		//已知车的s = value,查找point set中s与车s最近的点下标
		const int r = 5;
        int left= 0;
        int right = set.cols() - 1;
        while (left <= right) {
            // 找出中间下标 
            int mid = left + ((right - left) >> 1);
            if (set(r, mid) > value) {
                right = mid - 1;
            } else if (set(r, mid) < value) {
                left = mid + 1;
            } else {
				 //近似处理
				 if(fabs(set(r, mid) - value )> 0.5){
                    return mid + 1;
				 }else if(fabs(set(r, mid) - value )< 0.5){
					return mid - 1;
				 }else {
					return mid;
				 }
            }
        }
        return -1;
    }

	int cartesianToFrenet(const Eigen::VectorXd& car, const Eigen::Matrix<double, 7, Eigen::Dynamic>& path, frentPoint& carFrentPoint, int& index) {
		int minIndex;
		findClosestPoint(car, path, minIndex);
		const double dx = car(0) - path(0,minIndex);
		const double dy = car(1) - path(1,minIndex);

		const double cos_theta_r = std::cos(path(3,minIndex));
		const double sin_theta_r = std::sin(path(3,minIndex));

		const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
		// carFrentPoint.d = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
		carFrentPoint.d = std::copysign(cross_rd_nd, cross_rd_nd);
		carFrentPoint.s = path(6,minIndex);
		index = minIndex;
		return 0;
	}
	int getCarPosition(const double gpsX, const double gpsY, const double heading, const double speed, Eigen::VectorXd& car){
		//定义车及障碍物属性x, y, v, heading, R = L =2.647
		// car.resize(5,1);
		car(0) = gpsX - x0;
		car(1) = gpsY - y0;
		car(2) = speed;
		car(3) = 90 - heading;
		car(4) = 2.647;
		// std::cout << "Positioning successful!" << car(3)<<"heading" <<heading<< std::endl;
		return 0;
	}
	double d2r(double theta){
		return theta * M_PI / 180;
	}

	double r2d(double theta){
		return theta * 180 / M_PI;
	}

	void get_car_fourpoint(double length,double wigth,Eigen::VectorXd car_center,Eigen::MatrixXd& car_Fpoint){
		/*
            2-------3
            |       |
            1-------4
			车子的四个顶点 
        */
		car_Fpoint.resize(2,4);
		double thea = car_center[2]; //转换
        double zhouju = 1.308;
        double cos_thea = cos(thea);
        double sin_thea = sin(thea);
        double term0 = car_center[0] + zhouju * cos_thea;
        double term1 = car_center[1] + zhouju * sin_thea;//恢复到车体的中心 
		//1
		car_Fpoint(0,0) = (-length/2)*cos_thea-(-wigth/2)*sin_thea+term0;
		car_Fpoint(1,0) = (-length/2)*sin_thea+(-wigth/2)*cos_thea+term1;

		//2
		car_Fpoint(0,1) = (-length/2)*cos_thea-(wigth/2)*sin_thea+term0;
		car_Fpoint(1,1) = (-length/2)*sin_thea+(wigth/2)*cos_thea+term1;

		//3
		car_Fpoint(0,2) = (length/2)*cos_thea-(wigth/2)*sin_thea+term0;
		car_Fpoint(1,2) = (length/2)*sin_thea+(wigth/2)*cos_thea+term1;

		//4
		car_Fpoint(0,3) = (length/2)*cos_thea-(-wigth/2)*sin_thea+term0;
		car_Fpoint(1,3) = (length/2)*sin_thea+(-wigth/2)*cos_thea+term1;
	}

	void get_car_fourpoint_sl(Eigen::MatrixXd& car_Fpoint, int index, std::vector<std::pair<double, double>> &car_Fpoint_sl, Eigen::MatrixXd& globalpath){
		size_t start_index = std::max(index - 40, 0);
        size_t ebd_index = std::min(index+ 200,static_cast<int>(globalpath.cols()));
		for (size_t i = 0; i < 4; i++) { //每一个顶点寻找最小的编号 
			double x = car_Fpoint(0, i);
			double y = car_Fpoint(1, i);
			double x_diff, y_diff;
			double mindistance, d_min = std::numeric_limits<double>::max();
			size_t minindex;
			for (size_t j = start_index; j < ebd_index; ++j){
				x_diff = x - globalpath(0, j);
				y_diff = y - globalpath(1, j);
				mindistance = x_diff * x_diff + y_diff * y_diff;
				if (mindistance < d_min) {
					d_min = mindistance;
					minindex= j;
				}
			}
			// sl 
			std::pair<double, double> sl;
			double dx = x - globalpath(0, minindex);
			double dy = y - globalpath(1, minindex);
			double cos_theta_r = std::cos(globalpath(3,minindex));
			double sin_theta_r = std::sin(globalpath(3,minindex));
			double ref_s = globalpath(6, minindex);
			double path_s = dx * cos_theta_r + dy * sin_theta_r + ref_s;
			double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
			sl.second = std::copysign(cross_rd_nd, cross_rd_nd);
			sl.first = std::abs(path_s);
			car_Fpoint_sl.push_back(sl);
		}
	}

	bool IsPointInRect(std::vector<std::pair<double, double>> &car_Fpoint_sl, const  std::vector<std::tuple<std::pair<double, double>,
					std::pair<double, double>, std::pair<double, double>, std::pair<double, double>>> &obstacle) {
		if (obstacle.empty()) return false;
		double car_max_s = std::max({car_Fpoint_sl[0].first, car_Fpoint_sl[1].first,car_Fpoint_sl[2].first,car_Fpoint_sl[3].first});
		double car_min_s = std::min({car_Fpoint_sl[0].first, car_Fpoint_sl[1].first,car_Fpoint_sl[2].first,car_Fpoint_sl[3].first}); 
		double car_max_l = std::max({car_Fpoint_sl[0].second, car_Fpoint_sl[1].second, car_Fpoint_sl[2].second, car_Fpoint_sl[3].second});
    	double car_min_l = std::min({car_Fpoint_sl[0].second, car_Fpoint_sl[1].second, car_Fpoint_sl[2].second, car_Fpoint_sl[3].second});
		for (const auto &p : car_Fpoint_sl) {
			double path_s = p.first;
			double path_l = p.second;
			for (const auto &obs : obstacle) {
				double obs_max_s = std::max({std::get<0>(obs).first,std::get<1>(obs).first,std::get<2>(obs).first,std::get<3>(obs).first});
				double obs_min_s = std::min({std::get<0>(obs).first,std::get<1>(obs).first,std::get<2>(obs).first,std::get<3>(obs).first});
				double obs_max_l = std::max({std::get<0>(obs).second, std::get<1>(obs).second, std::get<2>(obs).second, std::get<3>(obs).second});
        		double obs_min_l = std::min({std::get<0>(obs).second, std::get<1>(obs).second, std::get<2>(obs).second, std::get<3>(obs).second});
				   // 如果车辆的包围框和障碍物的包围框没有交集，跳过
				if (car_max_s < obs_min_s || car_min_s > obs_max_s || car_max_l < obs_min_l || car_min_l > obs_max_l) {
					continue;
				}
 				Eigen::Vector2d vec1_p1(path_s - std::get<0>(obs).first, path_l - std::get<0>(obs).second);
				Eigen::Vector2d vec1_p2(path_s - std::get<1>(obs).first, path_l - std::get<1>(obs).second);
				Eigen::Vector2d vec1_p3(path_s - std::get<2>(obs).first, path_l - std::get<2>(obs).second);
				Eigen::Vector2d vec1_p4(path_s - std::get<3>(obs).first, path_l - std::get<3>(obs).second);

				Eigen::Vector2d vec12(std::get<1>(obs).first - std::get<0>(obs).first, std::get<1>(obs).second - std::get<0>(obs).second);
				Eigen::Vector2d vec23(std::get<2>(obs).first - std::get<1>(obs).first, std::get<2>(obs).second - std::get<1>(obs).second);
				Eigen::Vector2d vec34(std::get<3>(obs).first - std::get<2>(obs).first, std::get<3>(obs).second - std::get<2>(obs).second);
				Eigen::Vector2d vec41(std::get<0>(obs).first - std::get<3>(obs).first, std::get<0>(obs).second - std::get<3>(obs).second);

				if (PointInRect(vec12, vec23, vec34, vec41, vec1_p1, vec1_p2, vec1_p3, vec1_p4)) {
					return true;
				}
			}
		}
		for (const auto &obs : obstacle) { //总的障碍物的个数
			double obs_max_s = std::max({std::get<0>(obs).first,std::get<1>(obs).first,std::get<2>(obs).first,std::get<3>(obs).first});
			double obs_min_s = std::min({std::get<0>(obs).first,std::get<1>(obs).first,std::get<2>(obs).first,std::get<3>(obs).first});
			double obs_max_l = std::max({std::get<0>(obs).second, std::get<1>(obs).second, std::get<2>(obs).second, std::get<3>(obs).second});
			double obs_min_l = std::min({std::get<0>(obs).second, std::get<1>(obs).second, std::get<2>(obs).second, std::get<3>(obs).second});
				// 如果车辆的包围框和障碍物的包围框没有交集，跳过
			if (car_max_s < obs_min_s || car_min_s > obs_max_s || car_max_l < obs_min_l || car_min_l > obs_max_l) {
				continue;
			}
			for (size_t i = 0; i < 4; i++){ //每个障碍物的四个顶点 
				double obs_s ; //障碍物的S
				double obs_l ;//障碍物的L
				switch (i){
					case 0:
						obs_s = std::get<0>(obs).first; //障碍物的S
						obs_l = std::get<0>(obs).second;//障碍物的L
						break;
					case 1:
						obs_s = std::get<1>(obs).first; //障碍物的S
						obs_l = std::get<1>(obs).second;//障碍物的L
						break;
					case 2:
						obs_s = std::get<2>(obs).first; //障碍物的S
						obs_l = std::get<2>(obs).second;//障碍物的L
						break;
					case 3:
						obs_s = std::get<3>(obs).first; //障碍物的S
						obs_l = std::get<3>(obs).second;//障碍物的L
						break;
					default:
						break;
				}
				Eigen::Vector2d vec1_p1(obs_s - car_Fpoint_sl[0].first, obs_l - car_Fpoint_sl[0].second);
				Eigen::Vector2d vec1_p2(obs_s - car_Fpoint_sl[1].first, obs_l - car_Fpoint_sl[1].second);
				Eigen::Vector2d vec1_p3(obs_s - car_Fpoint_sl[2].first, obs_l - car_Fpoint_sl[2].second);
				Eigen::Vector2d vec1_p4(obs_s - car_Fpoint_sl[3].first, obs_l - car_Fpoint_sl[3].second);

				Eigen::Vector2d vec12(car_Fpoint_sl[1].first - car_Fpoint_sl[0].first, car_Fpoint_sl[1].second - car_Fpoint_sl[0].second);
				Eigen::Vector2d vec23(car_Fpoint_sl[2].first - car_Fpoint_sl[1].first, car_Fpoint_sl[2].second - car_Fpoint_sl[1].second);
				Eigen::Vector2d vec34(car_Fpoint_sl[3].first - car_Fpoint_sl[2].first, car_Fpoint_sl[3].second - car_Fpoint_sl[2].second);
				Eigen::Vector2d vec41(car_Fpoint_sl[0].first - car_Fpoint_sl[3].first, car_Fpoint_sl[0].second - car_Fpoint_sl[3].second);
				if (PointInRect(vec12, vec23, vec34, vec41, vec1_p1, vec1_p2, vec1_p3, vec1_p4)) {
					return true;
				}
			}
			
		}
		return false;
	}

	//判断点是否在多边形的内部 ，如果在内部的 叉乘应该全小于0 
	bool PointInRect(Eigen::Vector2d &vec12, Eigen::Vector2d &vec23, Eigen::Vector2d &vec34, Eigen::Vector2d &vec41
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
	bool HasOverlapUseSl(std::vector<std::pair<double, double>> &car_Fpoint_sl, std::vector<std::tuple<std::pair<double, double>,
					std::pair<double, double>, std::pair<double, double>, std::pair<double, double>>> obstaclelist) {
		return IsPointInRect(car_Fpoint_sl, obstaclelist);
	}



	//每一个点与所有障碍物进行比较 这样如果有碰撞 速度会加快 
    bool HasOverlap(Eigen::MatrixXd &car_point,Eigen::VectorXd &obs, double thea){
        if(obs.size() == 0){
            return false;
        }
        double car_x_min,car_y_min,car_x_max,car_y_max,car_centre_x,car_centre_y,car_half_length,car_half_width;
        car_half_length = 2.266 ; car_half_width = 0.907;
        double obs_x_min,obs_y_min,obs_x_max,obs_y_max,obs_centre_x,obs_centre_y,obs_half_length,obs_half_width;    
        //计算车辆的相关信息 最大x y 最小 x y 中心点坐标 
        car_x_min = std::min({car_point(0,0),car_point(0,1),car_point(0,2),car_point(0,3)});
        car_y_min = std::min({car_point(1,0),car_point(1,1),car_point(1,2),car_point(1,3)});
        car_x_max = std::max({car_point(0,0),car_point(0,1),car_point(0,2),car_point(0,3)});
        car_y_max = std::max({car_point(1,0),car_point(1,1),car_point(1,2),car_point(1,3)});
        car_centre_x = (car_point(0,0)+car_point(0,1)+car_point(0,2)+car_point(0,3))/4;
        car_centre_y = (car_point(1,0)+car_point(1,1)+car_point(1,2)+car_point(1,3))/4;
        double cos_heading_ = std::cos(thea);    
        double sin_heading_ = std::sin(thea);
        
            double obs_1_x = obs(0);
            double obs_1_y = obs(1);

            double obs_2_x = obs(0);
            double obs_2_y = obs(3);

            double obs_3_x = obs(2);
            double obs_3_y = obs(3);

            double obs_4_x = obs(2);
            double obs_4_y = obs(1);

            obs_x_min = std::min({obs_1_x,obs_2_x,obs_3_x,obs_4_x});
            obs_y_min = std::min({obs_1_y,obs_2_y,obs_3_y,obs_4_y});
            obs_x_max = std::max({obs_1_x,obs_2_x,obs_3_x,obs_4_x});
            obs_y_max = std::max({obs_1_y,obs_2_y,obs_3_y,obs_4_y});
            obs_centre_x = (obs_1_x+obs_2_x+obs_3_x+obs_4_x)/4;
            obs_centre_y = (obs_1_y+obs_2_y+obs_3_y+obs_4_y)/4;
            obs_half_length = std::abs(obs_4_x-obs_1_x)/2;
            obs_half_width  = std::abs(obs_2_y-obs_1_y)/2;
            double obs_cos_heading = std::cos(0);
            double obs_sin_heading = std::sin(0);
			if (obs_x_max < car_x_min || obs_x_min > car_x_max ||
				obs_y_max < car_y_min || obs_y_min > car_y_max){
				return false;
            }else{
                const double shift_x = obs_centre_x - car_centre_x;
                const double shift_y = obs_centre_y - car_centre_y;
                const double dx1 = cos_heading_ * car_half_length;
                const double dy1 = sin_heading_ * car_half_length;
                const double dx2 = sin_heading_ * car_half_width;
                const double dy2 = -cos_heading_ * car_half_width;

                const double dx3 = obs_cos_heading * obs_half_length;
                const double dy3 = obs_sin_heading * obs_half_length;
                const double dx4 = obs_sin_heading * obs_half_width;
                const double dy4 = -obs_cos_heading* obs_half_width;

                //f3
                if(std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
                            std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) +
                                std::abs(dx4 * cos_heading_ + dy4 * sin_heading_) +
                                car_half_length &&
                //f4
                        std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
                            std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) +
                                std::abs(dx4 * sin_heading_ - dy4 * cos_heading_) +
                                car_half_width &&
                //f5
                        std::abs(shift_x * obs_cos_heading + shift_y * obs_sin_heading) <=
                            std::abs(dx1 *obs_cos_heading + dy1 * obs_sin_heading) +
                                std::abs(dx2 * obs_cos_heading + dy2 * obs_sin_heading) +
                                obs_half_length &&
                //f6
                        std::abs(shift_x * obs_sin_heading - shift_y * obs_cos_heading) <=
                            std::abs(dx1 * obs_sin_heading - dy1 * obs_cos_heading) +
                                std::abs(dx2 * obs_sin_heading - dy2 * obs_cos_heading) +
                                obs_half_width){
                    return true;
                }else{
                    return false;
                }
            }
        
        return false;
    }
	
	//面积比较的方法进行碰撞检测  carpoint 为局部路径上的点 obses为障碍物上的坐标 
	int Area_Comparison(std::vector<Eigen::MatrixXd> carpoint,std::vector<Eigen::VectorXd> obses)
	{
		// std::cout<<"carpoint: "<<carpoint.size()<<std::endl;
        // std::cout<<"obses: "<<obses.size()<<std::endl;
        if(obses.size()==0){
            return 0;
        }
		for (size_t i = 0; i < carpoint.size(); ++i){
			for (size_t j = 0; j < obses.size(); ++j){
				// 获取障碍物包围盒的坐标 min_x min_y   max_x  max_y 四个顶点坐标
				/*  3-------2
					|       |
					4-------1*/
				double obs_1_x = obses[j](0);
				double obs_1_y = obses[j](1);

				double obs_2_x = obses[j](0);
				double obs_2_y = obses[j](3);

				double obs_3_x = obses[j](2);
				double obs_3_y = obses[j](3);

				double obs_4_x = obses[j](2);
				double obs_4_y = obses[j](1);
				
				double area = std::abs(obs_2_y-obs_1_y)*std::abs(obs_3_x-obs_2_x);//障碍物面积
				double l1_2 = std::sqrt(std::pow(obs_2_x-obs_1_x,2)+std::pow(obs_2_y-obs_1_y,2));
				double l2_3 = std::sqrt(std::pow(obs_3_x-obs_2_x,2)+std::pow(obs_3_y-obs_2_y,2));
				double l3_4 = std::sqrt(std::pow(obs_4_x-obs_3_x,2)+std::pow(obs_4_y-obs_3_y,2));
				double l4_1 = std::sqrt(std::pow(obs_1_x-obs_4_x,2)+std::pow(obs_1_y-obs_4_y,2));
				//对车辆的四个点进行判断 是否在障碍物相交 
				for (size_t k = 0; k < 4; k++){
					double L1 = std::sqrt(std::pow(carpoint[i](0,k)-obs_1_x,2)+std::pow(carpoint[i](1,k)-obs_1_y,2));
					double L2 = std::sqrt(std::pow(carpoint[i](0,k)-obs_2_x,2)+std::pow(carpoint[i](1,k)-obs_2_y,2));
					double L3 = std::sqrt(std::pow(carpoint[i](0,k)-obs_3_x,2)+std::pow(carpoint[i](1,k)-obs_3_y,2));
					double L4 = std::sqrt(std::pow(carpoint[i](0,k)-obs_4_x,2)+std::pow(carpoint[i](1,k)-obs_4_y,2));

					//计算三角形的面积之和 
					double p12 = (L1+L2+l1_2)/2;
					double s12 = std::sqrt(p12*(p12-L1)*(p12-L2)*(p12-l1_2));

					double p23 = (L2+L3+l2_3)/2;
					double s23 = std::sqrt(p23*(p23-L3)*(p23-L2)*(p23-l2_3));

					double p34 = (L3+L4+l3_4)/2;
					double s34 = std::sqrt(p34*(p34-L3)*(p34-L4)*(p34-l3_4));

					double p41 = (L1+L4+l4_1)/2;
					double s41= std::sqrt(p41*(p41-L1)*(p41-L4)*(p41-l4_1));
                    //没有碰撞 
					if ((s12+s23+s34+s41)>area){
						continue;
					}
					else
					{return 1;}					
				}				
			}
		}
        return 0;
	}

	//使用向量的方法计算点是否在矩形内部进行碰撞检测 
	// 检测点是否在矩形内部的函数
	bool isPointInsideRectangle(PointF point, PointF* vertexArray) {
		// 判断点是否在矩形内部
		float cross1, cross2;

		// 判断点是否在v[0]v[1]和v[2]v[3]两边的同侧
		cross1 = (point.X - vertexArray[0].X) * (vertexArray[1].Y - vertexArray[0].Y) - (point.Y - vertexArray[0].Y) * (vertexArray[1].X - vertexArray[0].X);
		cross2 = (point.X - vertexArray[3].X) * (vertexArray[2].Y - vertexArray[3].Y) - (point.Y - vertexArray[3].Y) * (vertexArray[2].X - vertexArray[3].X);

		if ((cross1 * cross2) > 0 || (cross1 == 0 && cross2 > 0) || (cross2 == 0 && cross1 > 0))
			return false;

		// 判断点是否在v[0]v[3]和v[1]v[2]两边的同侧
		cross1 = (point.X - vertexArray[0].X) * (vertexArray[3].Y - vertexArray[0].Y) - (point.Y - vertexArray[0].Y) * (vertexArray[3].X - vertexArray[0].X);
		cross2 = (point.X - vertexArray[1].X) * (vertexArray[2].Y - vertexArray[1].Y) - (point.Y - vertexArray[1].Y) * (vertexArray[2].X - vertexArray[1].X);

		if ((cross1 * cross2 > 0) || (cross1 == 0 && cross2 > 0) || (cross2 == 0 && cross1 > 0))
			return false;
    return true;
	}

	bool isPointInsideRectangle(const Eigen::Vector2d& point, const Eigen::Vector2d* vertexArray) {
		float cross1, cross2;
		cross1 = (point.x() - vertexArray[0].x()) * (vertexArray[1].y() - vertexArray[0].y()) - (point.y() - vertexArray[0].y()) * (vertexArray[1].x() - vertexArray[0].x());
		cross2 = (point.x() - vertexArray[3].x()) * (vertexArray[2].y() - vertexArray[3].y()) - (point.y() - vertexArray[3].y()) * (vertexArray[2].x() - vertexArray[3].x());

		if ((cross1 * cross2) > 0 || (cross1 == 0 && cross2 > 0) || (cross2 == 0 && cross1 > 0))
			return false;

		cross1 = (point.x() - vertexArray[0].x()) * (vertexArray[3].y() - vertexArray[0].y()) - (point.y() - vertexArray[0].y()) * (vertexArray[3].x() - vertexArray[0].x());
		cross2 = (point.x() - vertexArray[1].x()) * (vertexArray[2].y() - vertexArray[1].y()) - (point.y() - vertexArray[1].y()) * (vertexArray[2].x() - vertexArray[1].x());

		if ((cross1 * cross2 > 0) || (cross1 == 0 && cross2 > 0) || (cross2 == 0 && cross1 > 0))
			return false;

    return true;
	}

	int Rectanglecollisions(std::vector<Eigen::MatrixXd> carpoint,std::vector<Eigen::VectorXd> obses){
	// 获取障碍物包围盒的坐标 min_x min_y   max_x  max_y 四个顶点坐标
	/*  3-------2
		|       |
		4-------1*/
		if (obses.size()==0){
			return 0;
		}
		Eigen::Vector2d carcentrepoint;
		Eigen::Vector2d obscentrepoint;
		Eigen::Vector2d vertexArray[4] ;
		int flag = 0;
		//omp_set_num_threads(4); // 设置线程数为4
		
		for (size_t i = 0; i < carpoint.size(); ++i)
		{	
			carcentrepoint = (carpoint[i].col(0) + carpoint[i].col(3)) / 2.0;
			Eigen::Vector2d carSinglepoint_;
			for (size_t j = 0; j < obses.size(); ++j)
			{
				//计算障碍物中心
				obscentrepoint(0) = (obses[j](0)+obses[j](2))/2;
				obscentrepoint(1) = (obses[j](1)+obses[j](3))/2;
				if ((carcentrepoint - obscentrepoint).squaredNorm() > 25) {//大于5m 不进行碰撞检测 
            		continue;
        		}
				vertexArray[0] = obses[j].segment<2>(0);
				vertexArray[1] = {obses[j](0), obses[j](3)};
				vertexArray[2]=obses[j].tail<2>();;
				vertexArray[3] = {obses[j](2), obses[j](1)};
				//四个顶点 
				int local_flag = 0;
				for (size_t k = 0; k < 4; k++)
				{
					carSinglepoint_ = carpoint[i].col(k);
					//true 在内部 false 不在内部 
					bool ecollisionsflag = isPointInsideRectangle(carSinglepoint_,vertexArray);					
					if(ecollisionsflag==true){
							return 1;	
						}		
					}

			}
		}		
		return 0;
	}	
}