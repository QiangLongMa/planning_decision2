#include"toolKits.h"
/******************************************
 * 1.与Ver4相比
 * 增加了由x,y,计算theta和kappa函数
 **************************************** */
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
		static int startIndex = 0;
		static int end = path.cols();
		double distance, d_min = 999999;
		int i = startIndex;
		for (; i < end; i++) {
			distance = abs(path(0,i) - car(0)) + abs(path(1,i) - car(1));
			if (distance < d_min) {
				minIndex = i;
				d_min = distance;
				startIndex = i;
			}
		}
		startIndex = startIndex - 10 < 0 ? 0 : startIndex;
		end = startIndex + 30;
		end = end - path.cols() < 0 ? end : path.cols();
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

	void coordinateTransform(const Eigen::VectorXd& car, const Eigen::VectorXd& point_XYTheta, Eigen::VectorXd& point_car_xy){
		Eigen::Matrix2d Tr ;
		double oh = normalizeAngle(d2r(car(2)) - pi/2);
		Tr << cos(oh), -sin(oh), sin(oh), cos(oh);
		Eigen::Vector2d Tv, point_xy;
		Tv << point_XYTheta(0)-car(0), point_XYTheta(1) - car(1);
    	point_xy << Tr.inverse() * Tv;
		point_car_xy.resize(2, 1);
		point_car_xy  << point_xy(0), point_xy(1);
	}
	
	int getCarPosition(const double gpsX, const double gpsY, const double heading, const double speed, Eigen::VectorXd& car){
		//定义车及障碍物属性x, y, v, heading, R = L =2.647
		car(0) = gpsX - x0;
		car(1) = gpsY - y0;
		car(2) = speed;
		car(3) = 90 - heading;
		car(4) = 2.647;
		return 0;
	}

	// int cartesian2Frenet_l(const Eigen::VectorXd& realPosition, const Eigen::MatrixXd& globalPath, tool::frentPoint& carFrentPoint, int& minIndex,int &carIndex, double offset){
	// 	findClosestPoint(realPosition, globalPath, minIndex);
	// 	carIndex = minIndex;
    //     //std::cout <<"minIndex:" << minIndex << std::endl;
    //     //std::cout << "carIndex:" << carIndex << std::endl;
    //     const double dx = realPosition(0) - globalPath(0, minIndex);
    //     const double dy = realPosition(1) - globalPath(1, minIndex);
    //     const double cos_theta_r = std::cos(globalPath(3, minIndex));
    //     const double sin_theta_r = std::sin(globalPath(3, minIndex));
    //     const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
    //     //carFrentPoint.d = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
    //     carFrentPoint.d = cross_rd_nd + offset;
    //     carFrentPoint.s = globalPath(6, minIndex);
    //     return 0;
    // }

	// int cartesian2Frenet_r(const Eigen::VectorXd& realPosition, const Eigen::MatrixXd& globalPath, tool::frentPoint& carFrentPoint, int& minIndex,int &carIndex, double offset){
	// 	findClosestPoint(realPosition, globalPath, minIndex);
	// 	carIndex = minIndex;
    //     //std::cout <<"minIndex:" << minIndex << std::endl;
    //     //std::cout << "carIndex:" << carIndex << std::endl;
    //     const double dx = realPosition(0) - globalPath(0, minIndex);
    //     const double dy = realPosition(1) - globalPath(1, minIndex);
    //     const double cos_theta_r = std::cos(globalPath(3, minIndex));
    //     const double sin_theta_r = std::sin(globalPath(3, minIndex));
    //     const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
    //     //carFrentPoint.d = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
    //     carFrentPoint.d = cross_rd_nd + offset;
    //     carFrentPoint.s = globalPath(6, minIndex);
    //     return 0;
    // }


    int frenet2Cartesian(const Eigen::MatrixXd& globalPath, const double& s, const double& d, const int& minIndex, Eigen::Vector2d& cartesianXY){

        double cos_theta_r = cos(globalPath(3, minIndex));
        double sin_theta_r = sin(globalPath(3, minIndex));
        cartesianXY(0) = globalPath(0, minIndex) - sin_theta_r * d;
        cartesianXY(1) = globalPath(1, minIndex) + cos_theta_r * d;	
        return 0;
    }

	double d2r(double theta){
		return theta * M_PI / 180;
	}

	double r2d(double theta){
		return theta * 180 / M_PI;
	}

}