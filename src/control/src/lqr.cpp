#include"lqr.h"
#include "RiccatiSolver.h"

int lqr::stateSpaceMatrix(Eigen::Matrix<double, 3, 3>& A, Eigen::Matrix<double, 3, 2>& B, const Eigen::VectorXd& car,  tool::controlParams cps, 
                         double refDelta, const double refYaw) {
	A <<   1.0,     0.0,     -3 * sin(refYaw) * 0.02,
                0.0,     1.0,     3* cos(refYaw) * 0.02,
                0.0,     0.0,     1.0;
	B <<   cos(refYaw) * 0.02,                    0.0,
                sin(refYaw) * 0.02,                     0.0,
                tan(refDelta) * 0.02 / car(4),    3 * 0.02 / (car(4) * cos(refDelta) * cos(refDelta));
	return 0;
}

double lqr::lqrComputeCommand(double yaw, double delta_x ,double delta_y,double delta_yaw , 
            double curvature,double vel, double l, double dt,Eigen::MatrixXd& Q, Eigen::MatrixXd& R)
{
    double steer = 0.0;
    // MatrixXd Q = MatrixXd::Zero(3,3);  
    // MatrixXd R = MatrixXd::Zero(1,1);
    // Q << 1      , 0    , 0,
    //      0      , 1    , 0,
    //      0      , 0    , 1;
    // R << 1;
    // double curvature = match_point.path_point.kappa;
    // if(vel < 0) curvature = -curvature;
    double feed_forword = atan2(l * curvature, 1);
 
    MatrixXd A = MatrixXd::Zero(3, 3);
    A(0, 0) = 1.0;
    A(0, 2) = -vel*sin(yaw)*dt;
    A(1, 1) = 1;
    A(1, 2) = vel*cos(yaw)*dt;
    A(2, 2) = 1.0;
 
    MatrixXd B = MatrixXd::Zero(3,1);
    B(2, 0) = vel*dt/l/pow(cos(feed_forword),2);
    // double delta_x = x - match_point.path_point.x;
    // double delta_y = y - match_point.path_point.y;
    // double delta_yaw = NormalizeAngle(yaw - match_point.path_point.yaw);
    VectorXd dx(3); dx << delta_x, delta_y, delta_yaw;
 
    double eps = 0.01;
    double diff = std::numeric_limits<double>::max();
 
    MatrixXd P = Q;
    MatrixXd AT = A.transpose();
    MatrixXd BT = B.transpose();
    int num_iter = 0;
    while(num_iter++ < 1000 && diff > eps){
        MatrixXd Pn = AT * P * A - AT * P * B * (R + BT * P * B).inverse() * BT * P * A + Q;
        diff = ((Pn - P).array().abs()).maxCoeff();
        P = Pn;
    }
    MatrixXd feed_back = -((R + BT * P * B).inverse() * BT * P * A) * dx;
    steer = -(feed_back(0,0) + feed_forword);
    return steer;
}

// int  lqr::calcTrackError(const Eigen::VectorXd& car, const std::vector<tool::PathPoint>&  path, double& e, double& yaw, double& kappa, int& index) {
// 	//计算跟踪误差
// 	//x: 当前车辆的位置x, y: 当前车辆的位置y
// 	//寻找距目标点最近的参考轨迹点
// 	const double x = car.x;
// 	const double y = car.y;
// 	double distance, d_min;
// 	index = 0;
// 	d_min = pow(path[index].x - x, 2) + pow(path[index].y - y, 2);
// 	Eigen::Matrix<double, Eigen::Dynamic, 1 > delta_x, delta_y;
// 	delta_x.resize(path.size()), delta_y.resize(path.size());;
// 	for (int i = 0; i < path.size(); i++) {
// 		distance = pow(path[i].x - x, 2) + pow(path[i].y - y, 2);
// 		delta_x(i) = path[i].x - x;
// 		delta_y(i) = path[i].y - y;
// 		if (distance < d_min) {
// 			d_min = distance;
// 			index = i;
// 		}
// 	}
// 	yaw = path[index].theta;
// 	kappa = path[index].kappa;
// 	double angle = tool::normalizeAngle(yaw - atan2(delta_y(index), delta_x(index)));
// 	//e = d_min;  //误差
// 	e = sqrt(d_min);  //误差
// 	if (angle < 0)
// 		e *= -1;
// 	std::cout << "d_min:" << std::endl;
// 	std::cout << d_min << std::endl;
// 	return 0;
// }

Eigen::Vector2d lqr::calcU(const Eigen::VectorXd& car, const Eigen::MatrixXd&  path, int& minIndex,
                                         const Eigen::Matrix<double, 3, 3>& A, const Eigen::Matrix<double, 3, 2>& B,
                                         const Eigen::Matrix<double, 3, 3>& Q, const Eigen::Matrix<double, 2, 2>& R) {
	/*
		* LQR控制器
		* x为位置和航向误差
		*
	*/
	Eigen::Vector3d error_x;
	Eigen::Vector2d u;
	Eigen::MatrixXd K;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P;
	error_x << car(0) - path(0, minIndex), car(1) - path(1, minIndex), tool::normalizeAngle(tool::d2r(car(3)) - path(3, minIndex));
	solveRiccatiIterationD(A, B, Q, R, P, 0.01, 1000);
	K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
	u = -K * error_x;
	return u;
}



Eigen::Vector2d lqr::calcU(const Eigen::VectorXd& car, const Eigen::MatrixXd&  path, 
                                            std::array<double,5> &Projection_point_message,
                                         const Eigen::Matrix<double, 3, 3>& A, const Eigen::Matrix<double, 3, 2>& B,
                                         const Eigen::Matrix<double, 3, 3>& Q, const Eigen::Matrix<double, 2, 2>& R) {
	/*
    * LQR控制器
    * x为位置和航向误差
    *
	*/
	Eigen::Vector3d error_x;
	Eigen::Vector2d u;
	Eigen::MatrixXd K;
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P;
    error_x << car(0) -Projection_point_message[0], car(1) - Projection_point_message[1], 
                tool::normalizeAngle(tool::d2r(car(3)) - Projection_point_message[3]);
	solveRiccatiIterationD(A, B, Q, R, P, 1.E-2, 1000);
	K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
	u = -K * error_x;
	return u;
}