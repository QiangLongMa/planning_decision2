#pragma once
#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> //与图像处理有关
#include <cmath>
#include <iostream>

namespace tool{
    const double pi = 3.14159265358;
    struct frentPoint {
        double s = -1;
        double d = 9999999;
    };

    struct states {
        double x0;
        double vx0;
        double xT;
        double vxT;
        double y0;
        double vy0;
        double yT;
        double vyT;
    };

    struct controlParams {
        const double d = 2;  //道路宽度3.5
        const double deltaT = 0.05; //deltaT控制周期,
        const double t = 0.0; //开始时刻   
        const int n = 5; //采样点个数
        double vEnd = 10; // 局部路径终点速度
        double tT = 2.5;// 局部路径完成时间
        double l = vEnd * tT; //前驱路径长度
        double lp = tT/ deltaT+1; //前驱路径点数
    };

    double distance(const Eigen::Matrix<double, 20, 2>& finalKeyPoint, int i, int j);  
    int straight(Eigen::Vector3d start, Eigen::Vector3d end, double deltaT, Eigen::Matrix<double, 5, Eigen::Dynamic>& path);
    int arc8(Eigen::Vector4d start, double R, double theta, double deltaT, Eigen::Matrix<double, 5, Eigen::Dynamic>& path);
    int arc7(Eigen::Vector4d start, double R, double theta, double deltaT, Eigen::Matrix<double, 5, Eigen::Dynamic>& path);
    int arc6(Eigen::Vector4d start, double R, double theta, double deltaT, Eigen::Matrix<double, 5, Eigen::Dynamic>& path);
    int arc5(Eigen::Vector4d start, double R, double theta, double deltaT, Eigen::Matrix<double, 5, Eigen::Dynamic>& path);
    int arc4(Eigen::Vector4d start, double R, double theta, double deltaT, Eigen::Matrix<double, 5, Eigen::Dynamic>& path);
    int arc3(Eigen::Vector4d start, double R, double theta, double deltaT, Eigen::Matrix<double, 5, Eigen::Dynamic>& path);
    int arc2(Eigen::Vector4d start, double R, double theta, double deltaT, Eigen::Matrix<double, 5, Eigen::Dynamic>& path);
    int arc(Eigen::Vector4d start, double R, double theta, double deltaT, Eigen::Matrix<double, 5, Eigen::Dynamic>& path);
    double lineEq(const cv::Point2d& start, const cv::Point2d& end, const cv::Point2d& inflect);
    int realPosition2imagePosition(const double& x, const double& y, Eigen::Matrix<double, 1, 2>& imagePosition);
    int imagePosition2realPosition(const double& x, const double& y, Eigen::Matrix<double, 1, 2>& realPosition);
    int referencePath(const Eigen::MatrixXd& xyv, std::vector< Eigen::Matrix<double, 6, Eigen::Dynamic> >& reTrajt) ;
    double normalizeAngle(double angle);
    int deleteRepeatedData(const Eigen::Matrix<double, 5, Eigen::Dynamic>& subPath, Eigen::Matrix<double, 5, Eigen::Dynamic>& pathTemp, int& j);
    void findClosestPoint(const Eigen::VectorXd& car, const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& path, int& minIndex);
    void findClosestPointArray(const Eigen::VectorXd& car, const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& path, int& minIndex);
    int binarySerach(const Eigen::MatrixXd set, double  value);
    int cartesianToFrenet(const Eigen::VectorXd& car, const Eigen::Matrix<double, 7, Eigen::Dynamic>& path, frentPoint& carFrentPoint, int& index);
    int getCarPosition(const double gpsX, const double gpsY, const double heading, const double speed, Eigen::VectorXd& car);
    void coordinateTransform(const Eigen::VectorXd& car, const Eigen::VectorXd& point_XYTheta, Eigen::VectorXd& point_car_xy);
    // int cartesian2Frenet_l(const Eigen::VectorXd& realPosition, const Eigen::MatrixXd& globalPath, tool::frentPoint& carFrentPoint, int& minIndex,int &carIndex, double offset);
    // int cartesian2Frenet_r(const Eigen::VectorXd& realPosition, const Eigen::MatrixXd& globalPath, tool::frentPoint& carFrentPoint, int& minIndex,int &carIndex, double offset);
    int frenet2Cartesian(const Eigen::MatrixXd& globalPath, const double& s, const double& d, const int& minIndex, Eigen::Vector2d& cartesianXY);
    double d2r(double theta);
    double r2d(double theta);    
}
