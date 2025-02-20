#include <fstream>
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <chrono>
#include <limits>

namespace control_math {

    inline double NormalizeAngle(double angle) ;
		

    inline double slerp(const double s0, const double s1, const double s, const double theta0,
                const double theta1) ;

    inline double InterpolateUsingLinearApproximation(double s0,double s1,double s,double param0,double param1);

    void InterpolateUsingS (Eigen::MatrixXd &localpath, Eigen::MatrixXd &outputlocalpath);

    int findcloseindex (const Eigen::MatrixXd &path, int & closeindex, const double & preview_distance);






}