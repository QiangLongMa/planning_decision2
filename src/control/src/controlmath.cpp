#include"controlmath.h"


namespace control_math {

    int findcloseindex (const Eigen::MatrixXd &path, int & closeindex, const double & preview_distance) {
        int startindex = std::max(0, closeindex - 20);
        int endindex = std::min(static_cast<int>(path.cols() - 1),  closeindex + 60);
        double distance = std::numeric_limits<double>::max();
        int minindex = 0;
        for (size_t i = startindex; i < endindex; ++i) {
            double s = std::abs(preview_distance - path(5, i));
            if (s < distance) {
                distance = s;
                minindex = i;
            }
        }
        return minindex;
    }

    inline double NormalizeAngle(double angle) {
		angle = std::fmod(angle, 2 * M_PI);
		if (angle > M_PI)
			angle -= 2 * M_PI;
		else if (angle < -M_PI)
			angle += 2 * M_PI;
		return angle;
    }

    inline double slerp(const double s0, const double s1, const double s, const double theta0,
                const double theta1) {
        const double a0_n = NormalizeAngle(theta0);
        const double a1_n = NormalizeAngle(theta1);
        const double weight = (s - s0) / (s1 - s0);
        double a = (1-weight)*a0_n + weight*a1_n;
        return NormalizeAngle(a);
    }

    inline double InterpolateUsingLinearApproximation(double s0,double s1,double s,double param0,double param1){
        double weight = (s - s0) / (s1 - s0);
        double dkappa = (1-weight)*param0 + weight*param1;
        return dkappa;
    }
    //线性插值 使用s
    void InterpolateUsingS (Eigen::MatrixXd &localpath, Eigen::MatrixXd &outputlocalpath){
        double delta_s = 0.1;
        double current_s = 0;
        double path_length = localpath(5, localpath.cols() - 1);
        std::vector<std::vector<double>> vec(localpath.cols(), std::vector<double>(localpath.rows()));
        for (size_t i = 0; i < localpath.cols(); ++i){
            std::copy(localpath.col(i).data(), localpath.col(i).data() + localpath.rows(), vec[i].begin());
        }
        
        std::vector<std::vector<double>> outputlocalpathlist;
        auto comp = [](std::vector<double> & point, const double & s){
                return point[5] < s;
            };
        while (current_s <= path_length){
            auto it_lower = std::lower_bound(vec.begin(), vec.end(), current_s, comp);//自定义的比较方式
            if (it_lower == vec.end()) {
                outputlocalpathlist.emplace_back(vec.back());
            } else if (it_lower == vec.begin()) {
                outputlocalpathlist.emplace_back(vec.front());
            } else {
                auto &p0 = *(it_lower -1);
                auto &p1 = *(it_lower);
                double s0 = p0[5];
                double s1 = p1[5];
               // 使用线性插值来计算每个变量
                std::vector<double> interpolated_point = {
                    InterpolateUsingLinearApproximation(s0, s1, current_s, p0[0], p1[0]),  // x
                    InterpolateUsingLinearApproximation(s0, s1, current_s, p0[1], p1[1]),  // y
                    InterpolateUsingLinearApproximation(s0, s1, current_s, p0[2], p1[2]),  // v
                    slerp(s0, s1, current_s, p0[3], p1[3]),                                   // theta
                    InterpolateUsingLinearApproximation(s0, s1, current_s, p0[4], p1[4]),  // dk
                    InterpolateUsingLinearApproximation(s0, s1, current_s, p0[5], p1[5]),  // ddk
                    InterpolateUsingLinearApproximation(s0, s1, current_s, p0[6], p1[6]),  // s
                    InterpolateUsingLinearApproximation(s0, s1, current_s, p0[7], p1[7]),  // a
                    InterpolateUsingLinearApproximation(s0, s1, current_s, p0[8], p1[8]),  // realtivetime
                    InterpolateUsingLinearApproximation(s0, s1, current_s, p0[9], p1[9])   // absolutetime
                };
                outputlocalpathlist.push_back(interpolated_point);  // 添加插值后的点
            }    
            current_s += delta_s;
        }
         // 创建 Eigen 矩阵，避免多余的内存拷贝
        int rows = outputlocalpathlist[0].size();
        int cols = outputlocalpathlist.size();
        outputlocalpath.resize(rows, cols);
        // 使用 Eigen::MatrixXd 直接赋值而不是使用 std::vector 扁平化
        for (int col = 0; col < cols; ++col) {
            for (int row = 0; row < rows; ++row) {
                outputlocalpath(row, col) = outputlocalpathlist[col][row];
            }
        }
    }
}