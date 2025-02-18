// #include<iostream>
// #include<Eigen/Core>
// #include<Eigen/Dense>
// #include<Eigen/Eigen>
// #include<vector>


// class BSpline
// {
// private:
//     std::vector<std::tuple<double, double>> controlPoints;// 控制点
//     std::vector<double> knots; // 节点向量
//     Eigen::MatrixXd localpath;//局部路径  
// public:
//     BSpline(const Eigen::MatrixXd &localpath_);
//     Eigen::MatrixXd Process(const Eigen::MatrixXd &localpath_);
//     ~BSpline();
//     double basisFunction(int i, double t);
//     void generateKnots(const Eigen::MatrixXd &localpath_);
//     Eigen::MatrixXd generateCurve(int numPoints, const Eigen::MatrixXd &localpath_); 
// };

