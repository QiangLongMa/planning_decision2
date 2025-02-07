// #include"BSpline.h"





// BSpline::BSpline(const Eigen::MatrixXd &localpath_) : 
//     localpath(localpath_){
    

// }

// BSpline::~BSpline(){}

// Eigen::MatrixXd BSpline::Process(const Eigen::MatrixXd &localpath_) {
//     if(localpath_.cols() == 0){
//         return localpath_;
//     }
//     controlPoints.reserve(localpath_.cols());
//     for (size_t i = 0; i < localpath_.cols(); ++i){
//         controlPoints.emplace_back(std::make_tuple(localpath_(0,i), localpath_(1,i)));
//     }
//     // 创建非均匀节点向量
//     generateKnots(localpath_);





// }

// Eigen::MatrixXd BSpline::generateCurve(int numPoints, const Eigen::MatrixXd &localpath_){
//     Eigen::MatrixXd curve(numPoints, controlPoints.size()); // 存储生成的曲线点
//     double deltaT = 0.01;
//     double tStep = (knots.back() - knots.front()) / (numPoints - 1); // 计算t的步长
//     for (size_t i = 0; i < numPoints; ++i){
//         double t = knots.front() + i * tStep; // 当前参数值t


//         //计算基函数
//         for (size_t j = 0; i < 4; j++){
//             double basisval = basisFunction(i ,t);
//             double 
//         }
        


//     }
    


// }

// // 生成非均匀节点向量
// void BSpline::generateKnots(const Eigen::MatrixXd &localpath_){
//     int n = controlPoints.size() - 3;//段数
//     int degree = 3; // B样条的阶数
//     knots.resize(n + degree*2 +1); // 60个控制点 + 3（阶数） + 1

//     knots.push_back(0.0);
//     knots.push_back(0.0);
//     knots.push_back(0.0);
//     knots.push_back(0.0);
//     double L = localpath(5, localpath_.cols() - 1);
//     for (size_t i = 0; i < n; ++i){ //添加中间节点 
//         double length = localpath_(5, i + 3);
//         double t = length / L;
//     }
//     knots.push_back(1.0);
//     knots.push_back(1.0);
//     knots.push_back(1.0);
// }

// double BSpline::basisFunction(int i, double t){
//     double t_pow3 = t*t*t;
//     double t_pow = t*t;
//     double f1 = (-t_pow3 + 3*t_pow - 3*t + 1) / 6.0;
//     double f2 = (3*t_pow3 - 6*t_pow + 4) / 6.0;
//     double f3 = (-3*t_pow3 + 3*t_pow + 3*t + 1) / 6.0;
//     double f4 = t_pow3 / 6.0;

//     // 计算基函数的值
//     if (i == 0) return f1;
//     else if (i == 1) return f2;
//     else if (i == 2) return f3;
//     else return f4;
// }
