#include"quintic_polynomial_curve1d.h"

void quintic_polynomial_curve1d::setparam(double s0,double l0, double dl,double dll,double s1 ,double l1){
    s0_=s0;
    l0_=l0;
    dl_=dl;
    dll_=dl;
    s1_=s1;
    l1_=l1;
}
void quintic_polynomial_curve1d::Calcurver(Eigen::Vector4d &X){
    Eigen::Matrix4d A;
    //Eigen::Vector4d X;
    Eigen::Vector4d B;
    A<< 1 ,s0_,s0_*s0_,s0_*s0_*s0_,
        1, s1_,s1_*s1_,s1_*s1_*s1_,
        0,  1 , 2*s0_ ,3*s0_*s0_,
        0,   0 ,  2   , 6*s0_;
    B<< l0_ , l1_, dl_,dll_;
   
    X =  A.colPivHouseholderQr().solve(B);
    X_PARAM = X;
   
}

//计算相应的几阶导数
double quintic_polynomial_curve1d::Evaluate(const int order,const double s){
    switch (order)
    {
    case 0:
        return(X_PARAM(0)+X_PARAM(1)*s+X_PARAM(2)*s*s+X_PARAM(3)*s*s*s);
    case 1:
        return (X_PARAM(1)+2*X_PARAM(2)*s+3*X_PARAM(3)*s*s);
    case 2:
        return(2*X_PARAM(2)+6*X_PARAM(3)*s);
    default:
        return 0;
    }
}



void quintic_polynomial_curve5d::setparam(double s0,double l0, double dl,double dll,double s1 ,double l1,double dl1,double dll1){
    s0_= s0;
    l0_= l0;
    dl_= dl;
    dll_= dll;

    s1_= s1;
    l1_= l1;
    dl1_ = dl1;
    dll1_ = dll1;
}
void quintic_polynomial_curve5d::Calcurver(Eigen::VectorXd &X){
    Eigen::MatrixXd A;
    A.resize(6,6);
    //Eigen::Vector4d X;
    Eigen::VectorXd B;
    B.resize(6);
    A<< 1,s0_,pow(s0_,2),pow(s0_,3),pow(s0_,4),pow(s0_,5),
        0,1  ,2*s0_,3*pow(s0_,2),4*pow(s0_,3),5*pow(s0_,4),
        0, 0, 2 , 6*s0_ , 12*pow(s0_,2), 20*pow(s0_,3),
        1,s1_,pow(s1_,2),pow(s1_,3),pow(s1_,4),pow(s1_,5),
        0,1  ,2*s1_,3*pow(s1_,2),4*pow(s1_,3),5*pow(s1_,4),
        0, 0, 2 , 6*s1_ , 12*pow(s1_,2), 20*pow(s1_,3);

    B<< l0_ ,dl_,dll_,l1_,dl1_,dll1_;
    X =  A.colPivHouseholderQr().solve(B);
    X_PARAM.resize(6);
    X_PARAM = X;
    //LU分解来求解线性方程组 
    // Eigen::MatrixXd A;
    // A.resize(6,6);
    // Eigen::VectorXd B;
    // B.resize(6);

    // A << 1, s0_, pow(s0_, 2), pow(s0_, 3), pow(s0_, 4), pow(s0_, 5),
    //     0, 1, 2 * s0_, 3 * pow(s0_, 2), 4 * pow(s0_, 3), 5 * pow(s0_, 4),
    //     0, 0, 2, 6 * s0_, 12 * pow(s0_, 2), 20 * pow(s0_, 3),
    //     1, s1_, pow(s1_, 2), pow(s1_, 3), pow(s1_, 4), pow(s1_, 5),
    //     0, 1, 2 * s1_, 3 * pow(s1_, 2), 4 * pow(s1_, 3), 5 * pow(s1_, 4),
    //     0, 0, 2, 6 * s1_, 12 * pow(s1_, 2), 20 * pow(s1_, 3);

    // B << l0_, dl_, dll_, l1_, dl1_, dll1_;

    // // Perform LU decomposition
    // Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(A);
    // if (lu_decomp.isInvertible()) {
    //     X = lu_decomp.solve(B);
    //     X_PARAM.resize(6);
    //     X_PARAM = X;
    // } else {
    // // Handle the case when A is singular
    // // You may need to implement error handling here
    // }
}
//计算相应的几阶导数

void quintic_polynomial_curve5d::ComputeCoefficients(Eigen::VectorXd &X){
    double p = s1_-s0_;
    X_PARAM.resize(6);
    X.resize(6);
    X[0] = l0_;
    X[1]=dl_;
    X[2]=dll_;
    const double p2 = p * p;
    const double p3 = p * p2;

    // the direct analytical method is at least 6 times faster than using matrix
    // inversion.
    const double c0 = (l1_ - 0.5 * p2 * dll_ - dl_ * p - l0_) / p3;
    const double c1 = (dl1_ - dll_ * p - dl_) / p2;
    const double c2 = (dll1_ - dll_) / p;

    X[3] = 0.5 * (20.0 * c0 - 8.0 * c1 + c2);
    X[4] = (-15.0 * c0 + 7.0 * c1 - c2) / p;
    X[5] = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / p2;
    X_PARAM =X;
}


double quintic_polynomial_curve5d::Evaluate(const int order,const double s){
    switch (order)
    {
    case 0:
        return(X_PARAM(0)+X_PARAM(1)*s+X_PARAM(2)*s*s+X_PARAM(3)*s*s*s+X_PARAM(4)*s*s*s*s+X_PARAM(5)*s*s*s*s*s);
    case 1:
        return (X_PARAM(1)+2*X_PARAM(2)*s+3*X_PARAM(3)*s*s+4*X_PARAM(4)*s*s*s+5*X_PARAM(5)*s*s*s*s);
    case 2:
        return(2*X_PARAM(2)+6*X_PARAM(3)*s+12*X_PARAM(4)*s*s+20*X_PARAM(5)*s*s*s);
    default:
        return 0;
    }
}








