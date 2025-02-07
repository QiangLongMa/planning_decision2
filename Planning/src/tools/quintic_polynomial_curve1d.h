#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
using namespace std;
class quintic_polynomial_curve1d
{
private:
    double s0_, l0_, dl_, dll_, s1_, l1_;
    Eigen::Vector4d X_PARAM;

public:
    void setparam(double s0, double l0, double dl, double dll, double s1, double l1);
    void Calcurver(Eigen::Vector4d &X);
    double Evaluate(const int order, const double s);
};

class quintic_polynomial_curve5d
{

private:
    double s0_, l0_, dl_, dll_, s1_, l1_, dl1_, dll1_;
    Eigen::VectorXd X_PARAM;

public:
    void setparam(double s0, double l0, double dl, double dll, double s1, double l1, double dl1, double dll1);
    void Calcurver(Eigen::VectorXd &X);
    double Evaluate(const int order, const double s);

    void ComputeCoefficients(Eigen::VectorXd &X);
};
