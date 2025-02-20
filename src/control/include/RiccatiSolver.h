#ifndef RICCATI_SOLVER_H_
#define	RICCATI_SOLVER_H_

#include <Eigen/Dense>
#include <iostream>
#include <time.h>
#include <vector>
using namespace Eigen;

/* ����������ģ�� */
bool solveRiccatiIterationC(const MatrixXd& A, const MatrixXd& B,
	const MatrixXd& Q, const MatrixXd& R,
	MatrixXd& P, const double dt = 0.001,
	const double& tolerance = 1.E-2,
	const unsigned int iter_max = 1000)
{
	P = Q; // initialize

	MatrixXd P_next;

	MatrixXd AT = A.transpose();
	MatrixXd BT = B.transpose();
	MatrixXd Rinv = R.inverse();

	double diff;
	for (unsigned int i = 0; i < iter_max; ++i) {
		P_next = P + (P * A + AT * P - P * B * Rinv * BT * P + Q) * dt;
		diff = fabs((P_next - P).maxCoeff());
		P = P_next;
		if (diff < tolerance) {
			// std::cout << "iteration mumber = " << i << std::endl;
			return true;
		}
	}
	return false; // over iteration limit
}

/* ��������ɢģ�� */
bool solveRiccatiIterationD(const MatrixXd& Ad,
	const MatrixXd& Bd, const MatrixXd& Q,
	const MatrixXd& R, MatrixXd& P,
	const double& tolerance = 1.E-2,
	const unsigned int iter_max = 1000)
{
	P = Q; // initialize

	MatrixXd P_next;

	MatrixXd AdT = Ad.transpose();
	MatrixXd BdT = Bd.transpose();
	MatrixXd Rinv = R.inverse();

	double diff;
	for (unsigned int i = 0; i < iter_max; ++i) {
		// -- discrete solver --
		P_next = AdT * P * Ad -
			AdT * P * Bd * (R + BdT * P * Bd).inverse() * BdT * P * Ad + Q;

		diff = fabs((P_next - P).maxCoeff());
		P = P_next;
		if (diff < tolerance) {
			// std::cout << "iteration mumber = " << i << std::endl;
			return true;
		}
	}
	return false; // over iteration limit
}

/* ����ģ�͵�Arimoto Potter �㷨(matlabʹ��)*/
bool solveRiccatiArimotoPotter(const MatrixXd& A,
	const MatrixXd& B,
	const MatrixXd& Q,
	const MatrixXd& R, MatrixXd& P)
{
	const unsigned int dim_x = A.rows();
	const unsigned int dim_u = B.cols();

	// ���ܶپ���
	MatrixXd Ham = MatrixXd::Zero(2 * dim_x, 2 * dim_x);
	Ham << A, -B * R.inverse() * B.transpose(), -Q, -A.transpose();

	// ��������ֵ����������
	EigenSolver<MatrixXd> Eigs(Ham);

	// �������ֵ
	//std::cout << "eigen values��\n" << Eigs.eigenvalues() << std::endl;
	//std::cout << "eigen vectors��\n" << Eigs.eigenvectors() << std::endl;

	// extract stable eigenvectors into 'eigvec'
	MatrixXcd eigvec = MatrixXcd::Zero(2 * dim_x, dim_x);
	int j = 0;
	for (unsigned int i = 0; i < 2 * dim_x; ++i) {
		if (Eigs.eigenvalues()[i].real() < 0.) {
			eigvec.col(j) = Eigs.eigenvectors().block(0, i, 2 * dim_x, 1);
			++j;
		}
	}

	//std::cout << "eigvec==>\n" << eigvec  << std::endl;

	// calc P with stable eigen vector matrix
	MatrixXcd Vs_1, Vs_2;
	Vs_1 = eigvec.block(0, 0, dim_x, dim_x);
	Vs_2 = eigvec.block(dim_x, 0, dim_x, dim_x);
	P = (Vs_2 * Vs_1.inverse()).real();

	return true;
}


#endif	/* RICCATI_SOLVER_H_ */