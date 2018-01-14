/*
 * @file		Kinematics.h
 * @brief		Calculation Kinematics
 * @author	Ryu Yamamoto
 * @date		2016/03/02
 */

#ifndef _KINEMATICS_
#define _KINEMATICS_

#include "Link.h"
#include "Jacobian.h"

class Kinematics
{
public:
	struct Link *ulink;
public:
	Kinematics(Link *ulink)
	{
		this->ulink = ulink;
	}
	void calcForwardKinematics(int rootlink);
	bool calcInverseKinematics(int to, Link target);
	Matrix<double,3,3> computeMatrixFromAngles(double r, double p, double y);
	void computeAnglesFromMatrix(Matrix<double,3,3> R, double &r, double &p, double &y);
	vector<int> FindRoute(int to);
	Matrix<double,3,3> Rodrigues(Matrix<double,3,1> a, double q);
	Matrix<double,3,1> rot2omega(Matrix<double,3,3> R);
	Matrix<double,6,1> calcVWerr(Link Cref, Link Cnow);
	template <typename t_matrix> t_matrix 
	PseudoInverse(const t_matrix& m, const double &tolerance=1.e-6);
};

#endif
