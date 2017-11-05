/*
 * @file		Kinematics.h
 * @brief		Calculation Kinematics
 * @author	Ryu Yamamoto
 * @date		2016/03/02
 */

#ifndef _KINEMATICS_
#define _KINEMATICS_

#include "Link.h"

namespace MotionControl
{
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
		MatrixXd calcJacobian(vector<int> idx); 
		bool calcInverseKinematics(int to, int base, Link target);
		vector<int> FindRoute(int to, int base);
		Matrix<double,3,3> Rodrigues(Matrix<double,3,1> a, double q);
		Matrix<double,3,1> rot2omega(Matrix<double,3,3> R);
		Matrix<double,6,1> calcVWerr(Link Cref, Link Cnow);
		template <typename t_matrix> t_matrix 
		PseudoInverse(const t_matrix& m, const double &tolerance=1.e-6);
	};
};

#endif
