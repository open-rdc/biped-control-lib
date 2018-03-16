/*
 * @file		Link.h
 * @brief		Link Information
 * @author	Ryu Yamamoto
 * @date		2016/02/26
 */

#ifndef _LINK_
#define _LINK_

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "demoManipLink.h"

using namespace std;
using namespace Eigen;

struct Link
{
	string joint_name;					
	int sister;						
	int child;						
	int parent;						
	Matrix<double,3,1> p;	
	Matrix<double,3,3> R;	
	Matrix<double,3,1> v;	
	Matrix<double,3,1> w;	
	Matrix<double,3,1> a;	
	Matrix<double,3,1> b;	
	double q;							
	double dq;						
	double ddq;						
	double vartex;
	double face;
	double m;
	Matrix<double,3,1> c;
	Matrix<double,3,3> I;
	Link() : p(Matrix<double,3,1>::Zero()), R(Matrix<double,3,3>::Identity()),
					v(Matrix<double,3,1>::Zero()), w(Matrix<double,3,1>::Zero()),
					a(Matrix<double,3,1>::Zero()), b(Matrix<double,3,1>::Zero()),
					q(0.0), dq(0.0), ddq(0.0),
					c(Matrix<double,3,1>::Zero()), I(Matrix<double,3,3>::Identity()) 
	{
	}
};

extern "C" void SetJointInfo(struct Link *link);

#endif
