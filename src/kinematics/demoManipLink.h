/*
 * @file		LinkParameter.h
 * @brief		Constant Value of Link for Humanoid Robot
 * @author	Ryu Yamamoto
 * @date		2016/02/26
 */

#ifndef _LINK_PARAMETER_
#define _LINK_PARAMETER_

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <limits>
#include <boost/math/constants/constants.hpp>

using namespace std;
using namespace Eigen;

static const double eps = 1e-06;
static const double pi = boost::math::constants::pi<double>();

//JAXON
static const string joint_name[] = {
	"BASE",
	"ARM0",
	"ARM1",
	"ARM2",
	"ARM3",
	"ARM4",
	"ARM5",
};

enum
{
	BASE=0,
	ARM0,
	ARM1,
	ARM2,
	ARM3,
	ARM4,
	ARM5,
	JOINT_NUM
};

static const int parent[JOINT_NUM] = {-1,0,1,2,3,4,5};
static const int child[JOINT_NUM] = {1,2,3,4,5,6,-1};
static const int sister[JOINT_NUM] = {-1,-1,-1,-1,-1,-1,-1};

static const double LinkAxis[JOINT_NUM][3] = {
	//WASIT
	{0,0,0},

	//ARM
	{0,0,1},
	{0,1,0},
	{0,1,0},
	{0,0,1},
	{0,1,0},
	{0,0,1},
};

static const double LinkPos[JOINT_NUM][3] = {
	//WAIST
	{0.0f, 0.0f, 0.0f},	
	
	//ARM
	{0.0f, 0.0f, 0.02f},
	{0.0f, 0.0f, 0.12f},
	{0.0f, 0.0f, 0.12f},
	{0.0f, 0.0f, 0.0f},
	{0.0f, 0.0f, 0.12f},
	{0.0f, 0.0f, 0.06f},
};

#endif
