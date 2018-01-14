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
	"WAIST",
	"RLEG_JOINT0",
	"RLEG_JOINT1",
	"RLEG_JOINT2",
	"RLEG_JOINT3",
	"RLEG_JOINT4",
	"RLEG_JOINT5",
	"LLEG_JOINT0",
	"LLEG_JOINT1",
	"LLEG_JOINT2",
	"LLEG_JOINT3",
	"LLEG_JOINT4",
	"LLEG_JOINT5",
	"CHEST_JOINT0",
	"CHEST_JOINT1",
	"CHEST_JOINT2",
	"HEAD_JOINT0",
	"HEAD_JOINT1",
	"RARM_JOINT0",
	"RARM_JOINT1",
	"RARM_JOINT2",
	"RARM_JOINT3",
	"RARM_JOINT4",
	"RARM_JOINT5",
	"RARM_JOINT6",
	"RARM_JOINT7",
	"LARM_JOINT0",
	"LARM_JOINT1",
	"LARM_JOINT2",
	"LARM_JOINT3",
	"LARM_JOINT4",
	"LARM_JOINT5",
	"LARM_JOINT6",
	"LARM_JOINT7",
};

enum
{
	WAIST=0,
	RLEG_JOINT0,
	RLEG_JOINT1,
	RLEG_JOINT2,
	RLEG_JOINT3,
	RLEG_JOINT4,
	RLEG_JOINT5,
	LLEG_JOINT0,
	LLEG_JOINT1,
	LLEG_JOINT2,
	LLEG_JOINT3,
	LLEG_JOINT4,
	LLEG_JOINT5,
	CHEST_JOINT0,
	CHEST_JOINT1,
	CHEST_JOINT2,
	HEAD_JOINT0,
	HEAD_JOINT1,
	RARM_JOINT0,
	RARM_JOINT1,
	RARM_JOINT2,
	RARM_JOINT3,
	RARM_JOINT4,
	RARM_JOINT5,
	RARM_JOINT6,
	RARM_JOINT7,
	LARM_JOINT0,
	LARM_JOINT1,
	LARM_JOINT2,
	LARM_JOINT3,
	LARM_JOINT4,
	LARM_JOINT5,
	LARM_JOINT6,
	LARM_JOINT7,
	JOINT_NUM
};

static const int parent[JOINT_NUM] = {-1,0,1,2,3,4,5,0,7,8,9,10,11,0,13,14,15,16,15,18,19,20,21,22,23,24,15,26,27,28,29,30,31,32};
static const int child[JOINT_NUM] = {1,2,3,4,5,6,-1,8,9,10,11,12,-1,14,15,16,17,-1,19,20,21,22,23,24,25,-1,27,28,29,30,31,32,33,-1};
static const int sister[JOINT_NUM] = {-1,7,-1,-1,-1,-1,-1,13,-1,-1,-1,-1,-1,-1,-1,-1,18,-1,26,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};

static const double LinkAxis[JOINT_NUM][3] = {
	//WASIT
	{0,0,0},

	//RLEG
	{0,0,1},
	{1,0,0},
	{0,1,0},
	{0,1,0},
	{0,1,0},
	{1,0,0},

	//LLEG
	{0,0,1},
	{1,0,0},
	{0,1,0},
	{0,1,0},
	{0,1,0},
	{1,0,0},

	//CHEST
	{1,0,0},
	{0,1,0},
	{0,0,1},

	//HEAD
	{0,0,1},
	{0,1,0},

	//RARM
	{0,0,1},
	{0,1,0},
	{1,0,0},
	{0,0,1},
	{0,1,0},
	{0,0,1},
	{1,0,0},
	{0,1,0},

	//LARM
	{0,0,1},
	{0,1,0},
	{1,0,0},
	{0,0,1},
	{0,1,0},
	{0,0,1},
	{1,0,0},
	{0,1,0},
};

static const double LinkPos[JOINT_NUM][3] = {
	//WAIST
	{0.0f, 0.0f, 1022.5f},	
	
	//RLEG	
	{0.0f, -100.0f, 9.5f},	//1032
	{0.0f, 0.0f, -122.0f},
	{0.0f, 0.0f, -10.0f},
	{0.0f, 0.0f, -380.0f},
	{0.0f, 0.0f, -380.0f},
	{0.0f, 0.0f, -40.0f},

	//LLEG
	{0.0f, 100.0f, 9.5f},
	{0.0f, 0.0f, -122.0f},
	{0.0f, 0.0f, -10.0f},
	{0.0f, 0.0f, -380.0f},
	{0.0f, 0.0f, -380.0f},
	{0.0f, 0.0f, -40.0f},

	//CHEST
	{6.0f, 0.0f, 128.0f},	//1150.5
	{0.0f, 0.0f, 25.0f},
	{0.0f, 0.0f, 308.5f},

	//HEAD
	{-15.0f, 0.0f, -83.5f},
	{0.0f, 0.0f, -102.0f},

	//RARM
	{0.0f, -87.5f, 0.0f},	//1484
	{0.0f, -72.8f, 0.0f},
	{0.0f, -56.9f, 0.0f},
	{0.0f, 0.0f, -196.5f},
	{0.0f, 0.0f, -126.0f},
	{0.0f, 0.0f, -127.5f},
	{0.0f, 0.0f, -146.0f},
	{0.0f, 0.0f, -40.0f},

	//LARM
	{0.0f, 87.5f, 0.0f},
	{0.0f, 72.8f, 0.0f},
	{0.0f, 56.9f, 0.0f},
	{0.0f, 0.0f, -196.5f},
	{0.0f, 0.0f, -126.0f},
	{0.0f, 0.0f, -127.5f},
	{0.0f, 0.0f, -146.0f},
	{0.0f, 0.0f, -40.0f},
};

#endif
