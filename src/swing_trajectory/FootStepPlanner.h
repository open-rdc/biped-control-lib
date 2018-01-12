#ifndef _FOOTSTEPPLANNER_H_
#define _FOOTSTEPPLANNER_H_

#include <iostream>
#include <vector>

#include <Eigen/Core>

#include "PlanCommon.h"

class FootStepPlanner{
private:
	const double dt;
	double time, step_time;
	double zmp_offset;
	int foot_step_count;
	double target_distance;
	footstep_msgs::FootStatus next_leg_support;
	footstep_msgs::WalkingStatus walking_status;
	Eigen::Vector2d next_foot_point;
	Eigen::Vector2d stride, target_pos, control_point, next_control_point;
public:
	FootStepPlanner(const double _dt);
	~FootStepPlanner();
	bool SetTargetPos(const double target_x, const double target_y, const double target_th, footstep_msgs::FootStatus ref_leg_sup, footstep_msgs::WalkingStatus ref_walking_status);
	bool target_pos_2_foot_step(footstep_msgs::FootStatus ref_leg_sup);
	double DistanceTargetPos(Eigen::Vector2d current_pos);
	void start_walk(footstep_msgs::FootStatus ref_leg_sup);
	void stop_walk();
	void SetFootStepParameter(const double stride_x, const double stride_y, const double stride_th, const double zmp_offset, const double step_time)
	{
		stride << stride_x, stride_y;
		this->zmp_offset = zmp_offset;
		this->step_time = step_time; 
	}
	void foot_step_list_clear()
	{
		time = 0.f;
		foot_step_count = 0;
		foot_step_list.clear();
		support_leg_list.clear();	
		if(walking_status == footstep_msgs::StartWalking) control_point = Eigen::Vector2d::Zero();
	}
	void SetCurrentPos(Eigen::Vector2d next_foot_pos){
		next_foot_point = next_foot_pos;
	}
public:
	std::vector<footstep_msgs::FootStatus> support_leg_list;
	std::vector<Eigen::Vector3d> foot_step_list;
};

#endif
