#ifndef _GAITPATTERNGENERATOR_H_
#define _GAITPATTERNGENERATOR_H_

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include "PlanCommon.h"
#include "PreviewControl.h"
#include "FootStepPlanner.h"

class GaitPatternGenerator{
private:
	Eigen::Vector3d right_foot_position, right_foot_position_prev;
	Eigen::Vector3d left_foot_position, left_foot_position_prev;
	Eigen::Vector2d com_position_prev;
	Eigen::Vector2d ref_foot_pos, ref_foot_pos_prev;
	std::vector<Eigen::Vector3d> foot_step_list;
	Eigen::Matrix<double,2,4> swing_trajectory_coefficient;
public:
	PreviewControl *preview_control;
	FootStepPlanner *foot_planner;
public:
	GaitPatternGenerator(const double _half_gait_cycle, const double _height_z, const double _dt, footstep_msgs::FootStatus _sup_leg_state=footstep_msgs::RightLegSup);
	~GaitPatternGenerator();
	void setTrajectoryParameter(double _half_gait_cycle, double _height_z, double _zmp_offset, Eigen::Vector3d _right_foot_position, Eigen::Vector3d _left_foot_position){
		swing_step = 0;
		half_gait_cycle = _half_gait_cycle;
		height_z = _height_z;
		zmp_offset = _zmp_offset;
		right_foot_position_prev = _right_foot_position;
		left_foot_position_prev = _left_foot_position;
		DoubleSupportRatio = 0.08;
		SingleSupportRatio = half_gait_cycle - DoubleSupportRatio;
	}
	void goTargetPos(const double target_x, const double target_y, const double target_th){
		dist_x=target_x; dist_y = target_y; dist_th=target_th;
		planning_flag = true;
	}
	void setFootMaxStrideParameter(const double _max_stride_x, const double _max_stride_y, const double _max_stride_th){
		max_stride_x	= _max_stride_x; 
		max_stride_y	= _max_stride_y;
		max_stride_th = _max_stride_th;
	}
	Eigen::Vector2d get_preview_refzmp(){ return temp_refzmp; }
	Eigen::RowVector2d get_preview_outzmp() { return current_zmp; }
	bool update();
	void updateReferenceZmp();
	void updateStatusParameter();
	void generateReferenceFootTrajectory();
	void generateTrajectory();
	void generateSwingTrajectory();
	void calcSwingTrajectoryCoefficient(double stride_x, double stride_y, double prev_footpos_x, double prev_footpos_y);
	Eigen::Vector3d	getSwingReferenceFootPosition();
	Eigen::Vector3d getReferenceRightFootPosition(){ return right_foot_position; }
	Eigen::Vector3d getReferenceLeftFootPosition(){ return left_foot_position; }
	Eigen::Vector2d getCenterOfMassPosition(){ return com_pos; }
	footstep_msgs::WalkingStatus getWalkingStatus(){ return walking_status; }
private:
	double dt, half_gait_cycle, time_sec;
	double dist_x, dist_y, dist_th; 
	double max_stride_x, max_stride_y, max_stride_th;
	int swing_step, gait_phase_step;
	int SingleSupportRatio, DoubleSupportRatio;
	int calculation_walking_index;
	double height_z;
	double zmp_offset;
	bool swing_trajectory_init;
	bool side_step_move;
	bool planning_flag;
	bool walking_is_doing;
	bool disable_calc_ref;
	Eigen::Vector2d com_pos, com_vel, com_acc;
	Eigen::Vector2d temp_refzmp;
	Eigen::RowVector2d current_zmp;
	Eigen::Vector3d swing_trajectory;
	footstep_msgs::WalkingStatus walking_status;
	footstep_msgs::FootStatus sup_leg_state;
};

#endif
