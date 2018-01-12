#include "FootStepPlanner.h"

FootStepPlanner::FootStepPlanner(const double _dt)
	: dt(_dt)
{
}

FootStepPlanner::~FootStepPlanner()
{
}

bool FootStepPlanner::SetTargetPos(const double target_x, const double target_y, const double target_th, footstep_msgs::FootStatus ref_leg_sup, footstep_msgs::WalkingStatus ref_walking_status)
{
	target_pos << target_x, target_y; 
	next_leg_support = ref_leg_sup;
	walking_status = ref_walking_status;

	foot_step_list_clear();

	target_distance = sqrt(pow(target_x, 2) + pow(target_y, 2));

	if(target_pos.x() == 0 && target_pos.y() == 0){
		stride.x() = stride.y() = 0;
	}else if(fabs(target_pos.x()) > fabs(target_pos.y())){
		foot_step_count = static_cast<int>((fabs(target_pos.x()) + stride.x() - 0.001) / stride.x());
		if(target_pos.x() < 0) stride.x() *= -1.0f;
		stride.y() = target_pos.y() / foot_step_count;
	}else{
		foot_step_count = static_cast<int>((fabs(target_pos.y()) + stride.y() - 0.001) / stride.y());
		if(target_pos.y() < 0) stride.y() *= -1.0f;
		stride.x() = target_pos.x() / foot_step_count;
	}
	
	bool result = target_pos_2_foot_step(ref_leg_sup);
	return result;
}

bool FootStepPlanner::target_pos_2_foot_step(footstep_msgs::FootStatus ref_leg_sup)
{
	int counter = 0;
	
	start_walk(ref_leg_sup);

	while(1){
		counter++;

		if(DistanceTargetPos(control_point) <= sqrt(pow(stride.x(),2)+pow(stride.y(),2))) break;
		if(foot_step_count < counter) return false;

		next_control_point = control_point + stride;
		time += step_time;
		support_leg_list.push_back(next_leg_support);

		if(next_leg_support == footstep_msgs::RightLegSup){
			foot_step_list.push_back(Eigen::Vector3d(time, next_control_point.x(), next_control_point.y()+zmp_offset));
			next_leg_support = footstep_msgs::LeftLegSup;
		}else if(next_leg_support == footstep_msgs::LeftLegSup){
			foot_step_list.push_back(Eigen::Vector3d(time, next_control_point.x(), next_control_point.y()-zmp_offset));
			next_leg_support = footstep_msgs::RightLegSup;
		}
		control_point = next_control_point;
	}

	stop_walk();

	return true;
}

double FootStepPlanner::DistanceTargetPos(Eigen::Vector2d current_pos)
{
	return static_cast<double>(sqrt(pow((target_pos.x()-current_pos.x()),2)+pow((target_pos.y()-current_pos.y()),2)));
}

void FootStepPlanner::start_walk(footstep_msgs::FootStatus ref_leg_sup)
{
	if(walking_status == footstep_msgs::StartWalking){
		time += step_time;
		support_leg_list.push_back(footstep_msgs::BothLeg);
		foot_step_list.push_back(Eigen::Vector3d(0.f, 0.f, 0.f));
	}
		
	double offset = 0.f;
	Eigen::Vector2d next_step;
	if(ref_leg_sup == footstep_msgs::RightLegSup){
		offset = zmp_offset;
		next_leg_support = footstep_msgs::LeftLegSup;
	}else if(ref_leg_sup == footstep_msgs::LeftLegSup){
		offset = -1 * zmp_offset;
		next_leg_support = footstep_msgs::RightLegSup;
	}
	if(walking_status == footstep_msgs::StartWalking)
		next_step << 0.f, offset;
	else
		next_step = next_foot_point;
	support_leg_list.push_back(ref_leg_sup);
	foot_step_list.push_back(Eigen::Vector3d(time, next_step.x(), next_step.y()));
	if(walking_status == footstep_msgs::Walking){
		control_point.x() = next_step.x();
		control_point.y() = next_step.y() - offset;
	}
}

void FootStepPlanner::stop_walk()
{
	next_control_point = target_pos;

	if(walking_status != footstep_msgs::StopWalking){
		time += step_time;
		support_leg_list.push_back(next_leg_support);
		if(next_leg_support == footstep_msgs::RightLegSup)
			foot_step_list.push_back(Eigen::Vector3d(time, next_control_point.x(), next_control_point.y()+zmp_offset));
		else if(next_leg_support == footstep_msgs::LeftLegSup)
			foot_step_list.push_back(Eigen::Vector3d(time, next_control_point.x(), next_control_point.y()-zmp_offset));
	}
	time += step_time;
	support_leg_list.push_back(footstep_msgs::BothLeg);
	foot_step_list.push_back(Eigen::Vector3d(time, next_control_point.x(), next_control_point.y()));
	next_leg_support = footstep_msgs::BothLeg;
}
