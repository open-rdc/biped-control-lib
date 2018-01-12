#include "GaitPatternGenerator.h"

GaitPatternGenerator::GaitPatternGenerator(const double _half_gait_cycle, const double _height_z, const double _dt, footstep_msgs::FootStatus _sup_leg_state)
	: half_gait_cycle(_half_gait_cycle), com_position_prev(Eigen::Vector2d::Zero()),
		height_z(_height_z), swing_trajectory_init(false), swing_step(0), gait_phase_step(0), dt(_dt),
		sup_leg_state(_sup_leg_state), ref_foot_pos_prev(Eigen::Vector2d::Zero()), time_sec(0.0),
		right_foot_position_prev(Eigen::Vector3d::Zero()), left_foot_position_prev(Eigen::Vector3d::Zero()),
		swing_trajectory(Eigen::Vector3d::Zero()), walking_status(footstep_msgs::StartWalking), 
		side_step_move(false), planning_flag(false), walking_is_doing(false), disable_calc_ref(false)
{
	preview_control = new PreviewControl(dt,1.6, 0.28);
	foot_planner = new FootStepPlanner(dt);
}

GaitPatternGenerator::~GaitPatternGenerator()
{
}

// 目標値のアップデート
void GaitPatternGenerator::updateReferenceZmp()
{
	if(walking_is_doing){
		if(swing_trajectory_init) return;
		if(calculation_walking_index%static_cast<int>(half_gait_cycle/dt) != 0) {
			return ;
		}else if(calculation_walking_index%static_cast<int>(half_gait_cycle/dt) == 0) {
			int plan_index = calculation_walking_index / static_cast<int>(half_gait_cycle/dt);
			foot_planner->SetFootStepParameter(0.06, 0.04, 0.0, 0.065, 0.32);
			foot_planner->SetCurrentPos(Eigen::Vector2d(foot_planner->foot_step_list[plan_index](1), foot_planner->foot_step_list[plan_index](2)));
			foot_planner->SetTargetPos(dist_x, dist_y, dist_th, foot_planner->support_leg_list[plan_index], footstep_msgs::Walking);
		}
	}else if(!walking_is_doing){
		walking_is_doing = true;
		walking_status = footstep_msgs::StartWalking;
		if(0.0 <= dist_y) sup_leg_state = footstep_msgs::LeftLegSup;
		else sup_leg_state = footstep_msgs::RightLegSup;
		com_pos << 0.f, 0.f; com_vel << 0.f, 0.f; com_acc << 0.f, 0.f;
		com_position_prev = com_pos;
		foot_planner->SetFootStepParameter(max_stride_x, max_stride_y, max_stride_th, 0.065, 0.32);
		foot_planner->SetTargetPos(dist_x, dist_y, dist_th, sup_leg_state, walking_status);
	}
#if 0
	for(std::size_t i=0;i<foot_planner->foot_step_list.size();i++)
		std::cout << foot_planner->foot_step_list[i].x() << " " << foot_planner->foot_step_list[i].y() << " " << foot_planner->foot_step_list[i].z() << std::endl;
	std::cout << std::endl;
#endif
	gait_phase_step = 1;
	calculation_walking_index = 0;
	preview_control->interpolation_zmp_trajectory(foot_planner->foot_step_list);
	preview_control->set_com_param(com_pos, com_vel, com_acc);
	planning_flag = false; disable_calc_ref = false;
}

// 歩行軌道のアップデート
bool GaitPatternGenerator::update()
{
	// 再計算の要求
	if(planning_flag) updateReferenceZmp();
	// 重心軌道(予見制御)のアップデート
	if(!preview_control->update(com_pos, com_vel, com_acc)) {
		walking_is_doing = false;
		return false;
	}

	// ZMPのアップデート(予見制御)
	preview_control->get_ref_zmp(temp_refzmp);
	preview_control->output_zmp(current_zmp);
#if 0
	FILE *fp = fopen("com_pos.csv", "a");
	fprintf(fp, "%f %f %f %f\n", temp_refzmp.x(), temp_refzmp.y(), com_pos.x(), com_pos.y());
	fclose(fp);
#endif
	// 足先軌道のアップデート
	generateReferenceFootTrajectory();

	return true;
}

// 足先軌道の計算
void GaitPatternGenerator::generateReferenceFootTrajectory()
{
	right_foot_position.x() = right_foot_position_prev.x() - (com_pos.x() - com_position_prev.x());
	right_foot_position.y() = right_foot_position_prev.y() - (com_pos.y() - com_position_prev.y());
	right_foot_position.z() = 0.f;
	left_foot_position.x()  = left_foot_position_prev.x() - (com_pos.x() - com_position_prev.x());
	left_foot_position.y()  = left_foot_position_prev.y() - (com_pos.y() - com_position_prev.y());
	left_foot_position.z()  = 0.f;
	if(!swing_trajectory_init && !disable_calc_ref) generateSwingTrajectory();
	generateTrajectory();
	updateStatusParameter();
}

// パラメータのアップデート
void GaitPatternGenerator::updateStatusParameter()
{
	time_sec += dt;
	calculation_walking_index++;
	if(calculation_walking_index%static_cast<int>(half_gait_cycle/dt) == 0){
		swing_step = 0; swing_trajectory_init = false; time_sec = 0.f;
		if(gait_phase_step < (foot_planner->foot_step_list.size()-1)) gait_phase_step++;
		else disable_calc_ref = true;
	}
	if(walking_status == footstep_msgs::StartWalking){
		if(foot_planner->support_leg_list[gait_phase_step-1] != footstep_msgs::BothLeg)
			walking_status = footstep_msgs::Walking;
	}else if(walking_status == footstep_msgs::Walking){
		if(foot_planner->support_leg_list[gait_phase_step-1] == footstep_msgs::BothLeg)
			walking_status = footstep_msgs::StopWalking;
	}
	com_position_prev = com_pos;
	right_foot_position_prev = right_foot_position;
	left_foot_position_prev = left_foot_position;
}

// 予見制御で得られた重心軌道に基づく足先軌道軌道生成
void GaitPatternGenerator::generateTrajectory()
{
	if(DoubleSupportRatio/2.0 <= time_sec || time_sec < SingleSupportRatio){
		if(foot_planner->support_leg_list[gait_phase_step-1] == footstep_msgs::RightLegSup){
			//std::cout << "LeftLegSup" << std::endl;
			if(!disable_calc_ref) left_foot_position = getSwingReferenceFootPosition(); 
			if(!side_step_move) left_foot_position.y() =  left_foot_position_prev.y() - (com_pos.y() - com_position_prev.y());
		}else if(foot_planner->support_leg_list[gait_phase_step-1] == footstep_msgs::LeftLegSup){
			//std::cout << "RightLegSup" << std::endl;
			if(!disable_calc_ref) right_foot_position = getSwingReferenceFootPosition();
			if(!side_step_move) right_foot_position.y() =  right_foot_position_prev.y() - (com_pos.y() - com_position_prev.y());
		}
	}
}

// FootStepPlannerの結果に基づく遊脚軌道生成
void GaitPatternGenerator::generateSwingTrajectory()
{
	ref_foot_pos.x() = (foot_planner->foot_step_list[gait_phase_step](1) - foot_planner->foot_step_list[gait_phase_step-1](1))/2.0;
	if(foot_planner->support_leg_list[gait_phase_step-1] == footstep_msgs::RightLegSup){
		ref_foot_pos.y() = (foot_planner->foot_step_list[gait_phase_step](2) - foot_planner->foot_step_list[gait_phase_step-1](2))/2.0 + zmp_offset;
		calcSwingTrajectoryCoefficient(ref_foot_pos.x(), ref_foot_pos.y(), left_foot_position_prev.x(), left_foot_position_prev.y());
	}else if(foot_planner->support_leg_list[gait_phase_step-1] == footstep_msgs::LeftLegSup){
		ref_foot_pos.y() = (foot_planner->foot_step_list[gait_phase_step](2) - foot_planner->foot_step_list[gait_phase_step-1](2))/2.0 - zmp_offset;
		calcSwingTrajectoryCoefficient(ref_foot_pos.x(), ref_foot_pos.y(), right_foot_position_prev.x(), right_foot_position_prev.y());
	}
	//TODO 最後の歩幅の計算が上手くいって無さそう
	if(foot_planner->support_leg_list[gait_phase_step] == footstep_msgs::BothLeg) ref_foot_pos.y() = 0.0;
	if(ref_foot_pos.y() == 0.0) side_step_move = false;
	else side_step_move = true;
	swing_trajectory_init = true;
}

// 遊脚軌道生成のための係数計算
void GaitPatternGenerator::calcSwingTrajectoryCoefficient(double stride_x, double stride_y, double prev_footpos_x, double prev_footpos_y)
{
	Eigen::Vector2d p(Eigen::Vector2d::Zero());
	Eigen::Vector2d p_goal(Eigen::Vector2d(stride_x, stride_y));
	Eigen::Vector2d p_start(Eigen::Vector2d(prev_footpos_x, prev_footpos_y));

	swing_trajectory_coefficient << p_start(0), 0, 3*(p_goal(0)-p_start(0))/pow(half_gait_cycle,2), -2*(p_goal(0)-p_start(0))/pow(half_gait_cycle,3),
																	p_start(1), 0, 3*(p_goal(1)-p_start(1))/pow(half_gait_cycle,2), -2*(p_goal(1)-p_start(1))/pow(half_gait_cycle,3);
}

// 時刻tにおける遊脚軌道
Eigen::Vector3d GaitPatternGenerator::getSwingReferenceFootPosition()
{
	double t = swing_step*dt;
	Eigen::Vector2d p(Eigen::Vector2d::Zero());
	p = swing_trajectory_coefficient*Eigen::Vector4d(1,t,t*t,t*t*t);

	swing_trajectory.x() = p(0);
	swing_trajectory.y() = p(1);
	swing_trajectory.z() = height_z*0.5*(1-cos((2*M_PI/static_cast<int>((half_gait_cycle/dt)+0.001))*swing_step));

	swing_step++;

	return swing_trajectory;
}

