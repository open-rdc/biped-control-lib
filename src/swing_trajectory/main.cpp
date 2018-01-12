#include <stdio.h>

#include "PreviewControl.h"
#include "FootStepPlanner.h"
#include "GaitPatternGenerator.h"

const double dt = 0.01;
const double gait_cycle = 0.32;
const double foot_dist_y = 0.08;

// 1ステップ分の遊脚軌道を計算
// TODO 鉛直方向の軌道を多項式の中に含める
std::vector<Eigen::Vector3d> calcSwingTrajectory(double stx, double sty, double prev_stx, double prev_sty)
{
	Eigen::Vector2d p(Eigen::Vector2d::Zero());
	Eigen::Vector2d p_goal(Eigen::Vector2d(stx, sty));
	Eigen::Vector2d p_start(Eigen::Vector2d(prev_stx, prev_sty));
	Eigen::Matrix<double,2,4> swing_trajectory_coefficient;
	std::vector<Eigen::Vector3d> swing_trajectory;

	swing_trajectory_coefficient << p_start(0), 0, 3*(p_goal(0)-p_start(0))/pow(gait_cycle,2), -2*(p_goal(0)-p_start(0))/pow(gait_cycle,3),
																	p_start(1), 0, 3*(p_goal(1)-p_start(1))/pow(gait_cycle,2), -2*(p_goal(1)-p_start(1))/pow(gait_cycle,3);

	for(int step=0;step<static_cast<int>(gait_cycle/dt);step++)
	{
		double t = step * dt;
		Eigen::Vector2d p(Eigen::Vector2d::Zero());
		p = swing_trajectory_coefficient * Eigen::Vector4d(1,t,t*t,t*t*t);

		swing_trajectory.push_back(Eigen::Vector3d(p(0), p(1), 0.f));
	}

	return swing_trajectory;
}

int main()
{
	int count = 0;
	bool init_param = false;

	const double zmp_offset = 0.065;

	Eigen::Vector3d right_foot_position(Eigen::Vector3d(0,foot_dist_y,0));
	Eigen::Vector3d left_foot_position(Eigen::Vector3d(0,-foot_dist_y,0));
	Eigen::Vector3d right_foot_position_prev = right_foot_position;
	Eigen::Vector3d left_foot_position_prev = left_foot_position;

	std::vector<std::size_t> count_list;
	std::vector<Eigen::Vector2d> refzmp_list;
	std::vector<Eigen::Vector2d> com_list;
	std::vector<Eigen::Vector3d> right_foot_position_list, left_foot_position_list;

	GaitPatternGenerator gait_generator(0.32, 0.05, dt);

	gait_generator.setFootMaxStrideParameter(0.03, 0.03, 0.0);
	gait_generator.setTrajectoryParameter(gait_cycle, 0.05, zmp_offset, right_foot_position, left_foot_position);

	// 目標位置セット
	gait_generator.goTargetPos(0.1, 0.1, 0.0);
	// 歩行パターン生成, ZMP軌道のアップデート
	while(1){
		// TODO 重心軌道、支持脚、遊脚軌道の更新(仮)
		if(!gait_generator.update()) break;
		com_list.push_back(gait_generator.getCenterOfMassPosition());
		refzmp_list.push_back(gait_generator.get_preview_refzmp());
		count_list.push_back(count++);
	}

	// フットプリントのリスト
	std::vector<footstep_msgs::FootStatus> support_leg = gait_generator.foot_planner->support_leg_list;
	std::vector<Eigen::Vector3d> foot_step = gait_generator.foot_planner->foot_step_list;
#if 0
	for(std::size_t i=0;i<foot_step.size();i++){
		if(support_leg[i] == footstep_msgs::RightLegSup)
			std::cout << "Right "<< "[" << i << "] " << "x: " << foot_step[i](1) << " " << "y: " << foot_step[i](2) << std::endl;
		else if(support_leg[i] == footstep_msgs::LeftLegSup)
			std::cout << "Left " << "[" << i << "] " << "x: " << foot_step[i](1) << " " << "y: " << foot_step[i](2) << std::endl;
		else
			std::cout << "Both " << "[" << i << "] " << "x: " << foot_step[i](1) << " " << "y: " << foot_step[i](2) << std::endl;
	}
#endif
	// TODO 
	// 横移動の際、停止時に足先が少しズレる 
	// 目標脚接地点から遊脚軌道の計算(ワールド座標固定)
	// ここでは遊脚は予見時間分のインデックスを含んでない?
	std::size_t index = 1;
	while(1){
		std::vector<Eigen::Vector3d> swing_trajectory;
		if(support_leg[index-1] == footstep_msgs::RightLegSup){
			if(index == (foot_step.size()-1))
				swing_trajectory = calcSwingTrajectory(foot_step[index][1], left_foot_position_prev.y() , left_foot_position_prev.x(), left_foot_position_prev.y());	
			else
				swing_trajectory = calcSwingTrajectory(foot_step[index][1], foot_step[index][2] + zmp_offset - foot_dist_y, left_foot_position_prev.x(), left_foot_position_prev.y());
			for(int i=0;i<static_cast<int>(gait_cycle/dt);i++){
				right_foot_position_list.push_back(right_foot_position_prev);
				left_foot_position_list.push_back(swing_trajectory[i]);
			}
		}else if(support_leg[index-1] == footstep_msgs::LeftLegSup){
			if(index == (foot_step.size()-1))
				swing_trajectory = calcSwingTrajectory(foot_step[index][1], right_foot_position_prev.y() , right_foot_position_prev.x(), right_foot_position_prev.y());	
			else
				swing_trajectory = calcSwingTrajectory(foot_step[index][1], foot_step[index][2] - zmp_offset + foot_dist_y, right_foot_position_prev.x(), right_foot_position_prev.y());	
			for(int i=0;i<static_cast<int>(gait_cycle/dt);i++){
				right_foot_position_list.push_back(swing_trajectory[i]);
				left_foot_position_list.push_back(left_foot_position_prev);
			}
		// TODO footstep_msgs::BothLegは両足支持期間をしめしているのではなくここでは歩き始めと停止を明示的に示している
		}else if(support_leg[index-1] == footstep_msgs::BothLeg){
			for(int i=0;i<static_cast<int>(gait_cycle/dt);i++){
				right_foot_position_list.push_back(right_foot_position_prev);
				left_foot_position_list.push_back(left_foot_position_prev);
			}
		}
		right_foot_position_prev = right_foot_position_list.back();
		left_foot_position_prev = left_foot_position_list.back();
		index ++;
		if(foot_step.size() < index) break;
	}
#if 1
	for(int i=0;i<100;i++){
		right_foot_position_list.push_back(right_foot_position_prev);
		left_foot_position_list.push_back(left_foot_position_prev);
	}
#endif

	FILE *gp = popen("gnuplot -persist\n", "w");
#if 1
	fprintf(gp, "set key left top\n");
	fprintf(gp, "plot '-' with lines lw 2 title \"com\", '-' with lines lw 2 title \"refzmp\", '-' with lines lw 1 title \"right foot pos\", '-' with lines lw 1 title \"left foot pos\"\n");
	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%lu\t%f\n", count_list[i], com_list[i].x()); fprintf(gp,"e\n");
	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%lu\t%f\n", count_list[i], refzmp_list[i](0)); fprintf(gp,"e\n");
	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%lu\t%f\n", count_list[i], right_foot_position_list[i].x()); fprintf(gp,"e\n");
	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%lu\t%f\n", count_list[i], left_foot_position_list[i].x()); fprintf(gp,"e\n");
#endif
	fprintf(gp, "exit\n");
	pclose(gp);

	gp = popen("gnuplot -persist\n", "w");
	#if 1
	fprintf(gp, "set key left top\n");
	//fprintf(gp, "set yrange[0.085:-0.085]\n");
	fprintf(gp, "plot '-' with lines lw 2 title \"com\", '-' with lines lw 2 title \"refzmp\", '-' with lines lw 1 title \"right foot pos\", '-' with lines lw 1 title \"left foot pos\"\n");
	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%lu\t%f\n", count_list[i], com_list[i].y()); fprintf(gp,"e\n");
	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%lu\t%f\n", count_list[i], refzmp_list[i](1)); fprintf(gp,"e\n");
	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%lu\t%f\n", count_list[i], right_foot_position_list[i].y()); fprintf(gp,"e\n");
	for(std::size_t i=0;i<count_list.size();i++) fprintf(gp, "%lu\t%f\n", count_list[i], left_foot_position_list[i].y()); fprintf(gp,"e\n");
	#endif
	fprintf(gp, "exit\n");
	pclose(gp);

}
