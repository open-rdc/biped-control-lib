#include "PreviewControl.h"

void PreviewControl::interpolation_zmp_trajectory(vector<Vector3d> foot_step_list)
{
	this->refzmp.clear();
	preview_num = 0;

	// Add Virtual Control Point
	Vector3d virtual_control_point = foot_step_list.back();
	virtual_control_point[0] += preview_delay;
	foot_step_list.push_back(virtual_control_point);

	size_t foot_step_count = 1;
	size_t size = foot_step_list.size();
	foot_step_num = (foot_step_list[size-2](0)+stop_time)/dt;
	preview_step_num = (foot_step_list[size-2](0))/dt - 1; 

	for(int t=0;t<(foot_step_list[size-1](0)/dt);t++){
		double temp_time = t*dt;
		if(foot_step_list[foot_step_count](0) < temp_time) foot_step_count++;
		this->refzmp.push_back(Vector2d(foot_step_list[foot_step_count-1](1), foot_step_list[foot_step_count-1](2)));
	}
}

void PreviewControl::set_com_param(Vector2d com_pos, Vector2d com_vel, Vector2d com_acc)
{
	xk(0,0) = com_pos(0); xk(0,1) = com_pos(1);
	xk(1,0) = com_vel(0); xk(1,1) = com_vel(1);
	xk(2,0) = com_acc(0); xk(2,1) = com_acc(1);
}

void PreviewControl::calc_f()
{
	Matrix<double,3,3> AbK((A-b*K).transpose());
	for(int preview_step=1;preview_step<=(preview_delay/dt);preview_step++)
		fi.push_back((1.0/(R+b.transpose()*P*b))*b.transpose()*AbK.pow(preview_step-1)*Q*c.transpose());
}

void PreviewControl::calc_u()
{
	Matrix<double,1,2> du;
	for(int preview_step=1;preview_step<=(preview_delay/dt);preview_step++)
		du += fi[preview_step-1]*refzmp[preview_step+preview_num];
	u = -K*xk + du;
}

void PreviewControl::calc_xk(Vector2d &com_pos, Vector2d &com_vel, Vector2d &com_acc)
{
	xk = A*xk + b*u;

	com_pos << xk(0,0), xk(0,1);
	com_vel << xk(1,0), xk(1,1);
	com_acc << xk(2,0), xk(2,1);
}

bool PreviewControl::update(Vector2d &com_pos, Vector2d &com_vel, Vector2d &com_acc)
{
	if(foot_step_num <= preview_num)
		return false;

	p = c * xk;
	temp_refzmp = refzmp[preview_num];

	calc_u();
	calc_xk(com_pos, com_vel, com_acc);

	preview_num++;
	return true;
}
