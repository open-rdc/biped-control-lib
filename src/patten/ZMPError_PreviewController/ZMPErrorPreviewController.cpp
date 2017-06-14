#include "ZMPErrorPreviewController.h"

/* 目標ZMPの補間 */
void ZMPErrorPreviewControl::interpolation_zmp_trajectory(vector<Vector3d> foot_step_list)
{
	size_t foot_step_count = 1;
	size_t size = foot_step_list.size();
	gait_count = (foot_step_list[size-2](0)+stop_time)/dt;

	for(int t=0;t<(foot_step_list[size-1](0)/dt);t++)
	{
		double temp_time = t*dt;
		if(foot_step_list[foot_step_count](0) < temp_time) foot_step_count++;
		this->refzmp.push_back(Vector2d(foot_step_list[foot_step_count-1](1), foot_step_list[foot_step_count-1](2)));
	}
}

/* 予見制御ゲインの計算 */
void ZMPErrorPreviewControl::calc_f()
{
	Matrix<double,4,4> xi(xi0.transpose());
	for(int preview_step=1;preview_step<=(preview_delay/dt);preview_step++){
		fi.push_back(-(1.0/(R+G.transpose()*P*G))*G.transpose()*xi.pow(preview_step-1)*P*GR);
	}
}

/* 制御入力の計算 */
void ZMPErrorPreviewControl::calc_u()
{
	Matrix<double,1,2> du;
	du = K*xk_ex;
	for(int preview_step=1;preview_step<=(preview_delay/dt);preview_step++)
		du += fi[preview_step-1] * (refzmp[preview_step+loop_step] - refzmp[preview_step+loop_step-1]);
	u += du;
}

/* 重心軌道の計算 */
bool ZMPErrorPreviewControl::calc_xk(Vector2d &com_pos, Vector2d &com_vel, Vector2d &com_acc)
{
	if(gait_count <= loop_step)
	{
		loop_step = 0;
		return false;
	}

	//Output
	p = c * xk;
	this->temp_refzmp = refzmp[loop_step];

	//Error ZMP
	e = temp_refzmp.transpose() - p;

	//Extend xk
	xk_ex << e, xk-xkp;
	xkp = xk;

	//Calculation Input
	calc_u();

	//Calculation xk
	xk = A*xk + b*u;
	com_pos << xk(0,0), xk(0,1);
	com_vel << xk(1,0), xk(1,1);
	com_acc << xk(2,0), xk(2,1);
	
	//Increment
	loop_step++;
	return true;
}
