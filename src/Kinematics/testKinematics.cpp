#include "Kinematics.h"

using namespace std;
using namespace MotionControl;

double rad2deg(double radian)
{
	double degree = radian * 180.0f / pi;
	return degree;
}

double deg2rad(double degree)
{
	double radian = degree * pi / 180.0f;
	return radian;
}

int main(int argc, char* argv[])
{
	//Initialize
	Link ulink[JOINT_NUM];
	SetJointInfo(ulink);
	Kinematics kine(ulink);

	//Set Angle to Joint
	for(int i=0;i<JOINT_NUM;i++)
		kine.ulink[i].q = 0.0f;
	kine.ulink[J1].q = deg2rad(45.0f);
	kine.ulink[J2].q = deg2rad(30.0f);	
	kine.ulink[J4].q = deg2rad(60.0f);
	kine.ulink[J6].q = deg2rad(45.0f);

	//Calculation Forward Kinematics
	kine.calcForwardKinematics(WAIST);
	cout<<"Arm Link pos:"<<endl;
	cout<<kine.ulink[J6].p<<endl;
	
	Link RALink = kine.ulink[J6];
	Vector3d arm_p = kine.ulink[J6].p;
	Matrix3d arm_R = kine.ulink[J6].R;
	RALink.p(0) = arm_p(0);
	RALink.p(1) = arm_p(1) + 10.0;
	RALink.p(2) = arm_p(2);
	cout<<RALink.p<<endl;
	if(kine.calcInverseKinematics(J6, RALink)){
		cout<<"True"<<endl;
		for(int i=0;i<JOINT_NUM;i++)
			cout<<kine.ulink[i].joint_name<<":"<<rad2deg(kine.ulink[i].q)<<endl;
	}
	return 0;
}