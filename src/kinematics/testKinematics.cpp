#include "Kinematics.h"
#include "../util/func.h"

using namespace std;

int main(int argc, char* argv[])
{
	//Initialize
	Link ulink[JOINT_NUM];
	Kinematics kine(ulink);	
	SetJointInfo(ulink);

	//Set Angle to Joint
	for(int i=0;i<JOINT_NUM;i++)
		ulink[i].q = 0.0;

	ulink[ARM0].q = 0.0;
	ulink[ARM1].q = deg2rad(-40);
	ulink[ARM2].q = deg2rad( 80);
	ulink[ARM3].q = 0.0;
	ulink[ARM4].q = deg2rad(-40);
	ulink[ARM5].q = 0.0;
	//Calculation Forward Kinematics
	kine.calcForwardKinematics(BASE);
	Link ARM_LINK = ulink[ARM5];

	for(int i=0;i<1000;i++){
		ARM_LINK.p(2) -= 0.001;
		//Calculation Inverse Kinematics
		cout << "iteration:" << i << endl;
		if(kine.calcLMInverseKinematics(ARM5, ARM_LINK)){
			for(int i=1;i<=6;i++)
				cout << ulink[i].joint_name << ":" << rad2deg(ulink[i].q) << endl;
		}else{
			cout << "Calculation Inverse Kinematics Faild." << endl;
		}
		cout << "\n";
	}
	return 0;
}
