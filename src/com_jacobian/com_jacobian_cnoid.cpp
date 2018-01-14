#include <iostream>
#include <cstring>

#include <cnoid/Body>
#include <cnoid/JointPath>
#include <cnoid/Jacobian>
#include <cnoid/BodyLoader>
#include <cnoid/Link>

int main()
{
	cnoid::BodyPtr robot;
	cnoid::BodyLoader bodyloader;
	cnoid::JointPathPtr rleg, lleg, leg;
	
	std::string model_path("model/JAXON_RED/JAXON_JVRCmain.wrl");
	robot = bodyloader.load(model_path.c_str());

	rleg = cnoid::getCustomJointPath(robot, robot->link("RLEG_JOINT5"), robot->rootLink());
	lleg = cnoid::getCustomJointPath(robot, robot->link("LLEG_JOINT5"), robot->rootLink());
	leg	 = cnoid::getCustomJointPath(robot, robot->link("RLEG_JOINT5"), robot->link("LLEG_JOINT5"));

	Eigen::Vector3d rleg_pos = robot->link("RLEG_JOINT5")->p();
	Eigen::Vector3d lleg_pos = robot->link("LLEG_JOINT5")->p();

	Eigen::MatrixXd Jcom(3, robot->numJoints());
	Eigen::Vector3d com(robot->calcCenterOfMass());

	cnoid::Link *base = leg->baseLink();
	cnoid::calcCMJacobian(robot, base, Jcom);

	std::cout << Jcom << std::endl;

	return 0;
}
