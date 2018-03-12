#ifndef _PDCONTROLLER_
#define _PDCONTROLLER_

#include <iostream>
#include <string>
#include <cnoid/SimpleController>
#include <cnoid/Link>
#include <cnoid/BodyMotion>
#include <cnoid/ExecutablePath>

using namespace std;
using namespace cnoid;

class PDController : public cnoid::SimpleController
{
	Body* ioBody;
	vector<double> qref, qold;
	vector<double> pgain, dgain;
	double dt;

	public:
	virtual bool initialize(SimpleControllerIO* io) override
	{
		ioBody = io->body();
		dt = io->timeStep();

		for(int i=0; i < ioBody->numJoints(); ++i){
			Link* joint = ioBody->joint(i);
			joint->setActuationMode(Link::JOINT_TORQUE);
			io->enableIO(joint);
			qref.push_back(joint->q());
		}
		qold = qref;

		load_gain(ioBody->numJoints(), pgain, dgain);

		return true;
	}

	void load_gain(size_t numJoints, vector<double> &pgain, vector<double> &dgain)
	{
		FILE *fp;
		double temp_pgain, temp_dgain;
		std::string gain_filepath = "/usr/lib/choreonoid-1.6/simplecontroller/pdgain.txt";
	
		if((fp = fopen(gain_filepath.c_str(),"r")) == NULL)
		{
			std::cerr << "gain file not found." << std::endl;
			return ;
		}

		pgain.clear(); dgain.clear();

		pgain.resize(numJoints);
		dgain.resize(numJoints);

		int i=0;
		while(fscanf(fp,"%lf %lf",&temp_pgain,&temp_dgain) != EOF)
		{
			pgain[i] = temp_pgain;
			dgain[i] = temp_dgain;
			i++;
		}

	}

	virtual bool control() override
	{
		for(int i=0; i < ioBody->numJoints(); ++i){
			Link* joint = ioBody->joint(i);
			double q = joint->q();
			double dq = (q - qold[i]) / dt;
			double u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
			qold[i] = q;
			joint->u() = u;
		}
		return true;
	}
};

#endif

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(PDController)
