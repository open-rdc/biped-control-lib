#include "ArmInverseKinematicsTest.h"

static const char* arminversekinematicstest_spec[] =
{
	"implementation_id", "ArmInverseKinematicsTest",
	"type_name",         "ArmInverseKinematicsTest",
	"description",       "Test code for arm control using inverse kinematics",
	"version",           "1.0.0",
	"vendor",            "CIT",
	"category",          "test",
	"activity_type",     "PERIODIC",
	"kind",              "DataFlowComponent",
	"max_instance",      "1",
	"language",          "C++",
	"lang_type",         "compile",
	""
};

ArmInverseKinematicsTest::ArmInverseKinematicsTest(RTC::Manager* manager)
: RTC::DataFlowComponentBase(manager),
	m_qCurIn("qCur", m_qCur),
	m_axesIn("axes", m_axes),
	m_buttonIn("button", m_button),
	m_qRefOut("qRef", m_qRef),
	gain(0.1),
	dist(0.5),
	time(0.f),
	delta_t(0.001),
	init_pose(false)
{
//	double init_pose_temp[21] = {0,0,0,0,0,0,30,-20,-5,-80,0,0,-20,0,30,20,5,-80,0,0,-20};
	//for(int i=0;i<21;i++)
		//init_pose_list[i] = init_pose_temp[i];
}


ArmInverseKinematicsTest::~ArmInverseKinematicsTest()
{
}

RTC::ReturnCode_t ArmInverseKinematicsTest::onInitialize()
{
	addInPort("qCur", m_qCurIn);
	addInPort("axes", m_axesIn);
	addInPort("button", m_buttonIn);

	addOutPort("qRef",m_qRefOut);

	arm = new Kinematics(ulink);
	SetJointInfo(ulink);

	return RTC::RTC_OK;
}

RTC::ReturnCode_t ArmInverseKinematicsTest::onActivated(RTC::UniqueId ec_id)
{
	if(m_qCurIn.isNew()){
		m_qCurIn.read();
		m_qRef.data.length(m_qCur.data.length());
	}
	for(size_t i=0;i<JOINT_NUM;i++)
		ulink[i+1].q = m_qCur.data[i];
	for(size_t i=0;i<m_qCur.data.length();i++)
		m_qRef.data[i] = m_qCur.data[i];
	
	arm->calcForwardKinematics(WAIST);
	
	RARM_Link = ulink[RARM_JOINT7];
	std::cout << RARM_Link.p << std::endl;
	LARM_Link	= ulink[LARM_JOINT7];
	return RTC::RTC_OK;
}


RTC::ReturnCode_t ArmInverseKinematicsTest::onDeactivated(RTC::UniqueId ec_id)
{
	return RTC::RTC_OK;
}

void ArmInverseKinematicsTest::setBasePose(std::vector<double> base_pose, double goal_time)
{
	this->base_pose = base_pose;
	this->goal_time	= goal_time;
	this->calc_step = static_cast<int>(goal_time/delta_t);
	this->time			= 0.f;
#if 0
	std::cout << "Initial Pose: ";
	for(int i=0;i<21;i++)
		std::cout << init_pose_list[i] << " ";
	std::cout << "\n\n";
	std::cout << "Base Pose: ";
	for(int i=0;i<21;i++)
		std::cout << base_pose[i]*180.f/M_PI << " ";
	std::cout << "\n\n";
	std::cout << "Distance Pose: ";
	for(int i=0;i<21;i++){
		std::cout << init_pose_list[i] - base_pose[i]*180.f/M_PI << " ";
		dist_pose.push_back(((init_pose_list[i]*M_PI/180.f)-base_pose[i])/calc_step);
	}
	std::cout << "\n\n";
#endif
}

bool ArmInverseKinematicsTest::calcInterpolationPose(std::vector<double> &target_pose)
{
	double init_pose_list[21] = {0,0,0,0,0,0,30,-20,-5,-80,0,0,-20,0,30,20,5,-80,0,0,-20};
	target_pose.resize(21);
	for(std::size_t i=0;i<21;i++)
		target_pose[i] = base_pose[i] + (init_pose_list[i]-base_pose[i])*(time/goal_time);
	time+=delta_t;
	if(time <= goal_time){
		return true;
	}else if(goal_time < time){
		return false;
	}
}

RTC::ReturnCode_t ArmInverseKinematicsTest::onExecute(RTC::UniqueId ec_id)
{
	if(m_axesIn.isNew())
		m_axesIn.read();
	if(m_buttonIn.isNew())
		m_buttonIn.read();
	if(m_qCurIn.isNew()){
		m_qCurIn.read();

		if(m_button.data[0] == 1){
			m_qRef.data[35] += 0.001;
			m_qRef.data[36] -= 0.001;
		}else if(m_button.data[1] == 1){
			m_qRef.data[35] -= 0.001;
			m_qRef.data[36] += 0.001;
		}else if(m_button.data[6] == 1){
			if(!init_pose){
				std::vector<double> base_pose;
				for(std::size_t i=12;i<33;i++)
					base_pose.push_back(m_qCur.data[i]);
				setBasePose(base_pose, 3.0);
				init_pose = true;
			}
		}

		if(!init_pose){
			Eigen::Vector3d refP(-1*m_axes.data[1], -1*m_axes.data[0], m_axes.data[3]);
			if(m_button.data[4] == 1)
				LARM_Link.p = LARM_Link.p + (dist*refP);
			else if(m_button.data[5] == 1)
				RARM_Link.p = RARM_Link.p + (dist*refP);

			if(arm->calcInverseKinematics(RARM_JOINT7, CHEST_JOINT2, RARM_Link) && arm->calcInverseKinematics(LARM_JOINT7, CHEST_JOINT2, LARM_Link)){
				for(int i=13;i<=33;i++)
					m_qRef.data[i-1] = ulink[i].q;
			}else{
				cerr << "Calculation Inverse Kinematics Faild." << endl;
			}
		}else{
			std::vector<double> temp_angle;
			if(calcInterpolationPose(temp_angle)){
				for(std::size_t i=12;i<33;i++)
					m_qRef.data[i] = ulink[i+1].q = temp_angle[i-12];
			}else{
				arm->calcForwardKinematics(WAIST);
				RARM_Link = ulink[RARM_JOINT7];
				LARM_Link	= ulink[LARM_JOINT7];
				init_pose = false;
			}
		}
	}
	m_qRefOut.write();

	return RTC::RTC_OK;
}

extern "C"
{

	void ArmInverseKinematicsTestInit(RTC::Manager* manager)
	{
		coil::Properties profile(arminversekinematicstest_spec);
		manager->registerFactory(profile,
				RTC::Create<ArmInverseKinematicsTest>,
				RTC::Delete<ArmInverseKinematicsTest>);
	}

};


