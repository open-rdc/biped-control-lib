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
	gain(0.1)
{
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

	return RTC::RTC_OK;
}


RTC::ReturnCode_t ArmInverseKinematicsTest::onDeactivated(RTC::UniqueId ec_id)
{
	return RTC::RTC_OK;
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
		}

		RARM_Link.p(0) += m_axes.data[1]*0.5;
		RARM_Link.p(1) += m_axes.data[0]*0.5;
		RARM_Link.p(2) += m_axes.data[3]*0.5;

		if(arm->calcInverseKinematics(RARM_JOINT7, CHEST_JOINT2, RARM_Link)){
			for(int i=13;i<=25;i++)
				m_qRef.data[i-1] = ulink[i].q;
		}else{
			cerr << "Calculation Inverse Kinematics Faild." << endl;
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


