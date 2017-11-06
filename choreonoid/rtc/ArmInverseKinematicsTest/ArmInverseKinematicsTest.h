#ifndef ARMINVERSEKINEMATICSTEST_H
#define ARMINVERSEKINEMATICSTEST_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include "Kinematics.h"

using namespace MotionControl;
using namespace RTC;

class ArmInverseKinematicsTest
: public RTC::DataFlowComponentBase
{
	public:

		ArmInverseKinematicsTest(RTC::Manager* manager);
		~ArmInverseKinematicsTest();
		virtual RTC::ReturnCode_t onInitialize();
		virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
		virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
		virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
	protected:
		RTC::TimedDoubleSeq m_qCur;
		InPort<RTC::TimedDoubleSeq> m_qCurIn;
		RTC::TimedFloatSeq m_axes;
		InPort<RTC::TimedFloatSeq> m_axesIn;
		RTC::TimedBooleanSeq m_button;
		InPort<RTC::TimedBooleanSeq> m_buttonIn;

		RTC::TimedDoubleSeq m_qRef;
		OutPort<RTC::TimedDoubleSeq> m_qRefOut;
	private:
		Link ulink[JOINT_NUM];
		Link RARM_Link, LARM_Link;
		Kinematics *arm;
		double dist;
		float gain;
};


extern "C"
{
	DLL_EXPORT void ArmInverseKinematicsTestInit(RTC::Manager* manager);
};

#endif // ARMINVERSEKINEMATICSTEST_H
