// -*- C++ -*-
/*!
 * @file  PDController_and_Sensor.cpp
 * @brief PDController and Sensor reading
 * @date $Date$
 *
 * $Id$
 */

#include "PDController_and_Sensor.h"
#include <iostream>
#include <cnoid/BodyMotion>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>

using namespace std;
using namespace cnoid;

namespace {
// Module specification
// <rtc-template block="module_spec">
static const char* pdcontroller_and_sensor_spec[] =
  {
    "implementation_id", "PDController_and_Sensor",
    "type_name",         "PDController_and_Sensor",
    "description",       "PDController and Sensor reading",
    "version",           "1.0.0",
    "vendor",            "CIT",
    "category",          "Generic",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
}
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
PDController_and_Sensor::PDController_and_Sensor(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_angleIn("q", m_angle),
    m_gsensorIn("gsensor", m_gsensor),
    m_gyrometerIn("gyrometer", m_gyrometer),
    m_rfsensorIn("rfsensor", m_rfsensor),
    m_lfsensorIn("lfsensor", m_lfsensor),
    m_torqueOut("u", m_torque),
    dt(0.001),
    dq_old(0.0)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
PDController_and_Sensor::~PDController_and_Sensor()
{
}

void PDController_and_Sensor::ReadGain(size_t numJoints,std::vector<double> &pgain,std::vector<double> &dgain)
{
      FILE *PDGain;
      float temp_pgain,temp_dgain;
      char joint_name[20];
      PDGain = fopen("/usr/lib/choreonoid-1.5/rtc/pdgain.txt", "r");
                    
      pgain.clear(); dgain.clear();
       
      pgain.resize(numJoints);
      dgain.resize(numJoints);
      angleRef.resize(numJoints);
      q_old.resize(numJoints);
      q_old_ref.resize(numJoints);
      m_torque.data.length(numJoints);
      
      int i=0;
      while(fscanf(PDGain,"%f %f",&temp_pgain,&temp_dgain) != EOF)
      {
             pgain[i] = temp_pgain;
             dgain[i] = temp_dgain;
             //cout<<pgain[i]<<" "<<dgain[i]<<endl;
             i++;
      }
}

RTC::ReturnCode_t PDController_and_Sensor::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("q", m_angleIn);
  addInPort("gsensor", m_gsensorIn);
  addInPort("gyrometer", m_gyrometerIn);
  addInPort("rfsensor", m_rfsensorIn);
  addInPort("lfsensor", m_lfsensorIn);
  
  // Set OutPort buffer
  addOutPort("u", m_torqueOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PDController_and_Sensor::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PDController_and_Sensor::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PDController_and_Sensor::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t PDController_and_Sensor::onActivated(RTC::UniqueId ec_id)
{
    if(m_angleIn.isNew()){
        m_angleIn.read();
        ReadGain(m_angle.data.length(),pgain,dgain);
     }
      
     for(size_t i=0;i<m_angle.data.length();i++){
          angleRef[i] = q_old[i] = q_old_ref[i] = m_angle.data[i];
     }
     
  return RTC::RTC_OK;
}


RTC::ReturnCode_t PDController_and_Sensor::onDeactivated(RTC::UniqueId ec_id)
{
    cout<<"Deactivated"<<endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t PDController_and_Sensor::onExecute(RTC::UniqueId ec_id)
{
    if(m_angleIn.isNew()){
        m_angleIn.read();
    }
    if(m_gsensorIn.isNew()){
        m_gsensorIn.read();
    }
    if(m_gyrometerIn.isNew()){
        m_gyrometerIn.read();
    }
    if(m_rfsensorIn.isNew()){
        m_rfsensorIn.read();
    }
    if(m_lfsensorIn.isNew()){
        m_lfsensorIn.read();
    }

    for(size_t i=0; i < m_angle.data.length(); ++i){
        double q_ref = angleRef[i];
        double q = m_angle.data[i];
        double dq_ref = (q_ref - q_old_ref[i]) / dt;
        double dq = (q - q_old[i]) / dt;
        m_torque.data[i] = (q_ref - q) * pgain[i] + (dq_ref - dq) * dgain[i];
        if(i == 37){
            double ddq = (dq - dq_old)/dt;
            m_torque.data[i] = (1.0 - dq) * 100 - 0.2 * ddq;
            dq_old = dq;
        }
        q_old[i] = q;
    }
    q_old_ref = angleRef;
    m_torqueOut.write();

    cout<< "gsensor: ";
    for(size_t i=0; i < m_gsensor.data.length(); ++i){
        cout<< m_gsensor.data[i] << ", ";
    }
    cout<<endl;

    cout<< "gyrometer: ";
    for(size_t i=0; i < m_gyrometer.data.length(); ++i){
        cout<< m_gyrometer.data[i] << ", ";
    }
    cout<<endl;

    cout<< "rfsensor: ";
    for(size_t i=0; i < m_rfsensor.data.length(); ++i){
        cout<< m_rfsensor.data[i] << ", ";
    }
    cout<<endl;

    cout<< "lfsensor: ";
    for(size_t i=0; i < m_lfsensor.data.length(); ++i){
        cout<< m_lfsensor.data[i] << ", ";
    }
    cout<<endl;
    cout<<endl;

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PDController_and_Sensor::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PDController_and_Sensor::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PDController_and_Sensor::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PDController_and_Sensor::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PDController_and_Sensor::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void PDController_and_SensorInit(RTC::Manager* manager)
  {
    coil::Properties profile(pdcontroller_and_sensor_spec);
    manager->registerFactory(profile,
                             RTC::Create<PDController_and_Sensor>,
                             RTC::Delete<PDController_and_Sensor>);
  }
  
};


