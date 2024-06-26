//--------------------------------------------------------------------------
// Code generated by the SmartSoft MDSD Toolchain
// The SmartSoft Toolchain has been developed by:
//  
// Service Robotics Research Center
// University of Applied Sciences Ulm
// Prittwitzstr. 10
// 89075 Ulm (Germany)
//
// Information about the SmartSoft MDSD Toolchain is available at:
// www.servicerobotik-ulm.de
//
// This file is generated once. Modify this file to your needs. 
// If you want the toolchain to re-generate this file, please 
// delete it before running the code generator.
//--------------------------------------------------------------------------
#ifndef _BASESTATETASK_HH
#define _BASESTATETASK_HH

#include "BaseStateTaskCore.hh"
#include <CommBasicObjects/CommBasePositionUpdate.hh>
#include <CommBasicObjects/CommBaseState.hh>
#include <CommBasicObjects/CommBatteryEvent.hh>
#include <CommBasicObjects/CommBatteryParameter.hh>
#include <CommBasicObjects/CommBatteryState.hh>
#include <CommBasicObjects/CommNavigationVelocity.hh>
#include <CommBasicObjects/CommVoid.hh>
#include "BaseStateQueryHandlerCore.hh"

class BaseStateTask  : public BaseStateTaskCore
{
private:
public:
	BaseStateTask(SmartACE::SmartComponent *comp);
	virtual ~BaseStateTask();
	
	virtual int on_entry();
	virtual int on_execute();
	virtual int on_exit();
	void handle_push();
private:
	CommBasicObjects::CommTimeStamp time_stamp;
	CommBasicObjects::CommBasePose base_position;
	CommBasicObjects::CommBaseVelocity base_velocity;
	CommBasicObjects::CommBaseState base_state;
};

#endif
