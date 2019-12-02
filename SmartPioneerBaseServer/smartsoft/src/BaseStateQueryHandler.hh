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
#ifndef _BASESTATEQUERYHANDLER_USER_HH
#define _BASESTATEQUERYHANDLER_USER_HH
		
#include "BaseStateQueryHandlerCore.hh"

class BaseStateQueryHandler : public BaseStateQueryHandlerCore
{
protected:
	CommBasicObjects::CommTimeStamp time_stamp;
	CommBasicObjects::CommBasePose base_position;
	CommBasicObjects::CommBaseVelocity base_velocity;
	CommBasicObjects::CommBaseState base_state;
	SmartACE::SmartMutex mutex;

	virtual void on_update_from(const RobotTask* robotTask);
public:
	BaseStateQueryHandler(Smart::IQueryServerPattern<CommBasicObjects::CommVoid, CommBasicObjects::CommBaseState>* server);
	virtual ~BaseStateQueryHandler();
	virtual void handleQuery(const Smart::QueryIdPtr &id, const CommBasicObjects::CommVoid& request);
};
#endif
