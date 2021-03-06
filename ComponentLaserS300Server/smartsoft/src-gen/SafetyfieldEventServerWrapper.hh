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
// Please do not modify this file. It will be re-generated
// running the code generator.
//--------------------------------------------------------------------------
#ifndef _SAFETYFIELDEVENTSERVERWRAPPER_HH
#define _SAFETYFIELDEVENTSERVERWRAPPER_HH

#include <mutex>
#include "aceSmartSoft.hh"

// include communication objects
#include <CommBasicObjects/CommLaserSafetyEventParam.hh>
#include <CommBasicObjects/CommLaserSafetyEventParamACE.hh>
#include <CommBasicObjects/CommLaserSafetyEventState.hh>
#include <CommBasicObjects/CommLaserSafetyEventStateACE.hh>
#include <CommBasicObjects/CommLaserSafetyField.hh>
#include <CommBasicObjects/CommLaserSafetyFieldACE.hh>


class SafetyfieldEventServerWrapper
{
private:
	std::mutex update_mutex;
	Smart::StatusCode update_status;
	
	CommBasicObjects::CommLaserSafetyEventState updateEventState;
	CommBasicObjects::CommLaserSafetyField updateEvent;
	CommBasicObjects::CommLaserSafetyEventParam updateActivation;
	
	Smart::IEventServerPattern<CommBasicObjects::CommLaserSafetyEventParam, CommBasicObjects::CommLaserSafetyField, CommBasicObjects::CommLaserSafetyEventState> *safetyfieldEventServer;
	
public:
	SafetyfieldEventServerWrapper(Smart::IEventServerPattern<CommBasicObjects::CommLaserSafetyEventParam, CommBasicObjects::CommLaserSafetyField, CommBasicObjects::CommLaserSafetyEventState> *safetyfieldEventServer);
	virtual ~SafetyfieldEventServerWrapper();
	
	Smart::StatusCode put(CommBasicObjects::CommLaserSafetyEventState &eventState);
	
	Smart::StatusCode getLatestUpdate(CommBasicObjects::CommLaserSafetyEventState &eventState);
	
};

#endif // _SAFETYFIELDEVENTSERVERWRAPPER_HH
