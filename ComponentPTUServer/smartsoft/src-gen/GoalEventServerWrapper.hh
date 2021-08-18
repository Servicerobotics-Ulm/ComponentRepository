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
#ifndef _GOALEVENTSERVERWRAPPER_HH
#define _GOALEVENTSERVERWRAPPER_HH

#include <mutex>
#include "aceSmartSoft.hh"

// include communication objects
#include <DomainPTU/CommPTUGoalEventParameter.hh>
#include <DomainPTU/CommPTUGoalEventParameterACE.hh>
#include <DomainPTU/CommPTUGoalEventResult.hh>
#include <DomainPTU/CommPTUGoalEventResultACE.hh>
#include <DomainPTU/PTUGoalEventState.hh>
#include <DomainPTU/PTUGoalEventStateACE.hh>


class GoalEventServerWrapper
{
private:
	std::mutex update_mutex;
	Smart::StatusCode update_status;
	
	DomainPTU::PTUGoalEventState updateEventState;
	DomainPTU::CommPTUGoalEventResult updateEvent;
	DomainPTU::CommPTUGoalEventParameter updateActivation;
	
	Smart::IEventServerPattern<DomainPTU::CommPTUGoalEventParameter, DomainPTU::CommPTUGoalEventResult, DomainPTU::PTUGoalEventState> *goalEventServer;
	
public:
	GoalEventServerWrapper(Smart::IEventServerPattern<DomainPTU::CommPTUGoalEventParameter, DomainPTU::CommPTUGoalEventResult, DomainPTU::PTUGoalEventState> *goalEventServer);
	virtual ~GoalEventServerWrapper();
	
	Smart::StatusCode put(DomainPTU::PTUGoalEventState &eventState);
	
	Smart::StatusCode getLatestUpdate(DomainPTU::PTUGoalEventState &eventState);
	
};

#endif // _GOALEVENTSERVERWRAPPER_HH