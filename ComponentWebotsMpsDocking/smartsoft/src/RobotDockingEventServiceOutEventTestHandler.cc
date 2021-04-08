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
#include "RobotDockingEventServiceOutEventTestHandler.hh"

//bool RobotDockingEventServiceOutEventTestHandler::testEvent(
//	CommNavigationObjects::CommDockingEventParameter &p,
//	CommNavigationObjects::CommDockingEventResult &r,
//	const CommNavigationObjects::CommDockingEventState &s
//)
bool RobotDockingEventServiceOutEventTestHandler::testEvent(
	CommRobotinoObjects::CommRobotinoDockingEventParameter &p,
	CommRobotinoObjects::CommRobotinoDockingEventResult &r,
	const CommRobotinoObjects::RobotinoDockingEventState &s
)

throw() {
	// fire all events (without filtering) in the default implementation
	// implement your own (specific) event-filtering code using the event-parameter as input
	// true means that the current event will be fired to the according client
	// false means that the current event is ignored (it will not be communicated to the according client)

	bool result = false;

	CommRobotinoObjects::RobotinoDockingEventType oldState, newState;

	p.get(oldState);
	s.get(newState);
	if (oldState==newState)
	{
		result = false;
	}
	else
	{
		p.set(newState);
		r.set(newState);
		result = true;
		std::cout<<"Fire EVENT : new State"<<newState<<" oldState: "<<oldState<<std::endl;
	}

	return true;
}
