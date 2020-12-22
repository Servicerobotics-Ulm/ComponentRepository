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

// include wrapper header
#include "PlannerEventServerWrapper.hh"

// include component's main class
#include "SmartPlannerBreadthFirstSearch.hh"

// other extensin includes

PlannerEventServerWrapper::PlannerEventServerWrapper(Smart::IEventServerPattern<CommNavigationObjects::CommPlannerEventParameter, CommNavigationObjects::CommPlannerEventResult, CommNavigationObjects::PlannerEventState> *plannerEventServer) {
	this->plannerEventServer = plannerEventServer;
	update_status = Smart::SMART_NODATA;
}

PlannerEventServerWrapper::~PlannerEventServerWrapper() {
}


Smart::StatusCode PlannerEventServerWrapper::put(CommNavigationObjects::PlannerEventState &eventState) {
	std::unique_lock<std::mutex> lock(update_mutex);
	updateEventState = eventState;
	update_status = plannerEventServer->put(eventState);
	return update_status;
}

Smart::StatusCode PlannerEventServerWrapper::getLatestUpdate(CommNavigationObjects::PlannerEventState &eventState) {
	std::unique_lock<std::mutex> lock(update_mutex);
	eventState = updateEventState;
	return update_status;
}
