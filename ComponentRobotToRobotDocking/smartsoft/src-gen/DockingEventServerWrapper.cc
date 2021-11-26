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
#include "DockingEventServerWrapper.hh"

// include component's main class
#include "ComponentRobotToRobotDocking.hh"

// other extensin includes

DockingEventServerWrapper::DockingEventServerWrapper(Smart::IEventServerPattern<CommRobotinoObjects::CommRobotinoDockingEventParameter, CommRobotinoObjects::CommRobotinoDockingEventResult, CommRobotinoObjects::RobotinoDockingEventState> *dockingEventServer) {
	this->dockingEventServer = dockingEventServer;
	update_status = Smart::SMART_NODATA;
}

DockingEventServerWrapper::~DockingEventServerWrapper() {
}


Smart::StatusCode DockingEventServerWrapper::put(CommRobotinoObjects::RobotinoDockingEventState &eventState) {
	std::unique_lock<std::mutex> lock(update_mutex);
	updateEventState = eventState;
	update_status = dockingEventServer->put(eventState);
	return update_status;
}

Smart::StatusCode DockingEventServerWrapper::getLatestUpdate(CommRobotinoObjects::RobotinoDockingEventState &eventState) {
	std::unique_lock<std::mutex> lock(update_mutex);
	eventState = updateEventState;
	return update_status;
}