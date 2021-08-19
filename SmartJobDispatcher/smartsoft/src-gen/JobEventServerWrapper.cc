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
#include "JobEventServerWrapper.hh"

// include component's main class
#include "SmartJobDispatcher.hh"

// other extensin includes

JobEventServerWrapper::JobEventServerWrapper(Smart::IEventServerPattern<CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskMessage, CommBasicObjects::CommTaskEventState> *jobEventServer) {
	this->jobEventServer = jobEventServer;
	update_status = Smart::SMART_NODATA;
}

JobEventServerWrapper::~JobEventServerWrapper() {
}


Smart::StatusCode JobEventServerWrapper::put(CommBasicObjects::CommTaskEventState &eventState) {
	std::unique_lock<std::mutex> lock(update_mutex);
	updateEventState = eventState;
	update_status = jobEventServer->put(eventState);
	return update_status;
}

Smart::StatusCode JobEventServerWrapper::getLatestUpdate(CommBasicObjects::CommTaskEventState &eventState) {
	std::unique_lock<std::mutex> lock(update_mutex);
	eventState = updateEventState;
	return update_status;
}
