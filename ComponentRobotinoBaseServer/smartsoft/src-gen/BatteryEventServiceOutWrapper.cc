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
#include "BatteryEventServiceOutWrapper.hh"

// include component's main class
#include "ComponentRobotinoBaseServer.hh"

// other extensin includes

BatteryEventServiceOutWrapper::BatteryEventServiceOutWrapper(Smart::IEventServerPattern<CommBasicObjects::CommBatteryParameter, CommBasicObjects::CommBatteryEvent, CommBasicObjects::CommBatteryState> *batteryEventServiceOut) {
	this->batteryEventServiceOut = batteryEventServiceOut;
	update_status = Smart::SMART_NODATA;
}

BatteryEventServiceOutWrapper::~BatteryEventServiceOutWrapper() {
}


Smart::StatusCode BatteryEventServiceOutWrapper::put(CommBasicObjects::CommBatteryState &eventState) {
	std::unique_lock<std::mutex> lock(update_mutex);
	updateEventState = eventState;
	update_status = batteryEventServiceOut->put(eventState);
	return update_status;
}

Smart::StatusCode BatteryEventServiceOutWrapper::getLatestUpdate(CommBasicObjects::CommBatteryState &eventState) {
	std::unique_lock<std::mutex> lock(update_mutex);
	eventState = updateEventState;
	return update_status;
}
