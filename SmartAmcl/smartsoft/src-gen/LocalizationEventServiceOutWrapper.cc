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
#include "LocalizationEventServiceOutWrapper.hh"

// include component's main class
#include "SmartAmcl.hh"

// other extensin includes

LocalizationEventServiceOutWrapper::LocalizationEventServiceOutWrapper(Smart::IEventServerPattern<CommLocalizationObjects::CommLocalizationEventParameter, CommLocalizationObjects::CommLocalizationEventResult, CommLocalizationObjects::LocalizationEventState> *localizationEventServiceOut) {
	this->localizationEventServiceOut = localizationEventServiceOut;
	update_status = Smart::SMART_NODATA;
}

LocalizationEventServiceOutWrapper::~LocalizationEventServiceOutWrapper() {
}


Smart::StatusCode LocalizationEventServiceOutWrapper::put(CommLocalizationObjects::LocalizationEventState &eventState) {
	std::unique_lock<std::mutex> lock(update_mutex);
	updateEventState = eventState;
	update_status = localizationEventServiceOut->put(eventState);
	return update_status;
}

Smart::StatusCode LocalizationEventServiceOutWrapper::getLatestUpdate(CommLocalizationObjects::LocalizationEventState &eventState) {
	std::unique_lock<std::mutex> lock(update_mutex);
	eventState = updateEventState;
	return update_status;
}