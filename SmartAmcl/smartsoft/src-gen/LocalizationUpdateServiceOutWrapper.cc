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
#include "LocalizationUpdateServiceOutWrapper.hh"

// include component's main class
#include "SmartAmcl.hh"

// other extensin includes

LocalizationUpdateServiceOutWrapper::LocalizationUpdateServiceOutWrapper(Smart::ISendClientPattern<CommBasicObjects::CommBasePositionUpdate> *localizationUpdateServiceOut) {
	this->localizationUpdateServiceOut = localizationUpdateServiceOut;
	update_status = Smart::SMART_NODATA;
}

LocalizationUpdateServiceOutWrapper::~LocalizationUpdateServiceOutWrapper() {
}


Smart::StatusCode LocalizationUpdateServiceOutWrapper::send(CommBasicObjects::CommBasePositionUpdate &localizationUpdateServiceOutDataObject) {
	std::unique_lock<std::mutex> lock(update_mutex);
	updateData = localizationUpdateServiceOutDataObject;
	update_status = localizationUpdateServiceOut->send(localizationUpdateServiceOutDataObject);
	return update_status;
}

Smart::StatusCode LocalizationUpdateServiceOutWrapper::getLatestUpdate(CommBasicObjects::CommBasePositionUpdate &localizationUpdateServiceOutDataObject) {
	std::unique_lock<std::mutex> lock(update_mutex);
	localizationUpdateServiceOutDataObject = updateData;
	return update_status;
}
