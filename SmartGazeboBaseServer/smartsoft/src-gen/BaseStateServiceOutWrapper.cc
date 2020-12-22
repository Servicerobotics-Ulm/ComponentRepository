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
#include "BaseStateServiceOutWrapper.hh"

// include component's main class
#include "SmartGazeboBaseServer.hh"

// other extensin includes

BaseStateServiceOutWrapper::BaseStateServiceOutWrapper(Smart::IPushServerPattern<CommBasicObjects::CommBaseState> *baseStateServiceOut) {
	this->baseStateServiceOut = baseStateServiceOut;
	update_status = Smart::SMART_NODATA;
}

BaseStateServiceOutWrapper::~BaseStateServiceOutWrapper() {
}


Smart::StatusCode BaseStateServiceOutWrapper::put(CommBasicObjects::CommBaseState &baseStateServiceOutDataObject) {
	std::unique_lock<std::mutex> lock(update_mutex);
	updateData = baseStateServiceOutDataObject;
	update_status = baseStateServiceOut->put(baseStateServiceOutDataObject);
	return update_status;
}

Smart::StatusCode BaseStateServiceOutWrapper::getLatestUpdate(CommBasicObjects::CommBaseState &baseStateServiceOutDataObject) {
	std::unique_lock<std::mutex> lock(update_mutex);
	baseStateServiceOutDataObject = updateData;
	return update_status;
}
