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
#include "DepthImagePushServiceOutWrapper.hh"

// include component's main class
#include "ComponentKinectV2Server.hh"

// other extensin includes

DepthImagePushServiceOutWrapper::DepthImagePushServiceOutWrapper(Smart::IPushServerPattern<DomainVision::CommDepthImage> *depthImagePushServiceOut) {
	this->depthImagePushServiceOut = depthImagePushServiceOut;
	update_status = Smart::SMART_NODATA;
}

DepthImagePushServiceOutWrapper::~DepthImagePushServiceOutWrapper() {
}


Smart::StatusCode DepthImagePushServiceOutWrapper::put(DomainVision::CommDepthImage &depthImagePushServiceOutDataObject) {
	std::unique_lock<std::mutex> lock(update_mutex);
	updateData = depthImagePushServiceOutDataObject;
	update_status = depthImagePushServiceOut->put(depthImagePushServiceOutDataObject);
	return update_status;
}

Smart::StatusCode DepthImagePushServiceOutWrapper::getLatestUpdate(DomainVision::CommDepthImage &depthImagePushServiceOutDataObject) {
	std::unique_lock<std::mutex> lock(update_mutex);
	depthImagePushServiceOutDataObject = updateData;
	return update_status;
}
