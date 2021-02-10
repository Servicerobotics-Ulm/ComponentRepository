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
#include "LaserServiceOutWrapper.hh"

// include component's main class
#include "ComponentWebots2DLidar.hh"

// other extensin includes

LaserServiceOutWrapper::LaserServiceOutWrapper(Smart::IPushServerPattern<CommBasicObjects::CommMobileLaserScan> *laserServiceOut) {
	this->laserServiceOut = laserServiceOut;
	update_status = Smart::SMART_NODATA;
}

LaserServiceOutWrapper::~LaserServiceOutWrapper() {
}


Smart::StatusCode LaserServiceOutWrapper::put(CommBasicObjects::CommMobileLaserScan &laserServiceOutDataObject) {
	std::unique_lock<std::mutex> lock(update_mutex);
	updateData = laserServiceOutDataObject;
	update_status = laserServiceOut->put(laserServiceOutDataObject);
	return update_status;
}

Smart::StatusCode LaserServiceOutWrapper::getLatestUpdate(CommBasicObjects::CommMobileLaserScan &laserServiceOutDataObject) {
	std::unique_lock<std::mutex> lock(update_mutex);
	laserServiceOutDataObject = updateData;
	return update_status;
}