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
#include "SendTrajectoryOutWrapper.hh"

// include component's main class
#include "ComponentOpenRave.hh"

// other extensin includes

SendTrajectoryOutWrapper::SendTrajectoryOutWrapper(Smart::ISendClientPattern<CommManipulatorObjects::CommManipulatorTrajectory> *sendTrajectoryOut) {
	this->sendTrajectoryOut = sendTrajectoryOut;
	update_status = Smart::SMART_NODATA;
}

SendTrajectoryOutWrapper::~SendTrajectoryOutWrapper() {
}


Smart::StatusCode SendTrajectoryOutWrapper::send(CommManipulatorObjects::CommManipulatorTrajectory &sendTrajectoryOutDataObject) {
	std::unique_lock<std::mutex> lock(update_mutex);
	updateData = sendTrajectoryOutDataObject;
	update_status = sendTrajectoryOut->send(sendTrajectoryOutDataObject);
	return update_status;
}

Smart::StatusCode SendTrajectoryOutWrapper::getLatestUpdate(CommManipulatorObjects::CommManipulatorTrajectory &sendTrajectoryOutDataObject) {
	std::unique_lock<std::mutex> lock(update_mutex);
	sendTrajectoryOutDataObject = updateData;
	return update_status;
}