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
#include "TaskSendOutWrapper.hh"

// include component's main class
#include "ComponentSeq2SeqCom.hh"

// other extensin includes

TaskSendOutWrapper::TaskSendOutWrapper(Smart::ISendClientPattern<CommBasicObjects::CommTaskMessage> *taskSendOut) {
	this->taskSendOut = taskSendOut;
	update_status = Smart::SMART_NODATA;
}

TaskSendOutWrapper::~TaskSendOutWrapper() {
}


Smart::StatusCode TaskSendOutWrapper::send(CommBasicObjects::CommTaskMessage &taskSendOutDataObject) {
	std::unique_lock<std::mutex> lock(update_mutex);
	updateData = taskSendOutDataObject;
	update_status = taskSendOut->send(taskSendOutDataObject);
	return update_status;
}

Smart::StatusCode TaskSendOutWrapper::getLatestUpdate(CommBasicObjects::CommTaskMessage &taskSendOutDataObject) {
	std::unique_lock<std::mutex> lock(update_mutex);
	taskSendOutDataObject = updateData;
	return update_status;
}
