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
#include "SequencerEventServerWrapper.hh"

// include component's main class
#include "ComponentWebInterfaceComissioning.hh"

// other extensin includes

SequencerEventServerWrapper::SequencerEventServerWrapper(Smart::IEventServerPattern<CommBasicObjects::CommSkillMsg, CommBasicObjects::CommSkillMsg, CommBasicObjects::CommSkillMsg> *sequencerEventServer) {
	this->sequencerEventServer = sequencerEventServer;
	update_status = Smart::SMART_NODATA;
}

SequencerEventServerWrapper::~SequencerEventServerWrapper() {
}


Smart::StatusCode SequencerEventServerWrapper::put(CommBasicObjects::CommSkillMsg &eventState) {
	std::unique_lock<std::mutex> lock(update_mutex);
	updateEventState = eventState;
	update_status = sequencerEventServer->put(eventState);
	return update_status;
}

Smart::StatusCode SequencerEventServerWrapper::getLatestUpdate(CommBasicObjects::CommSkillMsg &eventState) {
	std::unique_lock<std::mutex> lock(update_mutex);
	eventState = updateEventState;
	return update_status;
}