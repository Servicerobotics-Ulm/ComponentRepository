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
#include "SpeechSendServiceOutWrapper.hh"

// include component's main class
#include "ComponentTTSClient.hh"

// other extensin includes

SpeechSendServiceOutWrapper::SpeechSendServiceOutWrapper(Smart::ISendClientPattern<DomainSpeech::CommSpeechOutputMessage> *speechSendServiceOut) {
	this->speechSendServiceOut = speechSendServiceOut;
	update_status = Smart::SMART_NODATA;
}

SpeechSendServiceOutWrapper::~SpeechSendServiceOutWrapper() {
}


Smart::StatusCode SpeechSendServiceOutWrapper::send(DomainSpeech::CommSpeechOutputMessage &speechSendServiceOutDataObject) {
	std::unique_lock<std::mutex> lock(update_mutex);
	updateData = speechSendServiceOutDataObject;
	update_status = speechSendServiceOut->send(speechSendServiceOutDataObject);
	return update_status;
}

Smart::StatusCode SpeechSendServiceOutWrapper::getLatestUpdate(DomainSpeech::CommSpeechOutputMessage &speechSendServiceOutDataObject) {
	std::unique_lock<std::mutex> lock(update_mutex);
	speechSendServiceOutDataObject = updateData;
	return update_status;
}
