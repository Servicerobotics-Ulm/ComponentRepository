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
// This file is generated once. Modify this file to your needs. 
// If you want the toolchain to re-generate this file, please 
// delete it before running the code generator.
//--------------------------------------------------------------------------
#include "ObjectEventServerEventTestHandler.hh"

bool ObjectEventServerEventTestHandler::testEvent(
	CommBasicObjects::CommVoid &p,
	CommObjectRecognitionObjects::CommObjectRecognitionEventResult &r,
	const CommObjectRecognitionObjects::CommObjectRecognitionEventState &s
) throw() {
	bool result = true;

	r.set_state(s.get_state());
	r.set_environment_id(s.get_environment_id());
	r.set_object_id_size(s.get_object_id_size());

	for (uint32_t i = 0; i < s.get_object_id_size(); ++i) {
		r.set_object_id(i, s.get_object_id(i));
	}
	std::cout << "[EventTestHandler]: Result size: " << r.get_object_id_size() << std::endl;

	return result;
}
