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
#include "SafetyfieldEventServerEventTestHandler.hh"

bool SafetyfieldEventServerEventTestHandler::testEvent(
	CommBasicObjects::CommLaserSafetyEventParam &p,
	CommBasicObjects::CommLaserSafetyField &r,
	const CommBasicObjects::CommLaserSafetyEventState &s
) throw() {
	bool changed = false;
	CommBasicObjects::SafetyFieldState changedState;
	//std::cout<<"Param: "<<p<<std::endl;
	//check if anything changed and remember the state it changed to
	//std::cout<<"State: "<<s<<std::endl;

	if(p.getWarningState() != s.getWarningState()){
		changed = true;
	}

	if(p.getProtectiveState() != s.getProtectiveState()){
		changed = true;
	}

	if(changed)
	{
		p.setProtectiveState(s.getProtectiveState());
		p.setWarningState(s.getWarningState());

		r.setProtectiveState(s.getProtectiveState());
		r.setWarningState(s.getWarningState());

		std::cout<<"fire event"<<std::endl;
	}

	// true --> send event
	// false --> don't send event
	return changed;
}
