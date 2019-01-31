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
#include "CommRobotinoDigitalEventOutEventTestHandler.hh"

bool CommRobotinoDigitalEventOutEventTestHandler::testEvent(
	CommRobotinoObjects::CommDigitalInputEventParameter &p,
	CommRobotinoObjects::CommDigitalInputEventResult &r,
	const CommRobotinoObjects::CommDigitalInputEventState &s
) throw() {


	bool result = false;

		int number = p.getDigitalInputValuesNumber();
		bool desired_value = p.getDigitalInputValue();

		if(number<s.getDigitalInputValuesSize() && number >=0){
			if(s.getDigitalInputValuesElemAtPos(number) == desired_value){
				result = true;
				std::cout<<__FUNCTION__<<"Digital Input Event: Fire event: "<<r<<" param: "<<p<<std::endl;
				r.setDigitalInputValues(s.getDigitalInputValuesCopy());
			}
		} else {
			std::cout<<__FUNCTION__<<"Error: Requested digital input value out of range!"<<std::endl;
		}


		// true --> send event
		// false --> don't send event
	   	return result;


	// fire all events (without filtering) in the default implementation
	// implement your own (specific) event-filtering code using the event-parameter as input
	// true means that the current event will be fired to the according client
	// false means that the current event is ignored (it will not be communicated to the according client)
	//return true;
}
