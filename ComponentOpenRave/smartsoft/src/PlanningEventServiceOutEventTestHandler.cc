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
#include "PlanningEventServiceOutEventTestHandler.hh"

bool PlanningEventServiceOutEventTestHandler::testEvent(
	CommManipulationPlannerObjects::CommManipulationPlannerEventParameter &p,
	CommManipulationPlannerObjects::CommManipulationPlannerEventResult &r,
	const CommManipulationPlannerObjects::CommManipulationPlannerEventState &s
) throw() {
	// fire all events (without filtering) in the default implementation
	// implement your own (specific) event-filtering code using the event-parameter as input
	// true means that the current event will be fired to the according client
	// false means that the current event is ignored (it will not be communicated to the according client)
	bool result = false;

	CommManipulationPlannerObjects::ManipulationPlannerEvent oldState(CommManipulationPlannerObjects::ManipulationPlannerEvent::UNKNOWN);
	CommManipulationPlannerObjects::ManipulationPlannerEvent newState(CommManipulationPlannerObjects::ManipulationPlannerEvent::UNKNOWN);

	oldState = p.get_event();
	newState = s.get_event();

	if (oldState == newState) {
		// no state change -> no event
		result = false;
	} else {
		// memorize new state (state change) for currently tested event
		// each activated event has its own parameter
		p.set_event(newState);

		// set result
		r.set_event(newState);
		r.set_pose(s.get_pose());
		result = true;
	}

	// true --> send event
	// false --> don't send event
   	return result;
}
