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
#include "PlanningEventServiceOutWrapper.hh"

// include component's main class
#include "ComponentOpenRave.hh"

// other extensin includes

PlanningEventServiceOutWrapper::PlanningEventServiceOutWrapper(Smart::IEventServerPattern<CommManipulationPlannerObjects::CommManipulationPlannerEventParameter, CommManipulationPlannerObjects::CommManipulationPlannerEventResult, CommManipulationPlannerObjects::CommManipulationPlannerEventState> *planningEventServiceOut) {
	this->planningEventServiceOut = planningEventServiceOut;
	update_status = Smart::SMART_NODATA;
}

PlanningEventServiceOutWrapper::~PlanningEventServiceOutWrapper() {
}


Smart::StatusCode PlanningEventServiceOutWrapper::put(CommManipulationPlannerObjects::CommManipulationPlannerEventState &eventState) {
	std::unique_lock<std::mutex> lock(update_mutex);
	updateEventState = eventState;
	update_status = planningEventServiceOut->put(eventState);
	return update_status;
}

Smart::StatusCode PlanningEventServiceOutWrapper::getLatestUpdate(CommManipulationPlannerObjects::CommManipulationPlannerEventState &eventState) {
	std::unique_lock<std::mutex> lock(update_mutex);
	eventState = updateEventState;
	return update_status;
}
