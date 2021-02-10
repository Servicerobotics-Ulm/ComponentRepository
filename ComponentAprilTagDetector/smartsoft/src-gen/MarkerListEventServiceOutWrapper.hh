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
#ifndef _MARKERLISTEVENTSERVICEOUTWRAPPER_HH
#define _MARKERLISTEVENTSERVICEOUTWRAPPER_HH

#include <mutex>
#include "aceSmartSoft.hh"

// include communication objects
#include <CommTrackingObjects/CommDetectedMarkerEventParameter.hh>
#include <CommTrackingObjects/CommDetectedMarkerEventParameterACE.hh>
#include <CommTrackingObjects/CommDetectedMarkerEventResult.hh>
#include <CommTrackingObjects/CommDetectedMarkerEventResultACE.hh>
#include <CommTrackingObjects/CommDetectedMarkerEventState.hh>
#include <CommTrackingObjects/CommDetectedMarkerEventStateACE.hh>


class MarkerListEventServiceOutWrapper
{
private:
	std::mutex update_mutex;
	Smart::StatusCode update_status;
	
	CommTrackingObjects::CommDetectedMarkerEventState updateEventState;
	CommTrackingObjects::CommDetectedMarkerEventResult updateEvent;
	CommTrackingObjects::CommDetectedMarkerEventParameter updateActivation;
	
	Smart::IEventServerPattern<CommTrackingObjects::CommDetectedMarkerEventParameter, CommTrackingObjects::CommDetectedMarkerEventResult, CommTrackingObjects::CommDetectedMarkerEventState> *markerListEventServiceOut;
	
public:
	MarkerListEventServiceOutWrapper(Smart::IEventServerPattern<CommTrackingObjects::CommDetectedMarkerEventParameter, CommTrackingObjects::CommDetectedMarkerEventResult, CommTrackingObjects::CommDetectedMarkerEventState> *markerListEventServiceOut);
	virtual ~MarkerListEventServiceOutWrapper();
	
	Smart::StatusCode put(CommTrackingObjects::CommDetectedMarkerEventState &eventState);
	
	Smart::StatusCode getLatestUpdate(CommTrackingObjects::CommDetectedMarkerEventState &eventState);
	
};

#endif // _MARKERLISTEVENTSERVICEOUTWRAPPER_HH