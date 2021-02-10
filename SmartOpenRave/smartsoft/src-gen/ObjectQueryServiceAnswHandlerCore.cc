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
#include "ObjectQueryServiceAnswHandlerCore.hh"
#include "ObjectQueryServiceAnswHandler.hh"

// include observers

ObjectQueryServiceAnswHandlerCore::ObjectQueryServiceAnswHandlerCore(Smart::IQueryServerPattern<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties, SmartACE::QueryId>* server)
:	Smart::IQueryServerHandler<CommObjectRecognitionObjects::CommObjectRecognitionId, CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties, SmartACE::QueryId>(server)
{
	
}

ObjectQueryServiceAnswHandlerCore::~ObjectQueryServiceAnswHandlerCore()
{
	
}

void ObjectQueryServiceAnswHandlerCore::updateAllCommObjects()
{
}

void ObjectQueryServiceAnswHandlerCore::notify_all_interaction_observers() {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	// try dynamically down-casting this class to the derived class 
	// (we can do it safely here as we exactly know the derived class)
	if(const ObjectQueryServiceAnswHandler* objectQueryServiceAnswHandler = dynamic_cast<const ObjectQueryServiceAnswHandler*>(this)) {
		for(auto it=interaction_observers.begin(); it!=interaction_observers.end(); it++) {
			(*it)->on_update_from(objectQueryServiceAnswHandler);
		}
	}
}

void ObjectQueryServiceAnswHandlerCore::attach_interaction_observer(ObjectQueryServiceAnswHandlerObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.push_back(observer);
}

void ObjectQueryServiceAnswHandlerCore::detach_interaction_observer(ObjectQueryServiceAnswHandlerObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.remove(observer);
}