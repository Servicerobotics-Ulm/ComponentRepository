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
#include "DigitalInputEventHandlerCore.hh"
#include "DigitalInputEventHandler.hh"

DigitalInputEventHandlerCore::DigitalInputEventHandlerCore(
	Smart::InputSubject<Smart::EventInputType<CommBasicObjects::CommDigitalInputEventResult>> *subject,
	const int &prescaleFactor)
	:	Smart::InputTaskTrigger<Smart::EventInputType<CommBasicObjects::CommDigitalInputEventResult>>(subject, prescaleFactor)
{
}
DigitalInputEventHandlerCore::~DigitalInputEventHandlerCore()
{  
}


void DigitalInputEventHandlerCore::updateAllCommObjects() {
}

void DigitalInputEventHandlerCore::notify_all_interaction_observers() {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	// try dynamically down-casting this class to the derived class 
	// (we can do it safely here as we exactly know the derived class)
	if(const DigitalInputEventHandler* digitalInputEventHandler = dynamic_cast<const DigitalInputEventHandler*>(this)) {
		for(auto it=interaction_observers.begin(); it!=interaction_observers.end(); it++) {
			(*it)->on_update_from(digitalInputEventHandler);
		}
	}
}

void DigitalInputEventHandlerCore::attach_interaction_observer(DigitalInputEventHandlerObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.push_back(observer);
}

void DigitalInputEventHandlerCore::detach_interaction_observer(DigitalInputEventHandlerObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.remove(observer);
}
