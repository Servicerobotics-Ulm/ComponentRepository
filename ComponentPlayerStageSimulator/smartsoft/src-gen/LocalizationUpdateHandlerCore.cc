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
#include "LocalizationUpdateHandlerCore.hh"
#include "LocalizationUpdateHandler.hh"

LocalizationUpdateHandlerCore::LocalizationUpdateHandlerCore(
	Smart::InputSubject<CommBasicObjects::CommBasePositionUpdate> *subject,
	const int &prescaleFactor)
	:	Smart::InputTaskTrigger<CommBasicObjects::CommBasePositionUpdate>(subject, prescaleFactor)
{
}
LocalizationUpdateHandlerCore::~LocalizationUpdateHandlerCore()
{  
}


void LocalizationUpdateHandlerCore::updateAllCommObjects() {
}

void LocalizationUpdateHandlerCore::notify_all_interaction_observers() {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	// try dynamically down-casting this class to the derived class 
	// (we can do it safely here as we exactly know the derived class)
	if(const LocalizationUpdateHandler* localizationUpdateHandler = dynamic_cast<const LocalizationUpdateHandler*>(this)) {
		for(auto it=interaction_observers.begin(); it!=interaction_observers.end(); it++) {
			(*it)->on_update_from(localizationUpdateHandler);
		}
	}
}

void LocalizationUpdateHandlerCore::attach_interaction_observer(LocalizationUpdateHandlerObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.push_back(observer);
}

void LocalizationUpdateHandlerCore::detach_interaction_observer(LocalizationUpdateHandlerObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.remove(observer);
}
