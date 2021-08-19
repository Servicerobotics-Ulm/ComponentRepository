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
#include "JobSource3EventHandlerCore.hh"
#include "JobSource3EventHandler.hh"

JobSource3EventHandlerCore::JobSource3EventHandlerCore(
	Smart::InputSubject<Smart::EventInputType<CommBasicObjects::CommTaskMessage>> *subject,
	const int &prescaleFactor)
	:	Smart::InputTaskTrigger<Smart::EventInputType<CommBasicObjects::CommTaskMessage>>(subject, prescaleFactor)
{
}
JobSource3EventHandlerCore::~JobSource3EventHandlerCore()
{  
}


void JobSource3EventHandlerCore::updateAllCommObjects() {
}

void JobSource3EventHandlerCore::notify_all_interaction_observers() {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	// try dynamically down-casting this class to the derived class 
	// (we can do it safely here as we exactly know the derived class)
	if(const JobSource3EventHandler* jobSource3EventHandler = dynamic_cast<const JobSource3EventHandler*>(this)) {
		for(auto it=interaction_observers.begin(); it!=interaction_observers.end(); it++) {
			(*it)->on_update_from(jobSource3EventHandler);
		}
	}
}

void JobSource3EventHandlerCore::attach_interaction_observer(JobSource3EventHandlerObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.push_back(observer);
}

void JobSource3EventHandlerCore::detach_interaction_observer(JobSource3EventHandlerObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.remove(observer);
}
