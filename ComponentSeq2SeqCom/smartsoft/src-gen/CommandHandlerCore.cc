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
#include "CommandHandlerCore.hh"
#include "CommandHandler.hh"

CommandHandlerCore::CommandHandlerCore(
	Smart::InputSubject<CommBasicObjects::CommTaskMessage> *subject,
	const int &prescaleFactor)
	:	Smart::InputTaskTrigger<CommBasicObjects::CommTaskMessage>(subject, prescaleFactor)
{  
	updateStatus = Smart::SMART_NODATA;
}
CommandHandlerCore::~CommandHandlerCore()
{  }

void CommandHandlerCore::updateAllCommObjects() {
}

void CommandHandlerCore::notify_all_interaction_observers() {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	// try dynamically down-casting this class to the derived class 
	// (we can do it safely here as we exactly know the derived class)
	if(const CommandHandler* commandHandler = dynamic_cast<const CommandHandler*>(this)) {
		for(auto it=interaction_observers.begin(); it!=interaction_observers.end(); it++) {
			(*it)->on_update_from(commandHandler);
		}
	}
}

void CommandHandlerCore::attach_interaction_observer(CommandHandlerObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.push_back(observer);
}

void CommandHandlerCore::detach_interaction_observer(CommandHandlerObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.remove(observer);
}
