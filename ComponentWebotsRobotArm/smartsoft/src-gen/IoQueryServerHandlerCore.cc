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
#include "IoQueryServerHandlerCore.hh"
#include "IoQueryServerHandler.hh"

// include observers

IoQueryServerHandlerCore::IoQueryServerHandlerCore(IQueryServer* server)
:	Smart::IInputHandler<std::pair<Smart::QueryIdPtr,CommBasicObjects::CommIOValues>>(server)
,	server(server)
{
}

IoQueryServerHandlerCore::~IoQueryServerHandlerCore()
{
}


void IoQueryServerHandlerCore::updateAllCommObjects()
{
}

void IoQueryServerHandlerCore::notify_all_interaction_observers() {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	// try dynamically down-casting this class to the derived class 
	// (we can do it safely here as we exactly know the derived class)
	if(const IoQueryServerHandler* ioQueryServerHandler = dynamic_cast<const IoQueryServerHandler*>(this)) {
		for(auto it=interaction_observers.begin(); it!=interaction_observers.end(); it++) {
			(*it)->on_update_from(ioQueryServerHandler);
		}
	}
}

void IoQueryServerHandlerCore::attach_interaction_observer(IoQueryServerHandlerObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.push_back(observer);
}

void IoQueryServerHandlerCore::detach_interaction_observer(IoQueryServerHandlerObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.remove(observer);
}