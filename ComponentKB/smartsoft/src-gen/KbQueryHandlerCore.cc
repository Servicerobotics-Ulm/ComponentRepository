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
#include "KbQueryHandlerCore.hh"
#include "KbQueryHandler.hh"

// include observers

KbQueryHandlerCore::KbQueryHandlerCore(IQueryServer* server)
:	Smart::IInputHandler<std::pair<Smart::QueryIdPtr,CommBasicObjects::CommKBRequest>>(server)
,	server(server)
{
	signalled_to_stop = false;
	this->start();
}

KbQueryHandlerCore::~KbQueryHandlerCore()
{
	this->stop();
}

// inputs are pushed to the request_queue
void KbQueryHandlerCore::handle_input(const std::pair<Smart::QueryIdPtr,CommBasicObjects::CommKBRequest> &input) {
	std::unique_lock<std::mutex> scoped_lock(handler_mutex);
	request_queue.push_back(input);
	scoped_lock.unlock();
	
	handler_cond_var.notify_all();
}
// inputs are processed from within the thread, thus implementing an active FIFO request-queue
int KbQueryHandlerCore::task_execution() {
	while(!signalled_to_stop) {
		std::unique_lock<std::mutex> scoped_lock(handler_mutex);
		if(request_queue.empty()) {
			handler_cond_var.wait(scoped_lock);
			if(signalled_to_stop)
				return 0;
		}
		std::pair<Smart::QueryIdPtr,CommBasicObjects::CommKBRequest> input = request_queue.front();
		request_queue.pop_front();
		scoped_lock.unlock();
		
		this->handleQuery(input.first, input.second);
	}
	return 0;
}

void KbQueryHandlerCore::updateAllCommObjects()
{
}

void KbQueryHandlerCore::notify_all_interaction_observers() {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	// try dynamically down-casting this class to the derived class 
	// (we can do it safely here as we exactly know the derived class)
	if(const KbQueryHandler* kbQueryHandler = dynamic_cast<const KbQueryHandler*>(this)) {
		for(auto it=interaction_observers.begin(); it!=interaction_observers.end(); it++) {
			(*it)->on_update_from(kbQueryHandler);
		}
	}
}

void KbQueryHandlerCore::attach_interaction_observer(KbQueryHandlerObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.push_back(observer);
}

void KbQueryHandlerCore::detach_interaction_observer(KbQueryHandlerObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.remove(observer);
}
