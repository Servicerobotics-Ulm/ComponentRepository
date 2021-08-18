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
#include "RecorderTaskCore.hh"
#include "RecorderTask.hh"
#include "ComponentTagRecorder.hh"

//FIXME: use logging
//#include "smartGlobalLogger.hh"

// include observers


RecorderTaskCore::RecorderTaskCore(Smart::IComponent *comp, const bool &useDefaultState) 
:	SmartACE::ManagedTask(comp)
,	useDefaultState(useDefaultState)
,	useLogging(false)
,	taskLoggingId(0)
,	currentUpdateCount(0)
{
}

RecorderTaskCore::~RecorderTaskCore()
{
}

int RecorderTaskCore::start() {
	this->resetTrigger();
	COMP->stateActivityManager->attach(this, "active");
	return SmartACE::Task::start();
}

int RecorderTaskCore::start(const ACE_Sched_Params &sched_params, const int &cpuAffinity) {
	return SmartACE::Task::start(sched_params, cpuAffinity);
}

int RecorderTaskCore::stop(const bool wait_till_stopped) {
	COMP->stateActivityManager->detach(this, "active");
	this->cancelTrigger();
	return SmartACE::Task::stop(wait_till_stopped);
}


void RecorderTaskCore::notify_all_interaction_observers() {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	// try dynamically down-casting this class to the derived class 
	// (we can do it safely here as we exactly know the derived class)
	if(const RecorderTask* recorderTask = dynamic_cast<const RecorderTask*>(this)) {
		for(auto it=interaction_observers.begin(); it!=interaction_observers.end(); it++) {
			(*it)->on_update_from(recorderTask);
		}
	}
}

void RecorderTaskCore::attach_interaction_observer(RecorderTaskObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.push_back(observer);
}

void RecorderTaskCore::detach_interaction_observer(RecorderTaskObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.remove(observer);
}

int RecorderTaskCore::execute_protected_region()
{
	if(useDefaultState) {
		Smart::StatusCode status = COMP->stateActivityManager->acquire("active", this);
		if(status == Smart::SMART_CANCELLED) {
			std::cout << "state canceled -> stop activity RecorderTask" << std::endl;
			return -1;
		} else if(status != Smart::SMART_OK) {
			std::cerr << "RecorderTaskCore: ERROR acquiring state: " << status << std::endl;
			usleep(500000);
			return 0;
		}
	}
	
	// update of comm-objects must be within the protected region to prevent aged comm-object values
	this->updateAllCommObjects();
	
	if(useLogging == true) {
		//FIXME: use logging
		//Smart::LOGGER->log(taskLoggingId, getCurrentUpdateCount(), getPreviousCommObjId());
	}
	
	// this is the user code (should not internally use the state-pattern any more)
	int retval = this->on_execute();
	
	// notify all attached interaction observers
	this->notify_all_interaction_observers();
	
	// inform all associated tasks about a new update
	this->trigger_all_tasks();
	
	// increment current currentUpdateCount for the next iteration
	currentUpdateCount++;
	
	if(useDefaultState) {
		COMP->stateActivityManager->release("active");
	}
	return retval;
}


void RecorderTaskCore::updateAllCommObjects()
{
	
}



void RecorderTaskCore::triggerLogEntry(const int& idOffset)
{
	if(useLogging == true) {
		int logId = taskLoggingId + 2*0 + idOffset;
		//FIXME: use logging
		//Smart::LOGGER->log(logId, getCurrentUpdateCount(), getPreviousCommObjId());
	}
}

int RecorderTaskCore::getPreviousCommObjId()
{
	// this method needs to be overloaded and implemented in derived classes
	return 0;
}
