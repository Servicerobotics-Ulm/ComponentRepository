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
#include "BaseStateTaskCore.hh"
#include "BaseStateTask.hh"
#include "SmartGazeboBaseServer.hh"

//FIXME: use logging
//#include "smartGlobalLogger.hh"

// include observers
#include "LocalizationUpdateHandler.hh"


BaseStateTaskCore::BaseStateTaskCore(Smart::IComponent *comp, const bool &useDefaultState) 
:	SmartACE::ManagedTask(comp)
,	useDefaultState(useDefaultState)
,	useLogging(false)
,	taskLoggingId(0)
,	currentUpdateCount(0)
{
}

BaseStateTaskCore::~BaseStateTaskCore()
{
}

int BaseStateTaskCore::start() {
	this->resetTrigger();
	return SmartACE::Task::start();
}

int BaseStateTaskCore::start(const ACE_Sched_Params &sched_params, const int &cpuAffinity) {
	return SmartACE::Task::start(sched_params, cpuAffinity);
}

int BaseStateTaskCore::stop(const bool wait_till_stopped) {
	this->cancelTrigger();
	return SmartACE::Task::stop(wait_till_stopped);
}


void BaseStateTaskCore::notify_all_interaction_observers() {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	// try dynamically down-casting this class to the derived class 
	// (we can do it safely here as we exactly know the derived class)
	if(const BaseStateTask* baseStateTask = dynamic_cast<const BaseStateTask*>(this)) {
		for(auto it=interaction_observers.begin(); it!=interaction_observers.end(); it++) {
			(*it)->on_update_from(baseStateTask);
		}
	}
}

void BaseStateTaskCore::attach_interaction_observer(BaseStateTaskObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.push_back(observer);
}

void BaseStateTaskCore::detach_interaction_observer(BaseStateTaskObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.remove(observer);
}

int BaseStateTaskCore::execute_protected_region()
{
	
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
	
	return retval;
}


void BaseStateTaskCore::updateAllCommObjects()
{
	
}


// this method is meant to be used in derived classes
Smart::StatusCode BaseStateTaskCore::baseStateServiceOutPut(CommBasicObjects::CommBaseState &baseStateServiceOutDataObject)
{
	Smart::StatusCode result = COMP->baseStateServiceOutWrapper->put(baseStateServiceOutDataObject);
	if(useLogging == true) {
		//FIXME: use logging
		//Smart::LOGGER->log(pushLoggingId+1, getCurrentUpdateCount(), getPreviousCommObjId());
	}
	return result;
}
// this method is meant to be used in derived classes
Smart::StatusCode BaseStateTaskCore::batteryEventServiceOutPut(CommBasicObjects::CommBatteryState &eventState)
{
	Smart::StatusCode result = COMP->batteryEventServiceOutWrapper->put(eventState);
	if(useLogging == true) {
		//FIXME: use logging
		//Smart::LOGGER->log(pushLoggingId+1, getCurrentUpdateCount(), getPreviousCommObjId());
	}
	return result;
}
// this method is meant to be used in derived classes
Smart::StatusCode BaseStateTaskCore::bumperEventServiceOutPut(CommBasicObjects::CommBumperEventState &eventState)
{
	Smart::StatusCode result = COMP->bumperEventServiceOutWrapper->put(eventState);
	if(useLogging == true) {
		//FIXME: use logging
		//Smart::LOGGER->log(pushLoggingId+1, getCurrentUpdateCount(), getPreviousCommObjId());
	}
	return result;
}

void BaseStateTaskCore::triggerLogEntry(const int& idOffset)
{
	if(useLogging == true) {
		int logId = taskLoggingId + 2*4 + idOffset;
		//FIXME: use logging
		//Smart::LOGGER->log(logId, getCurrentUpdateCount(), getPreviousCommObjId());
	}
}

int BaseStateTaskCore::getPreviousCommObjId()
{
	// this method needs to be overloaded and implemented in derived classes
	return 0;
}
