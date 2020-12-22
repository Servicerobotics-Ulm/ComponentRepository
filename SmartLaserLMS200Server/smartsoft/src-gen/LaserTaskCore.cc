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
#include "LaserTaskCore.hh"
#include "LaserTask.hh"
#include "SmartLaserLMS200Server.hh"

//FIXME: use logging
//#include "smartGlobalLogger.hh"

// include observers


LaserTaskCore::LaserTaskCore(Smart::IComponent *comp, const bool &useDefaultState) 
:	SmartACE::ManagedTask(comp)
,	useDefaultState(useDefaultState)
,	useLogging(false)
,	taskLoggingId(0)
,	currentUpdateCount(0)
,	baseStateInStatus(Smart::SMART_DISCONNECTED)
,	baseStateInObject()
{
}

LaserTaskCore::~LaserTaskCore()
{
}


void LaserTaskCore::notify_all_interaction_observers() {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	// try dynamically down-casting this class to the derived class 
	// (we can do it safely here as we exactly know the derived class)
	if(const LaserTask* laserTask = dynamic_cast<const LaserTask*>(this)) {
		for(auto it=interaction_observers.begin(); it!=interaction_observers.end(); it++) {
			(*it)->on_update_from(laserTask);
		}
	}
}

void LaserTaskCore::attach_interaction_observer(LaserTaskObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.push_back(observer);
}

void LaserTaskCore::detach_interaction_observer(LaserTaskObserverInterface *observer) {
	std::unique_lock<std::mutex> lock(interaction_observers_mutex);
	interaction_observers.remove(observer);
}

int LaserTaskCore::execute_protected_region()
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


void LaserTaskCore::updateAllCommObjects()
{
	baseStateInStatus = COMP->baseStateInInputTaskTrigger->getUpdate(baseStateInObject);
	
}


// this method is meant to be used in derived classes
Smart::StatusCode LaserTaskCore::laserServiceOutPut(CommBasicObjects::CommMobileLaserScan &laserServiceOutDataObject)
{
	Smart::StatusCode result = COMP->laserServiceOutWrapper->put(laserServiceOutDataObject);
	if(useLogging == true) {
		//FIXME: use logging
		//Smart::LOGGER->log(pushLoggingId+1, getCurrentUpdateCount(), getPreviousCommObjId());
	}
	return result;
}

void LaserTaskCore::triggerLogEntry(const int& idOffset)
{
	if(useLogging == true) {
		int logId = taskLoggingId + 2*1 + idOffset;
		//FIXME: use logging
		//Smart::LOGGER->log(logId, getCurrentUpdateCount(), getPreviousCommObjId());
	}
}

int LaserTaskCore::getPreviousCommObjId()
{
	// this method needs to be overloaded and implemented in derived classes
	return 0;
}
