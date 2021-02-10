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
#ifndef _PATHPLANNING_CORE_HH
#define _PATHPLANNING_CORE_HH
	
#include "aceSmartSoft.hh"

// include upcall interface
#include "PathPlanningRequestInUpcallInterface.hh"

// include communication-objects for output ports
#include <CommManipulatorObjects/CommManipulatorTrajectory.hh>

// include all interaction-observer interfaces
#include <PathPlanningObserverInterface.hh>


class PathPlanningCore
:	public SmartACE::ManagedTask
,	public Smart::TaskTriggerSubject
,	public PathPlanningRequestInUpcallInterface
{
private:
	bool useDefaultState; 
	bool useLogging;
	int taskLoggingId;
	unsigned int currentUpdateCount;
	
	Smart::StatusCode pathPlanningRequestInStatus;
	CommManipulatorObjects::CommManipulatorTrajectory pathPlanningRequestInObject;
	
	
protected:
	virtual int execute_protected_region();
	
	virtual void updateAllCommObjects();
	
	virtual int getPreviousCommObjId();
	
	void triggerLogEntry(const int& idOffset);
	
	
	// overload and implement this method in derived classes to immediately get all incoming updates from PathPlanningRequestIn (as soon as they arrive)
	virtual void on_PathPlanningRequestIn(const CommManipulatorObjects::CommManipulatorTrajectory &input) {
		// no-op
	}
	
	// this method can be safely used from the thread in derived classes
	inline Smart::StatusCode pathPlanningRequestInGetUpdate(CommManipulatorObjects::CommManipulatorTrajectory &pathPlanningRequestInObject) const
	{
		// copy local object buffer and return the last status code
		pathPlanningRequestInObject = this->pathPlanningRequestInObject;
		return pathPlanningRequestInStatus;
	}
	
	// this method is meant to be used in derived classes
	Smart::StatusCode sendPathTrajectoryOutPut(CommManipulatorObjects::CommManipulatorTrajectory &sendPathTrajectoryOutDataObject);
	
	
/**
 * Implementation of the Subject part of an InteractionObserver
 */
private:
	std::mutex interaction_observers_mutex;
	std::list<PathPlanningObserverInterface*> interaction_observers;
protected:
	void notify_all_interaction_observers();
public:
	void attach_interaction_observer(PathPlanningObserverInterface *observer);
	void detach_interaction_observer(PathPlanningObserverInterface *observer);

public:
	PathPlanningCore(Smart::IComponent *comp, const bool &useDefaultState=true);
	virtual ~PathPlanningCore();
	
	inline void setUpLogging(const int &taskNbr, const bool &useLogging=true) {
		this->taskLoggingId = taskNbr;
		this->useLogging = useLogging;
	}
	
	inline bool isLoggingActive() const {
		return useLogging;
	}
	
	inline int getLoggingID() const {
		return taskLoggingId;
	}
	
	inline int getCurrentUpdateCount() const {
		return currentUpdateCount;
	}
	
};
#endif
