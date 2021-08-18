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
#ifndef _PACKINGTASK_CORE_HH
#define _PACKINGTASK_CORE_HH
	
#include "aceSmartSoft.hh"

// include upcall interface

// include communication-objects for output ports
#include <CommObjectRecognitionObjects/CommObjectRecognitionEventResult.hh>
#include <CommObjectRecognitionObjects/CommObjectRecognitionEventState.hh>
#include <CommBasicObjects/CommVoid.hh>

// include all interaction-observer interfaces
#include <PackingTaskObserverInterface.hh>


class PackingTaskCore
:	public SmartACE::ManagedTask
,	public Smart::TaskTriggerSubject
{
private:
	bool useDefaultState; 
	bool useLogging;
	int taskLoggingId;
	unsigned int currentUpdateCount;
	
	
	
protected:
	virtual int execute_protected_region();
	
	virtual void updateAllCommObjects();
	
	virtual int getPreviousCommObjId();
	
	void triggerLogEntry(const int& idOffset);
	
	
	
	// this method is meant to be used in derived classes
	Smart::StatusCode objectEventServerPut(CommObjectRecognitionObjects::CommObjectRecognitionEventState &eventState);
	
	
/**
 * Implementation of the Subject part of an InteractionObserver
 */
private:
	std::mutex interaction_observers_mutex;
	std::list<PackingTaskObserverInterface*> interaction_observers;
protected:
	void notify_all_interaction_observers();
public:
	void attach_interaction_observer(PackingTaskObserverInterface *observer);
	void detach_interaction_observer(PackingTaskObserverInterface *observer);

public:
	PackingTaskCore(Smart::IComponent *comp, const bool &useDefaultState=true);
	virtual ~PackingTaskCore();
	
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
	
	
	virtual int start() override;
	virtual int start(const ACE_Sched_Params &sched_params, const int &cpuAffinity=-1) override;
	virtual int stop(const bool wait_till_stopped=true) override;
};
#endif
