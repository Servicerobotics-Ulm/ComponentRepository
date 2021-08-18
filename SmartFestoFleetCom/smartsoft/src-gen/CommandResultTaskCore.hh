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
#ifndef _COMMANDRESULTTASK_CORE_HH
#define _COMMANDRESULTTASK_CORE_HH
	
#include "aceSmartSoft.hh"

// include upcall interface
#include "TaskResultInUpcallInterface.hh"

// include communication-objects for output ports

// include all interaction-observer interfaces
#include <CommandResultTaskObserverInterface.hh>


class CommandResultTaskCore
:	public SmartACE::ManagedTask
,	public Smart::TaskTriggerSubject
,	public TaskResultInUpcallInterface
{
private:
	bool useDefaultState; 
	bool useLogging;
	int taskLoggingId;
	unsigned int currentUpdateCount;
	
	Smart::StatusCode taskResultInStatus;
	CommBasicObjects::CommTaskMessage taskResultInObject;
	
	
protected:
	virtual int execute_protected_region();
	
	virtual void updateAllCommObjects();
	
	virtual int getPreviousCommObjId();
	
	void triggerLogEntry(const int& idOffset);
	
	
	// overload and implement this method in derived classes to immediately get all incoming updates from TaskResultIn (as soon as they arrive)
	virtual void on_TaskResultIn(const CommBasicObjects::CommTaskMessage &input) {
		// no-op
	}
	
	// this method can be safely used from the thread in derived classes
	inline Smart::StatusCode taskResultInGetUpdate(CommBasicObjects::CommTaskMessage &taskResultInObject) const
	{
		// copy local object buffer and return the last status code
		taskResultInObject = this->taskResultInObject;
		return taskResultInStatus;
	}
	
	
	
/**
 * Implementation of the Subject part of an InteractionObserver
 */
private:
	std::mutex interaction_observers_mutex;
	std::list<CommandResultTaskObserverInterface*> interaction_observers;
protected:
	void notify_all_interaction_observers();
public:
	void attach_interaction_observer(CommandResultTaskObserverInterface *observer);
	void detach_interaction_observer(CommandResultTaskObserverInterface *observer);

public:
	CommandResultTaskCore(Smart::IComponent *comp, const bool &useDefaultState=true);
	virtual ~CommandResultTaskCore();
	
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
