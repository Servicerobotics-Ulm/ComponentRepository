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
#ifndef _TASKSENDINHANDLER_CORE_HH
#define _TASKSENDINHANDLER_CORE_HH

#include <aceSmartSoft.hh>

// include the main input-handler interface
#include "TaskSendInUpcallInterface.hh"
// include all other input interfaces (if any)

// include all interaction-observer interfaces
#include <TaskSendInHandlerObserverInterface.hh>

class TaskSendInHandlerCore
:	public Smart::InputTaskTrigger<CommBasicObjects::CommTaskMessage>
,	public TaskSendInUpcallInterface
{
private:
	
	virtual void updateAllCommObjects();
	
	// internal input handling method
	virtual void handle_input(const CommBasicObjects::CommTaskMessage& input) {
		this->updateAllCommObjects();
		// call the input handler method (which has to be implemented in derived classes)
		this->on_TaskSendIn(input);
		// notify all attached interaction observers
		this->notify_all_interaction_observers();
		// call implementation of base class
		Smart::InputTaskTrigger<CommBasicObjects::CommTaskMessage>::handle_input(input);
	}
	
/**
 * Implementation of the Subject part of an InteractionObserver
 */
private:
	std::mutex interaction_observers_mutex;
	std::list<TaskSendInHandlerObserverInterface*> interaction_observers;
protected:
	void notify_all_interaction_observers();
public:
	void attach_interaction_observer(TaskSendInHandlerObserverInterface *observer);
	void detach_interaction_observer(TaskSendInHandlerObserverInterface *observer);
	
protected:

public:
	TaskSendInHandlerCore(
		Smart::InputSubject<CommBasicObjects::CommTaskMessage> *subject,
		const int &prescaleFactor=1);
	virtual ~TaskSendInHandlerCore();
};
#endif