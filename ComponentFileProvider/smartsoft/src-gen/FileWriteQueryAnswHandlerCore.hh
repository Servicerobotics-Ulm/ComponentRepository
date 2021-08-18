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
#ifndef _FILEWRITEQUERYANSWHANDLER_CORE_HH
#define _FILEWRITEQUERYANSWHANDLER_CORE_HH
		
#include "aceSmartSoft.hh"


#include <CommBasicObjects/CommFileWriteAnswer.hh>
#include <CommBasicObjects/CommFileWriteRequest.hh>

// include the input interfaces (if any)

// include all interaction-observer interfaces
#include <FileWriteQueryAnswHandlerObserverInterface.hh>

class FileWriteQueryAnswHandlerCore 
:	public Smart::IInputHandler<std::pair<Smart::QueryIdPtr,CommBasicObjects::CommFileWriteRequest>>
,	public Smart::TaskTriggerSubject
{
private:
	// inputs are directly propagated to the implementation (passive handler)
	virtual void handle_input(const std::pair<Smart::QueryIdPtr,CommBasicObjects::CommFileWriteRequest> &input) override {
		this->handleQuery(input.first, input.second);
	}


	virtual void updateAllCommObjects();

/**
 * Implementation of the Subject part of an InteractionObserver
 */
private:
	std::mutex interaction_observers_mutex;
	std::list<FileWriteQueryAnswHandlerObserverInterface*> interaction_observers;
protected:
	void notify_all_interaction_observers();
public:
	void attach_interaction_observer(FileWriteQueryAnswHandlerObserverInterface *observer);
	void detach_interaction_observer(FileWriteQueryAnswHandlerObserverInterface *observer);

protected:
	
public:
	using IQueryServer = Smart::IQueryServerPattern<CommBasicObjects::CommFileWriteRequest, CommBasicObjects::CommFileWriteAnswer>;
	using QueryId = Smart::QueryIdPtr;
	FileWriteQueryAnswHandlerCore(IQueryServer *server);
	virtual ~FileWriteQueryAnswHandlerCore();
	
protected:
	IQueryServer *server;
	//this user-method has to be implemented in derived classes
	virtual void handleQuery(const QueryId &id, const CommBasicObjects::CommFileWriteRequest& request) = 0;
};
#endif
