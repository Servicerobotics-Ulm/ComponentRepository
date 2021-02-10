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
#ifndef _SYMBOLICPANNERQUERYHANDLER_CORE_HH
#define _SYMBOLICPANNERQUERYHANDLER_CORE_HH
		
#include "aceSmartSoft.hh"


#include <DomainSymbolicPlanner/CommSymbolicPlannerPlan.hh>
#include <DomainSymbolicPlanner/CommSymbolicPlannerRequest.hh>

// include the input interfaces (if any)

// include all interaction-observer interfaces
#include <SymbolicPannerQueryHandlerObserverInterface.hh>

class SymbolicPannerQueryHandlerCore 
:	public Smart::IInputHandler<std::pair<Smart::QueryIdPtr,DomainSymbolicPlanner::CommSymbolicPlannerRequest>>
,	public Smart::TaskTriggerSubject
{
private:
	// inputs are directly propagated to the implementation (passive handler)
	virtual void handle_input(const std::pair<Smart::QueryIdPtr,DomainSymbolicPlanner::CommSymbolicPlannerRequest> &input) override {
		this->handleQuery(input.first, input.second);
	}


	virtual void updateAllCommObjects();

/**
 * Implementation of the Subject part of an InteractionObserver
 */
private:
	std::mutex interaction_observers_mutex;
	std::list<SymbolicPannerQueryHandlerObserverInterface*> interaction_observers;
protected:
	void notify_all_interaction_observers();
public:
	void attach_interaction_observer(SymbolicPannerQueryHandlerObserverInterface *observer);
	void detach_interaction_observer(SymbolicPannerQueryHandlerObserverInterface *observer);

protected:
	
public:
	using IQueryServer = Smart::IQueryServerPattern<DomainSymbolicPlanner::CommSymbolicPlannerRequest, DomainSymbolicPlanner::CommSymbolicPlannerPlan>;
	using QueryId = Smart::QueryIdPtr;
	SymbolicPannerQueryHandlerCore(IQueryServer *server);
	virtual ~SymbolicPannerQueryHandlerCore();
	
protected:
	IQueryServer *server;
	//this user-method has to be implemented in derived classes
	virtual void handleQuery(const QueryId &id, const DomainSymbolicPlanner::CommSymbolicPlannerRequest& request) = 0;
};
#endif
