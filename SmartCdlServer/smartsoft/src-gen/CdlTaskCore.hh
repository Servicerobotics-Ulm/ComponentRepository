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
#ifndef _CDLTASK_CORE_HH
#define _CDLTASK_CORE_HH
	
#include "aceSmartSoft.hh"

// include upcall interface
#include "BaseStateClientUpcallInterface.hh"
#include "IRClientUpcallInterface.hh"
#include "LaserClientUpcallInterface.hh"
#include "LaserClient2UpcallInterface.hh"
#include "NavVelSendServerUpcallInterface.hh"
#include "PathNavigationGoalClientUpcallInterface.hh"
#include "PlannerClientUpcallInterface.hh"
#include "TrackingClientUpcallInterface.hh"

// include communication-objects for output ports
#include <CommNavigationObjects/CdlGoalEventState.hh>
#include <CommNavigationObjects/CommCdlGoalEventParameter.hh>
#include <CommNavigationObjects/CommCdlGoalEventResult.hh>
#include <CommBasicObjects/CommNavigationVelocity.hh>
#include <CommNavigationObjects/CommCdlRobotBlockedEventParameter.hh>
#include <CommNavigationObjects/CommCdlRobotBlockedEventResult.hh>
#include <CommNavigationObjects/CommCdlRobotBlockedState.hh>

// include all interaction-observer interfaces
#include <CdlTaskObserverInterface.hh>


class CdlTaskCore
:	public SmartACE::ManagedTask
,	public Smart::TaskTriggerSubject
,	public BaseStateClientUpcallInterface
,	public IRClientUpcallInterface
,	public LaserClientUpcallInterface
,	public LaserClient2UpcallInterface
,	public NavVelSendServerUpcallInterface
,	public PathNavigationGoalClientUpcallInterface
,	public PlannerClientUpcallInterface
,	public TrackingClientUpcallInterface
{
private:
	bool useDefaultState; 
	bool useLogging;
	int taskLoggingId;
	unsigned int currentUpdateCount;
	
	Smart::StatusCode baseStateClientStatus;
	CommBasicObjects::CommBaseState baseStateClientObject;
	Smart::StatusCode iRClientStatus;
	CommBasicObjects::CommMobileIRScan iRClientObject;
	Smart::StatusCode laserClientStatus;
	CommBasicObjects::CommMobileLaserScan laserClientObject;
	Smart::StatusCode laserClient2Status;
	CommBasicObjects::CommMobileLaserScan laserClient2Object;
	Smart::StatusCode navVelSendServerStatus;
	CommBasicObjects::CommNavigationVelocity navVelSendServerObject;
	Smart::StatusCode pathNavigationGoalClientStatus;
	CommNavigationObjects::CommCorridorNavigationGoal pathNavigationGoalClientObject;
	Smart::StatusCode plannerClientStatus;
	CommNavigationObjects::CommPlannerGoal plannerClientObject;
	Smart::StatusCode trackingClientStatus;
	CommTrackingObjects::CommTrackingGoal trackingClientObject;
	
	
protected:
	virtual int execute_protected_region();
	
	virtual void updateAllCommObjects();
	
	virtual int getPreviousCommObjId();
	
	void triggerLogEntry(const int& idOffset);
	
	
	// overload and implement this method in derived classes to immediately get all incoming updates from BaseStateClient (as soon as they arrive)
	virtual void on_BaseStateClient(const CommBasicObjects::CommBaseState &input) {
		// no-op
	}
	
	// this method can be safely used from the thread in derived classes
	inline Smart::StatusCode baseStateClientGetUpdate(CommBasicObjects::CommBaseState &baseStateClientObject) const
	{
		// copy local object buffer and return the last status code
		baseStateClientObject = this->baseStateClientObject;
		return baseStateClientStatus;
	}
	// overload and implement this method in derived classes to immediately get all incoming updates from IRClient (as soon as they arrive)
	virtual void on_IRClient(const CommBasicObjects::CommMobileIRScan &input) {
		// no-op
	}
	
	// this method can be safely used from the thread in derived classes
	inline Smart::StatusCode iRClientGetUpdate(CommBasicObjects::CommMobileIRScan &iRClientObject) const
	{
		// copy local object buffer and return the last status code
		iRClientObject = this->iRClientObject;
		return iRClientStatus;
	}
	// overload and implement this method in derived classes to immediately get all incoming updates from LaserClient (as soon as they arrive)
	virtual void on_LaserClient(const CommBasicObjects::CommMobileLaserScan &input) {
		// no-op
	}
	
	// this method can be safely used from the thread in derived classes
	inline Smart::StatusCode laserClientGetUpdate(CommBasicObjects::CommMobileLaserScan &laserClientObject) const
	{
		// copy local object buffer and return the last status code
		laserClientObject = this->laserClientObject;
		return laserClientStatus;
	}
	// overload and implement this method in derived classes to immediately get all incoming updates from LaserClient2 (as soon as they arrive)
	virtual void on_LaserClient2(const CommBasicObjects::CommMobileLaserScan &input) {
		// no-op
	}
	
	// this method can be safely used from the thread in derived classes
	inline Smart::StatusCode laserClient2GetUpdate(CommBasicObjects::CommMobileLaserScan &laserClient2Object) const
	{
		// copy local object buffer and return the last status code
		laserClient2Object = this->laserClient2Object;
		return laserClient2Status;
	}
	// overload and implement this method in derived classes to immediately get all incoming updates from NavVelSendServer (as soon as they arrive)
	virtual void on_NavVelSendServer(const CommBasicObjects::CommNavigationVelocity &input) {
		// no-op
	}
	
	// this method can be safely used from the thread in derived classes
	inline Smart::StatusCode navVelSendServerGetUpdate(CommBasicObjects::CommNavigationVelocity &navVelSendServerObject) const
	{
		// copy local object buffer and return the last status code
		navVelSendServerObject = this->navVelSendServerObject;
		return navVelSendServerStatus;
	}
	// overload and implement this method in derived classes to immediately get all incoming updates from PathNavigationGoalClient (as soon as they arrive)
	virtual void on_PathNavigationGoalClient(const CommNavigationObjects::CommCorridorNavigationGoal &input) {
		// no-op
	}
	
	// this method can be safely used from the thread in derived classes
	inline Smart::StatusCode pathNavigationGoalClientGetUpdate(CommNavigationObjects::CommCorridorNavigationGoal &pathNavigationGoalClientObject) const
	{
		// copy local object buffer and return the last status code
		pathNavigationGoalClientObject = this->pathNavigationGoalClientObject;
		return pathNavigationGoalClientStatus;
	}
	// overload and implement this method in derived classes to immediately get all incoming updates from PlannerClient (as soon as they arrive)
	virtual void on_PlannerClient(const CommNavigationObjects::CommPlannerGoal &input) {
		// no-op
	}
	
	// this method can be safely used from the thread in derived classes
	inline Smart::StatusCode plannerClientGetUpdate(CommNavigationObjects::CommPlannerGoal &plannerClientObject) const
	{
		// copy local object buffer and return the last status code
		plannerClientObject = this->plannerClientObject;
		return plannerClientStatus;
	}
	// overload and implement this method in derived classes to immediately get all incoming updates from TrackingClient (as soon as they arrive)
	virtual void on_TrackingClient(const CommTrackingObjects::CommTrackingGoal &input) {
		// no-op
	}
	
	// this method can be safely used from the thread in derived classes
	inline Smart::StatusCode trackingClientGetUpdate(CommTrackingObjects::CommTrackingGoal &trackingClientObject) const
	{
		// copy local object buffer and return the last status code
		trackingClientObject = this->trackingClientObject;
		return trackingClientStatus;
	}
	
	// this method is meant to be used in derived classes
	Smart::StatusCode goalEventServerPut(CommNavigationObjects::CdlGoalEventState &eventState);
	// this method is meant to be used in derived classes
	Smart::StatusCode navVelSendClientPut(CommBasicObjects::CommNavigationVelocity &navVelSendClientDataObject);
	// this method is meant to be used in derived classes
	Smart::StatusCode robotBlockedEventServerPut(CommNavigationObjects::CommCdlRobotBlockedState &eventState);
	
	
/**
 * Implementation of the Subject part of an InteractionObserver
 */
private:
	std::mutex interaction_observers_mutex;
	std::list<CdlTaskObserverInterface*> interaction_observers;
protected:
	void notify_all_interaction_observers();
public:
	void attach_interaction_observer(CdlTaskObserverInterface *observer);
	void detach_interaction_observer(CdlTaskObserverInterface *observer);

public:
	CdlTaskCore(Smart::IComponent *comp, const bool &useDefaultState=true);
	virtual ~CdlTaskCore();
	
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
