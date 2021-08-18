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
#ifndef _AMCLTASK_CORE_HH
#define _AMCLTASK_CORE_HH
	
#include "aceSmartSoft.hh"

// include upcall interface
#include "LaserServiceInUpcallInterface.hh"

// include communication-objects for output ports
#include <CommLocalizationObjects/CommLocalizationEventParameter.hh>
#include <CommLocalizationObjects/CommLocalizationEventResult.hh>
#include <CommLocalizationObjects/LocalizationEventState.hh>
#include <CommBasicObjects/CommBasePositionUpdate.hh>

// include all interaction-observer interfaces
#include <AmclTaskObserverInterface.hh>


class AmclTaskCore
:	public SmartACE::ManagedTask
,	public Smart::TaskTriggerSubject
,	public LaserServiceInUpcallInterface
{
private:
	bool useDefaultState; 
	bool useLogging;
	int taskLoggingId;
	unsigned int currentUpdateCount;
	
	Smart::StatusCode laserServiceInStatus;
	CommBasicObjects::CommMobileLaserScan laserServiceInObject;
	
	
protected:
	virtual int execute_protected_region();
	
	virtual void updateAllCommObjects();
	
	virtual int getPreviousCommObjId();
	
	void triggerLogEntry(const int& idOffset);
	
	
	// overload and implement this method in derived classes to immediately get all incoming updates from LaserServiceIn (as soon as they arrive)
	virtual void on_LaserServiceIn(const CommBasicObjects::CommMobileLaserScan &input) {
		// no-op
	}
	
	// this method can be safely used from the thread in derived classes
	inline Smart::StatusCode laserServiceInGetUpdate(CommBasicObjects::CommMobileLaserScan &laserServiceInObject) const
	{
		// copy local object buffer and return the last status code
		laserServiceInObject = this->laserServiceInObject;
		return laserServiceInStatus;
	}
	
	// this method is meant to be used in derived classes
	Smart::StatusCode localizationEventServiceOutPut(CommLocalizationObjects::LocalizationEventState &eventState);
	// this method is meant to be used in derived classes
	Smart::StatusCode localizationUpdateServiceOutPut(CommBasicObjects::CommBasePositionUpdate &localizationUpdateServiceOutDataObject);
	
	
/**
 * Implementation of the Subject part of an InteractionObserver
 */
private:
	std::mutex interaction_observers_mutex;
	std::list<AmclTaskObserverInterface*> interaction_observers;
protected:
	void notify_all_interaction_observers();
public:
	void attach_interaction_observer(AmclTaskObserverInterface *observer);
	void detach_interaction_observer(AmclTaskObserverInterface *observer);

public:
	AmclTaskCore(Smart::IComponent *comp, const bool &useDefaultState=true);
	virtual ~AmclTaskCore();
	
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
