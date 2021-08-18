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
#ifndef _STATEACTIVITYMANAGER_HH
#define _STATEACTIVITYMANAGER_HH
		
#include "aceSmartSoft.hh"

#include <map>
#include <memory>

class StateActivityManager : public SmartACE::StateChangeHandler
{
private:
	SmartACE::SmartRecursiveMutex mutex;
	struct StateStruct {
	public:
		std::string sub_state;
		std::shared_ptr<SmartACE::SmartConditionRecursiveMutex> cond_ptr;
		bool auto_restart_task = false;
		bool cancelled = false;
	};
	SmartACE::StateChangeHandler *user_handler;
	std::map<std::string, bool> substate_registry;
	std::map<SmartACE::ManagedTask*,StateStruct> task_registry;

protected:
    virtual void handleEnterState(const std::string& sub_state) override;
    virtual void handleQuitState(const std::string& sub_state) override;

public:
	StateActivityManager(SmartACE::StateChangeHandler *user_handler);
	virtual ~StateActivityManager() = default;

	void attach(SmartACE::ManagedTask *task, const std::string &sub_state, const bool &auto_restart_task=false);
	void detach(SmartACE::ManagedTask *task, const std::string &sub_state);

	Smart::StatusCode acquire(const std::string &sub_state, SmartACE::ManagedTask *task);
	Smart::StatusCode release(const std::string &sub_state);
};

#endif //_STATEACTIVITYMANAGER_HH
