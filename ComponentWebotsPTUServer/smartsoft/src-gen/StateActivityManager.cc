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

#include "StateActivityManager.hh"
#include "ComponentWebotsPTUServer.hh"

StateActivityManager::StateActivityManager(SmartACE::StateChangeHandler *user_handler)
:	user_handler(user_handler)
,	mutex()
{  }

void StateActivityManager::attach(SmartACE::ManagedTask *task, const std::string &sub_state, const bool &auto_restart_task)
{
	SmartACE::SmartRecursiveGuard guard(mutex);
		auto task_it = task_registry.find(task);
		if(task_it == task_registry.end()) {
			StateStruct state;
			state.sub_state = sub_state;
			state.cond_ptr = std::make_shared<SmartACE::SmartConditionRecursiveMutex>(mutex);
			state.auto_restart_task = auto_restart_task;
			task_registry[task] = state;
		} else {
			task_it->second.cancelled = false;
			task_it->second.sub_state = sub_state;
			task_it->second.auto_restart_task = auto_restart_task;
		}
		if(substate_registry.find(sub_state) == substate_registry.end()) {
			// sub state does not appear to have been activated thus far -> create a new entry
			substate_registry[sub_state] = false;
		}
}
void StateActivityManager::detach(SmartACE::ManagedTask *task, const std::string &sub_state)
{
	SmartACE::SmartRecursiveGuard guard(mutex);
	auto task_it = task_registry.find(task);
	if(task_it != task_registry.end()) {
		task_it->second.cancelled = true;
		task_it->second.cond_ptr->signal();
	}
}

Smart::StatusCode StateActivityManager::acquire(const std::string &sub_state, SmartACE::ManagedTask *task)
{
	SmartACE::SmartRecursiveGuard guard(mutex);
	auto state_it = substate_registry.find(sub_state);
	if(state_it != substate_registry.end()) {
		if(state_it->second == true) {
			return COMP->stateSlave->tryAcquire(sub_state);
		} else {
			auto task_it = task_registry.find(task);
			if(task_it != task_registry.end()) {
				if(task_it->second.cancelled == true) {
					return Smart::SMART_CANCELLED;
				} else {
					// blocking wait until either the state becomes active or the task gets cancelled
					task_it->second.cond_ptr->wait();
					if(task_it->second.cancelled == true) {
						return Smart::SMART_CANCELLED;
					} else {
						return COMP->stateSlave->tryAcquire(sub_state);
					}
				}
			}
		}
	}
	return Smart::SMART_ERROR;
}
Smart::StatusCode StateActivityManager::release(const std::string &sub_state)
{
	return COMP->stateSlave->release(sub_state);
}

void StateActivityManager::handleEnterState(const std::string& sub_state)
{
	SmartACE::SmartRecursiveGuard guard(mutex);
	substate_registry[sub_state] = true;
	for(auto it: task_registry) {
		if(it.second.sub_state == sub_state) {
			if(it.second.auto_restart_task == true) {
				it.first->start();
			}
			it.second.cond_ptr->signal();
		}
	}
	guard.release();
	user_handler->handleEnterState(sub_state);
}
void StateActivityManager::handleQuitState(const std::string& sub_state)
{
	SmartACE::SmartRecursiveGuard guard(mutex);
	substate_registry[sub_state] = false;
	for(auto it: task_registry) {
		if(it.second.sub_state == sub_state) {
			if(it.second.auto_restart_task == true) {
				it.first->stop();
			}
		}
	}
	guard.release();
	user_handler->handleQuitState(sub_state);
}
