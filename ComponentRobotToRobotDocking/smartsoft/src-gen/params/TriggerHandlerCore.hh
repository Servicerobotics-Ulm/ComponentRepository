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
#ifndef _TRIGGERHANDLERCORE_HH
#define _TRIGGERHANDLERCORE_HH

#include "aceSmartSoft.hh"
#include <queue>

#include <string>
#include <iostream>
#include <list>

class TriggerHandlerCore : public SmartACE::ManagedTask
{
	friend class ParamUpdateHandler;
	
public:
	TriggerHandlerCore() 
	:	SmartACE::ManagedTask(NULL) //TODO: a propper component pointer should be probably used here instead of NULL
	,	mutex()
	,	sema(0) // initialize semaphore in blocking mode
	,	current_trigger_enumerator(TriggerEnumerators::UNDEFINED_TRIGGER_ACTION)
	{  
		this->start();
	}
	virtual ~TriggerHandlerCore() {  }

	// trigger user methods
	
		
		virtual void handleCommRobotinoObjects_RobotinoDockingParameter_SAVE_BEFORE_DOCKING_POSE() = 0;
	
	// extended trigger user methods
	
protected:
	SmartACE::SmartMutex mutex;
	SmartACE::SmartSemaphore sema;
	int on_execute();

	class TriggerEnumerators {
	public:
		enum ENUM {
			UNDEFINED_TRIGGER_ACTION
			, COMMROBOTINOOBJECTS_ROBOTINODOCKINGPARAMETER_SAVE_BEFORE_DOCKING_POSE
		};
	};
	TriggerEnumerators::ENUM current_trigger_enumerator;
	std::queue<TriggerEnumerators::ENUM> trigger_queue;
	
	// active trigger SAVE_BEFORE_DOCKING_POSE
	struct CommRobotinoObjects_RobotinoDockingParameter_SAVE_BEFORE_DOCKING_POSEAttributes {
	}current_CommRobotinoObjects_RobotinoDockingParameter_SAVE_BEFORE_DOCKING_POSE;
	std::queue<CommRobotinoObjects_RobotinoDockingParameter_SAVE_BEFORE_DOCKING_POSEAttributes> CommRobotinoObjects_RobotinoDockingParameter_SAVE_BEFORE_DOCKING_POSE_queue;
	
private:
	// trigger internal methods
	void handleCommRobotinoObjects_RobotinoDockingParameter_SAVE_BEFORE_DOCKING_POSECore();
	
	// extended trigger internal methods 
};

#endif // _TRIGGERHANDLERCORE_HH
