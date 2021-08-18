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
	
		
		virtual void handleDomainRobotFleet_FleetManagerParameter_ADDROBOT(const std::string &robotName, const std::string &baseComponentName, const std::string &baseServiceName, const std::string &baseriOQueryServerName, const std::string &laserComponentName, const std::string &laserServiceName, const std::string &pathNavClientComponentName, const std::string &pathNavClientServiceName) = 0;
	
		
		virtual void handleDomainRobotFleet_FleetManagerParameter_RMROBOT(const std::string &robotName) = 0;
	
	// extended trigger user methods
	
protected:
	SmartACE::SmartMutex mutex;
	SmartACE::SmartSemaphore sema;
	int on_execute();

	class TriggerEnumerators {
	public:
		enum ENUM {
			UNDEFINED_TRIGGER_ACTION
			, DOMAINROBOTFLEET_FLEETMANAGERPARAMETER_ADDROBOT
			, DOMAINROBOTFLEET_FLEETMANAGERPARAMETER_RMROBOT
		};
	};
	TriggerEnumerators::ENUM current_trigger_enumerator;
	std::queue<TriggerEnumerators::ENUM> trigger_queue;
	
	// active trigger ADDROBOT
	struct DomainRobotFleet_FleetManagerParameter_ADDROBOTAttributes {
		std::string baseComponentName;
		std::string baseServiceName;
		std::string baseriOQueryServerName;
		std::string laserComponentName;
		std::string laserServiceName;
		std::string pathNavClientComponentName;
		std::string pathNavClientServiceName;
		std::string robotName;
	}current_DomainRobotFleet_FleetManagerParameter_ADDROBOT;
	std::queue<DomainRobotFleet_FleetManagerParameter_ADDROBOTAttributes> DomainRobotFleet_FleetManagerParameter_ADDROBOT_queue;
	
	// active trigger RMROBOT
	struct DomainRobotFleet_FleetManagerParameter_RMROBOTAttributes {
		std::string robotName;
	}current_DomainRobotFleet_FleetManagerParameter_RMROBOT;
	std::queue<DomainRobotFleet_FleetManagerParameter_RMROBOTAttributes> DomainRobotFleet_FleetManagerParameter_RMROBOT_queue;
	
private:
	// trigger internal methods
	void handleDomainRobotFleet_FleetManagerParameter_ADDROBOTCore(const std::string &robotName, const std::string &baseComponentName, const std::string &baseServiceName, const std::string &baseriOQueryServerName, const std::string &laserComponentName, const std::string &laserServiceName, const std::string &pathNavClientComponentName, const std::string &pathNavClientServiceName);
	void handleDomainRobotFleet_FleetManagerParameter_RMROBOTCore(const std::string &robotName);
	
	// extended trigger internal methods 
};

#endif // _TRIGGERHANDLERCORE_HH