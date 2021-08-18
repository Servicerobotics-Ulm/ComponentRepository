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

#include "TriggerHandlerCore.hh"

int TriggerHandlerCore::on_execute()
{
	// blocking wait until some active trigger request(s) come in
	sema.acquire();
	
	{
		SmartACE::SmartGuard g(mutex);
		
		// get the top trigger from the queue
		current_trigger_enumerator = trigger_queue.front();
		trigger_queue.pop();
		
		// retrieve the corresponding trigger attributes from the trigger specific queue
		switch(current_trigger_enumerator) {
		case TriggerEnumerators::DOMAINROBOTFLEET_FLEETMANAGERPARAMETER_ADDROBOT:
			current_DomainRobotFleet_FleetManagerParameter_ADDROBOT = DomainRobotFleet_FleetManagerParameter_ADDROBOT_queue.front();
			DomainRobotFleet_FleetManagerParameter_ADDROBOT_queue.pop();
			break;
		case TriggerEnumerators::DOMAINROBOTFLEET_FLEETMANAGERPARAMETER_RMROBOT:
			current_DomainRobotFleet_FleetManagerParameter_RMROBOT = DomainRobotFleet_FleetManagerParameter_RMROBOT_queue.front();
			DomainRobotFleet_FleetManagerParameter_RMROBOT_queue.pop();
			break;
		}
	} // mutex release

	// now call the corresponding trigger handler
	// (releasing the mutex before, allows to store newly incoming trigger commands on the queue in parallel)
	switch(current_trigger_enumerator) {
	case TriggerEnumerators::DOMAINROBOTFLEET_FLEETMANAGERPARAMETER_ADDROBOT:
		this->handleDomainRobotFleet_FleetManagerParameter_ADDROBOT(current_DomainRobotFleet_FleetManagerParameter_ADDROBOT.robotName, current_DomainRobotFleet_FleetManagerParameter_ADDROBOT.baseComponentName, current_DomainRobotFleet_FleetManagerParameter_ADDROBOT.baseServiceName, current_DomainRobotFleet_FleetManagerParameter_ADDROBOT.baseriOQueryServerName, current_DomainRobotFleet_FleetManagerParameter_ADDROBOT.laserComponentName, current_DomainRobotFleet_FleetManagerParameter_ADDROBOT.laserServiceName, current_DomainRobotFleet_FleetManagerParameter_ADDROBOT.pathNavClientComponentName, current_DomainRobotFleet_FleetManagerParameter_ADDROBOT.pathNavClientServiceName);
		break;
	case TriggerEnumerators::DOMAINROBOTFLEET_FLEETMANAGERPARAMETER_RMROBOT:
		this->handleDomainRobotFleet_FleetManagerParameter_RMROBOT(current_DomainRobotFleet_FleetManagerParameter_RMROBOT.robotName);
		break;
	}
	return 0;
}

//
// trigger internal handler methods
//

	// handle ADDROBOT
	void TriggerHandlerCore::handleDomainRobotFleet_FleetManagerParameter_ADDROBOTCore(const std::string &robotName, const std::string &baseComponentName, const std::string &baseServiceName, const std::string &baseriOQueryServerName, const std::string &laserComponentName, const std::string &laserServiceName, const std::string &pathNavClientComponentName, const std::string &pathNavClientServiceName)
	{
		SmartACE::SmartGuard g(mutex);

		// store the current set of trigger-attributes in a queue
		DomainRobotFleet_FleetManagerParameter_ADDROBOTAttributes attr;
		attr.baseComponentName = baseComponentName;
		attr.baseServiceName = baseServiceName;
		attr.baseriOQueryServerName = baseriOQueryServerName;
		attr.laserComponentName = laserComponentName;
		attr.laserServiceName = laserServiceName;
		attr.pathNavClientComponentName = pathNavClientComponentName;
		attr.pathNavClientServiceName = pathNavClientServiceName;
		attr.robotName = robotName;
		DomainRobotFleet_FleetManagerParameter_ADDROBOT_queue.push(attr);
		
		// store the current trigger call in the shared trigger-queue
		trigger_queue.push(TriggerEnumerators::DOMAINROBOTFLEET_FLEETMANAGERPARAMETER_ADDROBOT);
		
		// signal the task, in case it is waiting
		sema.release();
	}

	// handle RMROBOT
	void TriggerHandlerCore::handleDomainRobotFleet_FleetManagerParameter_RMROBOTCore(const std::string &robotName)
	{
		SmartACE::SmartGuard g(mutex);

		// store the current set of trigger-attributes in a queue
		DomainRobotFleet_FleetManagerParameter_RMROBOTAttributes attr;
		attr.robotName = robotName;
		DomainRobotFleet_FleetManagerParameter_RMROBOT_queue.push(attr);
		
		// store the current trigger call in the shared trigger-queue
		trigger_queue.push(TriggerEnumerators::DOMAINROBOTFLEET_FLEETMANAGERPARAMETER_RMROBOT);
		
		// signal the task, in case it is waiting
		sema.release();
	}

//
// extended trigger internal handler methods
//
