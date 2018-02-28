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
// This file is generated once. Modify this file to your needs. 
// If you want the toolchain to re-generate this file, please 
// delete it before running the code generator.
//--------------------------------------------------------------------------
#include "CompHandler.hh"
#include "SmartPioneerBaseServer.hh"

#include <iostream>

// include communication objects
#include <CommBasicObjects/CommBaseState.hh>
#include <CommBasicObjects/CommBatteryEvent.hh>
#include <CommBasicObjects/CommBatteryState.hh>
#include <CommBasicObjects/CommVoid.hh>
#include <CommBasicObjects/CommNavigationVelocity.hh>
#include <CommBasicObjects/CommBatteryParameter.hh>
#include <CommBasicObjects/CommBasePositionUpdate.hh>


void CompHandler::onStartup() 
{
	std::cout << "startup - put your startupCode in CompHandler::onStartup()!!!\n";

    int robotType = -1;
    if( strcmp("p3dx", COMP->getGlobalState().getRobot().getRobotType().c_str() ) == 0 ) {
      robotType = ROBOT_TYPE_P3DX;
    } else if( strcmp("p3dxsh", COMP->getGlobalState().getRobot().getRobotType().c_str() ) == 0 ) {
      robotType = ROBOT_TYPE_P3DX_SH;
    } else if( strcmp("p3atsh", COMP->getGlobalState().getRobot().getRobotType().c_str() ) == 0 ) {
      robotType = ROBOT_TYPE_P3AT_SH;
    }
    COMP->robotTask->setupRobotType(robotType);

    COMP->robotTask->openSerial( COMP->getGlobalState().getRobot().getSerialport(),
    	COMP->COMP->getGlobalState().getRobot().getEnable_motors(),
    	COMP->COMP->getGlobalState().getRobot().getEnable_sonar());

    COMP->robotTask->setParameters(	COMP->getGlobalState().getRobot().getMaxVel(),
    		COMP->getGlobalState().getRobot().getMaxRotVel(),
    		COMP->getGlobalState().getRobot().getMaxVelAcc(),
    		COMP->getGlobalState().getRobot().getMaxVelDecel(),
    		COMP->getGlobalState().getRobot().getMaxRotVelAcc(),
    		COMP->getGlobalState().getRobot().getMaxRotVelDecel() );

	Smart::StatusCode status;

	// Start all services. If you need manual control, use the content of this function to
	// connect and start each service individually, e.g:
	// COMP->connectMyPortName("SmartExampleComponent", "examplePort");
	status = COMP->connectAndStartAllServices();
	
	// Start all tasks. If you need manual control, use the content of this function to
	// start each task individually.
	COMP->startAllTasks();
	
	// Start all timers. If you need manual control, use the content of this function to
	// start each timer individually.
	COMP->startAllTimers();
	
	// Notify the component that setup/initialization is finished.
	// You may move this function to any other place.
	COMP->setStartupFinished(); 
}

void CompHandler::onShutdown() 
{
	std::cout << "shutdown - put your cleanup code in CompHandler::onShutdown()!!!\n";
	
}
