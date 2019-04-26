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
// --------------------------------------------------------------------------
//
//  Copyright (C) 2011, 2017 Matthias Lutz, Dennis Stampfer, Matthias Rollenhagen, Nayabrasul Shaik
//
//      lutz@hs-ulm.de
//      stampfer@hs-ulm.de
//      rollenhagen@hs-ulm.de
//      shaik@hs-ulm.de
//
//      ZAFH Servicerobotic Ulm
//      Christian Schlegel
//      University of Applied Sciences
//      Prittwitzstr. 10
//      89075 Ulm
//      Germany
//
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//--------------------------------------------------------------------------
#include "CompHandler.hh"
#include "ComponentRealSenseV2Server.hh"

#include <iostream>

// include communication objects


void CompHandler::onStartup() 
{
	std::cout << "startup - put your startupCode in CompHandler::onStartup()!!!\n";

	Smart::StatusCode status;

	// Start all services. If you need manual control, use the content of this function to
	// connect and start each service individually, e.g:
	// COMP->connectMyPortName("SmartExampleComponent", "examplePort");
	//status = COMP->connectAndStartAllServices();

	// connect to all services
	if (COMP->getGlobalState().getBase().getOn_base()) {
		std::cout << "## DEVICE ON BASE" << std::endl;
		std::cout << "connecting to: " << COMP->connections.basePushTimedClient.serverName << "; "
				<< COMP->connections.basePushTimedClient.serviceName << std::endl;
		status = COMP->basePushTimedClient->connect(COMP->connections.basePushTimedClient.serverName,
				COMP->connections.basePushTimedClient.serviceName);
		while (status != Smart::SMART_OK) {
			usleep(500000);
			status = COMP->basePushTimedClient->connect(COMP->connections.basePushTimedClient.serverName,
					COMP->connections.basePushTimedClient.serviceName);
		}
		std::cout << "connected.\n";

		COMP->basePushTimedClient->subscribe(COMP->connections.basePushTimedClient.interval);
	}

	if (COMP->getGlobalState().getBase().getOn_ptu()) {
		std::cout << "## DEVICE ON PTU" << std::endl;
		std::cout << "connecting to: "
				<< COMP->connections.ptuPosePushNewestClient.serverName << "; "
				<< COMP->connections.ptuPosePushNewestClient.serviceName << std::endl;
		status = COMP->ptuPosePushNewestClient->connect(
				COMP->connections.ptuPosePushNewestClient.serverName,
				COMP->connections.ptuPosePushNewestClient.serviceName);
		while (status != Smart::SMART_OK)
		{
			usleep(500000);
			status = COMP->ptuPosePushNewestClient->connect(
					COMP->connections.ptuPosePushNewestClient.serverName,
					COMP->connections.ptuPosePushNewestClient.serviceName);
		}
		std::cout << "connected.\n";

		COMP->ptuPosePushNewestClient->subscribe();
	}
	if (COMP->getGlobalState().getBase().getOn_ur()) {
		status = COMP->connectUrPosePushTimedClient(COMP->connections.urPosePushTimedClient.serverName, COMP->connections.urPosePushTimedClient.serviceName);
		std::cout<<"Connect manipulator query client result: "<<Smart::StatusCodeConversion(status)<<std::endl;

	}

	// start push timed server
	//COMP->colorImagePushTimedServer->start();

	// activate state server
	if (COMP->stateSlave->activate() != Smart::SMART_OK)
		std::cerr << "ERROR: activate state" << std::endl;

	
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