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

//--------------------------------------------------------------------------
//  Copyright (C) 2010 Matthias Lutz
//
//        Christian Schlegel (schlegel@hs-ulm.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
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
//-------------------------------------------------------------------------

#include "CompHandler.hh"
#include "ComponentRobotinoImageServer.hh"

#include <iostream>

// include communication objects
#include <CommBasicObjects/CommBaseState.hh>
#include <CommBasicObjects/CommDevicePoseState.hh>
#include <DomainVision/CommVideoImage.hh>
#include <CommBasicObjects/CommVoid.hh>


void CompHandler::onStartup() 
{
	std::cout << "startup - put your startupCode in CompHandler::onStartup()!!!\n";

	std::cout << "Format :" <<"h:"<<COMP->getGlobalState().getHardware_properties().getHeight() <<"w:"<<COMP->getGlobalState().getHardware_properties().getWidth() <<"\n";


	Smart::StatusCode status;

	// Start all services. If you need manual control, use the content of this function to
	// connect and start each service individually, e.g:
	// COMP->connectMyPortName("SmartExampleComponent", "examplePort");
	status = COMP->connectAndStartAllServices();
	
	if (COMP->getGlobalState().getBase().getOn_base()) {
		status = COMP->connectBaseStateServiceIn(COMP->connections.baseStateServiceIn.serverName, COMP->connections.baseStateServiceIn.serviceName);
	}

	if (COMP->getGlobalState().getBase().getOn_ptu()) {
		status = COMP->connectDevicePoseUpdateServiceIn(COMP->connections.devicePoseUpdateServiceIn.serverName, COMP->connections.devicePoseUpdateServiceIn.serviceName);
	}
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