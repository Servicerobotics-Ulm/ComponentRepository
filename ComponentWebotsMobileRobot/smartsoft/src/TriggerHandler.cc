//--------------------------------------------------------------------------
// Code generated by the SmartSoft MDSD Toolchain Version 2.2
// The SmartSoft Toolchain has been developed by:
//  
// Christian Schlegel (schlegel@hs-ulm.de)
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
//
//  Copyright (C) 2012 Matthias Hörger, Matthias Lutz
//
//        lutz@hs-ulm.de
//
//        Servicerobotic Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm
//        Germany
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

#include "TriggerHandler.hh"

#include "ComponentWebotsMobileRobot.hh"
#include "ComponentWebotsMobileRobot.hh"

// trigger user methods
void TriggerHandler::handleCommBasicObjects_BaseParams_BASE_RESET()
{
	//virtual void handleCommBasicObjects_BaseParams_BASE_RESET();
	// implement the trigger behavior here. Be aware, if you must use blocking calls here, please set this
	// trigger as active in the model. For an active trigger an active queue will be generated internally 
	// (transparent for the usage here). Thus an active trigger will be called within a separate task scope.
	COMP->robot->resetPosition();
	std::cout << "RESET BASE !!!!!!!!!!!!!!\n\n";
}
void TriggerHandler::handleCommBasicObjects_BaseParams_BASE_SONAR()
{
	// implement the trigger behavior here. Be aware, if you must use blocking calls here, please set this
	// trigger as active in the model. For an active trigger an active queue will be generated internally 
	// (transparent for the usage here). Thus an active trigger will be called within a separate task scope
	std::cout << "no BASE_SONAR available on RobotinoXT\n\n";
}
void TriggerHandler::handleCommBasicObjects_BaseParams_SIGNAL_STATE_IDLE()
{
	// implement the trigger behavior here. Be aware, if you must use blocking calls here, please set this
	// trigger as active in the model. For an active trigger an active queue will be generated internally 
	// (transparent for the usage here). Thus an active trigger will be called within a separate task scope.
	COMP->signalStateTask->setSignalState(CommBasicObjects::BaseTagType::SIGNAL_STATE_IDLE);
}
void TriggerHandler::handleCommBasicObjects_BaseParams_SIGNAL_STATE_ERROR()
{
	// implement the trigger behavior here. Be aware, if you must use blocking calls here, please set this
	// trigger as active in the model. For an active trigger an active queue will be generated internally 
	// (transparent for the usage here). Thus an active trigger will be called within a separate task scope.
	COMP->signalStateTask->setSignalState(CommBasicObjects::BaseTagType::SIGNAL_STATE_ERROR);
}
void TriggerHandler::handleCommBasicObjects_BaseParams_SIGNAL_STATE_BUSY()
{
	// implement the trigger behavior here. Be aware, if you must use blocking calls here, please set this
	// trigger as active in the model. For an active trigger an active queue will be generated internally 
	// (transparent for the usage here). Thus an active trigger will be called within a separate task scope.
	COMP->signalStateTask->setSignalState(CommBasicObjects::BaseTagType::SIGNAL_STATE_BUSY);
}
void TriggerHandler::handleCommBasicObjects_BaseParams_SIGNAL_STATE_LOCALIZATION_ERROR()
{
	// implement the trigger behavior here. Be aware, if you must use blocking calls here, please set this
	// trigger as active in the model. For an active trigger an active queue will be generated internally 
	// (transparent for the usage here). Thus an active trigger will be called within a separate task scope.
	COMP->signalStateTask->setSignalState(CommBasicObjects::BaseTagType::SIGNAL_STATE_LOCALIZATION_ERROR);
}
void TriggerHandler::handleCommBasicObjects_BaseParams_SIGNAL_STATE_SAFETY_FIELD()
{
	// implement the trigger behavior here. Be aware, if you must use blocking calls here, please set this
	// trigger as active in the model. For an active trigger an active queue will be generated internally 
	// (transparent for the usage here). Thus an active trigger will be called within a separate task scope.
	COMP->signalStateTask->setSignalState(CommBasicObjects::BaseTagType::SIGNAL_STATE_SAFETY_FIELD);
}

void TriggerHandler::handleSET_RELAY(const unsigned int &number, const bool &value){
	COMP->robot->setRelay(number,value);
}
