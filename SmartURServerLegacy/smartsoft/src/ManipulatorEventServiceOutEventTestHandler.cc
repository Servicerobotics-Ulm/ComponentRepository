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

//------------------------------------------------------------------------
//
//  Copyright (C) 2011 Manuel Wopfner
//
//        wopfner@hs-ulm.de
//
//        Christian Schlegel (schlegel@hs-ulm.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
//
//  This file is part of the "SmartURServer component".
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
//--------------------------------------------------------------------------

#include "ManipulatorEventServiceOutEventTestHandler.hh"

#include "SmartURServerLegacy.hh"

bool ManipulatorEventServiceOutEventTestHandler::testEvent(
	CommManipulatorObjects::CommManipulatorEventParameter &p,
	CommManipulatorObjects::CommManipulatorEventResult &r,
	const CommManipulatorObjects::CommManipulatorEventState &s
) throw() {

	bool result = false;
	
	CommManipulatorObjects::ManipulatorEvent oldState(CommManipulatorObjects::ManipulatorEvent::UNKNOWN);
	CommManipulatorObjects::ManipulatorEvent newState(CommManipulatorObjects::ManipulatorEvent::UNKNOWN);

	oldState = p.getEvent();
	newState = s.getEvent();

	if (oldState == newState)
	{
		// no state change --> no event
		result = false;
	} else
	{
		if (COMP->getGlobalState().getManipulator().getVerbose())
		{
			std::cout << "[EventTestHandler] SEND Event: " << newState.to_string() << std::endl;
		}

		// memorize new state (state change) for currently tested event
		// each activated event has its own parameter
		p.set_event(newState);
		// set result
		r.set_event(newState);
		result = true;
	}
	
	// true --> send event
	// false --> don't send event
   	return result;
}
