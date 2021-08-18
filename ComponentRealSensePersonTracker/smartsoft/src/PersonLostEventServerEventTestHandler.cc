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
//  Copyright (C) 2009-2017 Andreas Steck, Matthias Lutz
//
//        lutz@hs-ulm.de
//        shaik@hs-ulm.de
//
//        ZAFH Servicerobotik Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        D-89075 Ulm
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
// --------------------------------------------------------------------------
#include "PersonLostEventServerEventTestHandler.hh"

bool PersonLostEventServerEventTestHandler::testEvent(
	CommTrackingObjects::CommPersonLostEventParameter &p,
	CommTrackingObjects::CommPersonLostEventResult &r,
	const CommTrackingObjects::PersonLostEventState &s
) throw() {
	// fire all events (without filtering) in the default implementation
	// implement your own (specific) event-filtering code using the event-parameter as input
	// true means that the current event will be fired to the according client
	// false means that the current event is ignored (it will not be communicated to the according client)
	bool result = false;

   CommTrackingObjects::PersonLostEventType oldState, newState;
    p.get(oldState);
    s.get(newState);

    p.set(newState);

    if(oldState != newState) // && newState == PERSON_LOST)
    {
      r.set(newState);
      result = true;
    }
    else
    {
      result = false;
    }

    if(result ==true)
    	std::cout << "[Event Triggered]: Realsense lost the person" << std::endl;

   	return result;
}