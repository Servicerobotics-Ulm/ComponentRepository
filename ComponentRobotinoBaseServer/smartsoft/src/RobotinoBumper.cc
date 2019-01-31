//--------------------------------------------------------------------------
//
//  Copyright (C) 2015 Matthias Lutz
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


#include "RobotinoBumper.hh"
#include "CommBasicObjects/CommBumperEventState.hh"
#include "ComponentRobotinoBaseServer.hh"




void RobotinoBumper::bumperEvent( bool hasContact )
{
		CommBasicObjects::CommBumperEventState state;
		bumped = hasContact;
		if( bumped )
		{
			state.setNewState(CommBasicObjects::BumperEventType::BUMPER_PRESSED);
				std::cout << "Bumper has contact" << std::endl;
		} else {
			state.setNewState(CommBasicObjects::BumperEventType::BUMPER_NOT_PRESSED);
		}
		//COMP->bumperEventServer->put(state);

		COMP->bumperEventServiceOut->put(state);

}

bool bumped;





