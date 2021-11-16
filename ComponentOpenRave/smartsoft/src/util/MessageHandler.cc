//--------------------------------------------------------------------------
//  Copyright (C) 2010 Jonas Brich
//
//        brich@mail.hs-ulm.de
//
//        Christian Schlegel (schlegel@hs-ulm.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
//
//  This file is part of the "SmartOpenRave component".
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

#include "MessageHandler.hh"
#include "ComponentOpenRave.hh"

#include "colors.hh"

void MessageHandler::handleMessage(const std::string& message, MessageHandler::ErrorTypes type, bool sendMessage)
{
	if (sendMessage)
	{
		if (type == MessageHandler::INFO)
		{
			std::cout << COLOR_BLUE << "INFO: " << message << COLOR_DEFAULT << "\n";
		} else if (type == MessageHandler::WARNING)
		{
			std::cout << COLOR_MAGENTA << "WARNING: " << message << COLOR_DEFAULT << "\n";
		} else if (type == MessageHandler::ERROR)
		{
			std::cerr << COLOR_RED << "ERROR: " << message << COLOR_DEFAULT << "\n";
		}
	}
}

void MessageHandler::handleMessage(const std::string& message, CommManipulationPlannerObjects::ManipulationPlannerEvent event,
		MessageHandler::ErrorTypes type, bool sendMessage)
{

	std::cout << "Timo ERROR Type: " << type << std::endl;

	MessageHandler::handleMessage(message, type, sendMessage);
	// send event to other components
	CommManipulationPlannerObjects::CommManipulationPlannerEventState state;
	state.set_event(event);
	//COMP->eventServer->put(state);
	COMP->planningEventServiceOut->put(state);
}

void MessageHandler::handleMessage(const std::string& message, CommManipulationPlannerObjects::ManipulationPlannerEvent event,
		CommBasicObjects::CommPose3d& pose, MessageHandler::ErrorTypes type, bool sendMessage)
{
	MessageHandler::handleMessage(message, type, sendMessage);
	// send event to other components
	CommManipulationPlannerObjects::CommManipulationPlannerEventState state;
	state.set_event(event);
	state.set_pose(pose);
	std::cout << COLOR_BLUE << "INFO: " << "Pose past Iteration: " << pose << COLOR_DEFAULT << std::endl;
	//COMP->eventServer->put(state);
	COMP->planningEventServiceOut->put(state);
}
