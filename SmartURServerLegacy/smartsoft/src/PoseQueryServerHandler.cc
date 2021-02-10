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


#include "PoseQueryServerHandler.hh"
#include "SmartURServerLegacy.hh"

#include "UniversalRobotic.hh"

PoseQueryServerHandler::PoseQueryServerHandler(Smart::IQueryServerPattern<CommBasicObjects::CommVoid, CommManipulatorObjects::CommMobileManipulatorState>* server)
:	PoseQueryServerHandlerCore(server)
{
	CommBasicObjects::CommBasePose default_base_position;
	default_base_position.set_x(COMP->getGlobalState().getBase().getX());
	default_base_position.set_y(COMP->getGlobalState().getBase().getY());
	default_base_position.set_z(COMP->getGlobalState().getBase().getZ());

	// TODO: Set alpha, steer, turret to CommBasePose
	// default_base_position.set_base_alpha(COMP->getGlobalState().getBase().getBase_a());
	// default_base_position.set_steer_alpha(COMP->getGlobalState().getBase().getSteer_a());
	// default_base_position.set_turret_alpha(COMP->getGlobalState().getBase().getTurret_a());

	CommBasicObjects::CommBaseVelocity zero_velocity;
	zero_velocity.setVX(0);
	zero_velocity.setVY(0);
	zero_velocity.setVZ(0);
	zero_velocity.setWX(0);
	zero_velocity.setWY(0);
	zero_velocity.setWZ(0);

	default_baseState.set_base_position(default_base_position);
	default_baseState.set_base_velocity(zero_velocity);

}

PoseQueryServerHandler::~PoseQueryServerHandler()
{

}


void PoseQueryServerHandler::handleQuery(const Smart::QueryIdPtr &id, const CommBasicObjects::CommVoid& request)
{
	Smart::StatusCode status;
	CommManipulatorObjects::CommMobileManipulatorState answer;
	CommManipulatorObjects::CommManipulatorState state;

	if (COMP->getGlobalState().getManipulator().getVerbose())
	{
		std::cout << "Query Request received.." << std::endl;
	}

	UR->getCurrentState(state);
	answer.set_manipulator_state(state);

	// if baseServer is activated in the ini file get the baseState
	if (COMP->getGlobalState().getBase().getOn_base())
	{
		Smart::StatusCode status;
		CommBasicObjects::CommBaseState baseState;
		status = COMP->baseStateServiceIn->getUpdate(baseState);

		if (status == Smart::SMART_OK)
		{
			answer.set_base_state(baseState);
		} else
		{
			std::cout << "Error getting base state: " << Smart::StatusCodeConversion(status) << std::endl;
			answer.set_valid(false);
		}
	} else
	{
		answer.set_base_state(default_baseState);
	}

	if (COMP->getGlobalState().getManipulator().getVerbose() && answer.get_manipulator_state().is_valid())
	{
		if (answer.is_valid() && answer.get_manipulator_state().is_valid())
		{
			std::cout << "Query Request send valid.." << std::endl;
		} else
		{
			std::cout << "Query Request send NOT valid.." << std::endl;
		}
	}

   	this->server->answer(id, answer);
}
