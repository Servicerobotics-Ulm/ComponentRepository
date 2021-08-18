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
//  Copyright (C) 2010 Manuel Wopfner
//
//        wopfner@hs-ulm.de
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
//--------------------------------------------------------------------------

#include "PTUStateQueryServiceAnswHandler.hh"
#include "ComponentPTUServer.hh"

#include <iostream>
#include "CommBasicObjects/CommBasePose.hh"

PTUStateQueryServiceAnswHandler::PTUStateQueryServiceAnswHandler(IQueryServer *server)
:	PTUStateQueryServiceAnswHandlerCore(server)
{
	
}


void PTUStateQueryServiceAnswHandler::handleQuery(const Smart::QueryIdPtr &id, const CommBasicObjects::CommVoid& request) 
{
	std::cout << "Query Request received.." << std::endl;

		DomainPTU::CommMobilePTUState answer;
		DomainPTU::CommPTUState ptuState;
		CommBasicObjects::CommBaseState base_state;

		double pan,tilt;

		CommBasicObjects::CommBasePose default_base_position;
		default_base_position.set_x(COMP->getGlobalState().getBase().getX());
		default_base_position.set_y(COMP->getGlobalState().getBase().getY());
		default_base_position.set_z(COMP->getGlobalState().getBase().getZ());
		default_base_position.set_base_azimuth(COMP->getGlobalState().getBase().getBase_a());
		default_base_position.set_base_elevation(COMP->getGlobalState().getBase().getSteer_a());
		default_base_position.set_base_roll(COMP->getGlobalState().getBase().getTurret_a());

		CommBasicObjects::CommBaseVelocity zero_velocity;
		zero_velocity.set_vX(0);
		zero_velocity.set_vY(0);
		zero_velocity.set_vZ(0);
		zero_velocity.setWX(0);
		zero_velocity.setWY(0);
		zero_velocity.setWZ(0);


		if (COMP->getGlobalState().getBase().getOn_base())
		{
			base_state = COMP->baseStateTask->getBaseState();
		} else
		{
			// wait until the ptu moves. (because it is a push newest)
			base_state.set_base_position(default_base_position);
			base_state.set_base_raw_position(default_base_position);
			base_state.set_base_velocity(zero_velocity);
			base_state.set_time_stamp(CommBasicObjects::CommTimeStamp::now());
		}

		COMP->ptuTask->getPos(pan, tilt);
		ptuState.set_pan_angle(pan);
		ptuState.set_tilt_angle(tilt);
		ptuState.set_pose_ptu(COMP->getGlobalState().getBase().getX(), COMP->getGlobalState().getBase().getY(), COMP->getGlobalState().getBase().getZ(),
				COMP->getGlobalState().getBase().getBase_a(), COMP->getGlobalState().getBase().getSteer_a(), COMP->getGlobalState().getBase().getTurret_a(), 0.001);
		ptuState.set_valid(true);

		answer.set_base_state(base_state);
		answer.set_ptu_state(ptuState);

		if (answer.get_ptu_state().is_valid())
		{
			std::cout << "Query Request send valid.." << std::endl;
		} else
		{
			std::cout << "Query Request send NOT valid.." << std::endl;
		}

		//answer.set(result);
		server->answer(id, answer);

}