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
//
//  Copyright (C)  2018 Matthias Lutz
//
//              lutz@hs-ulm.de
//              schlegel@hs-ulm.de
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
//-------------------------------------------------------------------------
#include "UndockingTask.hh"
#include "ComponentRobotToRobotDocking.hh"

#include <iostream>

UndockingTask::UndockingTask(SmartACE::SmartComponent *comp) 
:	UndockingTaskCore(comp)
{
	std::cout << "constructor UndockingTask\n";

		_vxControl.push_back( Eigen::Vector2d( 0.01, 0 ) );
		_vxControl.push_back( Eigen::Vector2d( 0.01, 0.02 ) );
		_vxControl.push_back( Eigen::Vector2d( 0.1, 0.1 ) );
		_vxControl.push_back( Eigen::Vector2d( 0.4, COMP->getGlobalState().getDocking().getLaserDocking_transVelX() ) );

		_vyControl.push_back( Eigen::Vector2d( 0.01, 0 ) );
		_vyControl.push_back( Eigen::Vector2d( 0.01, 0.02 ) );
		_vyControl.push_back( Eigen::Vector2d( 0.1, 0.1 ) );
		_vyControl.push_back( Eigen::Vector2d( 0.4, COMP->getGlobalState().getDocking().getLaserDocking_transVelY() ) );
		_state = Backward;;
}
UndockingTask::~UndockingTask() 
{
	std::cout << "destructor UndockingTask\n";
}

void UndockingTask::undockInit(){
	_state = Backward;

}



int UndockingTask::on_entry()
{
	// do initialization procedures here, which are called once, each time the task is started
	// it is possible to return != 0 (e.g. when initialization fails) then the task is not executed further
	return 0;
}
int UndockingTask::on_execute()
{
	// this method is called from an outside loop,
	// hence, NEVER use an infinite loop (like "while(1)") here inside!!!
	// also do not use blocking calls which do not result from smartsoft kernel

	// to get the incoming data, use this methods:
	Smart::StatusCode status;
	CommRobotinoObjects::RobotinoDockingEventState goalEventState;

	status = COMP->stateSlave->acquire("UnDocking");
	if (status == Smart::SMART_OK)
	{
		if(COMP->savedBaseState_valid == true){

			CommBasicObjects::CommBaseState baseState;
			status = COMP->baseStatePushClient->getUpdateWait(baseState);
			if (status == Smart::SMART_OK) {

				double vx = 0.0;
				double vy = 0.0;

				Eigen::Vector2d pos( baseState.getBaseOdomPose().get_x(1), baseState.getBaseOdomPose().get_y(1));
				double orientation = baseState.getBaseOdomPose().get_base_azimuth();

				std::cout << "Robot " << baseState.getBaseOdomPose().get_x(1) << " " << baseState.getBaseOdomPose().get_y(1) << " " << rad2deg( baseState.getBaseOdomPose().get_base_azimuth() ) << "deg" << std::endl;
				std::cout<<  "Saved " << COMP->savedBaseState.getBaseOdomPose().get_x(1) << " " << COMP->savedBaseState.getBaseOdomPose().get_y(1) << " " << rad2deg( COMP->savedBaseState.getBaseOdomPose().get_base_azimuth() ) << "deg" << std::endl;

				Eigen::Vector2d goalPos;
				goalPos[0] = COMP->savedBaseState.getBaseOdomPose().get_x(1);
				goalPos[1] = COMP->savedBaseState.getBaseOdomPose().get_y(1);

				Eigen::Vector2d distVec = goalPos - pos;
				double dist = distVec.norm();

				std::cout<<"distVec: "<<distVec<<" dist: "<<dist<<std::endl;

				Eigen::Rotation2D< double > rotatiion_matrix_2d(-orientation);
				Eigen::Vector2d goalInRobotFrame = rotatiion_matrix_2d * distVec;

				std::cout<<"goalInRobotFrame: "<<goalInRobotFrame<<std::endl;


				vx = linearapproximation( _vxControl, fabs( goalInRobotFrame[0] ) );
				if( goalInRobotFrame[0] < 0 )
				{
					vx = -vx;
				}

				vy = linearapproximation( _vyControl, fabs( goalInRobotFrame[1] ) );
				if( goalInRobotFrame[1] < 0 )
				{
					vy = -vy;
				}

				CommBasicObjects::CommNavigationVelocity vel;

				switch( _state )
				{

				case Backward:
					vy = 0;
					if( 0 == vx )
					{
						_state = DirectDrive;
					}
					break;

				case DirectDrive:
					if(  0 == vx && 0 == vy )
					{
						_state = Finished;
						std::cout << "Drive -> Finished" << std::endl;
					}
					break;
				case Finished:
					vx = 0;
					vy = 0;

					//finished
					goalEventState.set(CommRobotinoObjects::RobotinoDockingEventType::UN_DOCKING_DONE);
					COMP->dockingEventServer->put(goalEventState);
					std::cout<<"GOALD EVENT UN_DOCKING_DONE FIRED!"<<std::endl;
					COMP->savedBaseState_valid = false;

					break;

				default:
					vx = 0;
					vy = 0;
					break;
				}

				std::cout << "[UndockingTask:] vx=" << vx << "m/s  vy=" << vy << "m/s  omega=" << rad2deg( 0 ) << "deg/s" << std::endl;
				vel.set_vX( vx, 1 ); //in m
				vel.set_vY( vy, 1 ); // in m
				vel.set_omega( 0 ); // in rad
				COMP->navVelSendClient->send(vel);
			}
			else
			{
				std::cerr << "[UnDockingTask] WARNING: failed to get current base state ("
						<< Smart::StatusCodeConversion(status) << ")" << std::endl;
			}

		} else {

			std::cout<<"Saved pose invalid - this should only happen once the goal is reached!"<<std::endl;
			ACE_OS::sleep(ACE_Time_Value(0,250000));

		}

		status = COMP->stateSlave->release("UnDocking");
	}

	// it is possible to return != 0 (e.g. when the task detects errors), then the outer loop breaks and the task stops
	return 0;
}
int UndockingTask::on_exit()
{
	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	return 0;
}