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
//  Copyright (C) 2010 Jonas Brich
//
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
//-------------------------------------------------------------------------

#include "DemonstrationTask.hh"
#include "ComponentOpenRave.hh"

#include <iostream>

DemonstrationTask::DemonstrationTask(SmartACE::SmartComponent *comp) 
:	DemonstrationTaskCore(comp)
{
	std::cout << "constructor DemonstrationTask\n";
}
DemonstrationTask::~DemonstrationTask() 
{
	std::cout << "destructor DemonstrationTask\n";
}


void DemonstrationTask::on_GripperStateServiceIn(const CommManipulatorObjects::CommGripperState &input)
{
	// upcall triggered from InputPort GripperStateServiceIn
	// - use a local mutex here, because this upcal is called asynchroneously from outside of this task
	// - do not use longer blocking calls here since this upcall blocks the InputPort GripperStateServiceIn
	// - if you need to implement a long-running procedure, do so within the on_execute() method and in
	//   there, use the method gripperStateServiceInGetUpdate(input) to get a copy of the input object
}
void DemonstrationTask::on_MobileManipulatorStateServiceIn(const CommManipulatorObjects::CommMobileManipulatorState &input)
{
	// upcall triggered from InputPort MobileManipulatorStateServiceIn
	// - use a local mutex here, because this upcal is called asynchroneously from outside of this task
	// - do not use longer blocking calls here since this upcall blocks the InputPort MobileManipulatorStateServiceIn
	// - if you need to implement a long-running procedure, do so within the on_execute() method and in
	//   there, use the method mobileManipulatorStateServiceInGetUpdate(input) to get a copy of the input object
}

int DemonstrationTask::on_entry()
{
	// do initialization procedures here, which are called once, each time the task is started
	// it is possible to return != 0 (e.g. when initialization fails) then the task is not executed further
	this->pose_synced = false;
	return 0;
}
int DemonstrationTask::on_execute()
{
	// this method is called from an outside loop,
	// hence, NEVER use an infinite loop (like "while(1)") here inside!!!
	// also do not use blocking calls which do not result from smartsoft kernel
	
	// to get the incoming data, use this methods:
	Smart::StatusCode status;

	//COMP->stateServer->acquire("demonstration");
	//COMP->stateServer->release("demonstration");

	COMP->stateSlave->acquire("demonstration");
	COMP->stateSlave->release("demonstration");

	if (!COMP->getGlobalState().getPortParameter().getWithManipulator())
	{
		MessageHandler::handleMessage("[DemonstrationTask] No real manipulator is attached. Demonstration Modus not possible.",
				MessageHandler::WARNING);

		sleep(1);
	}

	else {

		if (!pose_synced)
		{
			OPENRAVE->syncManipulator();
			pose_synced = true;
		}

		CommManipulatorObjects::CommMobileManipulatorState mobileManipulatorStateServiceInObject;
		status = this->mobileManipulatorStateServiceInGetUpdate(mobileManipulatorStateServiceInObject);
		if(status != Smart::SMART_OK) {
			std::cerr << status << std::endl;
			// return 0;
		} else {
			std::cout << "received: " << mobileManipulatorStateServiceInObject << std::endl;
		}

		if (status != Smart::SMART_OK)
		{
			MessageHandler::handleMessage("[DemonstrationTask] Timed update of ManipulatorState could not be get.", MessageHandler::WARNING);
		}

		double x, y, z, phi, theta, psi;
		mobileManipulatorStateServiceInObject.get_manipulator_state().get_pose_TCP_robot(x, y, z, phi, theta, psi);
		cout << "[DemonstrationTask] (x, y, z, phi, theta, psi): " << x << ", " << y
				<< ", " << z << ", " << phi << ", " << theta << ", " << psi << std::endl;

		std::vector<double> manipulatorAngles;

		// Get all Angles from the real Manipulator
		for (u_int32_t i = 0; i < mobileManipulatorStateServiceInObject.get_manipulator_state().get_joint_count(); ++i)
		{
			manipulatorAngles.push_back(mobileManipulatorStateServiceInObject.get_manipulator_state().get_joint_angle(i));
		}

		// Get gripper angles

		if (COMP->getGlobalState().getPortParameter().getWithGripper())
		{
			CommManipulatorObjects::CommGripperState gripperStateServiceInObject;
			status = this->gripperStateServiceInGetUpdate(gripperStateServiceInObject);
			if(status != Smart::SMART_OK) {
				std::cerr << status << std::endl;
				// return 0;
			} else {
				std::cout << "received: " << gripperStateServiceInObject << std::endl;
			}

			if (status != Smart::SMART_OK)
			{
				MessageHandler::handleMessage("[DemonstrationTask] Timed update of GripperState could not be get.", MessageHandler::WARNING);
			}

			std::cout << "[DemonstrationTask] gripper: ";
			for (uint32_t i = 0; i < gripperStateServiceInObject.get_size(); ++ i)
			{
				manipulatorAngles.push_back(gripperStateServiceInObject.get_pos(i));
				std::cout << gripperStateServiceInObject.get_pos(i) << ", ";
			}
			std::cout << "\n";
		}

		// Move the robot arm in OpenRave to the same position of real robot arm
		OPENRAVE->setJoints(manipulatorAngles);
	}

	// it is possible to return != 0 (e.g. when the task detects errors), then the outer loop breaks and the task stops
	return 0;
}
int DemonstrationTask::on_exit()
{
	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	return 0;
}
