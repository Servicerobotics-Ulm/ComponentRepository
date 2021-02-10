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
//  Copyright (C) 2012 Jonas Brich, Timo Hegele
//
//        hegele@mail.hs-ulm.de
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
#include "TrajectorySampling.hh"
#include "ComponentOpenRave.hh"

#include <iostream>

TrajectorySampling::TrajectorySampling(SmartACE::SmartComponent *comp) 
:	TrajectorySamplingCore(comp)
{
	std::cout << "constructor TrajectorySampling\n";
}
TrajectorySampling::~TrajectorySampling() 
{
	std::cout << "destructor TrajectorySampling\n";
}

int TrajectorySampling::on_entry()
{
	// do initialization procedures here, which are called once, each time the task is started
	// it is possible to return != 0 (e.g. when initialization fails) then the task is not executed further
	return 0;
}
int TrajectorySampling::on_execute()
{
	// this method is called from an outside loop,
	// hence, NEVER use an infinite loop (like "while(1)") here inside!!!
	// also do not use blocking calls which do not result from smartsoft kernel
	
	// to get the incoming data, use this methods:
	Smart::StatusCode status;

	// it is possible to return != 0 (e.g. when the task detects errors), then the outer loop breaks and the task stops
	return 0;
}
int TrajectorySampling::on_exit()
{
	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	return 0;
}

void TrajectorySampling::handleJointAngles(const CommManipulatorObjects::CommManipulatorTrajectory &r,
		std::vector<ORUtil::TrajectoryPoint>& openraveTrajectory)
{
	MessageHandler::handleMessage("[PathPlanningSendHandler::handleJointAngles] Planning Path.",
			CommManipulationPlannerObjects::ManipulationPlannerEvent::PLANNING_PATH, MessageHandler::INFO, COMP->getGlobalState().getOpenRave().getDebugSend());

	std::vector<double> angles;

	for (uint32_t j = 0; j < r.get_joint_count(); ++j)
	{
		angles.push_back(r.get_joint_angle(0, j));
	}

	if (OPENRAVE->getParameter().simulation_test_ik_only && (COMP->stateSlave->tryAcquire("simulation") == Smart::SMART_OK))
	{
		COMP->stateSlave->release("simulation");
		MessageHandler::handleMessage(
				"[PathPlanningSendHandler::handleJointAngles] IK Solution Found with SIMULATION_TEST_IK_ONLY parameter.",
				CommManipulationPlannerObjects::ManipulationPlannerEvent::PATH_FOUND, MessageHandler::INFO, COMP->getGlobalState().getOpenRave().getDebugSend());
	}

	/*else if (!OPENRAVE->planPath(angles, openraveTrajectory))
	{
		throw MessageHandler::ErrorException("[PathPlanningSendHandler::handleJointAngles] Path planning failed.",
				CommManipulationPlannerObjects::ManipulationPlannerEvent::NO_PATH_FOUND);
	}*/

}

void TrajectorySampling::handlePoses(const CommManipulatorObjects::CommManipulatorTrajectory &r,
		std::vector<ORUtil::TrajectoryPoint>& openraveTrajectory)
{
	try
	{
		std::vector<double> angles;
		double x, y, z, azimuth, elevation, roll;

		// Get TCP Pose in robot coordinate frame
		r.get_pose_TCP_robot(x, y, z, azimuth, elevation, roll, 1);

		/*cout << "[PathPlanningSendHandler::handlePoses] r.get_pose_TCP_robot(x, y, z, azimuth, elevation, roll): " << x << ", " << y
				<< ", " << z << ", " << azimuth << ", " << elevation << ", " << roll << std::endl;*/
		CommBasicObjects::CommPose3d resultPoseAfterIteration;

		if (!OPENRAVE->iterateToGetGraspingIKSolution(x, y, z, azimuth, elevation, roll, angles, resultPoseAfterIteration))
		{
			throw MessageHandler::ErrorException(
					"[PathPlanningSendHandler::handlePoses] Iteration exceeded to find grasping IK solution using.",
					CommManipulationPlannerObjects::ManipulationPlannerEvent::NO_IK_SOLUTION_FOUND);
		}

		// found IK solution for simple grasping
		// Send PLANNING_PATH event with final TCP pose where the path is planned to
		MessageHandler::handleMessage("[PathPlanningSendHandler::handlePoses] Planning Path.",
				CommManipulationPlannerObjects::ManipulationPlannerEvent::PLANNING_PATH, resultPoseAfterIteration, MessageHandler::INFO,
				COMP->getGlobalState().getOpenRave().getDebugSend());

		/*if (!OPENRAVE->planPath(angles, openraveTrajectory))
		{
			throw MessageHandler::ErrorException("[PathPlanningSendHandler::handlePoses] Path planning failed.",
					CommManipulationPlannerObjects::ManipulationPlannerEvent::NO_PATH_FOUND);
		}*/
	} catch (MessageHandler::ErrorException ex)
	{
		throw ex;
	} catch (...)
	{
		throw MessageHandler::ErrorException("[PathPlanningSendHandler::handlePoses] Unknown error.",
				CommManipulationPlannerObjects::ManipulationPlannerEvent::UNKNOWN);
	}
}
