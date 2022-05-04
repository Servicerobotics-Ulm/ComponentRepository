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

#include <list>
#include <set>
#include <string>
#include <fstream>
#include "ComponentOpenRave.hh"
#include "OpenRave.hh"
#include "util/ObjectXMLWriter.hh"
#include "util/ObjectDatabase.hh"

//#include <CommObjectRecognitionObjects/commObjectRecognitionId.hh>
//#include <CommObjectRecognitionObjects/commObjectRecognitionObjectProperties.hh>
//#include <CommBasicObjects/commPose3d.hh>

#include <boost/algorithm/string.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>

OpenRave* OpenRave::_instance = NULL;

//////////////////////////////////////////////////
//
//	public methods
//
//////////////////////////////////////////////////

OpenRave* OpenRave::instance()
{
	if (_instance == NULL)
	{
		SmartACE::SmartGuard guard(COMP->OpenRaveMutex);
		if (_instance == NULL)
		{
			_instance = new OpenRave();
            _instance->init();
		}
		guard.release();
	}
	return _instance;
}

OpenRave::~OpenRave()
{
}

void OpenRave::init()
{
	std::stringstream ss;
	ss << COMP->getGlobalState().getOpenRave().getObjectDatabasePath() << COMP->getGlobalState().getOpenRave().getObjectDatabaseXMLFilename();

	this->objectDatabase.init(ss.str());
}

void OpenRave::resetAll()
{
	// Delete all KinBodies except the manipulator
	COMP->OpenRaveWrapper.resetAll();
}

void OpenRave::syncManipulator()
{
	std::vector<double> manipulatorAngles;

	/////////////////////////////////////////////////////////////////
	// Sync Manipulator

	MessageHandler::handleMessage("Start synchronization. [OpenRave::syncManipulator]", MessageHandler::INFO,
			COMP->getGlobalState().getOpenRave().getDebugOpenRave());

	CommBasicObjects::CommVoid commVoid;

	// If the manipulator is attached, we work with it, otherwise we have a default behavior.
	if (COMP->getGlobalState().getPortParameter().getWithManipulator())
	{
		CommManipulatorObjects::CommMobileManipulatorState manipulatorState;
		do
		{
			Smart::StatusCode status = COMP->mobileManipulatorStateQueryServiceReq->query(commVoid, manipulatorState);
			//cout <<__FILE__ <<" "<<__LINE__<<std::endl;

			if (status != Smart::SMART_OK)
			{
				MessageHandler::handleMessage(
						" [OpenRave::syncManipulator] This is a WARNING, NOT AN ERROR: :-) Timed update of ManipulatorState could not be get.",
						CommManipulationPlannerObjects::ManipulationPlannerEvent::MANIPULATOR_SYNC_FAIL, MessageHandler::WARNING);
				return;
			}
		} while (!manipulatorState.is_valid());

		//DEBUG// std::cout << "[OpenRave::syncManipulator] Manipulator State: " << manipulatorState << std::endl;

		// Sync only once at the beginning. Arm should be stationary during the runtime of the component.
		if (!this->initial_sync)
		{
			// Translate robot arm so it fits into the environment
			// The 0,0,0 Point is in the second joint angle of the robot arm
			TransformMatrix transform;
			arma::mat kinTransform(4, 4);
			double x, y, z, azimuth, elevation, roll;

			COMP->OpenRaveWrapper.getKinBodyTransform(specificManipulator->getOpenRaveName(), transform);
			this->copy4x4TransformMatrixToMatrix(transform, kinTransform);

			manipulatorState.get_manipulator_state().get_pose_manipulator(x, y, z, azimuth, elevation, roll, 1);
			//DEBUG//std::cout << "Manipulator manipulatorState.get_manipulator_state().get_pose_manipulator():" << std::endl
			//DEBUG//		<< "   x,y,z, azimuth, elevation, roll [m] = " << x << " " << y << " " << z << " " << azimuth << " " << elevation
			//DEBUG//		<< " " << roll << std::endl;

			arma::mat baseOffset(4, 4);
			EulerTransformationMatrices::create_zyx_matrix(x, y, z, azimuth, elevation, roll, baseOffset);

			// get offset for the sake of coordinate differences: we have the origin in the first joint, open rave in the ground plate
			arma::mat offset = specificManipulator->getOffsetRealManipulatortoOpenRaveManipulator();
			arma::mat res = baseOffset * offset * kinTransform;

			this->copy4x4MatrixToTransformMatrix(res, transform);

			//DEBUG//std::cout << "Transform: (new pose of manipulator in open rave incl 'OffsetRealManipulatortoOpenRaveManipulator')" << std::endl
			//DEBUG//		<< "   x,y,z = " << transform[0][3] << ", " << transform[1][3] << ", " << transform[2][3] << std::endl;

			COMP->OpenRaveWrapper.moveKinbody(specificManipulator->getOpenRaveName(), transform); // Transforms the robot and updates the attached sensors and grabbed bodies.

			this->initial_sync = true;
		}

		// Get all Angles from the real Manipulator inclusive the gripper
		for (u_int32_t i = 0; i < manipulatorState.get_manipulator_state().get_joint_count(); ++i)
		{
			manipulatorAngles.push_back(manipulatorState.get_manipulator_state().get_joint_angle(i));
			std::cout << "manipulatorAngles[" << i << "]: " << manipulatorState.get_manipulator_state().get_joint_angle(i) << std::endl;
		}

		// Move the robot arm in OpenRave to the same position of real robot arm
		this->setJoints(manipulatorAngles);

	}

	// If we are in simulation (no arm attached)
	else
	{
		// Sync only once at the beginning. Arm should be stationary during the runtime of the component.
		if (!this->initial_sync)
		{
			// get offset for the sake of coordinate differences: we have the origin in the first joint, open rave in the ground plate
			arma::mat simulationTransform = specificManipulator->getSimulationTransform();
			// z-pose hard coded because only simulation and no real arm connected that can offer that pose
			TransformMatrix transform;
			this->copy4x4MatrixToTransformMatrix(simulationTransform, transform);
			COMP->OpenRaveWrapper.moveKinbody(specificManipulator->getOpenRaveName(), transform);

			specificManipulator->getSimulationJointAngles(manipulatorAngles);
			this->setJoints(manipulatorAngles);

			this->initial_sync = true;
		}

	}

	/////////////////////////////////////////////////////////////////
	// Sync Gripper

	if (COMP->getGlobalState().getPortParameter().getWithGripper() && !this->obj_grasped)
	{
		std::cout << "[OpenRave::syncManipulator] sync gripper..." << std::endl;
		CommManipulatorObjects::CommGripperState gripperState;
		do
		{
			Smart::StatusCode status = COMP->gripperStateQueryServiceReq->query(commVoid, gripperState);

			if (status != Smart::SMART_OK)
			{
				MessageHandler::handleMessage(
						" [OpenRave::syncManipulator] This is a WARNING, NOT AN ERROR: :-) Timed update of GripperState could not be get.",
						CommManipulationPlannerObjects::ManipulationPlannerEvent::MANIPULATOR_SYNC_FAIL, MessageHandler::WARNING);
				return;
			}

			if (!gripperState.is_valid())
			{
				std::cout << "[OpenRave::syncManipulator] invalid gripper state!!" << std::endl;
				sleep(1);
			}

		} while (!gripperState.is_valid());

		std::cout << "[OpenRave::syncManipulator] Gripper State: " << gripperState << std::endl;

		for (uint32_t i = 0; i < gripperState.get_size(); ++i)
		{
			manipulatorAngles.push_back(gripperState.get_pos(i));
		}

		// Move the robot arm/gripper in OpenRave to the same position of real robot arm/gripper
		this->setJoints(manipulatorAngles);
	}

	MessageHandler::handleMessage("Manipulator successfully synchronized. [OpenRave::syncManipulator]", MessageHandler::INFO,
			COMP->getGlobalState().getOpenRave().getDebugOpenRave());
}

OpenRave::OpenRaveParameter OpenRave::getParameter() const
{
	return this->localParameters;
}

void OpenRave::setJoints(const std::vector<double>& angles)
{
	COMP->OpenRaveWrapper.setJoints(angles);
}

void OpenRave::openGripper()
{
	std::cout << "[OpenRave::openGripper] Open gripper\n";
	std::vector<ORUtil::TrajectoryPoint> resultTraj;

	if (!COMP->OpenRaveWrapper.openGripper(resultTraj))
	{
		MessageHandler::handleMessage("[OpenRave::openGripper] Gripper could not be opened.", MessageHandler::WARNING);
	}
}

void OpenRave::closeGripper()
{
	std::cout << "[OpenRave::closeGripper] Close gripper\n";
	std::vector<ORUtil::TrajectoryPoint> resultTraj;

	if (!COMP->OpenRaveWrapper.closeGripper(resultTraj))
	{
		MessageHandler::handleMessage("[OpenRave::closeGripper] Gripper could not be closed.", MessageHandler::WARNING);
	}
}

bool OpenRave::calculateIKSolution(double x, double y, double z, double azimuth, double elevation, double roll,
		std::vector<double>& solution, CommBasicObjects::CommPose3d& pose)
{
	std::vector<double> cTarget;
	cTarget.push_back(x);
	cTarget.push_back(y);
	cTarget.push_back(z);

	ORUtil::PlanningParameter params;
	params.targetLoc3D = cTarget;

	params.azimuth = radToDeg(azimuth);
	params.elevation = radToDeg(elevation);
	params.roll = radToDeg(roll);


	if (COMP->OpenRaveWrapper.searchIKSolution(params, solution) == 0)
	{
		pose.set_x(x, 1);
		pose.set_y(y, 1);
		pose.set_z(z, 1);
		pose.set_azimuth(azimuth);
		pose.set_elevation(elevation);
		pose.set_roll(roll);
		return true;
	}

	return false;
}

bool OpenRave::iterateToGetGraspingIKSolution(double x, double y, double z, double azimuth, double elevation, double roll, std::vector<
		double>& solution, CommBasicObjects::CommPose3d& pose)
{
	int result = 0;
	vector<vector<double> > transform;
	std::vector<double> cTarget;
	cTarget.push_back(x);
	cTarget.push_back(y);
	cTarget.push_back(z);

	ORUtil::PlanningParameter params;
	params.verbose = (COMP->getGlobalState().getOpenRave().getDebugGrasping()) ? 1 : 0;
	params.targetLoc3D = cTarget;

	params.azimuth = radToDeg(azimuth);
	params.elevation = radToDeg(elevation);
	params.roll = radToDeg(roll);

	params.azimuthAngleFrom = localParameters.ikSolutionPoseBoundaries.fromAzimuth;
	params.azimuthAngleTo = localParameters.ikSolutionPoseBoundaries.toAzimuth;
	params.elevationAngleFrom = localParameters.ikSolutionPoseBoundaries.fromElevation;
	params.elevationAngleTo = localParameters.ikSolutionPoseBoundaries.toElevation;
	params.rollAngleFrom = localParameters.ikSolutionPoseBoundaries.fromRoll;
	params.rollAngleTo = localParameters.ikSolutionPoseBoundaries.toRoll;

	params.depthFrom = localParameters.ikSolutionPoseBoundaries.fromDepth;
	params.depthTo = localParameters.ikSolutionPoseBoundaries.toDepth;
	params.sideFrom = localParameters.ikSolutionPoseBoundaries.fromSide;
	params.sideTo = localParameters.ikSolutionPoseBoundaries.toSide;
	params.heightFrom = localParameters.ikSolutionPoseBoundaries.fromHeight;
	params.heightTo = localParameters.ikSolutionPoseBoundaries.toHeight;

	std::cout << "localParameters.ikSolutionPoseBoundaries.fromAzimuth: " << localParameters.ikSolutionPoseBoundaries.fromAzimuth << std::endl;
	std::cout << "localParameters.ikSolutionPoseBoundaries.toAzimuth: " << localParameters.ikSolutionPoseBoundaries.toAzimuth << std::endl;

	std::cout << "localParameters.ikSolutionPoseBoundaries.fromElevation: " << localParameters.ikSolutionPoseBoundaries.fromElevation << std::endl;
	std::cout << "localParameters.ikSolutionPoseBoundaries.toElevation: " << localParameters.ikSolutionPoseBoundaries.toElevation << std::endl;

	std::cout << "localParameters.ikSolutionPoseBoundaries.fromRoll: " << localParameters.ikSolutionPoseBoundaries.fromRoll << std::endl;
	std::cout << "localParameters.ikSolutionPoseBoundaries.toRoll: " << localParameters.ikSolutionPoseBoundaries.toRoll << std::endl;

	std::cout << "localParameters.ikSolutionPoseBoundaries.fromDepth: " << localParameters.ikSolutionPoseBoundaries.fromDepth << std::endl;
	std::cout << "localParameters.ikSolutionPoseBoundaries.toDepth: " << localParameters.ikSolutionPoseBoundaries.toDepth << std::endl;

	std::cout << "localParameters.ikSolutionPoseBoundaries.fromSide: " << localParameters.ikSolutionPoseBoundaries.fromSide << std::endl;
	std::cout << "localParameters.ikSolutionPoseBoundaries.toSide: " << localParameters.ikSolutionPoseBoundaries.toSide << std::endl;

	std::cout << "localParameters.ikSolutionPoseBoundaries.fromHeight: " << localParameters.ikSolutionPoseBoundaries.fromHeight << std::endl;
	std::cout << "localParameters.ikSolutionPoseBoundaries.toHeight: " << localParameters.ikSolutionPoseBoundaries.toHeight << std::endl;

	//while (std::cin.get() != '\n');

	params.depthStepSize = 0.005;
	params.heightStepSize = 0.005;

	params.azimuthAngleStepSize = 4;
	params.elevationAngleStepSize = 3;
	params.rollAngleStepSize = 2;

	std::cout << "now going to start iteration for an IK solution, with the following parameters" << "\n";
	std::cout << params.toString() << "\n";

	switch(OPENRAVE->getParameter().iterationMode)
	{
		case OpenRave::OpenRaveParameter::IterationMode::SIMPLE_COMPS:
		{
			// This case uses the generalik of the COMPs library and should work for Larry/UR5
			std::cout << "Applying SIMPLE Strategy mode" << std::endl;
			result = COMP->OpenRaveWrapper.iterForIKSolutionsCOMPS(params, solution, transform);
			break;
		}
		case OpenRave::OpenRaveParameter::IterationMode::CENTERED:
		{
			std::cout << "Applying CENTERED Strategy mode" << std::endl;
			result = COMP->OpenRaveWrapper.iterForIKSolutionsCentered(params, solution, transform);
			break;
		}
		default:
		{
			std::cout << "Applying No Iteration, just trying to grab a IK-SOlution at the Specified 6D Pose" << std::endl;
			result = COMP->OpenRaveWrapper.searchIKSolution(params, solution);
		}
	}


	if (result != 0)
	{
		MessageHandler::handleMessage("[OpenRave::iterateToGetGraspingIKSolution] Iteration returned with no Result.",
				MessageHandler::ERROR);
		return false;
	}

	arma::mat matrix;
	copy4x4TransformMatrixToMatrix(transform, matrix);
	pose = CommBasicObjects::CommPose3d(matrix, 1);

	return true;
}

bool OpenRave::planPath(const std::vector<double>& angles, std::vector<ORUtil::TrajectoryPoint>& trajectory)
{
	MessageHandler::handleMessage("Planning path.", MessageHandler::INFO, COMP->getGlobalState().getOpenRave().getDebugOpenRave());
	int result = 0;
	switch(OPENRAVE->getParameter().planningMode)
	{
		case OpenRave::OpenRaveParameter::PathPlanningMode::NORMAL:
		{
			result = COMP->OpenRaveWrapper.moveJoints(angles, trajectory);
			break;
		}
		case OpenRave::OpenRaveParameter::PathPlanningMode::CONSTRAINED:
		{
			ORUtil::PlanningParameter params;
			params.azimuthAngleFrom = localParameters.constrainedPathBoundaries.fromAzimuth;
			params.azimuthAngleTo = localParameters.constrainedPathBoundaries.toAzimuth;
			params.elevationAngleFrom = localParameters.constrainedPathBoundaries.fromElevation;
			params.elevationAngleTo = localParameters.constrainedPathBoundaries.toElevation;
			params.rollAngleFrom = localParameters.constrainedPathBoundaries.fromRoll;
			params.rollAngleTo = localParameters.constrainedPathBoundaries.toRoll;
			params.depthFrom = localParameters.constrainedPathBoundaries.fromDepth;
			params.depthTo = localParameters.constrainedPathBoundaries.toDepth;
			params.heightFrom = localParameters.constrainedPathBoundaries.fromHeight;
			params.heightTo = localParameters.constrainedPathBoundaries.toHeight;
            params.sideFrom = localParameters.constrainedPathBoundaries.fromSide;
			params.sideTo = localParameters.constrainedPathBoundaries.toSide;

			result = COMP->OpenRaveWrapper.moveJointsConstrained(angles, params, trajectory);
			break;
		}
		default:
		{
			MessageHandler::handleMessage("[OpenRave::planPath] Unrecognized planning Mode.",
							MessageHandler::ERROR);
			return false;
		}
	}

	// Blender: Test to draw tcp axes
	COMP->OpenRaveWrapper.drawTCPAxes();

	MessageHandler::handleMessage("Plan Path Finished.", MessageHandler::INFO, COMP->getGlobalState().getOpenRave().getDebugOpenRave());

	if (COMP->getGlobalState().getOpenRave().getSaveTrajectoryToFile())
	{
		MessageHandler::handleMessage("Writing Trajectory.", MessageHandler::INFO, COMP->getGlobalState().getOpenRave().getDebugOpenRave());
		std::ofstream output;
		output.open("./trajectoryOpenRave.txt", std::ofstream::trunc);

		for (size_t i = 0; i < trajectory.size(); i++)
		{
			for (size_t j = 0; j < trajectory[i].jointValues.size(); j++)
			{
				output << trajectory[i].jointValues[j];

				if (j < trajectory[i].jointValues.size() - 1)
					output << ", ";
			}

			if (i < trajectory.size() - 1)
				output << "\n";
		}

		output.close();
	}

	if (result >= 0)
	{
		std::cout << "[OpenRave::planPath] Planner returned true! (Trajectory size: " << result << ")\n";
		return true;
	} else
	{
		std::cout << "[OpenRave::planPath] Planner returned false! (ErrorCode: " << result << ")\n";
		return false;
	}
}

//////////////////////////////////////////////////
//
//	private methods
//
//////////////////////////////////////////////////

OpenRave::OpenRave()
{
	this->specificManipulator = MANIPULATORFACTORY->createManipulatorClass(COMP->getGlobalState().getOpenRave().getRobotName());

	this->localParameters = this->globalParameters;
	this->initial_sync = false;

	this->obj_grasped = false;
}

void OpenRave::loadSingleObjectFromObjRecognition(unsigned int objId)
{

	CommObjectRecognitionObjects::CommObjectRecognitionId request;
	request.set_id(objId);
	CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties response;

	std::cout << "Query Object from BoxDetection ************************** " << std::endl;

	Smart::StatusCode status = COMP->objectQueryServiceReq->query(request, response);

	if (status != Smart::SMART_OK)
	{
		MessageHandler::handleMessage("[OpenRave::loadSingleObjectFromObjRecognition] Object could not be get from object recognition.",
				MessageHandler::WARNING);
		return;
	} else
	{
		MessageHandler::handleMessage("[OpenRave::loadSingleObjectFromObjRecognition] Object queried.", MessageHandler::INFO);
	}

	double x = 0;
	double y = 0;
	double z = 0;
	ObjectXMLWriter writer;

	//////////////////////////////////////////////////////

	//ObjectDatabase database("");

	CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties object = response;

	std::string output;

	std::string type;
	object.get_type(type);

	std::cout << "[OpenRave::loadSingleObjectFromObjRecognition] Loading \"" << type << "\" object into environment." << std::endl;
	std::cout << "TYPE: " << type << std::endl;

	CommBasicObjects::CommPose3d pose = object.get_pose();

	std::stringstream stream;
	stream << object.get_id();

		if (type == "KITCHEN-COUNTER")
		{
			std::stringstream fileStream;
			fileStream <<"/usr/share/openrave/robots/ikeaKitchenZAFH.robot.xml";
			if (COMP->OpenRaveWrapper.loadFile(fileStream.str()))
			{
				MessageHandler::handleMessage("[OpenRave::loadEnvironmentFromFile] File environment load was successful File: " + fileStream.str(),
						MessageHandler::INFO, COMP->getGlobalState().getOpenRave().getDebugOpenRave());

				if(!COMP->OpenRaveWrapper.renameKinbody("ikeaKitchen", stream.str()))
				{
					std::cout<<"Error renaming kitchen!"<<std::endl;
				}
				TransformMatrix transform;
				arma::mat matrix = pose.getHomogeneousMatrix(1);
				this->copy4x4MatrixToTransformMatrix(matrix, transform);

				if(!COMP->OpenRaveWrapper.moveKinbody(stream.str(), transform))
				{
					std::cout<<"Error moving kitchen!"<<std::endl;
				}
				//PyObject* kitchenBody = COMP->OpenRaveWrapper.getKinBody("ikeaKitchen");
				//if (kitchenBody != Py_None){
				//	std::cout<<"got kitchen"<<std::endl;
				//}
				//else {
				//	std::cout<<"ERROR did not get kitchen ID"<<std::endl;
				//}
			} else
			{
				MessageHandler::handleMessage("[OpenRave::loadEnvironmentFromFile] File environment could not be loaded File: " + fileStream.str(),
						MessageHandler::ERROR);
			}
			return;
                }

		if (type == "SIDEBOARD-600")
		{
			std::stringstream fileStream;
			fileStream <<"/usr/share/openrave/robots/ikeaSideboard.robot.xml";
			if (COMP->OpenRaveWrapper.loadFile(fileStream.str()))
			{
				MessageHandler::handleMessage("[OpenRave::loadEnvironmentFromFile] File environment load was successful File: " + fileStream.str(),
						MessageHandler::INFO, COMP->getGlobalState().getOpenRave().getDebugOpenRave());

				if(!COMP->OpenRaveWrapper.renameKinbody("ikeaSideboard", stream.str()))
				{
					std::cout<<"Error renaming sideboard!"<<std::endl;
				}
				TransformMatrix transform;
				arma::mat matrix = pose.getHomogeneousMatrix(1);
				this->copy4x4MatrixToTransformMatrix(matrix, transform);

				if(!COMP->OpenRaveWrapper.moveKinbody(stream.str(), transform))
				{
					std::cout<<"Error moving sideboard!"<<std::endl;
				}
				//PyObject* kitchenBody = COMP->OpenRaveWrapper.getKinBody("ikeaKitchen");
				//if (kitchenBody != Py_None){
				//	std::cout<<"got kitchen"<<std::endl;
				//}
				//else {
				//	std::cout<<"ERROR did not get kitchen ID"<<std::endl;
				//}
			} else
			{
				MessageHandler::handleMessage("[OpenRave::loadEnvironmentFromFile] File environment could not be loaded File: " + fileStream.str(),
						MessageHandler::ERROR);
			}
			return;
                }

	// Move the table a little bit down to prevent collisions with objects
	if (type == "TABLE" || type == "table")
	{
		// move table 10cm down, then make it 20cm thick. this will prevent open rave
		// from planning under the table while the robot still can stand inside the contour of
		// the table
		std::cout << "Table pose from Objrec.: " << pose << std::endl;
		pose.set_z(pose.get_z(1) - 0.025, 1);
		object.get_dimension(x, y, z, 1);
		object.set_dimension(x, y, 0.05, 1);
		//object.set_dimension(x, y, 0.001, 1);
	}
	else if (type == "OBSTACLE-HULL")
	{

		std::cout << "type == OBSTACLE-HULL _____________________________________ " << std::endl;
		std::vector<std::vector<uint32_t> > vertices;
		std::vector<std::vector<double> > points;
		std::cout << "GET TRI MESH ************************************" << std::endl;
		object.get_triMesh(vertices, points);
		std::cout << "[OpenRave::loadSingleObjectFromObjRecognition] Obstacle mesh size vertices: " << vertices.size() << " points: " << points.size()
				<< std::endl;
		// Timo: Add TriMesh SingleObjectFromObjRecognition		
		COMP->OpenRaveWrapper.addTriMesh(stream.str(), vertices, points);
		return;
	}


	if (this->objectDatabase.isKnownObject(type))
	{
		if (this->objectDatabase.getObjectShape(type) == ObjectDatabase::BOX)
		{
			this->objectDatabase.getObjectBox(x, y, z);
			writer.writeKinBodyBox(stream.str(), x, y, z, output, COMP->getGlobalState().getOpenRave().getSaveObjectsToFile());
		} else if (this->objectDatabase.getObjectShape(type) == ObjectDatabase::CYLINDER)
		{
			this->objectDatabase.getObjectCylinder(x, y);
			writer.writeKinBodyCylinder(stream.str(), x, y, output, COMP->getGlobalState().getOpenRave().getSaveObjectsToFile());
		} else if (this->objectDatabase.getObjectShape(type) == ObjectDatabase::SPHERE)
		{
			this->objectDatabase.getObjectSphere(x);
			writer.writeKinBodySphere(stream.str(), x, output, COMP->getGlobalState().getOpenRave().getSaveObjectsToFile());
		} else if (this->objectDatabase.getObjectShape(type) == ObjectDatabase::MESH)
		{
			std::string filename;
			this->objectDatabase.getObjectMeshFilename(filename);
			writer.writeKinBodyMesh(stream.str(), filename, output, COMP->getGlobalState().getOpenRave().getSaveObjectsToFile());
		}
	} else
	{
		// x,y,z = dimensions in width, height, length
		object.get_dimension(x, y, z, 1);
		writer.writeKinBodyBox(stream.str(), x, y, z, output, COMP->getGlobalState().getOpenRave().getSaveObjectsToFile());
	}
	this->addKinBody(object.get_id(), output, pose.getHomogeneousMatrix(1));

	MessageHandler::handleMessage("[OpenRave::loadSingleObjectFromObjRecognition] Load single object done.", MessageHandler::INFO,
			COMP->getGlobalState().getOpenRave().getDebugOpenRave());

}

void OpenRave::loadEnvironment(unsigned int envId)
{
	CommObjectRecognitionObjects::CommObjectRecognitionId request;
	CommObjectRecognitionObjects::CommObjectRecognitionEnvironment response;

	request.set_id(envId);
	Smart::StatusCode status = COMP->environmentQueryServiceReq->query(request, response);

	if (status != Smart::SMART_OK)
	{
		MessageHandler::handleMessage("[OpenRave::loadEnvironment] Environment could not be get from object recognition.",
				MessageHandler::WARNING);
		return;
	} else
	{
		MessageHandler::handleMessage("[OpenRave::loadEnvironment] Environment from Objrec. queried.", MessageHandler::INFO);
	}

	ObjectXMLWriter writer;
	double x, y, z;

	// TODO: initialize database
	//ObjectDatabase database("");

	std::cout << "[OpenRave::loadEnvironment] Load environment with " << response.get_size() << " objects" << std::endl;

	for (uint32_t i = 0; i < response.get_size(); i++)
	{
		CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties object = response.get_object_properties(i);

		///////////////// DEBUG ///////////////////
//		{
//
//		int index;
//		std::string type;
//		double probability,x,y,z,yaw,pitch,roll;
//		arma::mat mat;
//		for(index=0;index<object.get_beliefs_size();index++){
//			object.get_belief(index, type, probability, x, y, z, yaw, pitch, roll, mat,1);
//
//			std::cout<<"[type P] x y z yaw pitch roll: "<<"["<<type<<" "<<probability<<"] "<<x<<" "<<y<<" "<<z<<" "<<yaw<<" "<<pitch<<" "<<roll<<std::endl;
//			std::cout<<"cov: "<<mat<<std::endl;
//		}
//
//		}
		///////////////// END DEBUG ///////////////

		std::string output;

		std::string type;
		object.get_type(type);

		std::stringstream stream;
		stream << object.get_id();

		std::cout << "[OpenRave::loadEnvironment] Loading object \"" << type << "\" in environment " << std::endl;

		CommBasicObjects::CommPose3d pose = object.get_pose();
		CommBasicObjects::CommPose3d sPose = object.getObjectSurfacePosesCopy()[0];

		if (type == "SIDEBOARD-600")
		{
			std::stringstream fileStream;
			fileStream <<"/usr/share/openrave/robots/ikeaSideboard.robot.xml";
			if (COMP->OpenRaveWrapper.loadFile(fileStream.str()))
			{
				MessageHandler::handleMessage("[OpenRave::loadEnvironmentFromFile] File environment load was successful File: " + fileStream.str(),
						MessageHandler::INFO, COMP->getGlobalState().getOpenRave().getDebugOpenRave());

				if(!COMP->OpenRaveWrapper.renameKinbody("ikeaSideboard-600", stream.str()))
				{
					std::cout<<"Error renaming sideboard!"<<std::endl;
				}
				TransformMatrix transform;
				arma::mat matrix = pose.getHomogeneousMatrix(1);
				this->copy4x4MatrixToTransformMatrix(matrix, transform);

				if(!COMP->OpenRaveWrapper.moveKinbody(stream.str(), transform))
				{
					std::cout<<"Error moving sideboard!"<<std::endl;
				}
				//PyObject* kitchenBody = COMP->OpenRaveWrapper.getKinBody("ikeaKitchen");
				//if (kitchenBody != Py_None){
				//	std::cout<<"got kitchen"<<std::endl;
				//}
				//else {
				//	std::cout<<"ERROR did not get kitchen ID"<<std::endl;
				//}
			} else
			{
				MessageHandler::handleMessage("[OpenRave::loadEnvironmentFromFile] File environment could not be loaded File: " + fileStream.str(),
						MessageHandler::ERROR);
			}
			continue;
		}
		if (type == "KITCHEN-COUNTER")
		{
			std::stringstream fileStream;
			fileStream <<"/usr/share/openrave/robots/ikeaKitchenZAFH.robot.xml";
			if (COMP->OpenRaveWrapper.loadFile(fileStream.str()))
			{
				MessageHandler::handleMessage("[OpenRave::loadEnvironmentFromFile] File environment load was successful File: " + fileStream.str(),
						MessageHandler::INFO, COMP->getGlobalState().getOpenRave().getDebugOpenRave());

				if(!COMP->OpenRaveWrapper.renameKinbody("ikeaKitchen", stream.str()))
				{
					std::cout<<"Error renaming kitchen!"<<std::endl;
				}
				TransformMatrix transform;
				arma::mat matrix = pose.getHomogeneousMatrix(1);
				this->copy4x4MatrixToTransformMatrix(matrix, transform);

				if(!COMP->OpenRaveWrapper.moveKinbody(stream.str(), transform))
				{
					std::cout<<"Error moving kitchen!"<<std::endl;
				}
				//PyObject* kitchenBody = COMP->OpenRaveWrapper.getKinBody("ikeaKitchen");
				//if (kitchenBody != Py_None){
				//	std::cout<<"got kitchen"<<std::endl;
				//}
				//else {
				//	std::cout<<"ERROR did not get kitchen ID"<<std::endl;
				//}
			} else
			{
				MessageHandler::handleMessage("[OpenRave::loadEnvironmentFromFile] File environment could not be loaded File: " + fileStream.str(),
						MessageHandler::ERROR);
			}
			continue;
		}
		// Move the table a little bit down to prevent collisions with objects
		if (type == "TABLE" || type == "table")
		{
			// move table 10cm down, then make it 20cm thick. this will prevent open rave
			// from planning under the table while the robot still can stand inside the contour of
			// the table
			std::cout << "[OpenRave::loadEnvironment] Table pose from Objrec.: " << pose.get_x(1) << " " << pose.get_y(1) << " "
					<< pose.get_z(1) << std::endl;
			pose.set_z(pose.get_z(1) - 0.025, 1);
			object.get_dimension(x, y, z, 1);
			object.set_dimension(x, y, 0.05, 1);
			//object.set_dimension(x, y, 0.001, 1);
		} else if (type == "OBSTACLE-HULL")
		{
		std::cout << "type == OBSTACLE-HULL _____________________________________ " << std::endl;
		std::vector<std::vector<uint32_t> > vertices;
		std::vector<std::vector<double> > points;
		std::cout << "GET TRI MESH ************************************" << std::endl;
			object.get_triMesh(vertices, points);

			std::cout << "[OpenRave::loadEnvironment] Obstacle mesh size vertices: " << vertices.size() << " points: " << points.size()
					<< std::endl;
			std::cout << "[OpenRave::loadEnvironment] Obstacle name: " << stream.str()<< std::endl;

			// Timo: Add TriMesh LoadEnvironment	
			COMP->OpenRaveWrapper.addTriMesh(stream.str(), vertices, points);
			continue;

		}



		if (this->objectDatabase.isKnownObject(type))
		{
			if (this->objectDatabase.getObjectShape(type) == ObjectDatabase::BOX)
			{
				this->objectDatabase.getObjectBox(x, y, z);
				writer.writeKinBodyBox(stream.str(), x, y, z, output, COMP->getGlobalState().getOpenRave().getSaveObjectsToFile());
			} else if (this->objectDatabase.getObjectShape(type) == ObjectDatabase::CYLINDER)
			{
				this->objectDatabase.getObjectCylinder(x, y);
				writer.writeKinBodyCylinder(stream.str(), x, y, output, COMP->getGlobalState().getOpenRave().getSaveObjectsToFile());
			} else if (this->objectDatabase.getObjectShape(type) == ObjectDatabase::MESH)
			{
				std::string filename;
				this->objectDatabase.getObjectMeshFilename(filename);
				writer.writeKinBodyMesh(stream.str(), filename, output, COMP->getGlobalState().getOpenRave().getSaveObjectsToFile());
			}
			else if (this->objectDatabase.getObjectShape(type) == ObjectDatabase::SPHERE)
			{
				std::string filename;
				this->objectDatabase.getObjectSphere(x);
				writer.writeKinBodySphere(stream.str(), x, output, COMP->getGlobalState().getOpenRave().getSaveObjectsToFile());
			}
		} else
		{
			std::cout<<__LINE__<<" Type: "<<type<<std::endl;
			if (type != "TABLE" && type != "table" )
			{
				std::cout<<"Change dimensions"<<std::endl;
				pose.set_z(pose.get_z(1) + 0.025, 1);
				object.get_dimension(x, y, z, 1);
				object.set_dimension(x+0.04, y+0.04, z+0.05, 1);
			}
			// x,y,z = dimensions in width, height, length
			object.get_dimension(x, y, z, 1);
			writer.writeKinBodyBox(stream.str(), x, y, z, output, COMP->getGlobalState().getOpenRave().getSaveObjectsToFile());
		}

		{ // block for debugging
			std::cout << "[OpenRave::loadEnvironment] Object Type: " << type << std::endl;
			std::cout << "[OpenRave::loadEnvironment]        Pose: " << pose << std::endl;
			object.get_dimension(x, y, z, 1);
			std::cout << "[OpenRave::loadEnvironment]        dimension x/y/z: " << x << "/" << y << "/" << z << std::endl;
		}

		this->addKinBody(object.get_id(), output, pose.getHomogeneousMatrix(1));
	}

	MessageHandler::handleMessage("[OpenRave::loadEnvironment] Environment load done!", MessageHandler::INFO,
			COMP->getGlobalState().getOpenRave().getDebugOpenRave());
}

void OpenRave::loadEnvironmentFromFile(unsigned int envId)
{
	std::stringstream stream;
	stream << COMP->getGlobalState().getOpenRave().getStoredEnvironmentPath();
	stream << COMP->getGlobalState().getOpenRave().getStoredEnvironmentName();
	stream << envId;
	stream << ".xml";

	if (COMP->OpenRaveWrapper.loadFile(stream.str()))
	{
		MessageHandler::handleMessage("[OpenRave::loadEnvironmentFromFile] File environment load was successful File: " + stream.str(),
				MessageHandler::INFO, COMP->getGlobalState().getOpenRave().getDebugOpenRave());
	} else
	{
		MessageHandler::handleMessage("[OpenRave::loadEnvironmentFromFile] File environment could not be loaded File: " + stream.str(),
				MessageHandler::ERROR);
	}

}

void OpenRave::saveEnvironmentToFile(unsigned int envId)
{
	std::stringstream stream;
	stream << COMP->getGlobalState().getOpenRave().getStoredEnvironmentPath();
	stream << COMP->getGlobalState().getOpenRave().getStoredEnvironmentName();
	stream << envId;
	stream << ".dae";

	if (COMP->OpenRaveWrapper.saveEnvironment(stream.str()))
	{
		MessageHandler::handleMessage("[OpenRave::saveEnvironmentToFile] Environment could not be saved. File: " + stream.str(),
				MessageHandler::ERROR);
	} else
	{
		MessageHandler::handleMessage("[OpenRave::saveEnvironmentToFile] Environment save was successful. File: " + stream.str(),
				MessageHandler::INFO, COMP->getGlobalState().getOpenRave().getDebugOpenRave());
	}
}

void OpenRave::addKinBody(unsigned int objId, const std::string& data, const arma::mat& matrix)
{
	TransformMatrix transform;
	this->copy4x4MatrixToTransformMatrix(matrix, transform);

	std::stringstream id;
	id << objId;

	if (COMP->OpenRaveWrapper.createKinbody(data) && COMP->OpenRaveWrapper.moveKinbody(id.str(), transform))
	{
		MessageHandler::handleMessage("[OpenRave::addKinBody] KinBody has been successfully added. ID: " + id.str(), MessageHandler::INFO,
				COMP->getGlobalState().getOpenRave().getDebugOpenRave());
	} else
	{
		MessageHandler::handleMessage("[OpenRave::addKinBody] KinBody could not be added. ID: " + id.str(), MessageHandler::ERROR);
	}
}

void OpenRave::deleteKinBody(unsigned int objId)
{
	std::stringstream stream;
	stream << objId;

	if (COMP->OpenRaveWrapper.deleteKinbody(stream.str()))
	{
		MessageHandler::handleMessage("[OpenRave::deleteKinBody] KinBody has been successfully removed. ID: " + stream.str(),
				MessageHandler::INFO, COMP->getGlobalState().getOpenRave().getDebugOpenRave());
	} else
	{
		MessageHandler::handleMessage("KinBody could not be removed. ID: " + stream.str() + " [deleteKinBody in OpenRave]",
				MessageHandler::ERROR);
	}
}

bool OpenRave::getPositonOfKinbody(unsigned int objId, double& x, double& y, double& z)
{
	std::stringstream stream;
	stream << objId;

	std::vector<double> pos;
	if (COMP->OpenRaveWrapper.getKinBodyPos(stream.str(), pos) != 0 || pos.size() != 3)
	{
		MessageHandler::handleMessage("KinBody cannot be found. ID: " + stream.str() + " [OpenRave::getPositonOfKinbody]",
				MessageHandler::WARNING);
		return false;
	}

	x = pos[0];
	y = pos[1];
	z = pos[2];

	std::cout << "  Position of obj: " << objId << ", x=" << x << ", y=" << y << ", z=" << z << std::endl;

	return true;
}

void OpenRave::graspKinBody(unsigned int objId)
{
	std::vector<ORUtil::TrajectoryPoint> resultTraj;

	std::stringstream stream;
	stream << objId;

	if (COMP->OpenRaveWrapper.grabObject(stream.str(), resultTraj) >= 0)
	{
		this->obj_grasped = true;

		MessageHandler::handleMessage("Object has been successfully grasped. [OpenRave::graspKinBody]", MessageHandler::INFO,
				COMP->getGlobalState().getOpenRave().getDebugOpenRave());
	} else
	{
		MessageHandler::handleMessage("Object could not be grasped. [OpenRave::graspKinBody]", MessageHandler::WARNING);
	}

}

void OpenRave::releaseKinBody()
{
	std::vector<ORUtil::TrajectoryPoint> resultTraj;

	if (COMP->OpenRaveWrapper.releaseGrabbedObject(resultTraj) >= 0)
	{
		this->obj_grasped = false;

		MessageHandler::handleMessage("Object has been successfully released. [OpenRave::releaseKinBody]", MessageHandler::INFO,
				COMP->getGlobalState().getOpenRave().getDebugOpenRave());
	} else
	{
		MessageHandler::handleMessage("Could not release object. [OpenRave::releaseKinBody]", MessageHandler::WARNING);
	}

}

bool OpenRave::copy4x4MatrixToTransformMatrix(const arma::mat& matrix, TransformMatrix& transform)
{
	if (!(matrix.n_cols == 4 && matrix.n_rows == 4))
	{
		MessageHandler::handleMessage("[OpenRave::copy4x4MatrixToTransformMatrix] No 4x4 Matrix as input.", MessageHandler::ERROR);
		return false;
	}

	transform.resize(4);
	for (int i = 0; i < 4; i++)
	{
		transform[i].resize(4);
		for (int j = 0; j < 4; j++)
		{
			transform[i][j] = matrix.at(i, j);
		}
	}

	return true;
}

bool OpenRave::copy4x4TransformMatrixToMatrix(const TransformMatrix& transform, arma::mat& matrix)
{
	if (!(transform.size() == 4 && transform[0].size() == 4))
	{
		MessageHandler::handleMessage("[OpenRave::copy4x4TransformMatrixToMatrix] No 4x4 Matrix as input.", MessageHandler::ERROR);
		return false;
	}

	matrix.set_size(4, 4);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			matrix.at(i, j) = transform[i][j];
		}
	}

	return true;
}

void OpenRave::deleteAllKinBodies(bool withManipulator, bool onlyManipulator)
{
	COMP->OpenRaveWrapper.deleteAllKinbodies(withManipulator, onlyManipulator);
}
