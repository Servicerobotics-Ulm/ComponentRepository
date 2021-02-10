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

#ifndef _OPENRAVE_HH
#define _OPENRAVE_HH

//#include "smartSoft.hh"

#include <vector>
#include <cmath>

#include "util/MessageHandler.hh"
#include "specificManipulator/ManipulatorFactory.hh"
#include "wrapper/ORUtil.hh"
#include "util/ObjectDatabase.hh"

//#include <CommManipulationPlannerObjects/CommManipulationPlannerEventState.hh>
#include <CommBasicObjects/CommPose3d.hh>
#include <EulerTransformationMatrices.hh>

#define OPENRAVE OpenRave::instance()

/**
 * This class handles all the communication from the Component to OpenRAVE.
 *
 * Currently this class is build upon OpenRAVE Revision 1518
 */
class OpenRave
{

public:
	typedef std::vector<std::vector<double> > TransformMatrix;

public:
	virtual ~OpenRave();

	/**
	 * Struct which holds all parameters that can be specified over the ParamServer.
	 */
	struct OpenRaveParameter
	{
		//encapsulate the enums in structs to enable clean "OpenRaveParameter::IterationMode::<mode>"-syntax and to allow duplicate names
		struct IterationMode{
			enum mode {NONE, SIMPLE_COMPS, CENTERED, ADVANCED};
		};

		struct PathPlanningMode{
			enum mode{NORMAL, CONSTRAINED};
		};

		struct BoundaryParameters
		{
			double fromSide, toSide;
			double fromHeight, toHeight;
			double fromDepth, toDepth;
			double fromAzimuth, toAzimuth;
			double fromElevation, toElevation;
			double fromRoll, toRoll;

			BoundaryParameters()
			{
				fromSide = toSide 			= 0;
				fromHeight = toHeight 		= 0;
				fromDepth = toDepth			= 0;
				fromAzimuth = toAzimuth 	= 0;
				fromElevation = toElevation = 0;
				fromRoll = toRoll 			= 0;
			}
		};

		BoundaryParameters constrainedPathBoundaries;
		BoundaryParameters ikSolutionPoseBoundaries;
		IterationMode::mode iterationMode;
		PathPlanningMode::mode planningMode;

		bool modified;
		bool computeGraspTable;
		int computeGraspTable_id;
		bool simulation_test_ik_only;

		OpenRaveParameter(){
			iterationMode 	= OpenRaveParameter::IterationMode::SIMPLE_COMPS;
			planningMode 	= OpenRaveParameter::PathPlanningMode::NORMAL;
			computeGraspTable = false;
			computeGraspTable_id = 0;
			simulation_test_ik_only = false;
			modified = false;
		}
	};

	/**
	 * Global OpenRave parameters which can be
	 * modified from external
	 */
	OpenRaveParameter globalParameters;

private:
	// Singleton Instance
	static OpenRave *_instance;

	bool initial_sync;

	bool obj_grasped;

	ObjectDatabase objectDatabase;

	// Holds the specific Manipulator
	SpecificManipulator::Manipulator* specificManipulator;


public:
	//TODO
	/**
	 * Local OpenRave parameters which can not
	 * be modified from external
	 */
	OpenRaveParameter localParameters;
	/**
	 * Returns the singleton instance of the OpenRave
	 */
	static OpenRave* instance();

	/**
	 * Initializes all the necessary things from OpenRAVE.
	 *
	 * For example: Planner, Robot, Default Environment
	 */
	void init();

	/**
	 * Resets OpenRAVE.
	 *
	 * All objects will be deleted except the manipulator.
	 * The default environment will be loaded after cleanup
	 */
	void resetAll();

	/**
	 * Synchronizes the real manipulator with the OpenRAVE manipulator.
	 *
	 * For this the timed update of the real manipulator is get and the
	 * current joint angles and the pose (Not the TCP) of the manipulator are read.
	 * The pose is used to fit the manipulator correctly into the environment.
	 * The angles are used to set the manipulator exactly the same way as in the real world.
	 *
	 * This method should be called before a path can be planned. Furthermore it should
	 * always be called whenever this component gets the access of the real manipulator.
	 */
	void syncManipulator();

	/**
	 * Returns the current parameter
	 */
	OpenRaveParameter getParameter() const;

	/**
	 * Set the joint values for the manipulator in OpenRAVE without planning.
	 *
	 * angles: 	the first values are the angles for the manipulator and if the gripper should be
	 * 			synchronized they must be appended after the manipulator angles
	 *
	 * This method should be called when the trajectory is executed on the real manipulator
	 * to ensure synchronization with it if a next path planning has to be performed.
	 */
	void setJoints(const std::vector<double>& angles);

	/**
	 * Opens the gripper in OpenRAVE.
	 */
	void openGripper();

	/**
	 * Closes the gripper in OpenRAVE.
	 */
	void closeGripper();

	/**
	 * Calculates the inverse kinematic to a given pose.
	 * The collision with objects at the given pose and with the robot itself are considered.
	 *
	 * x, y, z: 					coordinates of the pose in meters
	 * azimuth, elevation, roll: 	angles of the pose in rad
	 * solution: 					vector of joint angles which describes the solution of the given pose
	 *
	 * return: true if the calculation succeeds, otherwise false
	 */
	bool calculateIKSolution(double x, double y, double z, double azimuth, double elevation, double roll, std::vector<double>& solution,
			CommBasicObjects::CommPose3d& pose);

	/**
	 * Iterate from 0° (horizontal) to 90° (vertical) with the gripper to get a grasping position
	 *
	 * x, y, z:		coordinates of the pose in meters
	 * solution: 	vector of joint angles which describes the solution of the given pose
	 *
	 * return: true if the calculation succeeds, otherwise false
	 */
	bool iterateToGetGraspingIKSolution(double x, double y, double z, double azimuth, double elevation, double roll, std::vector<double>& solution, CommBasicObjects::CommPose3d& pose);

	/**
	 * Plans the collision free path from the current position of the robot the the given joint angles.
	 *
	 * angles:		vector of joint angles which specify the end position of the tool center point of the manipulator
	 * trajectory:	trajectory filled with the intermediate points of the tool center point from the current position to the given end position.
	 * 				Each element in the trajectory contains a set of joint angles.
	 *
	 * return: true if a path could be found, otherwise false
	 */
	bool planPath(const std::vector<double>& angles, std::vector<ORUtil::TrajectoryPoint>& trajectory);

	bool getPositonOfKinbody(unsigned int objId, double& x, double& y, double& z);

	SpecificManipulator::Manipulator& getSpecificManipulator()
	{
		return *this->specificManipulator;
	}

	/**
	 * Loads an environment from a given ID over the ObjectRecognition. This method will make a query to the ObjectRecognition
	 * and gets over this all the objects in the environment.
	 *
	 * envId:	ID which specifies the environment which should be loaded
	 */
	void loadEnvironment(unsigned int envId);

	/**
	 * Loads an environment with robot, default objects and other objects from a file. The current environment will be deleted.
	 *
	 * envId:	ID which specifies the environment which should be loaded
	 */
	void loadEnvironmentFromFile(unsigned int envId);

	/**
	 * Saves the complete environment with robot and all objects into a file.
	 *
	 * envId:	ID which specifies the environment
	 */
	void saveEnvironmentToFile(unsigned int envId);

	/**
	 * Deletes a KinBody from the OpenRAVE environment.
	 *
	 * objId:	ID of the object which should be deleted
	 */
	void deleteKinBody(unsigned int objId);

	/**
	 * Loads an single object from a given ID over the ObjectRecognition. This method will make a query to the ObjectRecognition
	 * and gets over this the object.
	 *
	 * objId:	ID which specifies the object which should be loaded
	 */

	void loadSingleObjectFromObjRecognition(unsigned int objId);

	/**
	 * Copies a 4x4 arma matrix into a OpenRAVE matrix.
	 *
	 * matrix: the 4x4 arma matrix from which will be copied from
	 * transform: the copied matrix in the OpenRAVE format
	 *
	 * return: true if copy succeed, otherwise false
	 */
	bool copy4x4MatrixToTransformMatrix(const arma::mat& matrix, TransformMatrix& transform);

	/**
	 * Grasps a KinBody in the OpenRAVE environment.
	 * The KinBody will be part of the manipulator and will be considered by planning.
	 *
	 * objId:	ID of the object which should be grasped
	 */
	void graspKinBody(unsigned int objId);

	/**
	 * Releases a KinBody in the OpenRAVE environment.
	 * The KinBody will no longer be part of the manipulator. The KinBody will be placed exactly where it is.
	 */
	void releaseKinBody();

private:
	OpenRave();

	/**
	 * Adds a KinBody to the OpenRAVE environment.
	 *
	 * data:	string which holds the XML-data which specifies the object
	 * matrix:	arma matrix which specifies the pose of the object
	 */
	void addKinBody(unsigned int objId, const std::string& data, const arma::mat& matrix, const arma::mat& matrix2);

	/**
	 * Adds a KinBody to the OpenRAVE environment.
	 *
	 * data:	string which holds the XML-data which specifies the object
	 * matrix:	arma matrix which specifies the pose of the object
	 */
	void addKinBody(unsigned int objId, const std::string& data, const arma::mat& matrix);


	/**
	 * Copies a 4x4 OpenRAVE matrix into a arma matrix.
	 *
	 * transform: the copied matrix in the OpenRAVE format
	 * matrix: the 4x4 arma matrix from which will be copied from
	 *
	 * return: true if copy succeed, otherwise false
	 */
	bool copy4x4TransformMatrixToMatrix(const TransformMatrix& transform, arma::mat& matrix);



	/**
	 * Deletes all KinBodies in the environment.
	 * Except those KindBodies grabbed by the manipulator.
	 *
	 * withManipulator:	if true all KinBodies including the manipulator will be deleted
	 * 					if false the manipulator will not be deleted
	 * onlyManipulator:	if true only the manipulator will be deleted
	 * 					if false, nothing will happen
	 *
	 * For deleting all KinBodies including the manipulator set "withManipulator" true and "onlyManipulator" false.
	 * For deleting only KinBodies set "withManipulator" false and "onlyManipulator" false.
	 * For deleting only the manipulator set "withManipulator" false and "onlyManipulator" true. It is possible to set "withManipulator" true.
	 */
	void deleteAllKinBodies(bool withManipulator = false, bool onlyManipulator = false);

	/**
	 * Convert rad to deg
	 */
	double radToDeg(double rad)
	{
		return rad * (180/M_PI);
	}

};

#endif /* _OPENRAVE_HH */
