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
//---------------------------------------------------------------------

#ifndef _MANIPULATOR_HH
#define _MANIPULATOR_HH

#include <vector>
#include <string>
#include "armadillo.hh"
#include "wrapper/ORUtil.hh"

namespace SpecificManipulator {

/**
 * Abstract class which interacts as an interface for all manipulator specific code.
 *
 * Each manipulator specific class must implement this class with all methods.
 */
class Manipulator {

public:

	/**
	 * Converts real joint angles into OpenRave joint angles.
	 *
	 * realAngles:		vector which holds all the real joint angles
	 * openRaveAngles:	vector which holds the converted joint angles
	 *
	 * The size of the vectors must be the same.
	 */
	virtual bool convertRealAnglesToOpenRaveAngles(const std::vector<double>& realAngles, std::vector<double>& openRaveAngles) = 0;

	/**
	 * Converts OpenRave joint angles into real joint angles.
	 *
	 * openRaveAngles:	vector which holds all the OpenRave joint angles
	 * realAngles:		vector which holds the converted joint angles
	 *
	 * The size of the vectors must be the same.
	 */
	virtual bool convertOpenRaveAnglesToRealAngles(const std::vector<double>& openRaveAngles, std::vector<double>& realAngles) = 0;


	/**
	 * Returns the active DoFs of the complete manipulator (= arm + gripper dofs).
	 * This method will return the DoFs with the gripper.
	 *
	 * dofs: vector which will include the active dofs
	 */
	virtual const vector<int>& getActiveDofs() = 0;

	/**
	 * Returns the active DoFs of the arm.
	 * This method will return the DoFs without the gripper.
	 *
	 * dofs: vector which will include the active dofs
	 */
	virtual const vector<int>& getActiveArmDofs() = 0;

	/**
	 * Returns the active DoFs of the gripper.
	 * This method will return the DoFs with the gripper.
	 *
	 * dofs: vector which will include the active dofs
	 */
	virtual const vector<int>& getActiveGripperDofs() = 0;



	/**
	 * Removes dummy joint angles
	 *
	 * angles: this angles will be modified. After the method has return with true the dummy joint angle is removed.
	 *
	 * return: true if the dummy joint angles could be removed otherwise false.
	 */
	virtual bool removeDummyJointAngles(std::vector<double>& angles) = 0;

	/**
	 * Returns the offset of the real manipulator coordinate system to the OpenRave manipulator coordinate system
	 *
	 * @return armadillo4x4 matrix
	 */
	virtual arma::mat getOffsetRealManipulatortoOpenRaveManipulator() = 0;

	/**
	 * provides the transformation of a manipulator which it should have as initial pose in the simulation mode
	 * @return armadillo4x4 matrix
	 */
	virtual arma::mat getSimulationTransform() = 0;

	/**
	 * provides the standby joint angles of a manipulator which it should have as initial pose in the simulation mode
	 */
	virtual void getSimulationJointAngles(std::vector<double>& angles) = 0;


	/**
	 * setter for the robotURI (= The URI to the manipulator file)
	 */
	virtual void setRobotURI(const std::string& uri)
	{
		this->robotURI = uri;
	}

	/**
	 * getter for the robotURI (= The URI to the manipulator file)
	 */
	virtual std::string getRobotURI() const
	{
		return this->robotURI;
	}

	/**
	 * getter for the manipulatiorName (= the name of the logical manipulator definition within the manipulator file)
	 */
	virtual std::string getManipulatorName() const
	{
		return this->manipulatorName;
	}

	/**
	 * setter for the manipulatiorName (= the name of the logical manipulator definition within the manipulator file)
	 */
	virtual void setManipulatorName(const std::string& name)
	{
		this->manipulatorName = name;
	}

	/**
	 * setter for the openRaveName (= the name of the manipulator-kinbody in the manipulator file)
	 */
	virtual std::string getOpenRaveName() const
	{
		return this->openRaveName;
	}


	/**
	 * setter for the manipulatorType (= the type or name of the real manipulator
	 * (e.g. "Katana" or "UR6855A". This is the same identifier used in the manipulatorFactory to create a specific manipulator))
	 */
	virtual std::string getManipulatorType() const
	{
		return this->manipulatorType;
	}

	/**
	 * parses the trajectory of a specific manipulator. Here the raw trajectory is converted into common trajectory points.
	 * Also specific corrections can be performed.
	 * This implementation converts the raw trajectory to the TrajectoryPoint style and converts the joint angles from openrave into the real manipulator's joint values.
	 * @return the number of trajectory points
	 */
	virtual int parseTrajectory(PyObject* pyTraj, vector<ORUtil::TrajectoryPoint>& traj)
	{
		//NOTE: incoming trajectory has the form: ArmJoint_1 ... ArmJoint_n [Gripperjoint_1 ... GripperJoint_n] Timestamp --> gripper joints optional
		vector<vector<double> > cppTraj;
		int convertResult = CPPDoubleVec2DFromPyFloatList2D(pyTraj, cppTraj);
		if ( convertResult != 0)
		{
			return convertResult;
		}

		for (unsigned int i = 0 ; i < cppTraj.size(); ++i)
		{
			ORUtil::TrajectoryPoint trajPoint;
			trajPoint.jointValues.clear();
			trajPoint.gripperJointValues.clear();
			for (unsigned int j = 0 ; j < this->getActiveArmDofs().size(); ++j)
			{
				trajPoint.jointValues.push_back(cppTraj[i][j]);
			}

			//if the incoming traj without the timestamp is longer than the number of ArmDofs then there are also gripper DOFs in the incoming traj
			if(cppTraj[i].size()-1 > getActiveArmDofs().size())
			{
				//number of gripper joints = total Number of DOFs - DOFs of the arm --> start parsing for gripper DOFs from end of arm DOFs to end of all DOFs.
				for (unsigned int j = this->getActiveArmDofs().size() ; j < getActiveDofs().size(); ++j)
				{
					trajPoint.gripperJointValues.push_back(cppTraj[i][j]);
				}
			}
			convertOpenRaveAnglesToRealAngles(trajPoint.jointValues, trajPoint.jointValues);
			convertOpenRaveAnglesToRealAngles(trajPoint.gripperJointValues, trajPoint.gripperJointValues);
			trajPoint.time = cppTraj[i][cppTraj[i].size()-1]; //the last value of the incoming traj is always the timestamp
			traj.push_back(trajPoint);
		}

		//debugging purpose
		/*cout << "activeArmDofs size = " << getActiveArmDofs().size() << endl;
		cout << "activeDofs size = " << getActiveDofs().size() << endl;
		cout << "--!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! c++ traj !!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
		for(unsigned int i = 0; i < traj.size(); ++i)
		{
			cout << "joints : ";
			for(unsigned int j = 0; j < traj[i].jointValues.size(); ++j)
			{
				cout << traj[i].jointValues[j] << ", ";
			}
			cout << endl;
			cout << "deg/sec : ";
			for(unsigned int j = 0; j < traj[i].jointValues.size(); ++j)
			{
				if(!(i == traj.size()-1))
				{
					cout << 180*((traj[i+1].jointValues[j]-traj[i].jointValues[j])/traj[i].time)/3.14 << ", ";
				}
			}
			cout << endl;
			cout << "gripper Joints : ";
			for(unsigned int j = 0; j < traj[i].gripperJointValues.size(); ++j)
			{
				 cout << traj[i].gripperJointValues[j] << ", ";
			}
			cout << endl;
			cout << "time : " << traj[i].time << endl;
		}
		cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! c++ traj !!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
		*/
		return traj.size();
	}

	virtual int parseTrajectoryTiagoArm(PyObject* pyTraj, vector<ORUtil::TrajectoryPoint>& traj) {
		//NOTE: incoming trajectory has the form: ArmJoint_1 ... ArmJoint_n [Gripperjoint_1 ... GripperJoint_n] Timestamp --> gripper joints optional
		vector<vector<double> > cppTraj;
		int convertResult = CPPDoubleVec2DFromPyFloatList2D(pyTraj, cppTraj);
		if ( convertResult != 0)
		{
			return convertResult;
		}

		for (unsigned int i = 0 ; i < cppTraj.size(); ++i)
		{
			ORUtil::TrajectoryPoint trajPoint;
			trajPoint.jointValues.clear();
			trajPoint.gripperJointValues.clear();
			for (unsigned int j = 0 ; j < 7; ++j)
			{
				trajPoint.jointValues.push_back(cppTraj[i][j]);
			}

			//if the incoming traj without the timestamp is longer than the number of ArmDofs then there are also gripper DOFs in the incoming traj
			/*if(cppTraj[i].size()-1 > getActiveArmDofs().size())
			{
				//number of gripper joints = total Number of DOFs - DOFs of the arm --> start parsing for gripper DOFs from end of arm DOFs to end of all DOFs.
				for (unsigned int j = this->getActiveArmDofs().size() ; j < getActiveDofs().size(); ++j)
				{
					trajPoint.gripperJointValues.push_back(cppTraj[i][j]);
				}
			}*/
			/*convertOpenRaveAnglesToRealAngles(trajPoint.jointValues, trajPoint.jointValues);
			convertOpenRaveAnglesToRealAngles(trajPoint.gripperJointValues, trajPoint.gripperJointValues);*/
			trajPoint.time = cppTraj[i][7]; //the last value of the incoming traj is always the timestamp
			traj.push_back(trajPoint);
		}

		return traj.size();
	}

	/**
	 * parses the iteration parameters. This way manipulation specific iteration parameters can be realized. For example the implicitly preset
	 * azimuth angle of the Katana can be realized.
	 * by default this method makes nothing. Overwrite it to realize specific functionalities
	 */
	virtual void parseIterParameters(ORUtil::PlanningParameter& params, ORUtil& orutil)
	{
	}

	/**
	 * Returns the closed gripper angle of OpenRave.
	 */
	double getClosedGripperAngleOpenRave() {
		return closedAngleOpenRave;
	}

	/**
	 * Returns the open gripper angle of OpenRave.
	 */
	virtual double getOpenGripperAngleOpenRave() {
		return openAngleOpenRave;
	}

	virtual ~Manipulator() {
	}

protected:
	/**
	 * These variables are there to match the OpenRave gripper angles to the real gripper angles.
	 */
	double closedAngleOpenRave;
	double openAngleOpenRave;
	double closedAngleReal;
	double openAngleReal;

	std::string robotURI;				//The URI to the manipulator file
	std::string manipulatorType;		//The type or name of the real manipulator (e.g. "Katana" or "UR6855A")
	std::string openRaveName;			//the name of the manipulator-kinbody in the manipulator file
	std::string manipulatorName;		//the name of the logical manipulator definition within the manipulator file
};

}

#endif /* _MANIPULATOR_HH */
