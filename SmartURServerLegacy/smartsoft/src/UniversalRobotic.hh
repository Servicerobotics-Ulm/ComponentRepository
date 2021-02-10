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

//------------------------------------------------------------------------
//
//  Copyright (C) 2011-2020 Manuel Wopfner, Matthias Lutz
//
//        lutz@hs-ulm.de
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



#ifndef _UNIVERSAL_ROBOTIC_HH
#define _UNIVERSAL_ROBOTIC_HH

#include "aceSmartSoft.hh"

// communication 0bjects
#include <CommManipulatorObjects/CommManipulatorState.hh>
#include <CommManipulatorObjects/CommManipulatorEventState.hh>
#include <CommManipulatorObjects/CommManipulatorTrajectory.hh>

#include "driver/URManipulator.hh"

#include <vector>
#include <list>

#define UR UniversalRobotic::instance()

/**
 * Singleton class which abstracts the UniversalRobotic and offers
 * an easy interface for performing trajectories, getting the
 * current state and setting different parameters for the UniversalRobotic
 */
class UniversalRobotic
{
public:
//	/**
//	 * Struct which includes all parameters of the UniversalRobotic
//	 * which can be set from external
//	 */
//	struct UniversalRoboticParameters
//	{
//		/**
//		 * Indicates if one of the parameters was modified since
//		 * the last apply
//		 */
//		bool modified;
//
//
//	};

private:

	struct Point3d
	{
		inline Point3d(double px = 0, double py = 0, double pz = 0) :
			x(px), y(py), z(pz)
		{
		}
		double x;
		double y;
		double z;
	};

	struct TrajElement
	{
		std::vector<double> joint_angles;
		double time;
		bool done;
		int id;

		TrajElement()
		{
			time = 0;
			done = false;
			id = 0;
		}
	};

	std::vector<double> target_joint_angles;
	std::vector<double> target_pose;

	bool goal_reached;

//	/**
//	 * Global UniversalRobotic parameters which can be
//	 * modified from external
//	 */
//	UniversalRoboticParameters globalParameters;
//
//	/**
//	 * Local UniversalRobotic parameters which can not
//	 * be modified from external
//	 */
//	UniversalRoboticParameters localParameters;

	/**
	 * Counter for the continuous id of the state
	 */
	uint32_t stateIdCounter;

	/**
	 * Indicates if the UniversalRobotic can be used (true).
	 * If an exception like a collision happened it is set to false.
	 */
	bool activated;

	/**
	 * The last captured point cloud.
	 */
	std::list<Point3d*> pointCloud;

	/**
	 * Counter for the captured point clouds.
	 */
	uint32_t pointCloudIdCounter;


	SmartACE::SmartRecursiveMutex mutex;

	/**
	 * Object representing the manipulator
	 */
	URManipulator* manipulator;

	/**
	 * singleton instance
	 */
	static UniversalRobotic *_instance;

	UniversalRobotic();

public:
	/**
	 * Returns the singleton instance of the UniversalRobotic
	 */
	static UniversalRobotic *instance();

	virtual ~UniversalRobotic();

	/**
	 * Initialize the connection to the UniversalRobotic set the
	 * default parameters and calibrate it.
	 * This method locks the UniversalRobotic until it is done.
	 */
	bool init();

	void calibrate();

	/**
	 * Send Programm to manip
	 * WARNING: FOR DEBUGING ONLY!!!!!
	 */
	void sendProgram(const std::string& program);

	/**
	 * Perform a given trajectory.
	 * This method locks the UniversalRobotic until it is done.
	 *
	 * @param trajectory The trajectory which should be performed
	 */
	void performTrajectory(const CommManipulatorObjects::CommManipulatorTrajectory &trajectory);

	/**
	 * Reads the current state if possible from the UniversalRobotic. If it is not possible
	 * the returned state is invalid.
	 * This method tries to lock the UniversalRobotic for reading the current state.
	 * If this is possible the state is valid, otherwiese it will be set to invalid
	 *
	 * @param state The current state is written to this parameter
	 */
	void getCurrentState(CommManipulatorObjects::CommManipulatorState &state);

//	/**
//	 * Set the given parameter to the UniversalRobotic.
//	 * This method locks the UniversalRobotic until it is done.
//	 *
//	 * @param param Parameter which should be set to the UniversalRobotic
//	 */
//	void setParameter(const CommManipulatorObjects::CommManipulatorParameter &param);

//	/**
//	 * Returns a reference to the global UniversalRobotic parameters.
//	 * @return Const reference to the global UniversalRobotic parameters
//	 */
//	const UniversalRoboticParameters getParameters() const;

	/**
	 * Switch to run so that a trajectory can be performed.
	 */
	void switchToRun();

	/**
	 * Switch to freedrive so that the arm can be moved by hand.
	 */
	void switchToFreedrive();

	void resetGoalAngles();

	/**
	 * Set activated=true and start the program.
	 */
	void start();

	/**
	 * Set activated=false and stop the program
	 */
	void stop();

	void shutdown();


	std::vector<bool> getDigitalInputValues();

	std::vector<float> getAnalogInputValues();

	void setDigitalOutputValue(unsigned int outputNumber, bool outputValue);

	void setAnalogOutputValue(unsigned int outputNumber, double outputValue);

private:

	void adjustTrajectory(std::list<TrajElement>& trajectory);

	void performTrajectoryJointAngles(const CommManipulatorObjects::CommManipulatorTrajectory& trajectory);

	void performTrajectoryPoseTCP(const CommManipulatorObjects::CommManipulatorTrajectory& trajectory);

	void checkState(std::shared_ptr<PrimaryAndSecondaryClientInterfaceParser::ClientInterfaceData> data);

	double getMaxAngle(const std::vector<double>& angles1, const std::vector<double>& angles2) const;

	/**
	 * calculate difference: angles2 - angles1
	 */
	void getAngleDiff(const std::vector<double>& angles1, const std::vector<double>& angles2, std::vector<double>& diff) const;

//	/**
//	 * Copies the global parameters to the local parameters and
//	 * apply them for the UniversalRobotic.
//	 */
//	void applyGlobalParameters();

	/**
	 * Prints the message and set activated=false
	 */
	void handleException(const std::string& message);

	/**
	 * Sends the given event to all listeners and set activated=false
	 */
	void handleException(const std::string& message, CommManipulatorObjects::ManipulatorEvent event);
};

#endif
