//------------------------------------------------------------------------
//
//  Copyright (C) 2010 Manuel Wopfner
//
//
//        Christian Schlegel (schlegel@hs-ulm.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
//      
//  This file is part of the "SmartRMPBaseServer component".
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

#ifndef ROBOT_H_
#define ROBOT_H_

#include "aceSmartSoft.hh"

#include <CommBasicObjects/CommBasePose.hh>
#include <CommBasicObjects/CommBasePositionUpdate.hh>

#include "rmp/RMPDriver.hh"

// boost-Library for Matrix
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/operation.hpp>

class Robot {

private:
	SmartACE::SmartMutex mutexRMP;
	RMPDriver rmp;

	RMPDriver::OdometryData odoData;
	RMPDriver::RMPStatusData statusData;

	SmartACE::SmartMutex mutexRobotPos;
	CommBasicObjects::CommBasePose robotPos, oldPos, rawPos;

	// probabilistic
	double lamdaSigmaD;
	double lamdaSigmaDeltaAlpha;
	double lamdaSigmaDeltaBeta;

	bool setupDone;

	int driveDirection;
	std::string serialNumber;
	RMPDriver::RMPParameters rmpParams;
	bool enableMotors;

	///----------
	//std::ofstream outputFile;
	double xspeed;

public:
	Robot();
	virtual ~Robot();

	void setup(bool enableMotors);

	void shutdown();

	void update();

	/**
	 * v in m/s
	 */
	void setV(double v);

	double getV();

	/**
	 * angle in rad/sec
	 */
	void setOmega(double angle);

	double getOmega();


	/**
	* @brief Returns the voltage of the battery
	*
	* @return Battery charge in volt
	*/
	double getBatteryVoltage();


	/**
	 * @brief Returns the CommBasePosition object (Position of the robot)
	 *
	 * @return Position of the Robot
	 */
	inline CommBasicObjects::CommBasePose getBasePosition() {
		SmartACE::SmartGuard posGuard(mutexRobotPos);
		return robotPos;
	}

	/**
	 * @brief Returns the uncorrected CommBasePosition object
	 *
	 * @return Uncorrected position of the Robot
	 */
	inline CommBasicObjects::CommBasePose getBaseRawPosition() {
		SmartACE::SmartGuard posGuard(mutexRobotPos);
		return rawPos;
	}

	void resetPosition();

	/**
	 * @brief Updates the current position of the robot
	 */
	void updatePosition(CommBasicObjects::CommBasePositionUpdate update);

	void setParameters(double maxVel, double maxRotVel, double maxAcc, double maxRotAcc);

	/////////////////////////////// private
private:
	void initRMP();

	//probabilistic
	bool updateCovMatrix(CommBasicObjects::CommBasePose pos0, CommBasicObjects::CommBasePose &pos1);

	double piToPiRad(double a);

};

#endif /* ROBOT_H_ */
