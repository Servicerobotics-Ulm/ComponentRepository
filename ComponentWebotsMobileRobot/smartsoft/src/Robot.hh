//--------------------------------------------------------------------------
//
//  Copyright (C) 2003 Christian Schlegel, Andreas Steck
//                2012 Matthias Hörger
//
//        schlegel@hs-ulm.de
//        steck@hs-ulm.de
//        hoerger@hs-ulm.de
//
//        Servicerobotic Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm
//        Germany
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
//--------------------------------------------------------------------------

//----------------------------------------------------------------------------
// This software makes use of Boost. License available at:
// http://www.boost.org/LICENSE_1_0.txt
//----------------------------------------------------------------------------


// header protection
#ifndef ROBOT_HH
#define ROBOT_HH

#include <string>
#include <math.h>
#include <iostream>
#include <sys/time.h>
#include <vector>

// boost-Library for Matrix
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/operation.hpp>

#include "aceSmartSoft.hh"

#include <CommBasicObjects/CommBasePose.hh>
#include <CommBasicObjects/CommBasePositionUpdate.hh>

/**
 * \class Robot
 * Encapsulates the Robot
 */
class Robot//: public Smart::ITimerHandler
{
public:

	/**
	 * @brief Default constructor
	 *
	 */
	Robot();

	/**
	 * @brief Default destructor
	 */
	~Robot( void );

	/**
	 * @brief Returns the CommBasePosition object (Position of the robot)
	 *
	 * @return Position of the Robot
	 */
	CommBasicObjects::CommBasePose getBasePosition();

	/**
	 * @brief Returns the uncorrected CommBasePosition object (based on the Robotino API2 Odometry)
	 *
	 * @return Uncorrected position of the Robot
	 */
	CommBasicObjects::CommBasePose getBaseRawPosition();

	CommBasicObjects::CommBaseVelocity getBaseVelocity();

	/**
	 * @brief Updates the current position of the robot
	 *
	 * @return ??
	 */
	int updatePosition( CommBasicObjects::CommBasePositionUpdate update );

	/**
	 * @brief Resets the current position of the robot (including the raw position!)
	 *
	 * @return ??
	 */
	int resetPosition();

	/**
	 * @brief Returns the voltage of the battery
	 *
	 * @return Battery charge in volt
	 */
	double getBatteryVoltage();


	/**
	 * @brief Returns the current power drain
	 *
	 * @return current power drain in A
	 */
	double getBatteryCurrent();

	/**
	 * @brief Returns if an external power supply is connected
	 *
	 * @return true if connected
	 */
	bool getExternalPower();

//	/**
//	 * @brief Returns the velocity of the robot in x-direction in mm/sec
//	 *
//	 * @return Velocity of the Robot in mm/sec
//	 */
//	double getVx();
//
//	/**
//	 * @brief Returns the velocity of the robot in y-direction in mm/sec
//	 *
//	 * @return Velocity of the Robot in mm/sec
//	 */
//	double getVy();

	// todo: m/s is what we get from Joystick Navigation, change where?
	/**
	 * @brief Sets the velocity of the robot in mm/sec and the rotational velocity rad/sec
	 *
	 * @param vX Velocity in x direction of the robot in mm/sec
	 * @param vY Velocity in y direction of the robot in mm/sec
	 * @param omega Rotational velocity of the robot in rad/sec
	 */
	void setVxVyOmega( double vX, double vY, double omega );

	/**
	 * @brief Returns the rotational velocity of the robot in rad/sec
	 *
	 * @return Rotational velocity of the Robot in radiant/sec
	 */
	double getOmegaRad();

	/**
	 * @brief Returns the rotational velocity of the robot in degree/sec
	 *
	 * @return Rotational velocity of the Robot in degree/sec
	 */
	double getOmegaDeg();

	/**
	 * @brief (not yet implemented) Returns the total distance the robot has travelled since startup in mm
	 *
	 * @return The distance the robot has travelled since startup in mm
	 */
	double getDistance();

	/**
	 * @brief (not yet implemented) Returns the total left rotation the robot has travelled since startup in radiant
	 *
	 * @return The total left rotation the robot has travelled since startup in radiant
	 */
	double getTotalRotationLeft();

	/**
	 * @brief (not yet implemented) Returns the total right rotation the robot has travelled since startup in radiant
	 *
	 * @return The total right rotation the robot has travelled since startup in radiant
	 */
	double getTotalRotationRight();

	void setParameters( double maxVelX, double maxVelY, double maxRotVel );
	void update(double newxpos, double newypos, double newalpha,double vx, double vy, double omega,  unsigned int sequence);

	void processEvents();



	/////////////////////////////// private
private:

	void initVariables();
	
	// thread
	int svc( void );

	//probabilistic
	bool updateCovMatrix( CommBasicObjects::CommBasePose pos0,
			CommBasicObjects::CommBasePose &pos1 );

	double piToPiRad( double a );

	double maxVelX, maxVelY, maxRotVel;

	CommBasicObjects::CommBasePose robotPos, oldPos, rawPos;
	CommBasicObjects::CommBaseVelocity robotVel;
	SmartACE::SmartMutex mutexRobotPos;

	// status
	unsigned char status;

	// old xpos, ypos and th -> necessary for calcutaling deltas
	double oldxpos, oldypos, oldalpha;

	// bumpers
	unsigned short frontbumpers, rearbumpers;

	// total distance the robot is travelled
	double totalDistance;

	// absolut left rotation the robot is travelled
	double totalRotationLeft;

	// absolut right rotation the robot is travelled
	double totalRotationRight;

	// probabilistic
	double lamdaSigmaD;
	double lamdaSigmaDeltaAlpha;
	double lamdaSigmaDeltaBeta;

	//RobotinoOdom robotinoOdom;
	bool _ignoreOdometryEvent;
	
	//RobotinoBumper robotinoBumper;

	// control Robotino's movement
	//rec::robotino::api2::OmniDrive robotinoDrive;

	// Model of Robotinos drive system (wheel diameter, gear, ...)
	//rec::robotino::api2::OmniDriveModel robotinoDriveModel;
	float actualVx, actualVy, actualOmega;

	// used to calculate the velocity of Robotino
	//rec::robotino::api2::Motor robotinoM1, robotinoM2, robotinoM3;

	//rec::robotino::api2::PowerManagement power;

};

#endif // ROBOT_HH