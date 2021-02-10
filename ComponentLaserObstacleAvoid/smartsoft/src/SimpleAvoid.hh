//------------------------------------------------------------------------
//
//  Copyright (C) 2008, 2009 Andreas Steck, Christian Schlegel
//                2021  Thomas Feldmeier
//
//        schlegel@hs-ulm.de
//
//        Christian Schlegel (schlegel@hs-ulm.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
//
//  This file is part of the "SmartKatana component".
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

/*
 * simple example library for obstacle avoidance based on laser scan.
 */

#ifndef SIMPLE_AVOID_HH
#define SIMPLE_AVOID_HH

#include <CommBasicObjects/CommMobileLaserScan.hh>
#include <CommBasicObjects/CommNavigationVelocity.hh>
#include <math.h>

class SimpleAvoid {

private:
	static double deadlockRotateSpeed;

public:
	static void runCycle(CommBasicObjects::CommMobileLaserScan scan,
			double &outSpeed, double &outTurnrate) {

		// all distances are measured in mm

		const double turnrateLimit = 30.0;
		const double minObstacleDistance = 400;
		const double sideMaxObstacleDistance = 700;
		const double frontMaxObstacleDistance = 20 * minObstacleDistance;
		const double robotSafetyRadius = 550;

		double minLeftLaserrange = sideMaxObstacleDistance;
		double minRightLaserrange = sideMaxObstacleDistance;
        double minFrontLaserrange = frontMaxObstacleDistance;

		uint count = scan.get_scan_size();
		for (uint i = 0; i < count; ++i) {
			// scan.get_scan_distance(i) returns the distance in mm
			double distance = scan.get_scan_distance(i);
			// scan.get_scan_angle is between 0 and 2*pi, 0=front of lidar, counterclockwise
			double angle = scan.get_scan_angle(i);

			// this ray hits an obstacle at this position relative of lidar:  x mm to front and y mm to left
			double x = distance * cos(angle);
			double y = distance * sin(angle);

			// we have a problem if the robot moves forward and the sideways distance to the obstacle is not big enough
			if(-robotSafetyRadius/2 < y && y < robotSafetyRadius/2) {
				if( x < minFrontLaserrange )
					minFrontLaserrange = distance;
			}

			// convert from radians to degrees
			angle = angle / M_PI * 180;
			// angle should be between -180 and +180 degrees, 0=front of lidar
			if( angle > 180 )
				angle -= 360;
			if (angle < 0.0 && angle > -60.0 && distance < minLeftLaserrange)
				minLeftLaserrange = distance;
			if (angle > 0.0 && angle < 60.0 && distance < minRightLaserrange)
				minRightLaserrange = distance;
		}

		// the robot should get slower if the lidar sees an obstacle in front within frontMaxObstacleDistance
		// the robot completely stops if the obstacle is at minObstacleDistance (and travels backwards if even closer)
		double speed = (minFrontLaserrange - minObstacleDistance) / 2;

		// turn more to the right if the distances to the right are bigger, and vice versa
		double turnrate = (minRightLaserrange - minLeftLaserrange) / 5;

		if (speed > minObstacleDistance / 2) {
			deadlockRotateSpeed = 0.0;
		} else {
			// speed is too slow (obstacle). choose a direction to rotate until the path is free again
			if (deadlockRotateSpeed == 0.0) {
				if (turnrate > 0)
					deadlockRotateSpeed = turnrateLimit;
				else
					deadlockRotateSpeed = -turnrateLimit;
			}
			turnrate = deadlockRotateSpeed;
		}
		if (turnrate < -turnrateLimit)
			turnrate = -turnrateLimit;
		else if (turnrate > turnrateLimit)
			turnrate = turnrateLimit;

		std::cout << "runCycle: count=" << count << " speed=" << speed
				<< " turn=" << turnrate << " left=" << minLeftLaserrange
				<< " front=" << minFrontLaserrange << " right=" << minRightLaserrange
				<< " count=" << count << " deadlock= " << deadlockRotateSpeed << std::endl;

		outSpeed = speed;
		outTurnrate = turnrate;
	}
};

double SimpleAvoid::deadlockRotateSpeed = 0.0;

#endif

