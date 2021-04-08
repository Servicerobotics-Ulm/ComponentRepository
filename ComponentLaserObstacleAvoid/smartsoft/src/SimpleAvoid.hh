//--------------------------------------------------------------------------
//
//  Copyright (C) 2008, 2009 Andreas Steck, Christian Schlegel
//                2021 Thomas Feldmeier
//
//        schlegel@hs-ulm.de
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


/*
 * simple example library for obstacle avoidance based on laser scan.
 */

#ifndef SIMPLE_AVOID_HH
#define SIMPLE_AVOID_HH

#include <CommBasicObjects/CommMobileLaserScan.hh>
#include <CommBasicObjects/CommNavigationVelocity.hh>
#include <math.h>

class SimpleAvoid {

public:
    static void runCycle(CommBasicObjects::CommMobileLaserScan scan,
                    double &outSpeed, double &outTurnrate) {

        const double maxTurnrate = 1.5;       // [radians/s]
        const double minSpeed = 125;          // [mm/s]
        const double maxSpeed = 500;          // [mm/s]
        const double maxSideDistance = 1000;  // [mm]
        const double maxFrontDistance = 4000; // [mm]
        const double robotSafetyRadius = 550; // [mm]

        double minLeftDistance  = maxSideDistance;
        double minRightDistance = maxSideDistance;
        double minFrontDistance = maxFrontDistance;

        uint count = scan.get_scan_size();
        for (uint i = 0; i < count; ++i) {
            // scan.get_scan_distance(i) returns the distance in mm
            double distance = scan.get_scan_distance(i);
            // scan.get_scan_angle returns an angle between 0 and 2*pi, angle 0 means front of lidar, counterclockwise
            double angle = scan.get_scan_angle(i);

            // this ray hits an obstacle at this position relative of lidar:  x mm to front and y mm to left
            double x = distance * cos(angle);
            double y = distance * sin(angle);

            // we have a problem if the robot moves forward and the sideways distance to an obstacle is not big enough
            if (-robotSafetyRadius / 2 < y && y < robotSafetyRadius / 2 && distance < minFrontDistance)
                minFrontDistance = distance;

            // convert from radians to degrees
            angle = angle / M_PI * 180;
            // angle should be between -180 and +180 degrees, 0=front of lidar
            if (angle > 180)
                angle -= 360;

            if (angle < 0.0 && angle > -90.0 && distance < minLeftDistance)
                minLeftDistance = distance;
            if (angle > 0.0 && angle < 90.0 && distance < minRightDistance)
                minRightDistance = distance;
        }

        // the robot should get slower if the lidar sees an obstacle in front within maxFrontDistance
        // the robot completely stops if the obstacle is at robotSafetyRadius
        // we set the translational velocity proportional to the distance difference
        outSpeed = (minFrontDistance - robotSafetyRadius) / 4; // [mm] -> [mm/s]
        if (outSpeed < 0)
            outSpeed = 0;
        if (outSpeed > 500)
            outSpeed = maxSpeed;
        // turn more to the right if the distances to the right are bigger, and vice versa
        // we set the turn rate proportional to the distance difference
        outTurnrate = (minRightDistance - minLeftDistance) * 0.01; // [mm] -> [radians/s]

        if (outSpeed < minSpeed)
            // obstacle is too close, translational speed is too slow
            // to avoid oscillations, rotate always counterclockwise
            outTurnrate = maxTurnrate;

        if (outTurnrate < -maxTurnrate)
            outTurnrate = -maxTurnrate;
        if (outTurnrate > maxTurnrate)
            outTurnrate = maxTurnrate;

        std::cout << " left=" << minLeftDistance
                  << " front=" << minFrontDistance
                  << " right=" << minRightDistance
                  << std::endl;
    }
};

#endif
