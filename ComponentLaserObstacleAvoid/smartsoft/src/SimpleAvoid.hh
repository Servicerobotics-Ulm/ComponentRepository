//------------------------------------------------------------------------
//
//  Copyright (C) 2008, 2009 Andreas Steck, Christian Schlegel
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


class SimpleAvoid {

private:
    static double  newSpeed;
    static double  newTurnrate;
    static double  minRightLaserrange;
    static double  minLeftLaserrange;
    static double  minFrontLaserrange;
    static double  l, r, f;
    static bool    deadlockRisk;
    static double  deadlockDirectionOfRotation;


public:

    static void runCycle(CommBasicObjects::CommMobileLaserScan scan, double &outSpeed, double &outTurnrate) {
          CommBasicObjects::CommNavigationVelocity vel;

          uint count = scan.get_scan_size();

          newSpeed            = 0;
          newTurnrate         = 0;
          minRightLaserrange  = 1e9;
          minLeftLaserrange   = 1e9;
          minFrontLaserrange  = 1e9;

          // search for right minimum in laserscan
          for (uint j=(30*2); j < count/2; ++j)
          {
            if (minRightLaserrange > (scan.get_scan_distance(j)) && (scan.get_scan_distance(j)) != 0.00)
              minRightLaserrange = (scan.get_scan_distance(j));
          }

          // search for left minimum in laserscan
          for (uint j=count/2; j < count-(30*2); ++j)
          {
            if (minLeftLaserrange > (scan.get_scan_distance(j)) && (scan.get_scan_distance(j)) != 0.00)
              minLeftLaserrange = (scan.get_scan_distance(j));
          }

          // search for front minimum in laserscan
          for (uint j=(75*2); j < (105*2); ++j)
          {
            if (minFrontLaserrange > (scan.get_scan_distance(j)) )
              minFrontLaserrange = (scan.get_scan_distance(j));
          }

          // delimitate left and right laserrange
          const double laserLimit = 700;
          if( minRightLaserrange > laserLimit) minRightLaserrange = laserLimit;
          if( minLeftLaserrange > laserLimit)  minLeftLaserrange = laserLimit;

          l = (100*minRightLaserrange)/500-100;
          r = (100*minLeftLaserrange)/500-100;
          //f = (1e5*minFrontLaserrange)/500-100;
          const double obstacleDistance = 400;
          f = (100*minFrontLaserrange)/500-(100*obstacleDistance)/500; // frontal minimum distance

          newSpeed = (f+f);

          // give the robot some EPO ;)
          newSpeed *= 1.2;

          newTurnrate = (r-l);

          // rotate robot if speed is too low - risk for deadlock
          // to further limitate the deadlock risk the robot will rotate in the same direction as long as
          // the speed is very low ( <-- an obstacle is in front of the robot)
          if( newSpeed <= fabs(200) )
          {
            if( deadlockRisk == false )
            {
              deadlockRisk = true;
              if( newTurnrate > 0 )
              {
                deadlockDirectionOfRotation = 1;
              }
              else
              {
                deadlockDirectionOfRotation = -1;
              }
            }
            newTurnrate = deadlockDirectionOfRotation * fabs(newTurnrate) * 30.0;
          }
          else
          {
            deadlockRisk = false;
          }

          // delimitate newSpeed
//          const double speedLimit = 400;
//          if( newSpeed >= speedLimit ) newSpeed = speedLimit;

          // delimitate newTurnrate
          const double turnrateLimit = 30.0;
          if( newTurnrate < -turnrateLimit )
          {
            newTurnrate = -turnrateLimit;
          }
          else if( newTurnrate > turnrateLimit )
          {
            newTurnrate = turnrateLimit;
          }

          // write commands to robot
          //vel.set_vX(newSpeed, 0.001);
          //vel.set_omega(newTurnrate * 0.0175);
          outSpeed = newSpeed;
          outTurnrate = newTurnrate;

    }
};

double SimpleAvoid::newSpeed = 0.0;
double SimpleAvoid::newTurnrate = 0.0;
double SimpleAvoid::minRightLaserrange = 1e9;
double SimpleAvoid::minLeftLaserrange = 1e9;
double SimpleAvoid::minFrontLaserrange = 1e9;
double  SimpleAvoid::l = 0.0;
double SimpleAvoid::r = 0.0;
double SimpleAvoid::f = 0.0;
bool SimpleAvoid::deadlockRisk = false;
double SimpleAvoid::deadlockDirectionOfRotation = 0.0;


#endif
