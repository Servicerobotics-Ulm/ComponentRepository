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
//
//  Copyright (C) 2010 Manuel Wopfner
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



#ifndef _LASERTASK_HH
#define _LASERTASK_HH

#include "LaserTaskCore.hh"
#include "CommBasicObjects/CommBasePose.hh"
#include "CommBasicObjects/CommBaseState.hh"
#include "CommBasicObjects/CommBaseVelocity.hh"
#include "CommBasicObjects/CommMobileLaserScan.hh"

#include <armadillo.hh>

#include <webots/Lidar.hpp>
#include <webots/Robot.hpp>


// modify these parameters for unit consistency
#define M_TO_CM 100.0      // conversion factor
#define M_TO_MM 1000.0     // conversion factor
#define S_TO_MS 1000.0     // conversion factor
#define UNIT_FACTOR 100.0  // value is expressed in 0.01 degree units
#define SHORT_LIMIT 65535  // max value
#define MEASURE_UNIT 1.0   // set the internal length unit of the scan in mm

class LaserTask  : public LaserTaskCore
{

private:
    CommBasicObjects::CommTimeStamp lastTimeStep;
    int horizontalResolution;
    unsigned int numberValidPoints;
    webots::Robot *webotsRobot;
    webots::Lidar *webotsLidar;

	ulong scanCounter = 0; // counter for scans

	CommBasicObjects::CommBasePose _default_base_position;

	// values from Smartsoft model, *.componentParameters
	double min_range;
	double max_range;
	double opening_angle;
	double resolution;
	double length_unit;
	double frequency;

	CommBasicObjects::CommMobileLaserScan scan;
//	CommBasicObjects::CommBaseState baseState;
	CommBasicObjects::CommBaseState base_state;
	CommBasicObjects::CommBasePose lastBasePosition;

	CommBasicObjects::CommBaseVelocity zero_velocity;

public:

	LaserTask(SmartACE::SmartComponent *comp);
	virtual ~LaserTask();
	
	virtual int on_entry();
	virtual int on_execute();
	virtual int on_exit();

private:

};

#endif
