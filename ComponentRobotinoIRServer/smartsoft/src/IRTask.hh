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

// --------------------------------------------------------------------------
//
//  Copyright (C) 2014 Matthias Lutz
//
//        schlegel@hs-ulm.de
//
//        ZAFH Servicerobotik Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        D-89075 Ulm
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
// --------------------------------------------------------------------------
#ifndef _IRTASK_HH
#define _IRTASK_HH

#include "IRTaskCore.hh"

#include "CommBasicObjects/CommPose3d.hh"


#include "rec/robotino/api2/all.h"

using namespace rec::robotino::api2;

class MyCom : public Com
{
public:
	MyCom() : Com( "SmartRobotinoIRserver")
	{
	}

	void errorEvent( const char* errorString )
	{
		std::cerr << "Error: " << errorString << std::endl;
	}

	void connectedEvent()
	{
		std::cout << "Connected." << std::endl;
	}

	void connectionClosedEvent()
	{
		std::cout << "Connection closed." << std::endl;
	}

	void logEvent( const char* message, int level )
	{
		std::cout << message << std::endl;
	}
};

class MyIRRangeFinder : public rec::robotino::api2::DistanceSensorArray
{
	//ulong scan_id;
	std::vector< CommBasicObjects::CommPose3d >sensorPositions;
	double max_range,min_range;
	unsigned int max_sensor_size;
public:
	MyIRRangeFinder()
	{
	}

	void distancesChangedEvent( const float* distances, unsigned int size );

	void initSensorPoses();

};

class MyAnlogInputArray : public rec::robotino::api2::AnalogInputArray
{

	std::vector< CommBasicObjects::CommPose3d >sensorPositions;

	//Function to transform was sensor values (potential) to user friendly distance in m
	float transferIRReadingsToDistances(float potential)
	{
		float x[16] = {0.035, 0.379, 0.485, 0.578, 0.69, 0.74, 0.897, 0.948, 1.133, 1.295, 1.45, 1.64, 1.85, 2.15, 2.54, 3.2};
		float y[16] = {30, 30, 25, 20, 18, 16, 14, 12, 10, 9, 8, 7, 6, 5, 4, 2};
		for (int i = 1; i < 16; ++i)
		{
			if (potential < x[i])
			{
				float dist = (((y[i - 1] - y[i]) / (x[i - 1] - x[i])) * (potential - x[i - 1])) + y[i - 1];
				return dist;
			}
		}

		return 0.0f;
	}

	void valuesChangedEvent	(	const float * 	values,	unsigned int 	size);

public:
	void initSensorPoses();

};

class IRTask  : public IRTaskCore
{
private:
	MyCom com;
	MyIRRangeFinder irFinder;
	MyAnlogInputArray analogInputArray;

	rec::robotino::api2::Relay relay;
public:
	IRTask(SmartACE::SmartComponent *comp);
	virtual ~IRTask();
	
	virtual int on_entry();
	virtual int on_execute();
	virtual int on_exit();


	void enableIRSensors();
	void disableIRSensors();
};

#endif