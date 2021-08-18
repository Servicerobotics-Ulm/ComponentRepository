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
//  Copyright (C) 2020 Nayabrasul Shaik
//
//        nayabrasul.shaik@thu.de
//
//        Christian Schlegel (christian.schlegel@thu.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
//
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
//-------------------------------------------------------------------------
#ifndef _PLANNERGRIDTASK_HH
#define _PLANNERGRIDTASK_HH

#include "PlannerGridTaskCore.hh"
#include "visualization/GridMapVisualization.hh"
#include "CommNavigationObjects/CommGridMap.hh"


// Different values the grid cells used by planner component
// values 8-15 are used to mark path found by the path planner
#define PLANNER_NORTH     0
#define PLANNER_WEST      1
#define PLANNER_SOUTH     2
#define PLANNER_EAST      3
#define PLANNER_FREE      16
#define PLANNER_START     17
#define PLANNER_GOAL      18
#define PLANNER_OBSTACLE  19
#define PLANNER_GROWING   20

class PlannerGridTask  : public PlannerGridTaskCore
{
private:
	GridMapVisualization* gridMap;
	CommNavigationObjects::CommGridMap map;
public:
	PlannerGridTask(SmartACE::SmartComponent *comp);
	virtual ~PlannerGridTask();
	
	virtual int on_entry();
	virtual int on_execute();
	virtual int on_exit();
	int connectServices();
	int disconnectServices();
};

#endif
