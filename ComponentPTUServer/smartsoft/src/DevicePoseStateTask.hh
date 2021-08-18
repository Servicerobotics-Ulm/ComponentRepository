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
//------------------------------------------------------------------------
//
//  Copyright (C) 2010 Manuel Wopfner
//
//        wopfner@hs-ulm.de
//
//        Christian Schlegel (schlegel@hs-ulm.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
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
#ifndef _DEVICEPOSESTATETASK_HH
#define _DEVICEPOSESTATETASK_HH

#include "DevicePoseStateTaskCore.hh"

#include "CommBasicObjects/CommBasePose.hh"
#include "CommBasicObjects/CommBaseVelocity.hh"
#include "CommBasicObjects/CommBaseState.hh"
#include "CommBasicObjects/CommDevicePoseState.hh"

class DevicePoseStateTask  : public DevicePoseStateTaskCore
{
private:
	double pan, tilt;

	arma::mat ptuRobotOffset;
	// calculate transformation matrix for robot offset

	CommBasicObjects::CommBasePose default_base_position;

	CommBasicObjects::CommBaseVelocity zero_velocity;

	CommBasicObjects::CommBaseState base_state;
	CommBasicObjects::CommDevicePoseState device_state;

public:
	DevicePoseStateTask(SmartACE::SmartComponent *comp);
	virtual ~DevicePoseStateTask();
	
	virtual int on_entry();
	virtual int on_execute();
	virtual int on_exit();
};

#endif
