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
//  Copyright (C) 2020 Nayabrasul Shaik
//
//        christian.schlegel@thu.de
//        nayabrasul.shaik@thu.de
//
//        ZAFH Servicerobotic Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm
//        Germany
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 2.1
//  of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this library; if not, write to the Free Software Foundation, Inc.,
//  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//
//  This work is based on previous work by the folks from PlayerStage.
//
//--------------------------------------------------------------------------

//----------------------------------------------------------------------------
//
// CREDITS:
//
// The code for the amcl algorithm was taken from the
// Playerstage Project, which is distributed under GPL, and you can find at
// http://playerstage.sourceforge.net/
//
// Player - One Hell of a Robot Server
// Copyright (C) 2000
//    Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
//
//----------------------------------------------------------------------------

#ifndef SMARTSOFT_SRC_SENSORS_AMCL_VISUAL_TAG_H_
#define SMARTSOFT_SRC_SENSORS_AMCL_VISUAL_TAG_H_

#include "amcl_sensor.h"
#include "CommBasicObjects/CommPose3d.hh"
#include "AmclTypes.hh"
namespace amcl {

class AMCLTagData : public AMCLSensorData
{
public:
	AMCLTagData () {};
	virtual ~AMCLTagData() {};
	CommBasicObjects::CommPose3d robot_pose_from_tags;
	double distance_variance;
	double orientation_variance;
	double distance_weight;
	double orientation_weight;
};

class AMCLVisualTag : public AMCLSensor{
public:
	AMCLVisualTag();
	virtual ~AMCLVisualTag();
    virtual bool UpdateSensor(pf_t *pf, AMCLSensorData *data);
private:
    static double TagSensorModel(AMCLTagData *data, pf_sample_set_t* set);
};

} /* namespace amcl */

#endif /* SMARTSOFT_SRC_SENSORS_AMCL_VISUAL_TAG_H_ */
