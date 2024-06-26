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
//  Copyright (C) 2009 Matthias Lutz
//
//        schlegel@hs-ulm.de
//        lutz@hs-ulm.de
//
//        ZAFH Servicerobotik Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        D-89075 Ulm
//        Germany
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
// --------------------------------------------------------------------------
#ifndef _COMPONENTGMAPPINGCORE_HH
#define _COMPONENTGMAPPINGCORE_HH
	
#include "aceSmartSoft.hh"
#include <iostream>

#include "values.h"
#include <utils/gvalues.h>
#include <gridfastslam/gridslamprocessor.h>
#include <utils/orientedboundingbox.h>
#include <utils/commandline.h>
class ComponentGMappingCore
{
private:

public:
	ComponentGMappingCore();
	double pi_to_pi(double angle){

	   angle+=M_PI;
	   double ret_angle = fmod(angle,2*M_PI);

	   if(angle<0)
	     ret_angle+=2*M_PI;

	   ret_angle-=M_PI;

	   return ret_angle;

	}
	GMapping::Map<double, GMapping::DoubleArray2D, false> * mymap;
	SmartACE::SmartMutex mapLock;
};
	
#endif
