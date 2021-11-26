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
//  Copyright (C) 2021 Timo Blender
//
//        timo.blender@thu.de
//        christian.schlegel@thu.de
//
//        Servicerobotic Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        D-89075 Ulm
//        Germany
//
//  Contributors:
//		Vineet Nagrath
//		Nayabrasul Shaik
//		Timo Blender
//		Christian Schlegel
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
//
// --------------------------------------------------------------------------

#ifndef _COMPONENTAASBRIDGECORE_HH
#define _COMPONENTAASBRIDGECORE_HH
	
#include "aceSmartSoft.hh"
#include <iostream>

class ComponentAASBridgeCore
{
	private:

	public:
		ComponentAASBridgeCore();

		enum TaskStates {
			PENDING,
			EXECUTING,
			SUCCESS,
			ERROR,
			DELETED
		};

		std::deque<long> TASK_QUEUE;

		std::map<long, std::string> TASK_COMMAND;
		std::map<long, TaskStates> TASK_STATE;

		long CURRENT_TASK_ID;

		std::map<long, std::string> TASK_OUT;

		std::map<long, std::string> TASK_ERROR_VALUE;

		SmartACE::SmartMutex mutex;

};
	
#endif
