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
#include "RecorderTask.hh"
#include "ComponentTagRecorder.hh"

#include <iostream>

RecorderTask::RecorderTask(SmartACE::SmartComponent *comp) 
:	RecorderTaskCore(comp)
{
	std::cout << "constructor RecorderTask\n";
}
RecorderTask::~RecorderTask() 
{
	std::cout << "destructor RecorderTask\n";
}

int RecorderTask::on_entry()
{
	return 0;
}
int RecorderTask::on_execute()
{

	Smart::StatusCode state_status = COMP->stateSlave->acquire("active");
	if(state_status == Smart::SMART_OK)
	{
		CommTrackingObjects::CommDetectedMarkerList marker_list;
		Smart::StatusCode status = COMP->markerListDetectionServiceIn->getUpdateWait(marker_list);
		if(status != Smart::SMART_OK)
		{
			std::cout << "Error while receiving the CommDetectedMarkerList : \n\t\t" << Smart::StatusCodeConversion(status) <<std::endl;
			COMP->stateSlave->release("active");
			return -1;
		}
		logger.write_markerlist(marker_list);
		COMP->stateSlave->release("active");
	}else{
		std::cout << "Error while acquiring active state : \n\t\t" << Smart::StatusCodeConversion(state_status)<<std::endl;
	}
	return 0;
}
int RecorderTask::on_exit()
{
	std::cout << "On Exit RecorderTask\n";
	return 0;
}

std::map<unsigned int, CommBasicObjects::CommPose3d>& RecorderTask::calculate_mean_poses()
{
	return logger.calculate_mean_poses();
}
