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
//  Copyright (C) 2011 Matthias Lutz
//
//        schlegel@hs-ulm.de
//
//        ZAFH Servicerobotik Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        D-89075 Ulm
//        Germany
//
//  This file is part of the "SmartSoft SpeechOutput component".
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


#include "ImageTask.hh"
#include "ComponentUnicapImageClient.hh"

#include <iostream>

ImageTask::ImageTask(SmartACE::SmartComponent *comp) 
:	ImageTaskCore(comp)
{
	std::cout << "constructor ImageTask\n";
}
ImageTask::~ImageTask() 
{
	std::cout << "destructor ImageTask\n";
}



int ImageTask::on_entry()
{
	// do initialization procedures here, which are called once, each time the task is started
	// it is possible to return != 0 (e.g. when initialization fails) then the task is not executed further
	return 0;
}
int ImageTask::on_execute()
{
	// this method is called from an outside loop,
	// hence, NEVER use an infinite loop (like "while(1)") here inside!!!
	// also do not use blocking calls which do not result from smartsoft kernel

		Smart::StatusCode req_status;
		DomainVision::CommVideoImage image;
		CommBasicObjects::CommVoid dummy;
		//static unsigned char *buffer = 0;

		std::cout << "[ImageTask] Getting Image ...\n";

		ParameterStateStruct localState = COMP->getGlobalState();
		if(localState.getSettings().getUseQuery()){
			req_status = COMP->queryClient->query(dummy,image);
			//CHS::QueryId id;
			//req_status = COMP->queryClient->queryRequest(dummy,id);
			//req_status = COMP->queryClient->queryReceiveWait(id,image);
		}

		if(localState.getSettings().getUseNewest() == true){
			req_status = COMP->pushNewestClient->getUpdateWait(image);
		}


		std::cout << "[ImageTask] status: " << Smart::StatusCodeConversion(req_status) << std::endl;

		if(localState.getSettings().getVerbose() == true){
			std::cout << "Is data valid?: " << (bool) image.is_data_valid() << std::endl;
		}

		if(!image.is_data_valid()) {
			std::cout << "IMAGE INVALID" << std::endl;
			return 0;
		}


		if(localState.getSettings().getVerbose() == true){
			std::cout<<"Intrinsic: "<<image.getIntrinsic_mSize() << std::endl;
			std::cout<<"Intrinsic: "<<image.get_intrinsic() <<std::endl;
			std::cout << "Height: " << image.get_height() << std::endl;
			std::cout << "Width: " << image.get_width() << std::endl;
			std::cout << "Format: " << image.get_format() << std::endl;
		}

		SmartACE::SmartGuard imgGuard(COMP->CurrentImageMutex);
		{
			cvReleaseImage(&COMP->currentImage);
			COMP->currentImage = NULL;
			COMP->currentImage = COMP->convertDataArrayToIplImage(image, cvSize(image.get_width(), image.get_height()));

			if(localState.getSettings().getVerbose() == true){
				if (COMP->currentImage == NULL)
					std::cout << "[ImageTask] Current Image NOT set!" << std::endl;
				else {
					std::cout << "[ImageTask] Current Image set!" << std::endl;
			}


				//std::cout << "Sensor Pose: " << image.get_sensor_pose() << std::endl;

				COMP->currentImagePose = CPose3D(
						image.get_sensor_pose().get_x(1),
						image.get_sensor_pose().get_y(1),
						image.get_sensor_pose().get_z(1),
						image.get_sensor_pose().get_azimuth(),
						image.get_sensor_pose().get_elevation(),
						image.get_sensor_pose().get_roll()
					);
			}


			if(localState.getSettings().getSaveImageToTask() == true){
				cvSaveImage("arm-img.jpg",COMP->currentImage);
			}
			/*
			{
			std::string filename;
			std::cout << "Filename or n for no file: ";
			cin >> filename;
			}
			*/

		}
		imgGuard.release();
		//	unsigned int size = image.get_size_as_rgb32();
		//	buffer = new unsigned char[size];
		//	get_as_rgb32(buffer);

		if(localState.getSettings().getSleepInImageTask() == true){
			std::cout << "[ImageTask] sleep ...\n";
			usleep(20000);
		}

	// it is possible to return != 0 (e.g. when the task detects errors), then the outer loop breaks and the task stops
	return 0;
}
int ImageTask::on_exit()
{
	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	return 0;
}
