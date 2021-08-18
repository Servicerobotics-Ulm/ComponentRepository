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
#include "DetectorTask.hh"
#include "ComponentAprilTagDetector.hh"

#include <iostream>

DetectorTask::DetectorTask(SmartACE::SmartComponent *comp) 
:	DetectorTaskCore(comp)
{
	std::cout << "constructor DetectorTask\n";
}
DetectorTask::~DetectorTask() 
{
	std::cout << "destructor DetectorTask\n";
}



int DetectorTask::on_entry()
{
	set_camara_params = true;
	//set aprilTag detector options
	ParameterStateStruct params = COMP->getGlobalState();
	marker_detector.create_and_add_tag_family(params.getOptions().getTag_family());
	//	marker_detector.set_detector_sigma(params.getOptions().getSigma());
	marker_detector.set_detector_decimate(params.getOptions().getDecimate());
	marker_detector.set_detector_num_threads(params.getOptions().getNumThreads());
	marker_detector.set_detector_debug(params.getOptions().getDebug());
	marker_detector.set_detector_refine_edges(params.getOptions().getRefine_edges());
	marker_detector.set_marker_size(params.getOptions().getTag_size());
	marker_detector.set_decision_threshold(params.getOptions().getDecision_threshold());
	marker_detector.display_options();
	return 0;
}
int DetectorTask::on_execute()
{

	DomainVision::CommVideoImage input_rgb;
	Smart::StatusCode status;

	status = COMP->stateSlave->acquire("active");
	if(status == Smart::SMART_OK)
	{
		status = COMP->rGBImagePushServiceIn->getUpdateWait(input_rgb);
		if(status != Smart::SMART_OK)
		{
			std::cout << "Error while receiving the rgb image : \n\t\t" << Smart::StatusCodeConversion(status) <<std::endl;
			COMP->stateSlave->release("active");
			return -1;
		}


		if(input_rgb.is_data_valid())
		{
			if(set_camara_params)
			{
				marker_detector.set_intrinsic_parameters(input_rgb);
				set_camara_params = false;
			}
			CommTrackingObjects::CommDetectedMarkerList dml;
			marker_detector.process_image(input_rgb, dml);
			//logger.write_markerlist(dml);
			print_markers_info(dml);

			//push marker list
			COMP->markerListDetectionServiceOut->put(dml);
#ifdef PUBLISH_2D_IMAGE_WITH_MARKER_DRAWN
			marker_detector.draw_markers(input_rgb);
			COMP->rGBImagePushServiceOut->put(input_rgb);
#endif

			//event service
			CommTrackingObjects::CommDetectedMarkerEventState event_state;
			event_state.setMarkers(dml);

			COMP->markerListEventServiceOut->put(event_state);

			//Display number of valid markers found and task frequency
			std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

			std::cout.precision(3);
			std::cout<<std::fixed;
			std::cout <<'\r'<<"Markers found = " <<dml.getMarkersSize() << "  : Detector frequency = " <<1000.0f/std::chrono::duration_cast<std::chrono::milliseconds>(now -last).count()<< " Hz" <<std::flush;
			last = now;

		}else
		{
			std::cout << "Received image data is invalid " <<std::endl;

		}
		COMP->stateSlave->release("active");
	}else
	{
		std::cout << "Error while acquiring active state : \n\t\t" << Smart::StatusCodeConversion(status)<<std::endl;
	}



	return 0;
}
int DetectorTask::on_exit()
{
	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	return 0;
}

void DetectorTask::print_markers_info(const CommTrackingObjects::CommDetectedMarkerList& dml)
{
	if(dml.getMarkersSize() != 0)
	{
		std::cout << "\n#######################  Markers found : "<<dml.getMarkersSize() <<"  ##########################" <<std::endl;
	}
	for(size_t i =0; i< dml.getMarkersSize(); ++i)
	{
		CommTrackingObjects::CommDetectedMarker current_marker = dml.getMarkersElemAtPos(i);
		std::cout << std::setw(40)<<" Id                    : " << current_marker.getId()<<std::endl;
		std::cout << std::setw(40)<< "Marker in Camera Frame: " << current_marker.getPose()<<std::endl;
		std::cout << std::setw(40)<< "Marker in Robot Frame(by Index) : " << dml.get_tag_pose_in_robot_frame_by_index(i)<<std::endl;
		std::cout << std::setw(40)<< "Marker in Robot Frame(by tag id) : " << dml.get_tag_pose_in_robot_frame_by_tag_id(current_marker.getId())<<std::endl;
		std::cout << std::setw(40)<< "Marker in World Frame(by Index) : " << dml.get_tag_pose_in_world_frame_by_index(i)<<std::endl;
		std::cout << std::setw(40)<< "Marker in World Frame(by tag id) : " << dml.get_tag_pose_in_world_frame_by_tag_id(current_marker.getId())<<std::endl;
	}
}
