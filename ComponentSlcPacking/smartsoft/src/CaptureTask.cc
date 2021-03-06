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
#include "CaptureTask.hh"
#include "ComponentSlcPacking.hh"

#include "utils/VisualizationHelper.hh"

#include <iostream>

CaptureTask::CaptureTask(SmartACE::SmartComponent *comp) 
:	CaptureTaskCore(comp)
{
	std::cout << "constructor CaptureTask\n";

	_cloud_window_idx = -1;
	_rgb_window_idx = -1;

}
CaptureTask::~CaptureTask() 
{
	std::cout << "destructor CaptureTask\n";
}



int CaptureTask::on_entry()
{
	// do initialization procedures here, which are called once, each time the task is started
	// it is possible to return != 0 (e.g. when initialization fails) then the task is not executed further
	return 0;
}
int CaptureTask::on_execute()
{
	

	COMP->start_capturing.acquire();

	CommBasicObjects::CommVoid v;
	DomainVision::CommRGBDImage rgbd_image;

	// Get new image by query
	Smart::StatusCode status = COMP->rgbdQueryClient->query(v, rgbd_image);
	if (status != Smart::SMART_OK) {
		std::cerr << "[CaptureTask] Error in Camera Server communication: " << Smart::StatusCodeConversion(status) << std::endl;
		//errorQuit();
		return 0;
	}

	if (!rgbd_image.getIs_valid()) {
		std::cerr << "[CaptureTask] Error: Data is not valid!" << std::endl;
		//errorQuit();
		return 0;
	}

	DomainVision::CommVideoImage current_video_image;
	DomainVision::CommDepthImage current_depth_image;
	current_video_image = rgbd_image.getColor_image();
	current_depth_image = rgbd_image.getDepth_image();

	PointManipulation pointManipulator = PointManipulation();
	pointManipulator.setMembers(rgbd_image.getSensor_pose(), current_depth_image.getIntrinsic_mCopy(), current_video_image.getIntrinsic_mCopy(), current_depth_image.getExtrinsic_mCopy(), &current_depth_image, &current_video_image);

	std::cout << "[CaptureTask] Creating Point Cloud..." << std::flush;
	COMP->point_cloud = pointManipulator.createColoredPointCloud(&current_video_image, &current_depth_image);
	std::cout << " Done!" << std::endl;

	COMP->environmentIdCounter++;

	showData(COMP->point_cloud.makeShared(), &current_video_image);

	CommObjectRecognitionObjects::CommObjectRecognitionEventState event;
	event.set_object_id_size(0);
	event.set_environment_id(0);
	event.set_state(CommObjectRecognitionObjects::ObjectRecognitionState::FINISHED);

	std::cout << "[CaptureTask] end of capturing ...\n";
	COMP->objectEventServer->put(event);
	// it is possible to return != 0 (e.g. when the task detects errors), then the outer loop breaks and the task stops
	return 0;
}
int CaptureTask::on_exit()
{
	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	return 0;
}

void CaptureTask::showData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, DomainVision::CommVideoImage* video_image) {
	if(_cloud_window_idx < 0){
		_cloud_window_idx = _vHelper.createPointCloudWindow("[PackingTask] cloud");
		_rgb_window_idx = _vHelper.createRgbWindow("[PackingTask] RGB");
	}

	std::cout << "[PackingTask] Rendering Data..." << std::flush;
	cv::Mat rgb_matrix = cv::Mat((int)video_image->getParameter().height, (int)video_image->getParameter().width, CV_8UC3, const_cast< unsigned char*>(video_image->get_data()));

	_vHelper.showPointCloud(cloud, _cloud_window_idx);
	_vHelper.showRgbImage(_rgb_window_idx, rgb_matrix);
	std::cout << " Done!" << std::endl;

}
