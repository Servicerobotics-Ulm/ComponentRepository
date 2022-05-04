// --------------------------------------------------------------------------
//
//  Copyright (C) 2017 Matthias Rollenhagen
//
//      rollenhagen@hs-ulm.de
//		schlegel@hs-ulm.de
//
//      ZAFH Servicerobotic Ulm
//      Christian Schlegel
//      University of Applied Sciences
//      Prittwitzstr. 10
//      89075 Ulm
//      Germany
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
//--------------------------------------------------------------------------
#ifndef _DETECTIONTASK_HH
#define _DETECTIONTASK_HH

#include "DetectionTaskCore.hh"
#include <DomainVision/CommRGBDImage.hh>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Box.hh"
#include "basic/VisualizationHelper.hh"
#include "basic/PointManipulation.hh"
#include <vector>

// every searched box can have three different sides
struct SearchedObject{
		float sides[3];                //object side length in meter. [0] = longest side, [2] = shortest side
		std::string object_type;       // is set by trigger ADDOBJECT, all other data comes from database
		float shelf_level_height[2];   //height from ground in meter [0] = min value , [1] = max value
	};

class DetectionTask  : public DetectionTaskCore
{
private:
	VisualizationHelper vHelper;
	PointManipulation _pointManipulator;

	const static bool _DO_TEST = true;
	const static int thresh = 50;
	const static int N = 10;
	double _length_deviation_factor;
	double _max_box_cosine;
	double _x_detection_distance_min;
	double _x_detection_distance_max;
	double _y_detection_distance_min;
	double _y_detection_distance_max;
	double _z_detection_distance_min;
	double _z_detection_distance_max;
	bool _send_obstacle_mesh;
	bool _use_hsv;

	CommBasicObjects::CommPose3d _base_pose;

	std::vector<Box> _deleted_boxes;

	SearchedObject _searched_obj;
public:
	DetectionTask(SmartACE::SmartComponent *comp);
	virtual ~DetectionTask();
	

	void findRectangles(const cv::Mat& image_rgb, std::vector<Box>& boxes, float ratio_deviation_factor, float max_box_cosine);
	void eraseWrongShelfLevelBoxes(std::vector<Box>& boxes);
	void eraseDuplicates(std::vector<Box>& boxes);
	void eraseEmptyRectangles (std::vector<Box>& boxes);
	void eraseFarRectangles (std::vector<Box>& boxes);
	void eraseWrongLenthRecangles (std::vector<Box>& boxes, float deviation_factor = 0.1);
	double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
	double calcRectRatio(std::vector<cv::Point> points);
	bool isPointInRect(std::vector<cv::Point> rectPoints, cv::Point point);
	void createPointClouds(DomainVision::CommVideoImage* color_image, std::vector<Box>& boxes, pcl::PointCloud<pcl::PointXYZRGB>::Ptr surroundings_cloud);
	void createPointClouds(DomainVision::CommVideoImage* color_image, std::vector<Box>& boxes);
	void calcRectPose(std::vector<Box>& boxes);
	void setDetectedObjects(std::vector<Box>& boxes);
	void setObstacleObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstacles_cloud);
	void sendFinishedEvent();
	void setObjectIds(std::vector<Box>& boxes);
	void errorQuit();
	void eraseNanPoses (std::vector<Box>& boxes);
	void createRackCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	bool setSearchProperties();
	void sortBoxesForVacuumGripper(std::vector<Box>& boxes);
	void rearrangePose(std::vector<Box>& boxes);
	void setEnvId();

	virtual int on_entry();
	virtual int on_execute();
	virtual int on_exit();
};

#endif
