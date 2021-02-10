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

#ifndef VISUALIZATIONHELPER_H_
#define VISUALIZATIONHELPER_H_

#include "PointManipulation.hh"

#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/poses/CPose3D.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>



using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;

class VisualizationHelper {

private:
	std::vector<CDisplayWindow*> rgbImageWindows;
	std::vector<CDisplayWindow3D*> cartesianWindows;

	void showRgbImage(CDisplayWindow* rgbImageWindow, const uint8_t* imageData, int width, int height);
	void showRgbImage(CDisplayWindow* rgbImageWindow, cv::Mat rgb);


public:
	VisualizationHelper();
	virtual ~VisualizationHelper();

	void showPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, int window_index);
	void showRgbImage(int window_index, const uint8_t* imageData, int width, int height);
	void showRgbImage(int window_index, cv::Mat rgb_mat);
	void addSmallCoordinateFrame(int window_index, mrpt::poses::CPose3D pose);
	int createRgbWindow(std::string title);
	int createPointCloudWindow(std::string title);
	void showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, int window_index);
	void showPointCloud(mrpt::opengl::CPointCloudColoured::Ptr cloud_mrpt, int window_index);

};

#endif /* VISUALIZATIONHELPERS_H_ */
