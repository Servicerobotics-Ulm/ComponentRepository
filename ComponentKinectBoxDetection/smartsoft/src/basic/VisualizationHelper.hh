//------------------------------------------------------------------------
//
//  Copyright (C) 2011, 2017 Manuel Wopfner, Matthias Rollenhagen
//
//        wopfner@hs-ulm.de
//		  rollenhagen@hs-ulm.de
//
//        Christian Schlegel (schlegel@hs-ulm.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
//
//  This file is part of the "SmartColorToFObjectRecognition".
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

#ifndef VISUALIZATIONHELPER_H_
#define VISUALIZATIONHELPER_H_

#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/poses/CPose3D.h>
#include "Box.hh"
#include <opencv2/opencv.hpp>

#include "DomainVision/CommRGBDImage.hh"
#include "PointManipulation.hh"

#include <mrpt/gui/include/mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/include/mrpt/opengl/COpenGLScene.h>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;

class VisualizationHelper {

private:
	CDisplayWindow* rgbImageWindow;
	CDisplayWindow* depthImageWindow;
	CDisplayWindow3D* cartesianWindow;
	CDisplayWindow3D* cartesianSmallCloudWindow;
	CDisplayWindow3D* cartesianSurroundingCloudWindow;

	PointManipulation _pointManipulator;

public:
	VisualizationHelper();

	void run_window_test(DomainVision::CommRGBDImage* rgbd_image, std::vector<Box> rectangles, pcl::PointCloud<pcl::PointXYZRGB>::Ptr surroundings_cloud);
	void show_rgb(DomainVision::CommVideoImage* comm_rgb_image);
	void drawRectangles(cv::Mat& image, std::vector<Box> boxes);
	void drawEstimatedBoxContours(std::vector<double> color_intrinsic, cv::Mat& image, std::vector<Box>& boxes);
	void show_rgb_image(CDisplayWindow* rgbImageWindow, DomainVision::CommVideoImage* comm_rgb_image);
	void show_box_image(DomainVision::CommVideoImage* comm_rgb_image, std::vector<Box>& boxes);
	void show_rgb_image(cv::Mat* rgb_matrix);
	void show_rgb_contour_image(cv::Mat rgb) ;
	void show_depth_image(mrpt::gui::CDisplayWindow* depthImageWindow,DomainVision::CommDepthImage* depth_image);
	void add_coordinate_systems(mrpt::opengl::COpenGLScene::Ptr &theScene,mrpt::gui::CDisplayWindow3D* cartesianWindowPtr, DomainVision::CommRGBDImage* rgbd_image, std::vector<Box> rectangles);

	virtual ~VisualizationHelper();

	void setPointManipulator(const PointManipulation& pointManipulator) {
		_pointManipulator = pointManipulator;
	}
};

#endif /* VISUALIZATIONHELPERS_H_ */
