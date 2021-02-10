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

#include "VisualizationHelper.hh"

#include <math.h>

#include <opencv2/core/core.hpp>

#include <mrpt/img/include/mrpt/img/CImage.h>
#include <mrpt/img/include/mrpt/img/TColor.h>

using namespace std;

VisualizationHelper::VisualizationHelper(){

}

VisualizationHelper::~VisualizationHelper() {
	delete &rgbImageWindows;
	delete &cartesianWindows;
}

int VisualizationHelper::createRgbWindow(std::string title) {
	int windowIndex = rgbImageWindows.size();
	rgbImageWindows.push_back(new CDisplayWindow(title));

	return windowIndex;
}

int VisualizationHelper::createPointCloudWindow(std::string title) {
	int windowIndex = cartesianWindows.size();
	cartesianWindows.push_back(new CDisplayWindow3D(title, 640, 480));

	cartesianWindows[windowIndex]->setCameraProjective(true);
	cartesianWindows[windowIndex]->setCameraElevationDeg(25.0f);
	cartesianWindows[windowIndex]->setCameraAzimuthDeg(160.0f);
	cartesianWindows[windowIndex]->setCameraPointingToPoint(4, 0, 0);
	cartesianWindows[windowIndex]->setCameraZoom(12.0f);

	return windowIndex;
}

void VisualizationHelper::showPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, int window_index) {

	if(cartesianWindows.size() <= window_index || window_index < 0){
		return;
	}

	mrpt::opengl::COpenGLScene::Ptr scene3D = cartesianWindows[window_index]->get3DSceneAndLock();
	scene3D->clear();

	mrpt::opengl::CPointCloudColoured::Ptr cloud_mrpt = opengl::CPointCloudColoured::Create();
	cloud_mrpt->setName("cloud");
	cloud_mrpt->setPointSize(2.0);
	scene3D->insert(cloud_mrpt);


	mrpt::poses::CPose3D robot_pose(0, 0, 0, 0, 0, 0);
	mrpt::opengl::CSetOfObjects::Ptr robot_coord_frame = opengl::stock_objects::CornerXYZSimple(0.5,2.0);
	robot_coord_frame->setPose(robot_pose);
	scene3D->insert(robot_coord_frame);

	// show the colored point cloud in cartesian coordinate system
	cloud_mrpt->clear();

	for (int i = 0; i < point_cloud->size(); i++) {
		cloud_mrpt->push_back(point_cloud->points[i].x, point_cloud->points[i].y , point_cloud->points[i].z, (float)point_cloud->points[i].r/255.0, (float)point_cloud->points[i].g/255.0, (float)point_cloud->points[i].b/255.0);
	}

	cartesianWindows[window_index]->unlockAccess3DScene();
	cartesianWindows[window_index]->forceRepaint();
}


void VisualizationHelper::showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, int window_index) {

	if(cartesianWindows.size() <= window_index || window_index < 0){
		return;
	}

	mrpt::opengl::CPointCloudColoured::Ptr cloud_mrpt = opengl::CPointCloudColoured::Create();
	cloud_mrpt->setName("cloud");
	cloud_mrpt->setPointSize(2.0);

	cloud_mrpt->clear();

	for (int i = 0; i < point_cloud->size(); i++) {
		cloud_mrpt->push_back(point_cloud->points[i].x, point_cloud->points[i].y , point_cloud->points[i].z, 0, 250, 0);
	}

	this->showPointCloud(cloud_mrpt, window_index);
}

void VisualizationHelper::showPointCloud(mrpt::opengl::CPointCloudColoured::Ptr cloud_mrpt, int window_index) {

	if(cartesianWindows.size() <= window_index || window_index < 0){
		return;
	}

	std::cout << "----> cloud size: " <<  cloud_mrpt->size() << std::endl;

	mrpt::opengl::COpenGLScene::Ptr scene3D = cartesianWindows[window_index]->get3DSceneAndLock();
	scene3D->clear();
	scene3D->insert(cloud_mrpt);

	mrpt::poses::CPose3D robot_pose(0, 0, 0, 0, 0, 0);
	mrpt::opengl::CSetOfObjects::Ptr robot_coord_frame = opengl::stock_objects::CornerXYZSimple(0.5,2.0);
	robot_coord_frame->setPose(robot_pose);
	scene3D->insert(robot_coord_frame);

	cartesianWindows[window_index]->unlockAccess3DScene();
	cartesianWindows[window_index]->forceRepaint();
}


void VisualizationHelper::showRgbImage(int window_index, const uint8_t* imageData, int width, int height) {
	if(rgbImageWindows.size() <= window_index || window_index < 0){
		return;
	}
	showRgbImage(rgbImageWindows[window_index], imageData, width, height);
}

void VisualizationHelper::showRgbImage(int window_index, cv::Mat rgb_mat) {
	showRgbImage(rgbImageWindows[window_index], rgb_mat);
}

void VisualizationHelper::showRgbImage(CDisplayWindow* rgbImageWindow, const uint8_t* imageData, int width, int height) {

	mrpt::img::CImage colorImage(width, height);
	//const uint8_t* imageData = kinect_image->get_rgb_image();

	for (uint32_t i = 0; i < height; i++) {
		for (uint32_t j = 0; j < width; j++) {

			const uint8_t* pixel = (imageData
					+ i * 3 * width + j * 3);

			uint8_t r = pixel[0];
			uint8_t g = pixel[1];
			uint8_t b = pixel[2];

			mrpt::img::TColor color(r, g, b);
			colorImage.setPixel(j, i, color);
		}
	}

	//rgbImageWindow->resize(1000, 600);
	
	// Timo - 20.04
	//---------------
	// new:	
	mrpt::img::CImage plotImage = colorImage.scaleHalf(mrpt::img::IMG_INTERP_NN);
	// old:
	// mrpt::img::CImage plotImage = colorImage.scaleHalf();
	//---------------

	//Save every shown image
	//colorImage.saveToFile("/tmp/rackdetectImage.jpg", 100);

	rgbImageWindow->showImage(plotImage);

}


void VisualizationHelper::showRgbImage(CDisplayWindow* rgbImageWindow, cv::Mat rgb) {

	mrpt::img::CImage colorImage(rgb.cols, rgb.rows);
	//std::cout << "width x height: " << rgb.cols << "x" << rgb.rows << std::endl;

	for (uint32_t i = 0; i < rgb.cols; i++) {
		for (uint32_t j = 0; j < rgb.rows; j++) {
			uint8_t r = rgb.at<cv::Vec3b>(j, i)[0];
			uint8_t g = rgb.at<cv::Vec3b>(j, i)[1];
			uint8_t b = rgb.at<cv::Vec3b>(j, i)[2];

			mrpt::img::TColor color(r, g, b);
			colorImage.setPixel(i, j, color);
		}
	}
	
	// Timo - 20.04
	//---------------
	// new:	
	mrpt::img::CImage plotImage = colorImage.scaleHalf(mrpt::img::IMG_INTERP_NN);
	// old:
	// mrpt::img::CImage plotImage = colorImage.scaleHalf();
	//---------------
	
	rgbImageWindow->showImage(colorImage);
}

void VisualizationHelper::addSmallCoordinateFrame(int window_index, mrpt::poses::CPose3D pose) {
	mrpt::opengl::COpenGLScene::Ptr scene3D = cartesianWindows[window_index]->get3DSceneAndLock();

	mrpt::opengl::CSetOfObjects::Ptr coord_frame = opengl::stock_objects::CornerXYZSimple(0.15, 2.0);
	coord_frame->setPose(pose);
	scene3D->insert(coord_frame);

	cartesianWindows[window_index]->unlockAccess3DScene();
	cartesianWindows[window_index]->forceRepaint();
}



