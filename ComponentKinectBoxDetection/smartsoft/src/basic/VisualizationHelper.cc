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

#include "VisualizationHelper.hh"

#include <math.h>

#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <mrpt/img/include/mrpt/img/CImage.h>
#include <mrpt/img/include/mrpt/img/TColor.h>

#include <opencv4/opencv2/core/types_c.h>
#include <opencv4/opencv2/core/core_c.h>

using namespace std;

VisualizationHelper::VisualizationHelper(){

	rgbImageWindow = new CDisplayWindow("RGB Image v2");
	depthImageWindow = new CDisplayWindow("Depth Image v2");
	cartesianWindow = new CDisplayWindow3D("Cartesian Points v2", 640, 480);
	cartesianSmallCloudWindow = new CDisplayWindow3D("Cartesian Small Cloud Points v2", 640, 480);
	cartesianSurroundingCloudWindow = new CDisplayWindow3D("Cartesian Surrounding Cloud Points v2", 640, 480);

	// place windows on screen
	rgbImageWindow->setPos(0, 0);
	cartesianWindow->setPos(0, 260);
	cartesianSmallCloudWindow->setPos(0, 300);

	cartesianWindow->setCameraProjective(true);
	cartesianWindow->setCameraElevationDeg(25.0f);
	cartesianWindow->setCameraAzimuthDeg(160.0f);
	cartesianWindow->setCameraPointingToPoint(4, 0, 0);
	cartesianWindow->setCameraZoom(12.0f);

	cartesianSmallCloudWindow->setCameraProjective(true);
	cartesianSmallCloudWindow->setCameraElevationDeg(25.0f);
	cartesianSmallCloudWindow->setCameraAzimuthDeg(160.0f);
	cartesianSmallCloudWindow->setCameraPointingToPoint(4, 0, 0);
	cartesianSmallCloudWindow->setCameraZoom(12.0f);

	cartesianSurroundingCloudWindow->setCameraProjective(true);
	cartesianSurroundingCloudWindow->setCameraElevationDeg(25.0f);
	cartesianSurroundingCloudWindow->setCameraAzimuthDeg(160.0f);
	cartesianSurroundingCloudWindow->setCameraPointingToPoint(4, 0, 0);
	cartesianSurroundingCloudWindow->setCameraZoom(12.0f);

}

VisualizationHelper::~VisualizationHelper() {
	delete rgbImageWindow;
	delete depthImageWindow;
	delete cartesianWindow;
	delete cartesianSmallCloudWindow;
	delete cartesianSurroundingCloudWindow;
}


void VisualizationHelper::run_window_test(DomainVision::CommRGBDImage* rgbd_image, std::vector<Box> rectangles, pcl::PointCloud<pcl::PointXYZRGB>::Ptr surroundings_cloud) {
	// clear windows

	mrpt::opengl::COpenGLScene::Ptr &theScene = cartesianWindow->get3DSceneAndLock();
	theScene->clear();
	mrpt::opengl::COpenGLScene::Ptr &sceneSmallCloud = cartesianSmallCloudWindow->get3DSceneAndLock();
	sceneSmallCloud->clear();
	mrpt::opengl::COpenGLScene::Ptr &sceneSurroundingCloud = cartesianSurroundingCloudWindow->get3DSceneAndLock();
	sceneSurroundingCloud->clear();

	mrpt::opengl::CPointCloudColoured::Ptr cloud = opengl::CPointCloudColoured::Create();
	cloud->setName("cloud");
	cloud->setPointSize(2.0);
	theScene->insert(cloud);

	mrpt::opengl::CPointCloudColoured::Ptr small_cloud = opengl::CPointCloudColoured::Create();
	small_cloud->setName("smallCloud");
	small_cloud->setPointSize(2.0);
	sceneSmallCloud->insert(small_cloud);

	mrpt::opengl::CPointCloudColoured::Ptr sur_cloud = opengl::CPointCloudColoured::Create();
	sur_cloud->setName("surrounding cloud");
	sur_cloud->setPointSize(2.0);
	sceneSurroundingCloud->insert(sur_cloud);

	for(int i = 0; i < surroundings_cloud->size(); i++){
		float x, y, z;

		x = surroundings_cloud->at(i).x;
		y = surroundings_cloud->at(i).y;
		z = surroundings_cloud->at(i).z;
		//Cloud can not be visualized with invalid values (NaN, inf)
		if(isinf(x) || isinf(y) || isinf(z) || x != x || y != y || z != z){
			continue;
		}

		sur_cloud->push_back(x, y , z,(float) surroundings_cloud->at(i).r / 255, (float) surroundings_cloud->at(i).g / 255, (float) surroundings_cloud->at(i).b / 255);
	}



	{
		float x_in_m, y_in_m, z_in_m;
		float r, g, b;
		for(int i = 0; i < rectangles.size(); i++){
			for(int j = 0; j < rectangles[i].getPointCloud()->size(); j++){
				x_in_m = rectangles[i].getPointCloud()->at(j).x;
				y_in_m = rectangles[i].getPointCloud()->at(j).y;
				z_in_m = rectangles[i].getPointCloud()->at(j).z;
				r = rectangles[i].getPointCloud()->at(j).r;
				g = rectangles[i].getPointCloud()->at(j).g;
				b = rectangles[i].getPointCloud()->at(j).b;

				small_cloud->push_back(x_in_m, y_in_m , z_in_m,(float) r / 255, (float) g / 255, (float) b / 255);
			}

		}
	}

	DomainVision::CommVideoImage video_image;
	DomainVision::CommDepthImage depth_image;
	video_image = rgbd_image->getColor_image();
	depth_image = rgbd_image->getDepth_image();

	add_coordinate_systems(sceneSurroundingCloud, cartesianSurroundingCloudWindow, rgbd_image, rectangles);
	add_coordinate_systems(sceneSmallCloud, cartesianSmallCloudWindow, rgbd_image, rectangles);
	add_coordinate_systems(theScene, cartesianWindow, rgbd_image, rectangles);

	// draw the identified planes into rgb image, that they are also shown in the point cloud
	cv::Mat rgb_matrix2 = cv::Mat((int)video_image.get_height(), (int)video_image.get_width(), CV_8UC3, const_cast< unsigned char*>(video_image.get_data()));
	drawRectangles(rgb_matrix2, rectangles);
	video_image.set_data(rgb_matrix2.data);

	cloud->clear();
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	//_pointManipulator.createColoredPointCloud(pcl_cloud_ptr, &video_image, true);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_ptr = _pointManipulator.getColoredPointCloud().makeShared();

	for(int i = 0; i < pcl_cloud_ptr->size(); i++){
		float x, y, z;
		x = pcl_cloud_ptr->at(i).x;
		y = pcl_cloud_ptr->at(i).y;
		z = pcl_cloud_ptr->at(i).z;

		cloud->push_back(x, y , z,(float) pcl_cloud_ptr->at(i).r / 255, (float) pcl_cloud_ptr->at(i).g / 255, (float) pcl_cloud_ptr->at(i).b / 255);
	}


	//drawEstimatedBoxContours(kinect_image, rgb_matrix, rectangles);
	show_rgb_image(rgbImageWindow, &video_image);

	show_depth_image(depthImageWindow, &depth_image);

}


/*
 * Draws all rectangeles an their center points into the image
 */
void VisualizationHelper::drawRectangles(cv::Mat& image, vector<Box> boxes) {

	cv::Mat boxes_img(image.size(), CV_8UC3);
	boxes_img = cv::Scalar::all(0);

	//srand(time(NULL));
	//double red = rand() % 256, green = rand() % 256, blue = rand() % 256;
	double red = 80, green = 200, blue = 80;
	double red_center = 255, green_center = 255, blue_center = 255;

	for (size_t k = 0; k < boxes.size(); k++) {
		cv::fillConvexPoly(boxes_img, boxes.at(k).getPoints(),cv::Scalar(red, green, blue), 1);

		cv::Point rect_center = boxes.at(k).getRectCenter();

		//cv::circle(boxes_img, boxes[k].some_debug_point_2d, 2, cv::Scalar(250, 0, 0), 2);
		//circle(boxes_img, rect_center, 10, Scalar(red_center, green_center, blue_center), 3);
		//putText(Mat& img, const string& text, Point org, int fontFace, double fontScale, Scalar color, int thickness=1, int lineType=8, bool bottomLeftOrigin=false )
		std::stringstream text;
		text << (boxes[k].getId());

		rect_center.x = rect_center.x - 10;
		cv::putText(boxes_img, text.str(), rect_center,cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255,255), 2);
	}

	image = 0.5 * image + 0.5 * boxes_img;
}



/*
 * Calculating the box pose out of 2D and calculate the .
 */
void VisualizationHelper::drawEstimatedBoxContours(vector<double> color_intrinsic, cv::Mat& image, vector<Box>& boxes) {
	cv::RNG rng(12345);

	for (uint i = 0; i < boxes.size(); i++) {
		// 2D image points. If you change the image, you need to change vector
		std::vector<cv::Point2d> image_points;

		// Points already have to be sorted: top left, top right, bottom right,bottom left
		image_points.push_back(cv::Point2d(boxes[i].getPoints()[0].x, boxes[i].getPoints()[0].y));
		image_points.push_back(cv::Point2d(boxes[i].getPoints()[1].x, boxes[i].getPoints()[1].y));
		image_points.push_back(cv::Point2d(boxes[i].getPoints()[2].x, boxes[i].getPoints()[2].y));
		image_points.push_back(cv::Point2d(boxes[i].getPoints()[3].x, boxes[i].getPoints()[3].y));

		// 3D model points. imaginary coordinate frame is fixed at bottom left corner of detected plane
		std::vector<cv::Point3d> model_points;
		model_points.push_back(cv::Point3d(0.0f, boxes[i].getModelSideLengths()[0], 0.0f));								// upper left
		model_points.push_back(cv::Point3d(boxes[i].getModelSideLengths()[2],boxes[i].getModelSideLengths()[0], 0.0f));	// upper right
		model_points.push_back(cv::Point3d(boxes[i].getModelSideLengths()[2], 0.0f, 0.0f));								// lower right
		model_points.push_back(cv::Point3d(0.0f, 0.0f, 0.0f));															// lower left

		// Camera internals
		//double focal_length = kinect_image->getRgb_width(); // Approximate focal length.
		double fx = color_intrinsic[0];
		double fy = color_intrinsic[5];
		double cx = color_intrinsic[2];
		double cy = color_intrinsic[6];

		cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << fx, 0, cx, 0 , fy, cy, 0, 0, 1);
		cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion

		// Output rotation and translation
		cv::Mat rotation_vector; // Rotation in axis-angle form
		cv::Mat translation_vector;

		// Solve for pose
		cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);

		vector<cv::Point3d> corner_points_3d;
		vector<cv::Point2d> corner_points_2d;

		// corners of undetected plane (y = object depth)
		corner_points_3d.push_back(cv::Point3d(model_points[0].x, model_points[0].y, - boxes[i].getModelSideLengths()[1]));
		corner_points_3d.push_back(cv::Point3d(model_points[1].x, model_points[1].y, - boxes[i].getModelSideLengths()[1]));
		corner_points_3d.push_back(cv::Point3d(model_points[2].x, model_points[2].y, - boxes[i].getModelSideLengths()[1]));
		corner_points_3d.push_back(cv::Point3d(model_points[3].x, model_points[3].y, - boxes[i].getModelSideLengths()[1]));

		// project 3D model into 2D image
		cv::projectPoints(corner_points_3d, rotation_vector, translation_vector, camera_matrix, dist_coeffs, corner_points_2d);

		cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

		// Draw lines (box contours) between the calculated points
		cv::line(image,image_points[0], corner_points_2d[0], color, 2);
		cv::line(image,image_points[1], corner_points_2d[1], color, 2);
		cv::line(image,image_points[2], corner_points_2d[2], color, 2);
		cv::line(image,image_points[3], corner_points_2d[3], color, 2);
		cv::line(image,corner_points_2d[0], corner_points_2d[1], color, 2);
		cv::line(image,corner_points_2d[1], corner_points_2d[2], color, 2);
		cv::line(image,corner_points_2d[2], corner_points_2d[3], color, 2);
		cv::line(image,corner_points_2d[0], corner_points_2d[3], color, 2);
	}
}



void VisualizationHelper::show_box_image(DomainVision::CommVideoImage* comm_rgb_image, vector<Box>& boxes) {
	cv::Mat rgb_matrix = cv::Mat((int)comm_rgb_image->get_height(), (int)comm_rgb_image->get_width(), CV_8UC3, const_cast< unsigned char*>(comm_rgb_image->get_data()));
	drawRectangles(rgb_matrix, boxes);
	DomainVision::CommVideoImage tmp_img(comm_rgb_image->get_width(), comm_rgb_image->get_height(), comm_rgb_image->get_format(), rgb_matrix.data);
	show_rgb_image(rgbImageWindow, &tmp_img);
}

void VisualizationHelper::show_rgb_image(cv::Mat* rgb_matrix) {
	DomainVision::CommVideoImage tmp_img(rgb_matrix->cols, rgb_matrix->rows, DomainVision::FormatType::RGB24, rgb_matrix->data);
	show_rgb_image(rgbImageWindow, &tmp_img);
}

void VisualizationHelper::show_rgb(DomainVision::CommVideoImage* comm_rgb_image) {
	show_rgb_image(rgbImageWindow, comm_rgb_image);
}

void VisualizationHelper::show_rgb_image(CDisplayWindow* rgbImageWindow, DomainVision::CommVideoImage* comm_rgb_image) {

	uint32_t height = comm_rgb_image->get_height();
	uint32_t width  = comm_rgb_image->get_width();

	mrpt::img::CImage colorImage(width,height);

	const uint8_t* imageData = comm_rgb_image->get_data();
	assert(imageData != nullptr);

	for (uint32_t i = 0; i < height; i++) {
		for (uint32_t j = 0; j < width; j++) {

			const uint8_t* pixel = (imageData + i * 3 *width + j * 3);

			uint8_t r = pixel[0];
			uint8_t g = pixel[1];
			uint8_t b = pixel[2];

			mrpt::img::TColor color(r, g, b);
			colorImage.setPixel(j, i, color);
		}
	}
	
	// Timo - 20.04
	//---------------
	// new:	
	mrpt::img::CImage plotImage = colorImage.scaleHalf(mrpt::img::IMG_INTERP_NN);
	// old:
	// mrpt::img::CImage plotImage = colorImage.scaleHalf();
	//---------------

	rgbImageWindow->resize(plotImage.getWidth(), plotImage.getHeight());
	rgbImageWindow->showImage(plotImage);
}


void VisualizationHelper::show_rgb_contour_image(cv::Mat rgb) {
	//mrpt::utils::CImage colorImage(rgb.cols, rgb.rows,1);
//	//std::cout << "width x height: " << rgb.cols << "x" << rgb.rows << std::endl;
//
//	for (uint32_t i = 0; i < rgb.cols; i++) {
//		for (uint32_t j = 0; j < rgb.rows; j++) {
//
//			uint8_t r = rgb.at<cv::Vec3b>(j, i)[0];
//			uint8_t g = rgb.at<cv::Vec3b>(j, i)[1];
//			uint8_t b = rgb.at<cv::Vec3b>(j, i)[2];
//
//			mrpt::utils::TColor color(r, g, b);
//
//			colorImage.setPixel(i, j, color);
//		}
//	}
//	//rgbContoursImageWindow->showImage(colorImage);


		IplImage* image2;
		image2 = cvCreateImage(cvSize(rgb.cols,rgb.rows),8,1);
		
		// Timo - 20.04
		//---------------
		// new:	
		IplImage ipltemp = cvIplImage(rgb);
		// old:
		// IplImage ipltemp=rgb;
		//---------------
		
		cvCopy(&ipltemp,image2);
//		IplImage* image2=cvCloneImage(&(IplImage)rgb);

		// Timo - 20.04
		//---------------
		// new:	
		mrpt::img::CImage colorImage(cv::cvarrToMat(image2), mrpt::img::SHALLOW_COPY);
		// old:
		// mrpt::img::CImage colorImage(image2);
		//---------------

		std::cout << "show contour image" << std::endl;

	depthImageWindow->showImage(colorImage);


}



void VisualizationHelper::show_depth_image(CDisplayWindow* depthImageWindow, DomainVision::CommDepthImage* comm_depth_image) {

	DomainVision::DepthFormatType depth_format = comm_depth_image->getFormat();
	uint32_t height	= comm_depth_image->getHeight();
	uint32_t width	= comm_depth_image->getWidth();

	mrpt::img::CImage depthImage(width, height);

	//std::cout << "depth width x height: " << width<< "x" << height << std::endl;

	const uint16_t* depth_data_uint16;
	const float* depth_data_float;

	if(depth_format==DomainVision::DepthFormatType::UINT16)
	{
		depth_data_uint16 = comm_depth_image->get_distances_uint16();
		assert(depth_data_uint16 != nullptr);
	}else if (depth_format==DomainVision::DepthFormatType::FLOAT)
	{
		depth_data_float = comm_depth_image->get_distances_float();
		assert(depth_data_float != nullptr);
	}


	for (uint32_t i = 0; i < height; i++) {
		for (uint32_t j = 0; j < width; j++) {

			long int depth_val;
			if(depth_format==DomainVision::DepthFormatType::UINT16)
			{
				const uint16_t *depth_val_ptr = depth_data_uint16+(width*i+j);
				depth_val = static_cast<long int>(depth_val_ptr[0]);
			}else if(depth_format==DomainVision::DepthFormatType::FLOAT)
			{
				const float *depth_val_ptr = depth_data_float+(width*i+j);
				depth_val = static_cast<long int>(depth_val_ptr[0]*1000);
			}

			uint8_t r = (depth_val) /  255;
			uint8_t g = (depth_val) /  255;
			uint8_t b = (depth_val) /  255;

			mrpt::img::TColor color(r, g, b);
			depthImage.setPixel(j, i, color);
		}
	}
	depthImageWindow->showImage(depthImage);
}



void VisualizationHelper::add_coordinate_systems(mrpt::opengl::COpenGLScene::Ptr &theScene, CDisplayWindow3D* cartesianWindowPtr, DomainVision::CommRGBDImage* rgbd_image, std::vector<Box> rectangles) {

	//////////////////////////////
	// Add robot coordinate frame
	double robot_yaw = rgbd_image->getBase_state().get_base_position().get_base_pose3d().get_azimuth(), robot_pitch = rgbd_image->getBase_state().get_base_position().get_base_pose3d().get_elevation(), robot_roll = rgbd_image->getBase_state().get_base_position().get_base_pose3d().get_roll();
	double robot_x = rgbd_image->getBase_state().get_base_position().get_base_pose3d().getPosition().getX() / 1000, robot_y = rgbd_image->getBase_state().get_base_position().get_base_pose3d().getPosition().getY() / 1000, robot_z = rgbd_image->getBase_state().get_base_position().get_base_pose3d().getPosition().getZ() / 1000;

	mrpt::poses::CPose3D robot_pose(robot_x, robot_y, robot_z,robot_yaw, robot_pitch, robot_roll);
	mrpt::opengl::CSetOfObjects::Ptr robot_coord_frame = opengl::stock_objects::CornerXYZSimple(0.5,2.0);
	robot_coord_frame->setPose(robot_pose);
	theScene->insert(robot_coord_frame);

	//////////////////////////////
	// Add sensor coordinate frame
	double sensor_yaw = rgbd_image->getSensor_pose().get_azimuth(), sensor_pitch = rgbd_image->getSensor_pose().get_elevation(), sensor_roll = rgbd_image->getSensor_pose().get_roll();
	double sensor_x =  rgbd_image->getSensor_pose().getPosition().getX() / 1000, sensor_y = rgbd_image->getSensor_pose().getPosition().getY() / 1000, sensor_z = rgbd_image->getSensor_pose().getPosition().getZ() / 1000;

	mrpt::poses::CPose3D sensor_pose(sensor_x, sensor_y, sensor_z,sensor_yaw, sensor_pitch, sensor_roll);
	mrpt::opengl::CSetOfObjects::Ptr sensor_coord_frame = opengl::stock_objects::CornerXYZSimple(0.25,2.0);
	sensor_coord_frame->setPose(sensor_pose);
	theScene->insert(sensor_coord_frame);

	//////////////////////////////
	// Add box coordinate frames
	for (int i = 0; i < rectangles.size(); i++) {
		mrpt::poses::CPose3D box_pose = rectangles[i].getSurfacePose();
		//mrpt::poses::CPose3D box_pose = rectangles[i].getObjCenterPose();

		mrpt::opengl::CSetOfObjects::Ptr box_coord_frame = opengl::stock_objects::CornerXYZSimple(0.1,2.0);
		box_coord_frame->setPose(box_pose);
		theScene->insert(box_coord_frame);
	}

	cartesianWindowPtr->unlockAccess3DScene();
	cartesianWindowPtr->forceRepaint();

}

