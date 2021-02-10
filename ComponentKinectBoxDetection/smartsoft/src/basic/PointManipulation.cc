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

#include "PointManipulation.hh"
#include <math.h>

#include <pcl/visualization/cloud_viewer.h>
 
#include <mrpt/poses/include/mrpt/poses/CPoint3D.h>

PointManipulation::PointManipulation() {

}

PointManipulation::PointManipulation(CommBasicObjects::CommPose3d sensor_pose,	std::vector<double> depth_intrinsic,
		                             std::vector<double> color_intrinsic, std::vector<double> extrinsic,
									 DomainVision::CommDepthImage* depth_image, DomainVision::CommVideoImage* color_image) {
	this->sensor_pose = sensor_pose;
	this->depth_intrinsic = depth_intrinsic;
	this->color_intrinsic = color_intrinsic;
	this->_depth_image = depth_image;
	this->_color_image = color_image;
	this->extrinsic = extrinsic;

	//this->_point_cloud_in_rgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	this->_point_cloud_in_rgb = pcl::PointCloud<pcl::PointXYZRGB>();
}

void PointManipulation::setMembers(CommBasicObjects::CommPose3d sensor_pose,	std::vector<double> depth_intrinsic,
		                           std::vector<double> color_intrinsic, std::vector<double> extrinsic,
								   DomainVision::CommDepthImage* depth_image, DomainVision::CommVideoImage* color_image) {
	this->sensor_pose = sensor_pose;
	this->depth_intrinsic = depth_intrinsic;
	this->color_intrinsic = color_intrinsic;
	this->_depth_image = depth_image;
	this->_color_image = color_image;
	this->extrinsic = extrinsic;

	//this->_point_cloud_in_rgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	this->_point_cloud_in_rgb = pcl::PointCloud<pcl::PointXYZRGB>();
}

PointManipulation::~PointManipulation() {

}


/*
 * Transforms the inserted point from sensor coordinate system
 * to robot coordinate system.
 */
mrpt::poses::CPoint3D PointManipulation::transormPointToRobotCoord(const mrpt::poses::CPoint3D & point) {
	double sensor_yaw = this->sensor_pose.get_azimuth(), sensor_pitch = this->sensor_pose.get_elevation(), sensor_roll = this->sensor_pose.get_roll();
	double sensor_x = this->sensor_pose.getPosition().getX() / 1000, sensor_y = this->sensor_pose.getPosition().getY() / 1000, sensor_z = this->sensor_pose.getPosition().getZ() / 1000;

	mrpt::poses::CPose3D sensorPose(sensor_x, sensor_y, sensor_z,sensor_yaw, sensor_pitch, sensor_roll);
	mrpt::poses::CPoint3D result =  sensorPose + point;

	return result;
}


/*
 * Transfers 2D point (row / column) into 3D.
 */
void PointManipulation::pixelToXyz (uint32_t& depth_row, uint32_t& depth_col, float &depth_val_meters, float &x, float &y, float &z, bool transform_to_robot_frame){

	const float bad_point = std::numeric_limits<float>::quiet_NaN();
	if (std::isnan(depth_val_meters) || depth_val_meters <= 0.001) {
		//depth value is not valid
		x = y = z = bad_point;
	} else {
		const float cx(this->depth_intrinsic[2]), cy(this->depth_intrinsic[6]);
		const float inv_fx(1/this->depth_intrinsic[0]), inv_fy(1/this->depth_intrinsic[5]);

		x = (depth_col + 0.5 - cx) * inv_fx * depth_val_meters;
		y = (depth_row + 0.5 - cy) * inv_fy * depth_val_meters;
		z = depth_val_meters;

		if (transform_to_robot_frame) {
			mrpt::poses::CPoint3D tmp_point(x, y, z);

			tmp_point = transormPointToRobotCoord(tmp_point);

			x = tmp_point.x();
			y = tmp_point.y();
			z = tmp_point.z();
		}
	}
}


void PointManipulation::pixelToXyz (uint32_t& rgb_row, uint32_t& rgb_col, float &x, float &y, float &z, bool transform_to_robot_frame){

	//_point_cloud_in_rgb->points[rgb_row * color_image->get_width() + rgb_col];
	x = _point_cloud_in_rgb.points[rgb_row * _color_image->get_width() + rgb_col].x;
	y = _point_cloud_in_rgb.points[rgb_row * _color_image->get_width() + rgb_col].y;
	z = _point_cloud_in_rgb.points[rgb_row * _color_image->get_width() + rgb_col].z;

	//transformPointFromDepthToRgb(x,y,z);

}


//
//void PointManipulation::pixelToXyz (uint32_t& depth_row, uint32_t& depth_col, float &x, float &y, float &z, bool transform_to_robot_frame){
//	float depth_val;
//	getDepthValInM(depth_col, depth_row, depth_val);
//	pixelToXyz (depth_row, depth_col, depth_val, x, y, z, transform_to_robot_frame);
//}


/*
 * transform x,y,z from depth coordinate frame to rgb coordinate frame
 */
void PointManipulation::transformPointFromDepthToRgb (float &x, float &y, float &z){
	float rgb_x, rgb_y, rgb_z;

	rgb_x = extrinsic[0] * x + extrinsic[4] * y + extrinsic[8] * z + extrinsic[3];
    rgb_y = extrinsic[1] * x + extrinsic[5] * y + extrinsic[9] * z + extrinsic[7];
	rgb_z = extrinsic[2] * x + extrinsic[6] * y + extrinsic[10] * z + extrinsic[11];

	x = rgb_x;
	y = rgb_y;
	z = rgb_z;
}

void PointManipulation::project (float &x, float &y, float &z, float &fx, float &fy, float &cx, float &cy, int &rgb_row_out, int &rgb_col_out){
	float x_projected = x/z;
	float y_projected = y/z;

	rgb_col_out =  x_projected * fx + cx;
	rgb_row_out =  y_projected * fy + cy;

}


void PointManipulation::getDepthValInM(uint32_t& depth_row, uint32_t& depth_col, float &depth_val_m){
	DomainVision::DepthFormatType depth_format = this->_depth_image->getFormat();
	int depth_width = _depth_image->getWidth();


	depth_val_m = 0.0;
	if (depth_format == DomainVision::DepthFormatType::UINT16) {
		const uint16_t depth_val_ptr = _depth_image->get_distances_uint16()[depth_row * depth_width + depth_col];
		depth_val_m = depth_val_ptr / 1000.0f;
	} else if (depth_format	== DomainVision::DepthFormatType::FLOAT) {
		const float depth_val_ptr = _depth_image->get_distances_float()[depth_row * depth_width + depth_col];
		depth_val_m = depth_val_ptr;

	} else {
		std::cout << "Unknow Depth Format" << std::endl;
		std::abort();
	}


}



pcl::PointCloud<pcl::PointXYZRGB> PointManipulation::createColoredPointCloud(DomainVision::CommVideoImage *color_image, bool dense_cloud) {
	_point_cloud_in_rgb.clear();
	_point_cloud_in_rgb.resize(color_image->get_width() * color_image->get_height());
	_point_cloud_in_rgb.is_dense = dense_cloud;


	float cx(this->color_intrinsic[2]), cy(this->color_intrinsic[6]);
	float fx(this->color_intrinsic[0]), fy(this->color_intrinsic[5]);

	float x_in_m, y_in_m, z_in_m, depth_val_in_m;
	const uint8_t* imageData = color_image->get_data();

	for (uint32_t depth_row = 0; depth_row < color_image->get_height(); depth_row++) {
		for (uint32_t depth_col = 0; depth_col < color_image->get_width(); depth_col++) {
			//float depth_val = this->_depth_image->get_distance<float>(rgb_col, rgb_row);
			depth_val_in_m = 0.0;
			getDepthValInM(depth_row, depth_col, depth_val_in_m);


			// returns x,y,z in depth frame
			pixelToXyz(depth_row, depth_col, depth_val_in_m, x_in_m, y_in_m, z_in_m, false);


			// check if measured distance is NaN
			// if NaN or inf values are in point cloud it will not be shown
			// To get the mapping form RGB pixel (row / column) to
			// cloud object index (both having size of (1920x1080)) the invalid values are set to max values
			if(dense_cloud && (std::isinf(x_in_m) || std::isinf(z_in_m) || std::isinf(y_in_m) || x_in_m != x_in_m || y_in_m != y_in_m || z_in_m != z_in_m)){
				continue;
			}

			int rgb_c, rgb_r;
			project(x_in_m, y_in_m, z_in_m, fx, fy, cx, cy, rgb_r, rgb_c);
			bool rgb_out_of_bounds = false;
			uint8_t r, b, g;
			// if point not projected to the picture fill some color
			if(rgb_r<0 || rgb_r >= color_image->get_height() || rgb_c < 0 || rgb_c >= color_image->get_width() ){
				r = g = b = 255;
				rgb_out_of_bounds = true;
			}
			else {
				const uint8_t* pixel = (imageData + rgb_r * 3 * color_image->get_width() + rgb_c * 3);

				r = pixel[0];
				g = pixel[1];
				b = pixel[2];
			}



			mrpt::poses::CPoint3D tmp_point(x_in_m, y_in_m, z_in_m);
			tmp_point = transormPointToRobotCoord(tmp_point);
			x_in_m = tmp_point.x();
			y_in_m = tmp_point.y();
			z_in_m = tmp_point.z();


			//const uint8_t* pixel = (imageData + rgb_row * 3 * color_image->get_width() + rgb_col * 3);
//			uint8_t r = pixel[0];
//			uint8_t g = pixel[1];
//			uint8_t b = pixel[2];

			pcl::PointXYZRGB p(r, g, b);
			p.x = x_in_m;
			p.y = y_in_m;
			p.z = z_in_m;

			if(rgb_out_of_bounds){
				//_point_cloud_in_rgb->points[rgb_r * color_image->get_width() + rgb_c] = pcl::PointXYZRGB p(, g, b);;
			}else{
				_point_cloud_in_rgb.points[rgb_r * color_image->get_width() + rgb_c] =	p;
			}
		}
	}

	return _point_cloud_in_rgb;

// DEBUG code: show created point cloud
//  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//  viewer.showCloud(point_cloud_out);
//  while (!viewer.wasStopped ())
//  {
//  }

}

/*
void PointManipulation::createColoredPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_out,	CommVisionObjects::CommVideoImage *color_image, bool dense_cloud) {
	_point_cloud_in_rgb->clear();
	_point_cloud_in_rgb->resize(color_image->get_width() * color_image->get_height());

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	point_cloud_ptr->resize(color_image->get_width() * color_image->get_height());
	point_cloud_ptr->is_dense = dense_cloud;

	float cx(this->color_intrinsic[2]), cy(this->color_intrinsic[6]);
	float fx(this->color_intrinsic[0]), fy(this->color_intrinsic[5]);

	float x_in_m, y_in_m, z_in_m, depth_val_in_m;
	const uint8_t* imageData = color_image->get_data();

	for (uint32_t depth_row = 0; depth_row < color_image->get_height(); depth_row++) {
		for (uint32_t depth_col = 0; depth_col < color_image->get_width(); depth_col++) {
			//float depth_val = this->_depth_image->get_distance<float>(rgb_col, rgb_row);
			depth_val_in_m = 0.0;
			getDepthValInM(depth_row, depth_col, depth_val_in_m);


			// returns x,y,z in depth frame
			pixelToXyz(depth_row, depth_col, depth_val_in_m, x_in_m, y_in_m, z_in_m, false);


			// check if measured distance is NaN
			// if NaN or inf values are in point cloud it will not be shown
			// To get the mapping form RGB pixel (row / column) to
			// cloud object index (both having size of (1920x1080)) the invalid values are set to max values
			if(dense_cloud && (std::isinf(x_in_m) || std::isinf(z_in_m) || std::isinf(y_in_m) || x_in_m != x_in_m || y_in_m != y_in_m || z_in_m != z_in_m)){
				continue;
			}

			int rgb_c, rgb_r;
			project(x_in_m, y_in_m, z_in_m, fx, fy, cx, cy, rgb_r, rgb_c);
			bool rgb_out_of_bounds = false;
			uint8_t r, b, g;
			// if point not projected to the picture fill some color
			if(rgb_r<0 || rgb_r > color_image->get_height() || rgb_c < 0 || rgb_c > color_image->get_width() ){
				r = g = b = 255;
				rgb_out_of_bounds = true;
			}
			else {
				const uint8_t* pixel = (imageData + rgb_r * 3 * color_image->get_width() + rgb_c * 3);

				r = pixel[0];
				g = pixel[1];
				b = pixel[2];
			}



			mrpt::poses::CPoint3D tmp_point(x_in_m, y_in_m, z_in_m);
			tmp_point = transormPointToRobotCoord(tmp_point);
			x_in_m = tmp_point.x();
			y_in_m = tmp_point.y();
			z_in_m = tmp_point.z();


			//const uint8_t* pixel = (imageData + rgb_row * 3 * color_image->get_width() + rgb_col * 3);
//			uint8_t r = pixel[0];
//			uint8_t g = pixel[1];
//			uint8_t b = pixel[2];

			pcl::PointXYZRGB p(r, g, b);
			p.x = x_in_m;
			p.y = y_in_m;
			p.z = z_in_m;

			point_cloud_ptr->points[depth_row * color_image->get_width() + depth_col] =	p;
			if(rgb_out_of_bounds){
				//_point_cloud_in_rgb->points[rgb_r * color_image->get_width() + rgb_c] = pcl::PointXYZRGB p(, g, b);;
			}else{
				_point_cloud_in_rgb->points[rgb_r * color_image->get_width() + rgb_c] =	p;
			}
		}
	}
	point_cloud_out->points.clear();
	point_cloud_out->points = point_cloud_ptr->points;

// DEBUG code: show created point cloud
//  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//  viewer.showCloud(point_cloud_out);
//  while (!viewer.wasStopped ())
//  {
//  }

}


void PointManipulation::createPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_out, bool dense_cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
	point_cloud_ptr->is_dense = dense_cloud;

	if(point_cloud_ptr->is_dense){
		point_cloud_ptr->resize(_depth_image->getWidth() * _depth_image->getHeight());
	}

	float x_in_m, y_in_m, z_in_m, depth_val;

	for (uint32_t row = 0; row < _depth_image->getHeight(); row++) {
		for (uint32_t col = 0; col < _depth_image->getWidth(); col++) {
			//float depth_val = this->_depth_image->get_distance<float>(col, row);
			depth_val = 0.0;
			getDepthValInM(row, col, depth_val);

			pixelToXyz(row, col, depth_val, x_in_m, y_in_m, z_in_m, true);

			if (dense_cloud){
				// check if measured distance is NaN
				// if NaN or inf values are in point cloud it will not be shown
				// To get the mapping form RGB pixel (row / column) to
				// cloud object index (both having size of (1920x1080)) the invalid values are set to max values
				if(std::isinf(x_in_m) || std::isinf(z_in_m) || std::isinf(y_in_m) || x_in_m != x_in_m || y_in_m != y_in_m || z_in_m != z_in_m){
					continue;
				}
				point_cloud_ptr->points.push_back(pcl::PointXYZ(x_in_m, y_in_m, z_in_m));
			}else{
				pcl::PointXYZ p(x_in_m, y_in_m, z_in_m);
				point_cloud_ptr->points[row * _depth_image->getWidth() + col] =	p;
			}

		}
	}
	point_cloud_out->points.clear();
	point_cloud_out->points = point_cloud_ptr->points;
}
*/
pcl::PointCloud<pcl::PointXYZRGB> PointManipulation::getColoredPointCloud() {
	return _point_cloud_in_rgb;
}
