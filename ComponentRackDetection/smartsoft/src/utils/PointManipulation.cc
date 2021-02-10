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

#include <pcl/filters/passthrough.h>

#include <mrpt/poses/include/mrpt/poses/CPoint3D.h>

PointManipulation::PointManipulation() {

}

PointManipulation::PointManipulation(CommBasicObjects::CommPose3d sensor_pose,	std::vector<double> depth_intrinsic, std::vector<double> color_intrinsic, DomainVision::CommDepthImage* depth_image) {
	this->sensor_pose = sensor_pose;
	this->depth_intrinsic = depth_intrinsic;
	this->color_intrinsic = color_intrinsic;
	this->_depth_image = depth_image;
}

PointManipulation::~PointManipulation() {
	// TODO Auto-generated destructor stub
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

		float temp_x, temp_y, temp_z;
		temp_x = x;
		temp_y = y;
		temp_z = z;

		//This is now done by the sensor pose rotation
		//x = temp_z;
		//y = temp_x * (-1);
		//z = temp_y * (-1);

		if (transform_to_robot_frame) {
			mrpt::poses::CPoint3D tmp_point(x, y, z);

			tmp_point = transormPointToRobotCoord(tmp_point);

			x = tmp_point.x();
			y = tmp_point.y();
			z = tmp_point.z();
		}

	}
}

void PointManipulation::pixelToXyz (uint32_t& depth_row, uint32_t& depth_col, float &x, float &y, float &z, bool transform_to_robot_frame){
	float depth_val = this->_depth_image->get_distance<float>(depth_col, depth_row);
	pixelToXyz (depth_row, depth_col, depth_val, x, y, z, transform_to_robot_frame);
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


void PointManipulation::createColoredPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_out,	DomainVision::CommVideoImage *color_image, bool dense_cloud) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
	point_cloud_ptr->resize(color_image->get_width() * color_image->get_height());
	point_cloud_ptr->is_dense = dense_cloud;

	float x_in_m, y_in_m, z_in_m;
	const uint8_t* imageData = color_image->get_data();

	for (uint32_t rgb_row = 0; rgb_row < color_image->get_height(); rgb_row++) {
		for (uint32_t rgb_col = 0; rgb_col < color_image->get_width(); rgb_col++) {
			float depth_val = this->_depth_image->get_distance<float>(rgb_col, rgb_row);
			//depth_val = 0.0;
			//getDepthValInM(rgb_row, rgb_col, depth_val);

			pixelToXyz(rgb_row, rgb_col, depth_val, x_in_m, y_in_m, z_in_m, true);

			// check if measured distance is NaN
			// if NaN or inf values are in point cloud it will not be shown
			// To get the mapping form RGB pixel (row / column) to
			// cloud object index (both having size of (1920x1080)) the invalid values are set to max values
			if(dense_cloud && (std::isinf(x_in_m) || std::isinf(z_in_m) || std::isinf(y_in_m) || x_in_m != x_in_m || y_in_m != y_in_m || z_in_m != z_in_m)){
				continue;
			}

			const uint8_t* pixel = (imageData + rgb_row * 3 * color_image->get_width() + rgb_col * 3);
			uint8_t r = pixel[0];
			uint8_t g = pixel[1];
			uint8_t b = pixel[2];

			pcl::PointXYZRGB p(r, g, b);
			p.x = x_in_m;
			p.y = y_in_m;
			p.z = z_in_m;

			point_cloud_ptr->points[rgb_row * color_image->get_width() + rgb_col] =	p;

		}
	}
	point_cloud_out->points.clear();
	point_cloud_out->points = point_cloud_ptr->points;
}


void PointManipulation::createPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_out, bool dense_cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
	point_cloud_ptr->is_dense = dense_cloud;

	if(point_cloud_ptr->is_dense){
		point_cloud_ptr->resize(_depth_image->getWidth() * _depth_image->getHeight());
	}

	float x_in_m, y_in_m, z_in_m;

	for (uint32_t row = 0; row < _depth_image->getHeight(); row++) {
		for (uint32_t col = 0; col < _depth_image->getWidth(); col++) {
			float depth_val = this->_depth_image->get_distance<float>(col, row);
			//depth_val = 0.0;
			//getDepthValInM(row, col, depth_val);

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


pcl::PointCloud<pcl::PointXYZ>::Ptr PointManipulation::removeObjectFromPC(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, std::vector<float> obj_center_xyz, std::vector<float> obj_dimensions_xyz, mrpt::poses::CPose3D obj_center_pose) {

	float min_x, min_y, min_z;
	float max_x, max_y, max_z;

	min_x = obj_center_xyz[0] - obj_dimensions_xyz[0] / 2;
	max_x = obj_center_xyz[0] + obj_dimensions_xyz[0] / 2;
	min_y = obj_center_xyz[1] - obj_dimensions_xyz[1] / 2;
	max_y = obj_center_xyz[1] + obj_dimensions_xyz[1] / 2;
	min_z = obj_center_xyz[2] - obj_dimensions_xyz[2] / 2;
	max_z = obj_center_xyz[2] + obj_dimensions_xyz[2] / 2;

	// Using object pose for min/max point. This calculation is more accurate,
	// but causes problems in sideway picking. Hence, Must be improved for
	// side picking.
//	min_x = (obj_center_pose - mrpt::poses::CPose3D(obj_dimensions_xyz[0] / 2, 0, 0, 0, 0, 0)).x();
//	max_x = (obj_center_pose + mrpt::poses::CPose3D(obj_dimensions_xyz[0] / 2, 0, 0, 0, 0, 0)).x();
//	min_y = (obj_center_pose - mrpt::poses::CPose3D(0, obj_dimensions_xyz[1] / 2, 0, 0, 0, 0)).y();
//	max_y = (obj_center_pose + mrpt::poses::CPose3D(0, obj_dimensions_xyz[1] / 2, 0, 0, 0, 0)).y();
//	min_z = (obj_center_pose - mrpt::poses::CPose3D(0, 0, obj_dimensions_xyz[2] / 2, 0, 0, 0)).z();
//	max_z = (obj_center_pose + mrpt::poses::CPose3D(0, 0, obj_dimensions_xyz[2] / 2, 0, 0, 0)).z();

	//check each point, if it is within range and delete it
	for(int i = 0; i < point_cloud->size(); i ++){
		if(point_cloud->at(i).x > min_x && point_cloud->at(i).x < max_x){
			if(point_cloud->at(i).y > min_y && point_cloud->at(i).y < max_y){
				if(point_cloud->at(i).z > min_z && point_cloud->at(i).z < max_z){
					point_cloud->points.erase(point_cloud->points.begin() + i);
					i--;
				}
			}
		}

	}

	return point_cloud;
}

/*
 * Delete all points out of range. Get rid of NaN and inf values
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr PointManipulation::distanceFilterCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PassThrough<pcl::PointXYZ> pass_env_x;
	pass_env_x.setInputCloud(cloud);
	pass_env_x.setFilterFieldName("x");
	pass_env_x.setFilterLimits(min_x, max_x);
	pass_env_x.filter(*cloud_filtered_x);

	pcl::PassThrough<pcl::PointXYZ> pass_env_y;
	pass_env_y.setInputCloud(cloud_filtered_x);
	pass_env_y.setFilterFieldName("y");
	pass_env_y.setFilterLimits(min_y, max_y);
	pass_env_y.filter(*cloud_filtered_y);

	pcl::PassThrough<pcl::PointXYZ> pass_env_z;
	pass_env_z.setInputCloud(cloud_filtered_y);
	pass_env_z.setFilterFieldName("z");
	pass_env_z.setFilterLimits(min_z, max_z);
	pass_env_z.filter(*cloud_filtered_z);

	return cloud_filtered_z;
}

