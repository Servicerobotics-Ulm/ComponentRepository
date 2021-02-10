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

#ifndef POINTMANIPULATION_H_
#define POINTMANIPULATION_H_

#include <mrpt/poses/CPose3D.h>
#include "CommBasicObjects/CommPose3d.hh"
#include "DomainVision/CommDepthImage.hh"
#include "DomainVision/CommVideoImage.hh"

//#include <mrpt/utils.h>
//#include <mrpt/poses.h>
//#include <mrpt/math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointManipulation {
private:
	CommBasicObjects::CommPose3d sensor_pose;
	std::vector<double> depth_intrinsic;
	std::vector<double> color_intrinsic;
	std::vector<double> extrinsic;
	DomainVision::CommDepthImage* _depth_image;
	DomainVision::CommVideoImage* _color_image;
	pcl::PointCloud<pcl::PointXYZRGB> _point_cloud_in_rgb;

public:

	PointManipulation();
	PointManipulation(CommBasicObjects::CommPose3d sensor_pose,	std::vector<double> depth_intrinsic, std::vector<double> color_intrinsic, std::vector<double> extrinsic, DomainVision::CommDepthImage* depth_image, DomainVision::CommVideoImage* color_image);
	virtual ~PointManipulation();


	mrpt::poses::CPoint3D transormPointToRobotCoord(const mrpt::poses::CPoint3D & point);
	void getDepthValInM(uint32_t& depth_row, uint32_t& depth_col, float &depth_val_m);
	void transformPointFromDepthToRgb (float &x, float &y, float &z);
	void project (float &x, float &y, float &z, float &fx, float &fy, float &cx, float &cy, int &rgb_row_out, int &rgb_col_out);
	void pixelToXyz (uint32_t& rgb_row, uint32_t& rgb_col, float &depth_val_meters, float &x, float &y, float &z, bool transform_to_robot_frame = true);
	void pixelToXyz (uint32_t& rgb_row, uint32_t& rgb_col, float &x, float &y, float &z, bool transform_to_robot_frame = true);
	void createColoredPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_out, DomainVision::CommVideoImage *color_image, bool dense_cloud = false);

	pcl::PointCloud<pcl::PointXYZRGB> createColoredPointCloud(DomainVision::CommVideoImage *color_image, bool dense_cloud);
	pcl::PointCloud<pcl::PointXYZRGB> getColoredPointCloud();
	void setMembers(CommBasicObjects::CommPose3d sensor_pose,	std::vector<double> depth_intrinsic, std::vector<double> color_intrinsic, std::vector<double> extrinsic, DomainVision::CommDepthImage* depth_image, DomainVision::CommVideoImage* color_image);

	void createPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_out, bool dense_cloud = false);


	CommBasicObjects::CommPose3d getSensorPose(){
		return this->sensor_pose;
	}

};

#endif /* POINTMANIPULATION_H_ */
