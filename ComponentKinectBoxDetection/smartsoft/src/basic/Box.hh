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

#ifndef BOX_HH_
#define BOX_HH_

//#include <mrpt/poses.h>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "basic/ObjectBeliefs.hh"
#include "PointManipulation.hh"

#include <mrpt/math/include/mrpt/math/TPolygon2D.h>


class Box {
public:
	Box();
	virtual ~Box();

	bool isPointInRect(int x, int y);
	cv::Point getRectCenter();
	void getRectMinMaxValues (int &minX, int &maxX, int &minY, int &maxY);

	//mrpt::poses::CPoint3D some_debug_point;
	cv:: Point some_debug_point_2d;

	float getDepth() const {
		return depth;
	}

	void setDepth(float depth) {
		this->depth = depth;
	}

	int getId() const {
		return id;
	}

	void setId(int id) {
		this->id = id;
	}

	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& getPointCloud() const {
		return point_cloud;
	}

	void setPointCloud(
			const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloud) {
		point_cloud = pointCloud;
	}

	const std::vector<cv::Point>& getPoints() const {
		return points;
	}

	void setPoints(const std::vector<cv::Point>& points) {
		if(points.size() == 4){
			this->points = points;
			sortPoints();

			std::vector<mrpt::math::TPoint2D> mrpt_points;
			mrpt_points.push_back(mrpt::math::TPoint2D(this->points[0].x, this->points[0].y));
			mrpt_points.push_back(mrpt::math::TPoint2D(this->points[1].x, this->points[1].y));
			mrpt_points.push_back(mrpt::math::TPoint2D(this->points[2].x, this->points[2].y));
			mrpt_points.push_back(mrpt::math::TPoint2D(this->points[3].x, this->points[3].y));
			this->mrpt_rect = mrpt::math::TPolygon2D(mrpt_points);
		}else {
			//TODO: Throw exception
		}
	}

	const mrpt::poses::CPose3D getSurfacePose() const {
		mrpt::poses::CPose3D tmp_pose(this->pose);
		return tmp_pose;
	}

	void setSurfacePose(const mrpt::poses::CPose3D pose) {
		this->pose = pose;
		//rearrangePose();
	}

	const mrpt::poses::CPose3D getObjCenterPose() const {
		// move surface pose into x direction
		mrpt::poses::CPose3D tmp_pose(this->pose + mrpt::poses::CPose3D(this->depth / 2, 0, 0, 0, 0, 0));
		return tmp_pose;
	}

	void setModelSideLengths(float side_lengths[3]) {
		this->model_side_lengths.resize(3);
		this->model_side_lengths[0] = side_lengths[0];
		this->model_side_lengths[1] = side_lengths[1];
		this->model_side_lengths[2] = side_lengths[2];
	}

	const std::vector<float>& getModelSideLengths() const {
		return model_side_lengths;
	}

	void setVisibleSides(int side_id1, int side_id2) {
		this->visible_sides.clear();
		this->visible_sides.resize(2);

		//because the shorter sides are at the end of the sides-array
		// => sorting IDs small to big
		if(side_id1 < side_id2){
			this->visible_sides[0] = side_id1;
			this->visible_sides[1] = side_id2;
		}else{
			this->visible_sides[0] = side_id2;
			this->visible_sides[1] = side_id1;
		}
	}

	//return: [0] = longer side; [1] = shorter side;
	std::vector<int> getVisibleSides() const {
		return visible_sides;
	}

	const PointManipulation& getPointManipulator() const {
		return *point_manipulator;
	}

	void setPointManipulator( PointManipulation* pointManipulator) {
		point_manipulator = pointManipulator;
	}

	void setPointCloudIndices(std::vector<uint> pointCloudIndices) {
		this->box_point_indices = pointCloudIndices;
	}

	const std::vector<uint>& getPointCloudIndices() const {
		return box_point_indices;
	}

private:
	int id;
	std::vector<cv::Point> points; // four corners
	float depth; 	// in meter
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud;
	mrpt::poses::CPose3D pose; // with respect to the robot
	std::vector<float> model_side_lengths;
	PointManipulation* point_manipulator;
	std::vector<int> visible_sides; //IDs of the sides (model), that are on top. [0] =  longer side, [1] = shorter side

	// only internally used in isPointInRect() function
	mrpt::math::TPolygon2D mrpt_rect;

	std::vector<uint> box_point_indices;


	void sortPoints();
	void rearrangePose();
};

#endif /* BOX_HH_ */
