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

#ifndef LAYER_HH_
#define LAYER_HH_

//#include <mrpt/poses.h>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "PointManipulation.hh"
#include <mrpt/math/include/mrpt/math/TPlane.h>
#include <mrpt/math/include/mrpt/math/TPose3D.h>



class Layer {
public:
	Layer();
	virtual ~Layer();

	friend bool operator< (Layer a, Layer b) {
		mrpt::math::TPoint3D origin(0, 0, 0);
		double dist_a = a.getPlane().distance(origin);
		double dist_b = b.getPlane().distance(origin);

		return dist_a < dist_b;
	}

	float getThickness() const {
		return thickness;
	}

	void setThickness(float thickness) {
		this->thickness = thickness;
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

	void setPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloud) {
		point_cloud = pointCloud;
	}


	const mrpt::math::TPlane getPlane() const {
		mrpt::math::TPlane tmp_plane(this->plane);
		return tmp_plane;
	}

	void setPlane(const mrpt::math::TPlane plane) {
		this->plane = plane;
	}

	const mrpt::math::TPose3D getPlaneAsPose() const {
		//mrpt::poses::CPose3D pose;
		mrpt::math::TPose3D pose;
		plane.getAsPose3D(pose);
		return pose;
	}

	std::vector< std::vector<bool> > getGrid() {
		return grid;
	}

	void setGrid(std::vector< std::vector<bool> > grid) {
		this->grid = grid;
	}


private:
	int id;
	float thickness; 	// in meter
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud;
	mrpt::math::TPlane plane;
	std::vector< std::vector<bool> > grid;
};

#endif /* LAYER_HH_ */
