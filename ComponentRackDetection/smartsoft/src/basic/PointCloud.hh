//------------------------------------------------------------------------
//
//  Copyright (C) 2011 Manuel Wopfner
//
//        wopfner@hs-ulm.de
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

#ifndef POINTCLOUD_H_
#define POINTCLOUD_H_


#include <vector>
#include <mrpt/poses/CPose3D.h>

#include "pcl_typedefs.hh"


using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;
//using namespace mrpt::utils;
using namespace mrpt::system;

class PointCloud {

private:
	bool normalVectorUpToDate;
	bool boundingBoxUpToDate;

	PclPointCloudPtr cloud;
	SurfaceNormalsPtr normals;

//	std::vector<double> xVal;
//	std::vector<double> yVal;
//	std::vector<double> zVal;

//	std::vector<double> vecX;
//	std::vector<double> vecY;
//	std::vector<double> vecZ;

	TPoint3D minPoint;
	TPoint3D maxPoint;
	CPose3D pose;

	std::vector<double> features;

public:
	PointCloud();

	virtual ~PointCloud();

	/**
	 * Deletes all points.
	 */
	void clear();

	/**
	 * Resize the point cloud.
	 */
	void resize(size_t size);

	/**
	 * Returns the number of points in the point cloud.
	 */
	size_t size() const {
		return this->cloud->size();
	}

	/**
	 * Returns the values of the point specified by index in the x, y, z parameters.
	 */
	void getPoint(size_t index, double& x, double& y, double& z) const;

	/**
	 * Returns the values of the point specified by index as MRPT TPoint3D.
	 */
	void getPoint(size_t index, TPoint3D& point) const;

	/**
	 * Returns the values of the point specified by index as MRPT CPoint3D.
	 */
	void getPoint(size_t index, CPoint3D& point) const;


	/**
	 * Returns the values of the normal vector specified by index in the x, y, z parameters.
	 * The return value is false if the normal vectors are not calculated yet.
	 */
//	bool getNormalVector(size_t index, double& x, double& y, double& z) const;

	/**
	 * Adds a new point to the point cloud.
	 */
	void addPoint(double x, double y, double z);

//	/**
//	 * Adds a new MRPT TPoint3D to the point cloud.
//	 */
//	void addPoint(const TPoint3D& point);

	/**
	 * Adds a new MRPT TPoint3D to the point cloud.
	 */
	void addPoint(const TPoint3D& point);

	/**
	 * Adds a new MRPT CPoint3D to the point cloud.
	 */
	void addPoint(const CPoint3D& point);

	/**
	 * Calculate the normal vectors for all points.
	 * The viewpoint, to which the normal vectors are aligned
	 * is given by x, y, z.
	 * The normal vector is calculated with the number of nearest neighbors specified
	 * by neighbors.
	 */
//	void calculateNormalVectors(double x = 0, double y = 0, double z = 0, int neighbors = 8);

	/**
	 * Copies all points in newCloud which are above the plane and have a bigger distance then minDist.
	 */
	void getPointsAbovePlane(const TPlane& plane, double minDist, PointCloud& newCloud);

	/**
	 * Copies all points in newCloud which distance to the plane is smaller then maxDist.
	 */
	void getPointsInPlane(const TPlane& plane, double maxDist, PointCloud& newCloud);

	/**
	 * Returns the minimal fitting bounding box of the point cloud.
	 */
	bool getBoundingBox(CPose3D& pose, TPoint3D& min, TPoint3D& max) const;

	/**
	 * Returns the minimal fitting bounding box of the point cloud.
	 */
	bool getBoundingBox(CPose3D& pose, CPoint3D& min, CPoint3D& max) const;

	/**
	 * set bounding box
	 */
	void setBoundingBox(CPose3D& pose, CPoint3D& min, CPoint3D& max);

	/**
	 * return only pose of bb
	 */
	CPose3D getBoundingBoxPose() const;

	/**
	 * Calculates the bounding box
	 */
	//void calculateBoundingBox();

	/**
	 * Calculates the minimal fitting bounding box
	 */
	void calculateMinimalVolumeBoundingBox();

	/**
	 * calculate bounding box based on 2d projection
	 */
	void calculateSimplifiedBoundingBox(CPose3D &tablePose);

//	/**
//	 * Calculates the eigenvectors for the point cloud
//	 */
//	void calculateEigenvectors(TPoint3D& xAxis, TPoint3D& yAxis, TPoint3D& zAxis);

//	/**
//	 * Calculates the centroid of the cluster
//	 */
//	void calculateCentroid();

	/**
	 * Changes the origin of the cloud
	 * @param pose
	 */
	//void changeOrigin(const CPose3D& pose);

	/**
	 * Returns the pose of the cluster
	 * @return Pose of the cluster
	 */
	const CPose3D& getPose() const {
		return this->pose;
	}

	/**
	 * Returns the pose of the cluster
	 * @return Pose of the cluster
	 */
	void setPose(CPose3D p) {
		this->pose = p;
	}

	const std::vector<double>& getFeatures() {
		std::sort (this->features.begin(), this->features.end());
		return this->features;
	}

	/**
	 * tells wheter bb up to date but not which bb was calculated (minimal or simple)
	 */
	bool isBoundingBoxUpToDate() const {
		return boundingBoxUpToDate;
	}

	 PclPointCloudPtr getPclPointCloudPtr(){
		return cloud;
	}
	void setPclPointCloud(const PclPointCloudPtr cloud){
		this->cloud = cloud;
		normalVectorUpToDate = false;
		boundingBoxUpToDate = false;
	}

	std::vector< std::vector<double> > getAsStdVector() const;

private:

};

#endif /* POINTCLOUD_H_ */
