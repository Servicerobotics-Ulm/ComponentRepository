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

#include "PointCloud.hh"
#include "MVBB/Mvbb.h"
#include "MVBB/BoundingBox.h"
#include <iostream>
#include <time.h>
#include <colors.hh>
//#include <mrpt/utils.h>

#include <mrpt/poses/include/mrpt/poses/CPoint3D.h>
#include <mrpt/math/include/mrpt/math/TPlane.h>

#include <mrpt/system/CTicTac.h>

PointCloud::PointCloud():
	minPoint(800, 800, 800), maxPoint(-800, -800, -800) {
	this->cloud = PclPointCloudPtr(new PclPointCloud);
	this->normals = SurfaceNormalsPtr(new SurfaceNormals);
	normalVectorUpToDate = false;
	boundingBoxUpToDate = false;
	features.resize(3);
	pose.setFromValues(0,0,0,0,0,0);
}

PointCloud::~PointCloud() {
	//TODO delete point cloud ?
}

void PointCloud::clear() {
	this->cloud->clear();
	this->normals->clear();

	features.clear();


	minPoint.x =0;
	minPoint.y =0;
	minPoint.z =0;
	maxPoint.x =0;
	maxPoint.y =0;
	maxPoint.z =0;
	pose.setFromValues(0,0,0,0,0,0);
}

void PointCloud::resize(size_t size) {
	this->cloud->resize(size);
	this->normals->resize(size);
}

void PointCloud::addPoint(double x, double y, double z) {
	//TODO what about color?
	 PointT pclPoint;
	 pclPoint.x = x;
	 pclPoint.y = y;
	 pclPoint.z = z;
//	 pclPoint.r = 255;
//	 pclPoint.g = 0;
//	 pclPoint.b = 0;

	this->cloud->points.push_back(pclPoint);

	normalVectorUpToDate = false;
	boundingBoxUpToDate = false;
}


void PointCloud::addPoint(const TPoint3D& point) {
	//TODO what about color?
	 PointT pclPoint;
	 pclPoint.x = point.x;
	 pclPoint.y = point.y;
	 pclPoint.z = point.z;
//	 pclPoint.r = 255;
//	 pclPoint.g = 0;
//	 pclPoint.b = 0;

	 this->cloud->points.push_back(pclPoint);

	normalVectorUpToDate = false;
	boundingBoxUpToDate = false;
}

void PointCloud::addPoint(const CPoint3D& point) {
	//TODO what about color?
	 PointT pclPoint;
	 pclPoint.x = point.x();
	 pclPoint.y = point.y();
	 pclPoint.z = point.z();
//	 pclPoint.r = 255;
//	 pclPoint.g = 0;
//	 pclPoint.b = 0;

	 this->cloud->points.push_back(pclPoint);

	normalVectorUpToDate = false;
	boundingBoxUpToDate = false;
}

void PointCloud::getPoint(size_t index, double& x, double& y, double& z) const {
	PointT pclPoint;
	pclPoint = this->cloud->at(index);
	x = pclPoint.x;
	y = pclPoint.y;
	z = pclPoint.z;
}

void PointCloud::getPoint(size_t index, TPoint3D& point) const {
	PointT pclPoint;
	pclPoint = this->cloud->at(index);
	point.x = pclPoint.x;
	point.y = pclPoint.y;
	point.z = pclPoint.z;
}
void PointCloud::getPoint(size_t index, CPoint3D& point) const {
	PointT pclPoint;
	pclPoint = this->cloud->at(index);
	point.x(pclPoint.x);
	point.y(pclPoint.y);
	point.z(pclPoint.z);
}


std::vector< std::vector<double> > PointCloud::getAsStdVector() const {

		std::vector< std::vector<double> > vector;
		std::cout<<"[PointCloud::getAsStdVector] PointCloud size: "<<this->cloud->size () << std::endl;
		vector.resize(this->cloud->size ());
		 for (size_t i = 0; i < this->cloud->size (); ++i)
			  {
				 vector[i].resize(3);
				 vector[i][0] = this->cloud->points[i].x;
				 vector[i][1] = this->cloud->points[i].y;
				 vector[i][2] = this->cloud->points[i].z;
			  }
		 return vector;
	}


//bool PointCloud::getNormalVector(size_t index, double& x, double& y, double& z) const {
//	if (normalVectorUpToDate) {
//		x = vecX[index];
//		y = vecY[index];
//		z = vecZ[index];
//	}
//	return normalVectorUpToDate;
//}
//
//void PointCloud::calculateNormalVectors(double x, double y, double z, int neighbors) {
//	TPoint3D viewPoint(x, y, z);
//	vecX.resize(size());
//	vecY.resize(size());
//	vecZ.resize(size());
//
//	int k = neighbors; // number of nearest neighbors
//	int dim = 3; // dimension
//	double eps = 0; // error bound
//	int maxPts = size(); // maximum number of data points
//
//	int nPts; // actual number of data points
//	ANNpointArray dataPts; // data points
//	ANNpoint queryPt; // query point
//
//	ANNidxArray nnIdx; // near neighbor indices
//	ANNdistArray dists; // near neighbor distances
//	ANNkd_tree* kdTree; // search structure
//
//	queryPt = annAllocPt(dim); // allocate query point
//	dataPts = annAllocPts(maxPts, dim); // allocate data points
//	nnIdx = new ANNidx[k]; // allocate near neigh indices
//	dists = new ANNdist[k]; // allocate near neighbor dists
//	nPts = 0; // read data points
//
//	while (nPts < maxPts) {
//		dataPts[nPts][0] = xVal[nPts];
//		dataPts[nPts][1] = yVal[nPts];
//		dataPts[nPts][2] = zVal[nPts];
//		nPts++;
//	}
//
//	kdTree = new ANNkd_tree( // build search structure
//			dataPts, // the data points
//			nPts, // number of points
//			dim); // dimension of space
//
//	///////////////////////////////////
//	// calculate normal vector
//	///////////////////////////////////
//
//	for (size_t i = 0; i < size(); i++) {
//
//		queryPt[0] = xVal[i];
//		queryPt[1] = yVal[i];
//		queryPt[2] = zVal[i];
//
//		kdTree->annkSearch(queryPt, k, nnIdx, dists, eps);
//
//		std::vector<TPoint3D> pointVec;
//		TPlane plane;
//		TPoint3D point;
//
//		for (int j = 0; j < k; ++j) {
//			point.x = xVal[nnIdx[j]];
//			point.y = yVal[nnIdx[j]];
//			point.z = zVal[nnIdx[j]];
//			pointVec.push_back(point);
//		}
//
//		mrpt::math::getRegressionPlane(pointVec, plane);
//
//		double vec[3];
//		plane.getUnitaryNormalVector(vec);
//
//		//		std::cout << "plane: " << plane.coefs[0] << "," << plane.coefs[1] << ", " << plane.coefs[2] << ", "
//		//				<< plane.coefs[0] << "\n";
//
//		if (plane.evaluatePoint(viewPoint) < 0) {
//			vec[0] *= -1;
//			vec[1] *= -1;
//			vec[2] *= -1;
//		}
//
//		vecX[i] = vec[0];
//		vecY[i] = vec[1];
//		vecZ[i] = vec[2];
//	}
//
//	delete kdTree;
//	delete nnIdx;
//	delete dists;
//
//	annDeallocPt(queryPt); // dealloc query point
//	annDeallocPts(dataPts); // dealloc data points
//
//	normalVectorUpToDate = true;
//}

void PointCloud::getPointsAbovePlane(const TPlane& plane, double minDist, PointCloud& newCloud) {
	newCloud.clear();
	double dist = 0;
	TPoint3D point;

	for (size_t i = 0; i < size(); ++i) {
		getPoint(i, point);
		dist = plane.evaluatePoint(point);

		if (dist > minDist) {
			newCloud.addPoint(point);
		}
	}
}

void PointCloud::getPointsInPlane(const TPlane& plane, double maxDist, PointCloud& newCloud) {
	newCloud.clear();
	double dist = 0;
	TPoint3D point;

	for (size_t i = 0; i < size(); ++i) {
		getPoint(i, point);
		dist = plane.distance(point);

		if (dist < maxDist) {
			newCloud.addPoint(point);
		}
	}
}

bool PointCloud::getBoundingBox(CPose3D& pose, TPoint3D& min, TPoint3D& max) const {
	 pose = this->pose;
	 min = this->minPoint;
	 max = this->maxPoint;

	 return boundingBoxUpToDate;
}

bool PointCloud::getBoundingBox(CPose3D& pose, CPoint3D& min, CPoint3D& max) const {
	 pose = this->pose;
	 min = CPoint3D(this->minPoint);
	 max = CPoint3D(this->maxPoint);

	 return boundingBoxUpToDate;
}

void PointCloud::setBoundingBox(CPose3D& pose, CPoint3D& min, CPoint3D& max) {
	 this->pose = pose;
	 
	// Timo - 20.04
	//---------------
	// new:	
	this->minPoint = min.asTPoint();
	this->maxPoint = max.asTPoint();
	// old:
	// this->minPoint = min;
	// this->maxPoint = max;
	//---------------


	 boundingBoxUpToDate = true;
}

CPose3D PointCloud::getBoundingBoxPose() const {
	 return this->pose;
}

/*
* calculate bounding box based on 2d projection
*/
void PointCloud::calculateSimplifiedBoundingBox(CPose3D &tablePose){


	double insertTime,calcTime;
	static mrpt::system::CTicTac  stopwatch1;
	stopwatch1.Tic();

	bool REDUX = true;
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	// first, find zmin and zmax of this object, do this in table coord.
	//!!!!!! double zmin = numeric_limits<float>::max(), zmax = numeric_limits<float>::min();
	//!!!!!! numeric_limits::min does not work for float/double as it returns the smallest POSITIVE number.
	//!!!!!! for int, it returns the smallest number which is then negative as you would expect!
	double zmin = 9999, zmax = -9999;
	for(unsigned int i =  0;i<this->cloud->size();++i){
		// transform points in table coord system. required to eliminate the possible
		// non-zero pose/angle of table
		CPoint3D point;
		this->getPoint(i,point);
		CPoint3D pointInTableCoord(point - tablePose);

		zmin = std::min(pointInTableCoord.z(), zmin);

		zmax = std::max(pointInTableCoord.z(), zmax);
	}
	vector<double> vector_points;
	for(unsigned int i =  0;i<this->cloud->size();++i){
		// transform points in table coord system. required to eliminate the possible
		// non-zero pose/angle of table
		//CPoint3D point(xVal[i], yVal[i], zVal[i]);
		CPoint3D point;
		this->getPoint(i,point);
		CPoint3D pointInTableCoord(point - tablePose);

		vector_points.push_back(pointInTableCoord.x());
		vector_points.push_back(pointInTableCoord.y());
//		vector_points.push_back(pointInTableCoord.z());

		// distribute on top/bottom of object. we do this to force the minimal
		// bounding box to be aligned with the table plan and object. if points
		// are not equally sampled on the object, the bounding box may be "schief",
		// rotated around x/z which is not the case of an object standing on a
		// surface.
		vector_points.push_back(i % 2 ? zmin : zmax);
	}
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;

	insertTime = stopwatch1.Tac();
	// point cloud now in table coordinate system

    vector<BoxInfo> data;
    try{
    stopwatch1.Tic();
	MVBB mvbb(vector_points, "", REDUX);
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	//mvbb.setGrid(3);
	mvbb.setSamples(100);
    //mvbb.setGain(0.9);
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
    	data = mvbb.iterate(); //just the one root bounding box is generated
    calcTime = stopwatch1.Tac();
    std::cout << "[PointCloud]"<<__LINE__ << std::endl;
    printf("\n[MvBB] Insert time: %fms | Calc time: %fms\n\n",1000*insertTime,1000*calcTime);


    }
    catch(...)
    {
    	cout <<COLOR_RED "Error: failed to calculate SimplifiedBoundingBox!"<<COLOR_DEFAULT<<endl;
    	boundingBoxUpToDate = false;
    	this->features[0] = 0;
    	this->features[1] = 0;
    	this->features[2] = 0;
    	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
    	return;
    }

    std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	assert (data[0].coords.size() == 24); //8 vertices of box * 3 coordinates

	BoxInfo info = data[0];
	BoundingBox bb;
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	bb.createBoxFromCoords(info.id, info.parentId, info.coords);

	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	Point center, size;
	center = bb.getCenter(); // center in table coord system

	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
  	std::vector<Face> faces = bb.getFaces();
	std::cout << "[PointCloud] Faces size: "<< faces.size() << std::endl;

        //TODO if faces == 0 was dann ? -> besser auf 6 prüfen??
	if(faces.size() != 6) {
    		cout <<COLOR_RED "Error: Faces.size() not as expected!!"<<COLOR_DEFAULT<<endl;
    		boundingBoxUpToDate = false;
	    	this->features[0] = 0;
	    	this->features[1] = 0;
	    	this->features[2] = 0;
	    	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	    	return;
	}





	/////////////////////////////////////
	// find the maximum normal vector component [x && y && z] values within all faces
	/////////////////////////////////////
	double maxX=0;
	double maxY=0;
	double maxZ=0;
	int maxXi=-1;
	int maxYi=-1;
	int maxZi=-1;
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	for(unsigned int i = 0; i < faces.size(); i++) {
		//std::cout << "Face " << i << " " << faces[i] << std::endl;
		//std::cout << "   area: " << faces[i].getArea() << std::endl;
		//std::cout << "   normal: " << faces[i].getNormal() << std::endl;

		if(maxX<faces[i].getNormal()[0])
		{
		   maxX = faces[i].getNormal()[0];
		   maxXi = i;
		};
		if(maxY<faces[i].getNormal()[1])
		{
		   maxY = faces[i].getNormal()[1];
		   maxYi = i;
		};
		if(maxZ<faces[i].getNormal()[2])
		{
		   maxZ = faces[i].getNormal()[2];
		   maxZi = i;
		};
	}
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;


	std::vector<double> x(3);
//	mrpt::vector_double x(3);
	std::vector<double> y(3);
//	mrpt::vector_double y(3);
	std::vector<double> z(3);
//	mrpt::vector_double z(3);
	std::vector<double> coordsys(3);
//	mrpt::vector_double coordsys(3);
	double yaw,pitch,roll;
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;

	//rot X --> yaw
	coordsys[0] = 1;
	coordsys[1] = 0;
	coordsys[2] = 0;
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	x[0] = faces[maxXi].getNormal()[0];
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	x[1] = faces[maxXi].getNormal()[1];
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	x[2] = faces[maxXi].getNormal()[2]*0; //project the normal value down to the xy plane --> only for angle calculation (length is scaled down)!
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	//scalar_prod = coordsys[0]*x[0] + coordsys[1]*x[1] + coordsys[2]*x[2];	//scalar calculation but without the direction of the rotation
	//length1 = sqrt((x[0]*x[0] + x[1]*x[1] + x[2]*x[2]));
	//length2 = sqrt(coordsys[0]*coordsys[0] + coordsys[1]*coordsys[1] + coordsys[2]*coordsys[2]);
      	//angle = acos(scalar_prod/length1 * length2);
	//std::cout<<"angle: "<<angle<<" deg: "<<(angle*180/M_PI)<<std::endl;
	yaw = atan2(x[1],x[0]);
//	std::cout<<"yaw angle in deg: "<<yaw*180/M_PI<<std::endl;

	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	//rot y -->roll
	coordsys[0] = 0;
	coordsys[1] = 1;
	coordsys[2] = 0;
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	y[0] = faces[maxYi].getNormal()[0]*0; //project down to yz plane
	y[1] = faces[maxYi].getNormal()[1];
	y[2] = faces[maxYi].getNormal()[2];
	roll = atan2(y[2],y[1]);
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
//	std::cout<<"pitch angle in deg: "<<pitch*180/M_PI<<std::endl;

	//rot z --> pitch
	coordsys[0] = 0;
	coordsys[1] = 0;
	coordsys[2] = 1;
	z[0] = faces[maxZi].getNormal()[0];
	z[1] = faces[maxZi].getNormal()[1]*0; //project down to xz plane
	z[2] = faces[maxZi].getNormal()[2];
	pitch = atan2(z[0],z[2]);
//	std::cout<<"roll angle in deg: "<<roll*180/M_PI<<std::endl;
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	CPose3D center_pose(center[0],center[1],center[2],yaw,pitch,roll); // pose of object in table coords

	vector<CPoint3D> bbPointsLocalCoord;
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	//Transform the 8 bounding box points into local coord system
	for (std::size_t i = 0; i < data[0].coords.size(); i += 3){
		CPoint3D tmp(data[0].coords[i], data[0].coords[i+1], data[0].coords[i+2]);
	    tmp = tmp - center_pose;
		bbPointsLocalCoord.push_back(tmp);
	}

	//search for local min and max point to estimate the size of the bounding box
	// set bounding box
	this->minPoint.x = this->minPoint.y = this->minPoint.z = 800000;
	this->maxPoint.x = this->maxPoint.y = this->maxPoint.z = -800000;
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	for(unsigned int i = 0; i < bbPointsLocalCoord.size(); i++){
		double* x,*y,*z;
		x = &bbPointsLocalCoord[i].x();
		y = &bbPointsLocalCoord[i].y();
		z = &bbPointsLocalCoord[i].z();

		// minPoint
		if (*x < this->minPoint.x)
			this->minPoint.x = *x;

		if (*y < this->minPoint.y)
			this->minPoint.y = *y;

		if (*z < this->minPoint.z)
			this->minPoint.z = *z;

		// maxPoint
		if (*x > this->maxPoint.x)
			this->maxPoint.x = *x;

		if (*y > this->maxPoint.y)
			this->maxPoint.y = *y;

		if (*z > this->maxPoint.z)
			this->maxPoint.z = *z;
	}
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	// set histogram values
	this->features[0] = (this->maxPoint.x - this->minPoint.x);
	this->features[1] = (this->maxPoint.y - this->minPoint.y);
	this->features[2] = (this->maxPoint.z - this->minPoint.z);

	this->pose = tablePose + center_pose;
	boundingBoxUpToDate = true;
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
}


void PointCloud::calculateMinimalVolumeBoundingBox(){
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	bool REDUX = true;

	double insertTime,calcTime;
	static mrpt::system::CTicTac  stopwatch1;
	stopwatch1.Tic();

	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	vector<double> vector_points;
	{
	double x, y, z;
	for(unsigned int i =  0;i<this->cloud->size();++i){
		this->getPoint(i,x,y,z);
		vector_points.push_back(x);
		vector_points.push_back(y);
		vector_points.push_back(z);
	}
	}

	MVBB mvbb(vector_points, "", REDUX);
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	insertTime = stopwatch1.Tac();

	stopwatch1.Tic();
	//mvbb.setGrid(3);
	mvbb.setSamples(100);
    //mvbb.setGain(0.9);
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	vector<BoxInfo> data = mvbb.iterate(); //just the one root bounding box is generated
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	calcTime = stopwatch1.Tac();

	printf("\n[MvBB] Insert time: %fms | Calc time: %fms\n\n",1000*insertTime,1000*calcTime);

	assert (data[0].coords.size() == 24); //8 vertices of box * 3 coordinates
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;

	BoxInfo info = data[0];
	BoundingBox bb;
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
	bb.createBoxFromCoords(info.id, info.parentId, info.coords);
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;

	Point center, size;
	center = bb.getCenter();
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;

  	std::vector<Face> faces = bb.getFaces();

	/////////////////////////////////////
	// find the maximum normal vector component [x && y && z] values within all faces
	/////////////////////////////////////
	double maxX=0;
	double maxY=0;
	double maxZ=0;
	int maxXi=-1;
	int maxYi=-1;
	int maxZi=-1;

	for(unsigned int i = 0; i < faces.size(); i++) {
		//std::cout << "Face " << i << " " << faces[i] << std::endl;
		//std::cout << "   area: " << faces[i].getArea() << std::endl;
		//std::cout << "   normal: " << faces[i].getNormal() << std::endl;

		if(maxX<faces[i].getNormal()[0])
		{
		   maxX = faces[i].getNormal()[0];
		   maxXi = i;
		};
		if(maxY<faces[i].getNormal()[1])
		{
		   maxY = faces[i].getNormal()[1];
		   maxYi = i;
		};
		if(maxZ<faces[i].getNormal()[2])
		{
		   maxZ = faces[i].getNormal()[2];
		   maxZi = i;
		};
	}

	std::vector<double> x(3);
//	mrpt::vector_double x(3);
	std::vector<double> y(3);
//	mrpt::vector_double y(3);
	std::vector<double> z(3);
//	mrpt::vector_double z(3);
	std::vector<double> coordsys(3);
//	mrpt::vector_double coordsys(3);
	double yaw,pitch,roll;

	//rot X --> yaw
	coordsys[0] = 1;
	coordsys[1] = 0;
	coordsys[2] = 0;
	x[0] = faces[maxXi].getNormal()[0];
	x[1] = faces[maxXi].getNormal()[1];
	x[2] = faces[maxXi].getNormal()[2]*0; //project the normal value down to the xy plane --> only for angle calculation (length is scaled down)!
	//scalar_prod = coordsys[0]*x[0] + coordsys[1]*x[1] + coordsys[2]*x[2];	//scalar calculation but without the direction of the rotation
	//length1 = sqrt((x[0]*x[0] + x[1]*x[1] + x[2]*x[2]));
	//length2 = sqrt(coordsys[0]*coordsys[0] + coordsys[1]*coordsys[1] + coordsys[2]*coordsys[2]);
      	//angle = acos(scalar_prod/length1 * length2);
	//std::cout<<"angle: "<<angle<<" deg: "<<(angle*180/M_PI)<<std::endl;
	yaw = atan2(x[1],x[0]);
	//std::cout<<"yaw angle in deg: "<<yaw*180/M_PI<<std::endl;


	//rot y -->roll
	coordsys[0] = 0;
	coordsys[1] = 1;
	coordsys[2] = 0;
	y[0] = faces[maxYi].getNormal()[0]*0; //project down to yz plane
	y[1] = faces[maxYi].getNormal()[1];
	y[2] = faces[maxYi].getNormal()[2];
	roll = atan2(y[2],y[1]);
	//std::cout<<"pitch angle in deg: "<<pitch*180/M_PI<<std::endl;

	//rot z --> pitch
	coordsys[0] = 0;
	coordsys[1] = 0;
	coordsys[2] = 1;
	z[0] = faces[maxZi].getNormal()[0];
	z[1] = faces[maxZi].getNormal()[1]*0; //project down to xz plane
	z[2] = faces[maxZi].getNormal()[2];
	pitch = atan2(z[0],z[2]);
	//std::cout<<"roll angle in deg: "<<roll*180/M_PI<<std::endl;

	CPose3D center_pose(center[0],center[1],center[2],yaw,pitch,roll);

	//Do not use changeOrigin, since the min and max Points (not avialable in robot coord.) are transformed as well
	this->pose = this->pose + center_pose;

	vector<CPoint3D> bbPointsLocalCoord;

	//Transform the 8 bounding box points into local coord system
	for (std::size_t i = 0; i < data[0].coords.size(); i += 3){
		CPoint3D tmp(data[0].coords[i], data[0].coords[i+1], data[0].coords[i+2]);
	    tmp = tmp - center_pose;
		bbPointsLocalCoord.push_back(tmp);
	}

	//search for local min and max point to estimate the size of the bounding box
	// set bounding box
	this->minPoint.x = this->minPoint.y = this->minPoint.z = 800000;
	this->maxPoint.x = this->maxPoint.y = this->maxPoint.z = -800000;

	for(unsigned int i = 0; i < bbPointsLocalCoord.size(); i++){
		double* x,*y,*z;
		x = &bbPointsLocalCoord[i].x();
		y = &bbPointsLocalCoord[i].y();
		z = &bbPointsLocalCoord[i].z();

		// minPoint
		if (*x < this->minPoint.x)
			this->minPoint.x = *x;

		if (*y < this->minPoint.y)
			this->minPoint.y = *y;

		if (*z < this->minPoint.z)
			this->minPoint.z = *z;

		// maxPoint
		if (*x > this->maxPoint.x)
			this->maxPoint.x = *x;

		if (*y > this->maxPoint.y)
			this->maxPoint.y = *y;

		if (*z > this->maxPoint.z)
			this->maxPoint.z = *z;
	}

	// set histogram values
	this->features[0] = (this->maxPoint.x - this->minPoint.x);
	this->features[1] = (this->maxPoint.y - this->minPoint.y);
	this->features[2] = (this->maxPoint.z - this->minPoint.z);

	boundingBoxUpToDate = true;
	std::cout << "[PointCloud]"<<__LINE__ << std::endl;
}






// axis aligned bb
//void PointCloud::calculateBoundingBox(){
//	this->minPoint.x = this->minPoint.y = this->minPoint.z = 800000;
//	this->maxPoint.x = this->maxPoint.y = this->maxPoint.z = -800000;
//
//	for(unsigned int i = 0; i < this->xVal.size(); i++){
//		double* x,*y,*z;
//		x = &this->xVal[i];
//		y = &this->yVal[i];
//		z = &this->zVal[i];
//
//		// minPoint
//		if (*x < this->minPoint.x)
//			this->minPoint.x = *x;
//
//		if (*y < this->minPoint.y)
//			this->minPoint.y = *y;
//
//		if (*z < this->minPoint.z)
//			this->minPoint.z = *z;
//
//		// maxPoint
//		if (*x > this->maxPoint.x)
//			this->maxPoint.x = *x;
//
//		if (*y > this->maxPoint.y)
//			this->maxPoint.y = *y;
//
//		if (*z > this->maxPoint.z)
//			this->maxPoint.z = *z;
//	}
//
////	// set histogram values
//	this->features[0] = (this->maxPoint.x - this->minPoint.x);
//	this->features[1] = (this->maxPoint.y - this->minPoint.y);
//	this->features[2] = (this->maxPoint.z - this->minPoint.z);
//}


//void PointCloud::calculateEigenvectors(TPoint3D& xAxis, TPoint3D& yAxis, TPoint3D& zAxis) {
//	vector<double> means;
//	CMatrixDouble33 covars, eigenVal, eigenVec;
//
//	//TODO this could be done much faster
//	std::vector<TPoint3D> points;
//	for(unsigned int i = 0;i<this->xVal.size();i++){
//		TPoint3D point;
//		point.x = xVal[i]; point.y = yVal[i]; point.z = zVal[i];
//		points.push_back(point);
//	}
//
//	covariancesAndMean(points, covars, means);
//	covars.eigenVectors(eigenVec, eigenVal);
//
//	zAxis.x = eigenVec(0, 0);
//	zAxis.y = eigenVec(1, 0);
//	zAxis.z = eigenVec(2, 0);
//
//	yAxis.x = eigenVec(0, 1);
//	yAxis.y = eigenVec(1, 1);
//	yAxis.z = eigenVec(2, 1);
//
//	xAxis.x = eigenVec(0, 2);
//	xAxis.y = eigenVec(1, 2);
//	xAxis.z = eigenVec(2, 2);
//}

//void PointCloud::calculateCentroid() {
//	TPoint3D centroid;
//
//	for (unsigned int i = 0; i < this->xVal.size(); i++) {
//		centroid.x += this->xVal[i];
//		centroid.y += this->yVal[i];
//		centroid.z += this->zVal[i];
//	}
//
//	// calculate mean
//	centroid.x /= this->xVal.size();
//	centroid.y /= this->xVal.size();
//	centroid.z /= this->xVal.size();
//}


//void PointCloud::changeOrigin(const CPose3D& p) {
//	this->pose = this->pose + p;
//
//	// move bounding box
//	CPoint3D p1(this->minPoint);
//	this->minPoint = p1 - p;
//	CPoint3D p2(this->maxPoint);
//	this->maxPoint = p2 - p;
//
//
//	// move origin of the point cloud to it's center
//	for (unsigned int i = 0; i < this->xVal.size(); i++) {
//		//TODO this could be done much faster!
//		CPoint3D point(this->xVal[i],this->yVal[i],zVal[i]);
//		CPoint3D out;
//
//			out = point - p;
//		this->xVal[i] = out.x();
//		this->yVal[i] = out.y();
//		this->zVal[i] = out.z();
//
//	}
//}
