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

#ifndef CONCRETEOBJECT_H_
#define CONCRETEOBJECT_H_

#include <string>
#include <vector>
//#include <mrpt/gui/include/mrpt/gui.h>
#include <mrpt/poses/CPose3D.h>

#include "basic/ObjectBeliefs.hh"
#include "basic/PointCloud.hh"

#include <CommObjectRecognitionObjects/CommObjectRecognitionObjectProperties.hh>


using namespace mrpt;
using namespace mrpt::math;
//using namespace mrpt::utils;
//using namespace mrpt::gui;

class ConcreteObject {

private:
	/**
	 * unique id of the object
	 */
	uint32_t id;

	/**
	 * name of the object class the object belongs to.
	 */
	std::string objectClass;

	/**
	 * world pose of the object
	 */
	CPose3D pose;

	/**
	 * world pose of the objects surface
	 */
	CPose3D surfacePose;

	/**
	 * min point (left, bottom, front) of the bounding box
	 */
	TPoint3D minPoint;

	/**
	 * max point (right, top, back) of the bounding box
	 */
	TPoint3D maxPoint;

	/**
	* object dimensions (size)
	*/
	CommBasicObjects::CommPosition3d dimension;


public:
	/**
	 * the beliefs: a list of objects with probabilities what the OR thinks this the
	 * object under consideration might be
	 */
	ObjectBeliefs objectBeliefs;

	PointCloud obstacleHullPointCloud;
	std::vector< std::vector <uint32_t> > obstacleHullvertices;
	std::vector< std::pair <CommObjectRecognitionObjects::RelationType,unsigned int> > relations;
	double fill_level;


	ConcreteObject();
	virtual ~ConcreteObject();

public:

	ConcreteObject(const ConcreteObject& p) {
	    id = p.getId();
	    objectClass = p.getObjectClass();
	    pose = p.getPose();
	    surfacePose = p.getSurfacePose();
	    dimension = p.getDimension();
	    minPoint = p.getMinPoint();
	    maxPoint = p.getMaxPoint();
	    objectBeliefs = p.getBeliefs();
	    obstacleHullPointCloud = p.getObstacleHullPointCloud();
	    obstacleHullvertices = p.getObstacleHullvertices();
	    fill_level = p.getFillLevel();

	    //std::cout << "beliefs in copy constr.: " << p.getBeliefs() << std::endl;
	}

	ConcreteObject& operator=(const ConcreteObject& p) {
	    id = p.getId();
	    objectClass = p.getObjectClass();
	    pose = p.getPose();
	    surfacePose = p.getSurfacePose();
	    dimension = p.getDimension();
	    minPoint = p.getMinPoint();
	    maxPoint = p.getMaxPoint();
	    objectBeliefs = p.getBeliefs();
	    obstacleHullPointCloud = p.getObstacleHullPointCloud();
	    obstacleHullvertices = p.getObstacleHullvertices();
	    std::cout << "Timo, Zuweisung, Points: " << p.getObstacleHullPointCloud().getAsStdVector().size() << std::endl;
	    std::cout << "Timo, Zuweisung, Vertices: " << p.getObstacleHullvertices().size() << std::endl;
	    fill_level = p.getFillLevel();
	   // std::cout << "beliefs in = operator.: " << p.getBeliefs() << std::endl;
	    return *this;
	}



	/**
	 * Set the unique id of the object.
	 */
	void setId(uint32_t id) {
		this->id = id;
	}

	/**
	 * Returns the unique id of the object.
	 */
	uint32_t getId() const {
		return id;
	}

	/**
	 * Set the name of the object class the object belongs to.
	 */
	void setObjectClass(const std::string& objClass) {
		this->objectClass = objClass;
	}

	/**
	 * Returns the name of the object class the object belongs to.
	 */
	const std::string& getObjectClass() const {
		return objectClass;
	}

	/**
	 * Set the real world pose of the object.
	 */
	void setPose(const CPose3D pose) {
		this->pose = pose;
	}

	/**
	 * Get the real world pose of the object.
	 */
	const CPose3D getPose() const {
		return pose;
	}

	/**
	 * Set the real world surface pose of the object.
	 */
	void setSurfacePose(const CPose3D surfacePose) {
		this->surfacePose = surfacePose;
	}

	/**
	 * Get the real world surface pose of the object.
	 */
	const CPose3D getSurfacePose() const {
		return surfacePose;
	}

	void setDimension(const CommBasicObjects::CommPosition3d dimension) {
		this->dimension = dimension;
	}

	const CommBasicObjects::CommPosition3d getDimension() const {
		return this->dimension;
	}

	/**
	 * Set the min point (left, bottom, front) of the bounding box
	 */
	void setMinPoint(const mrpt::math::TPoint3D& point) {
		minPoint = point;
	}

	/**
	 * Get the min point (left, bottom, front) of the bounding box
	 */
	const TPoint3D& getMinPoint() const {
		return minPoint;
	}

	/**
	 * Set the max point (right, top, back) of the bounding box
	 */
	void setMaxPoint(const TPoint3D& point) {
		maxPoint = point;
	}

	/**
	 * Get the max point (right, top, back) of the bounding box
	 */
	const TPoint3D& getMaxPoint() const {
		return maxPoint;
	}

	/**
	 * Returns the euclidean distance between the to real world poses of the objects
	 */
	double distanceTo(const ConcreteObject& o) const {
		return pose.distanceTo(o.pose);
	}

	/**
	 * Compares the features of two concrete objects and returns if they are similar,
	 * otherwise false;
	 */
	bool compareFeatures(const ConcreteObject& o) const;

	/**
	 * add a belief to the list of beliefs
	 */
	void setBeliefs(ObjectBeliefs belief) {
		objectBeliefs = belief;
	}

	/**
	 * get the list of object beliefs
	 */
	ObjectBeliefs getBeliefs() const {
		return objectBeliefs;
	}

	void setObstacleHullPointCloud(const PointCloud& cloud ){
		obstacleHullPointCloud = cloud;
	}

	PointCloud getObstacleHullPointCloud() const{
		return obstacleHullPointCloud;
	}

	void setObstacleHullvertices(const std::vector< std::vector <uint32_t> >& hull){
		obstacleHullvertices = hull;
	}

	const std::vector< std::vector <uint32_t> >& getObstacleHullvertices() const{
		return obstacleHullvertices;
	}

	void setFillLevel(double level){
		this->fill_level = level;
	}

	double getFillLevel() const{
		return this->fill_level;
	}


};

#endif /* CONCRETEOBJECT_H_ */
