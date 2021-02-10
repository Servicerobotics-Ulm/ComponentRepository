/*
 * Container.h
 *
 *  Created on: Jul 18, 2018
 *      Author: rollenhagen
 */

#ifndef CONTAINER_H_
#define CONTAINER_H_

#include "Layer.hh"

class Container {
private:
	std::vector<Layer> layers;
	std::vector<mrpt::math::TPoint3D> topPoints; // order: point of pose, point on x axis of pose, point on y axis of pose, last point
	mrpt::math::TPose3D pose;
	std::vector<float> dimensions; //in m, width, length, height

	float gridUnit;

	void findFreeSpace(float width, float length, float height);

public:
	Container();
	virtual ~Container();

	std::vector<mrpt::math::TPoint3D> getTopPoints() const {
		return topPoints;
	}

	// required order: point of pose, point on x axis of pose, point on y axis of pose, last point
	void setTopPoints(std::vector<mrpt::math::TPoint3D> points) {
		if(points.size() == 4){
			this->topPoints = points;
		}else {
			//TODO: Throw exception
		}
	}

	mrpt::math::TPose3D getPose() {
		mrpt::math::TPose3D tmp_pose(this->pose);
		return tmp_pose;
	}

	void setPose(const mrpt::math::TPose3D pose) {
		this->pose = pose;
	}

	//dimensions in m, relative to container pose. order: x,y,z
	//Todo no hard coding!!
	std::vector<float> getDimensions() {
		if(dimensions.size() == 0){
//			dimensions.push_back(topPoints[0].distanceTo(topPoints[1]));
//			dimensions.push_back(topPoints[0].distanceTo(topPoints[2]));
//			dimensions.push_back(layers[0].getPlane().distance(topPoints[0]));
			dimensions.push_back(0.36);
			dimensions.push_back(0.26);
			//dimensions.push_back(0.22); // regular big SLC
			//dimensions.push_back(0.13); // regular lower SLC
			dimensions.push_back(0.105); // blue low SLC
		}
		return dimensions;
	}
};

#endif /* CONTAINER_H_ */
