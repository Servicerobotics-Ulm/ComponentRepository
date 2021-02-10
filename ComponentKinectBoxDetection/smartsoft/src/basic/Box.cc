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

#include "Box.hh"
#include <cmath>

#include <mrpt/math/include/mrpt/math/TPose3D.h>
#include <mrpt/poses/include/mrpt/poses/CPoint3D.h>

Box::Box() {
	// TODO Auto-generated destructor stub
}

Box::~Box() {
	// TODO Auto-generated destructor stub
}

/*
 * Sorts the points vector to: top left, top right, bottom right, bottom left
 */
void Box::sortPoints(){
	std::sort(points.begin(),points.end(),[](const cv::Point2d pt1, const cv::Point2d pt2) { return (pt1.y < pt2.y);});
	std::sort(points.begin(),points.begin() + 2, [](const cv::Point2d pt1, const cv::Point2d pt2) { return (pt1.x < pt2.x);});
	std::sort(points.begin() + 2,points.end(), [](const cv::Point2d pt1, const cv::Point2d pt2) { return (pt1.x > pt2.x);});
}


/*
 * Transforms the box pose to:
 * 		x is going into the plane
 * 		y is parallel to the short side
 * 		z is parallel to the long side
 * 	This is possible since, the plane points are sorted.
 */
void Box::rearrangePose(){

	mrpt::poses::CPose3D pose = this->pose;
	CommBasicObjects::CommPose3d sensor_pose = point_manipulator->getSensorPose();

	///////////////////////////////////////
	// Rotate the pose with x into the object

	// Rotate to make pose x axis to plane normal (ZYX Konvention)
	pose += mrpt::poses::CPose3D(0, 0, 0, 0, 1.57079, 0);
	// Should also be 1.57079, but then the delta velues are too small,
	// which results in inaccurate positions. Hence only 1.45.
	pose += mrpt::poses::CPose3D(0, 0, 0, 0, 0, -1.45);

	mrpt::math::TPoint3D sensor_origin(sensor_pose.getPosition().getX(), sensor_pose.getPosition().getY(), sensor_pose.getPosition().getZ());
	
	
	// Timo - 20.04
	//---------------
	// new:	
	mrpt::math::TPoint3D point1 = mrpt::math::TPoint3D(pose.asTPose()) + mrpt::math::TPoint3D(0.1,0,0);
	mrpt::math::TPoint3D point2 = mrpt::math::TPoint3D(pose.asTPose()) + mrpt::math::TPoint3D(-0.1,0,0);
	// old:
	// mrpt::math::TPoint3D point1 = pose + mrpt::math::TPoint3D(0.1,0,0);
	// mrpt::math::TPoint3D point2 = pose + mrpt::math::TPoint3D(-0.1,0,0);
	//---------------


	double dist1 = point1.distanceTo(sensor_origin);
	double dist2 = point2.distanceTo(sensor_origin);

	// check, whether the box coordinate frame is rotated correctly
	// by checking the distance to sensor frame origin.
	if(dist1 < dist2){
		// if box coordinate frame is wrong, it is rotated (180 deg)
		//pose += mrpt::poses::CPose3D(0, 0, 0, 3.14, 0, 3.14);
		pose += mrpt::poses::CPose3D(0, 0, 0, 3.14, 0, 0);
	}

	/////////////////////////////////
	// Find smaller plane side (top or left) and find mid point of it

	int dx, dy;
	// calculate length of plane's top side by Pythagoras' theorem
	dx = this->points[0].x - this->points[1].x;
	dy = fabs(this->points[0].y - this->points[1].y);
	float length_top_side = sqrt(pow(dx, 2) + pow(dy, 2));

	// calculate length of plane's left side by Pythagoras' theorem
	dx = this->points[0].x - this->points[3].x;
	dy = fabs(this->points[0].y - this->points[3].y);
	float length_left_side = sqrt(pow(dx, 2) + pow(dy, 2));

	// mid point of the shorter side, hence on the z axis of the pose
	cv::Point z_target;
	if(length_top_side < length_left_side){
		z_target.x = (this->points[0].x + this->points[1].x) / 2;
		// adding 10 pixels to get closer to the plane center for preventing errors in 3D
		z_target.y = ((this->points[0].y + this->points[1].y) / 2) + 10;
	}else {
		// adding 10 pixels to get closer to the plane center for preventing errors in 3D
		z_target.x = ((this->points[0].x + this->points[3].x) / 2) + 10;
		z_target.y = (this->points[0].y + this->points[3].y) / 2;
	}

	////////////////////////////////////
	// Define two straights:
	// 1. plane center -> pose z axis
	// 2. plane center -> mid point of smaller plane side
	float x_in_m, y_in_m, z_in_m;
	// transform short side mid point from 2D into robot coordinate frame
	// all calculation is done with respect to robot coordinate frame
	uint32_t row = z_target.y , col = z_target.x;
	point_manipulator->pixelToXyz(row , col, x_in_m, y_in_m, z_in_m, true);

	mrpt::poses::CPoint3D z_target_3d(x_in_m, y_in_m, z_in_m);

	// define point on object pose z axis
	mrpt::poses::CPoint3D z_axis_vector = pose +  mrpt::poses::CPoint3D(0.0, 0.0, 0.02);

	// when the angle is too small calculation errors can appear --> rotate the pose a bit to enlarge the angle
	if (z_target_3d.y() + 0.005 > z_axis_vector.y() && z_target_3d.y() - 0.005 < z_axis_vector.y()){
		pose += mrpt::poses::CPose3D(0, 0, 0, 0, 0, -0.4);
		z_axis_vector = pose +  mrpt::poses::CPoint3D(0.0, 0.0, 0.02);
	}

	// pose origin is intersection point between the two straights
	mrpt::poses::CPoint3D intersection_point (pose.x(), pose.y(), pose.z());

	// calculate slopes of the two straights
	mrpt::poses::CPoint3D gradient1 = z_target_3d - intersection_point;
	mrpt::poses::CPoint3D gradient2 = z_axis_vector - intersection_point;


	/////////////////////////////////////
	// Calculate angle between the two straights
	// arccos( |g1 * g2| / ( |g1| * |g2| ) )

	float numerator = gradient1.x() * gradient2.x() + gradient1.y() * gradient2.y() + gradient1.z() * gradient2.z();
	numerator = fabs(numerator);

	float abs_g1 = sqrt(pow(gradient1.x(), 2) + pow(gradient1.y(), 2) + pow(gradient1.z(), 2));
	float abs_g2 = sqrt(pow(gradient2.x(), 2) + pow(gradient2.y(), 2) + pow(gradient2.z(), 2));
 	float denominator = abs_g1 * abs_g2;

 	int direction_factor = 1;
 	//originally: if (z_target_3d.y() > z_axis_vector.y()){
 	if (z_target_3d.y() < z_axis_vector.y()){
 		// multiplication of angle with -1 for rotation into left direction
 		direction_factor = -1;
 	}

 	float angle_rad = (acos(numerator / denominator) * direction_factor) - 3.14;

 	// rotate pose around x axis
	pose += mrpt::poses::CPose3D(0, 0, 0, 0, 0, angle_rad);

	this->pose = pose;


	////////////////////
	//Debug code

	this->some_debug_point_2d = cv::Point();
	this->some_debug_point_2d.x = z_target.x;
	this->some_debug_point_2d.y = z_target.y;

//	this->some_debug_point = mrpt::poses::CPoint3D(z_axis_vector.x(), z_axis_vector.y(), z_axis_vector.z());

//	std::cout << "------------" << std::endl;
//	std::cout << "midpoint: " << z_target_3d << std::endl;
//	std::cout << "z axis vector: " << z_axis_vector << std::endl;
//	std::cout << "center intersection: " << intersection_point << std::endl;
//	std::cout << "slope mid point: " << gradient1 << std::endl;
//	std::cout << "slope z axis: " << gradient2 << std::endl;
//	std::cout << "numerator: " << numerator << std::endl;
//	std::cout << "denominator: " << denominator << std::endl;
//	std::cout << "angle_rad: " << angle_rad << std::endl;
}




/*
 * Defines MRPT rectangle to check, whether the given point is within this shape
 * this is necessary to get rid of surrounding points of diagonal boxes
 */
bool Box::isPointInRect(int x, int y){
	return mrpt_rect.contains(mrpt::math::TPoint2D(x,y));
}


/*
 * Calculates the center point of the rectangle.
 */
cv::Point Box::getRectCenter() {
	return cv::Point((points[0].x + points[2].x) / 2,(points[0].y + points[2].y) / 2);
}


/*
 * Returns the min and max row (Y) and column (X) in the RGB image.
 * This is necessary, when a diagonal rectangle was detected.
 */
void Box::getRectMinMaxValues (int &minX, int &maxX, int &minY, int &maxY){
	maxX = std::max(points[0].x, points[1].x);
	maxX = std::max(points[2].x, maxX);
	maxX = std::max(points[3].x, maxX);

	minX = std::min(points[0].x, points[1].x);
	minX = std::min(points[2].x, minX);
	minX = std::min(points[3].x, minX);

	maxY = std::max(points[0].y, points[1].y);
	maxY = std::max(points[2].y, maxY);
	maxY = std::max(points[3].y, maxY);

	minY = std::min(points[0].y, points[1].y);
	minY = std::min(points[2].y, minY);
	minY = std::min(points[3].y, minY);
}

