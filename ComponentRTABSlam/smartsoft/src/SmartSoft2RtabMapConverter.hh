/*--------------------------------------------------------------------------

 Copyright (C) 2011 

 Created on: Sep 19, 2017
 Author    : Nayabrasul Shaik (shaik@hs-ulm.de)

 ZAFH Servicerobotik Ulm
 University of Applied Sciences
 Prittwitzstr. 10
 D-89075 Ulm
 Germany

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version 2.1
 of the License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 General Public License for more details.

 You should have received a copy of the GNU General Public License along
 with this library; if not, write to the Free Software Foundation, Inc.,
 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

--------------------------------------------------------------------------*/

#ifndef SMARTSOFT2RTABMAPCONVERTER_HH_
#define SMARTSOFT2RTABMAPCONVERTER_HH_

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Statistics.h>

#include <DomainVision/CommRGBDImage.hh>


enum class Odomtype
    {
        WheelOdom,
        VisualOdom
    };

// Color intrinsics
struct st_intrinsics{
	int cols, rows;
	float cx, cy, fx, fy;
	float distortion_coeffs[5];
	DomainVision::ImageDistortionModel distortion_model;
};//color_intrinsics, depth_intrinsics;

//Depth extrinsics with respect to color
struct st_extrinsics{
	float rotation[9];
	float translation[3];
};//depth_to_color_extrinsics;

rtabmap::CameraModel cameraModelFromSmartSoft(DomainVision::CommRGBDImage &scan, const rtabmap::Transform & localTransform);
rtabmap::Transform transformFromSmartSoftCommPose3d( const CommBasicObjects::CommPose3d &base_pose);
rtabmap::Transform transformFromSmartSoftBasePose( const CommBasicObjects::CommBasePose &base_pose);

#endif /* SMARTSOFT2RTABMAPCONVERTER_HH_ */
