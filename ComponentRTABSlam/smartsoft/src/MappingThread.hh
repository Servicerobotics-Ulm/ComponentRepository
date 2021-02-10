//--------------------------------------------------------------------------
// Code generated by the SmartSoft MDSD Toolchain
// The SmartSoft Toolchain has been developed by:
//  
// Service Robotics Research Center
// University of Applied Sciences Ulm
// Prittwitzstr. 10
// 89075 Ulm (Germany)
//
// Information about the SmartSoft MDSD Toolchain is available at:
// www.servicerobotik-ulm.de
//
// This file is generated once. Modify this file to your needs. 
// If you want the toolchain to re-generate this file, please 
// delete it before running the code generator.
//--------------------------------------------------------------------------
// --------------------------------------------------------------------------
//
//  Copyright (C) 2018 Nayabrasul Shaik
//
//        shaik@hs-ulm.de
//
//        ZAFH Servicerobotik Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        D-89075 Ulm
//        Germany
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
// --------------------------------------------------------------------------
#ifndef _MAPPINGTHREAD_HH
#define _MAPPINGTHREAD_HH

#include "MappingThreadCore.hh"
#include "SmartSoft2RtabMapConverter.hh"

#include "DomainVision/CommRGBDImage.hh"

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Transform.h>

#include <rtabmap/core/Memory.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/odometry/OdometryF2M.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>




#include "MapBuilder.hh"

//#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <limits>
#include <memory>

#include <qt5/QtWidgets/qapplication.h>

#include <rtabmap/core/Odometry.h>

class MappingThread  : public MappingThreadCore
{
private:
	Smart::StatusCode status;
		  DomainVision::CommRGBDImage scan;

		  //rs::core::image_info m_colorInfo;
		  //rs::core::image_info m_depthInfo;
		  //rs2_extrinsics d2c_extrinsics;
		  //rs2_intrinsics m_colorInt;
		  //rs2_intrinsics m_depthInt;

		  cv::Mat m_imageColor;
		  cv::Mat m_imageDepth;

		  bool first_image_flag;                  /**< use this flag to initialize intrinsic and extrinsic parameters after receiving first image from server    */
		  int m_frame_number;                     /**< Local frame number zero for the first frame received by tracker 											 */
	      short int number_of_no_persons_frames;  /**< Counter to set goal (0,0) if person is not found in frame for long time 									 */


	     // RtabMapCoreWrapper cw;
	      //rtabmap::Odometry * odometry_;
	      //SmartSoftOdometry smartodom_;
	      rtabmap::OdometryF2M Odomfm;

	      QApplication *app;
	      MapBuilder *mapBuilder;
public:
	MappingThread(SmartACE::SmartComponent *comp);
	virtual ~MappingThread();
	
	virtual int on_entry();
	virtual int on_execute();
	virtual int on_exit();

	void release_images();
	//void DisplayTrackingResult(Intel::RealSense::PersonTracking::PersonTrackingData &mtrackingData, cv::Mat &rgb, cv::Mat &depth, std::string str_framenum);
	//void SendGoal(Intel::RealSense::PersonTracking::PersonTrackingData &mtrackingData);
	//mrpt::poses::CPoint3D transormPointToRobotCoord(const mrpt::poses::CPoint3D & point, CommBasicObjects::CommPose3d sensor_pose);
	bool startStopTracking(bool isStart, int personId);


	const wchar_t *GetWC(const char *c);
	void init_odometry();
	rtabmap::Rtabmap rtabmap;
	Odomtype OdomType_;
};

#endif
