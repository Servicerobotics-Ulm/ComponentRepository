/*--------------------------------------------------------------------------

 Copyright (C) 2011 

 Created on: Aug 25, 2017
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

#ifndef SMARTSOFT2REALSENSESDKCONVERTER_H_
#define SMARTSOFT2REALSENSESDKCONVERTER_H_

//#include <rs_core.h>
#include <3rdParty/include/rs/core/correlated_sample_set.h>
#include <3rdParty/include/rs/core/video_module_interface.h>
#include <3rdParty/include/rs/core/projection_interface.h>
#include <opencv2/core/core.hpp>

class SmartSoft2RealSenseSdkConverter {
public:
	SmartSoft2RealSenseSdkConverter();
	virtual ~SmartSoft2RealSenseSdkConverter();
	/**
	 * @brief Create Realsense SDK image from image data
	 * @param image data
	 * @return RealSense sdk image, IMPORTANT shalow copy - image memory managed by ImageConstPtr
	 */
	rs::core::image_interface* CreateSdkImage(rs::core::stream_type streamType, uchar* framedata, rs::core::image_info &info);

	 /**
	   * @brief Create RealSense SDK sample set from images
	   * @param color image data
	   * @param depth image data
	   * @return RealSense SDK sample set, IMPORTANT images memory managed by ImageConstPtr & ImageConstPtr,
	   * sample set should be release with ReleaseSampleSet method
	   */
	  rs::core::correlated_sample_set CreateSdkSampleSet(uchar* color_framedata, rs::core::image_info &color_info,uchar* depth_framedata, rs::core::image_info &depth_info);


	  /**
	   * @brief Create RealSense SDK module config from camera parameters
	   * @param colorCameraInfo color camera info
	   * @param depthCameraInfo depth camera info
	   * @return RealSense SDK module config
	   */
	  rs::core::video_module_interface::actual_module_config CreateSdkModuleConfig(rs::core::intrinsics  &colorIntrinsics, rs::core::intrinsics  &depthIntrinsics,rs::core::extrinsics  &extrinsics);

};


#endif /* SMARTSOFT2REALSENSESDKCONVERTER_H_ */
