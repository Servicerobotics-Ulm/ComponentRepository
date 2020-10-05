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
//  Copyright (C) 2011, 2017 Matthias Lutz, Dennis Stampfer, Matthias Rollenhagen
//
//      lutz@hs-ulm.de
//      stampfer@hs-ulm.de
//      rollenhagen@hs-ulm.de
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
#ifndef _IMAGETASK_HH
#define _IMAGETASK_HH

#include "ImageTaskCore.hh"

#include <DomainVision/CommRGBDImage.hh>
#include <CommBasicObjects/CommPose3d.hh>
#include <CommBasicObjects/CommBaseState.hh>

#ifdef WITH_OPENCV_4_2_VERSION
#include <opencv4/opencv2/highgui.hpp>
#else
#include <highgui.h>
#endif
#include <opencv2/opencv.hpp>

#include <chrono>

class ImageTask  : public ImageTaskCore
{
public:
	ImageTask(SmartACE::SmartComponent *comp);
	virtual ~ImageTask();
	
	virtual int on_entry();
	virtual int on_execute();
	virtual int on_exit();

	void startCapturing();
	void stopCapturing();
	cv::Mat get_low_res_rgb_image();
	void set_low_res_rgb_image(const cv::Mat& in_image);

private:
	void visualization();
	unsigned int _ring_buffer_index;
	std::vector<DomainVision::CommRGBDImage*> _ring_buffer;
	std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> previous_time;
	cv::Mat low_res_rgb_image;
};

#endif
