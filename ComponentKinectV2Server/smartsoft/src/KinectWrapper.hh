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

#ifndef KINECTWRAPPER_HH_
#define KINECTWRAPPER_HH_

#include <iostream>
#include <stdint.h>
#include <math.h>
#include <vector>

#include "ImageTaskCore.hh"
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>

#include "DomainVision/CommRGBDImage.hh"
#include "DomainVision/CommVideoImage.hh"
#include "DomainVision/CommDepthImage.hh"


class KinectWrapper {

private:
	std::string debug_msg_name;
	bool verbose;

	ulong image_counter;
	libfreenect2::SyncMultiFrameListener* listener;
	bool enable_rgb;
	bool enable_depth;
	bool enable_ir;
	std::string serial;
	long img_count = 0;

	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev;
	libfreenect2::Registration* registration;
	libfreenect2::FrameMap frames;
	libfreenect2::Freenect2Device::ColorCameraParams colorParams;
	libfreenect2::Freenect2Device::IrCameraParams irParams;
	libfreenect2::Frame *undistorted_depth_frame;
	libfreenect2::Frame *big_depth_frame;
	libfreenect2::Frame *registered_frame;

	bool read_camera_params;
	std::vector<double> rgb_intrinsics;
	std::vector<double> rgb_extrinsics;
	std::vector<double> depth_intrinsics;
	std::vector<double> depth_extrinsics;

	void printDebugMsg(std::string msgText);

	void printErrorMsg(std::string msgText);

	int initializeCam();

public:
	KinectWrapper();

	virtual ~KinectWrapper();

	void startVideo();
	void stopVideo();

	void getImage(DomainVision::CommRGBDImage& image);
	enum class Resolution
		{
			RES_1920_X_1080,
			RES_512_X_424
		};
	Resolution resolution;
	void set_resolution(const Resolution& res);

};

#endif /* KINECTWRAPPER_HH_ */
