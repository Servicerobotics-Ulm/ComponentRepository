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

	void printDebugMsg(std::string msgText);

	void printErrorMsg(std::string msgText);

	int initializeCam();

	void calcPointXYZ (const libfreenect2::Frame *depth_frame, int r, int c, float &x, float &y, float &z);

public:
	KinectWrapper();

	virtual ~KinectWrapper();

	void startVideo();

	void startDepth() {
	}

	void stopVideo();

	void stopDepth() {

	}

	void getImage(DomainVision::CommRGBDImage& image);

};

#endif /* KINECTWRAPPER_HH_ */
