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

#include "KinectWrapper.hh"

#include "ComponentKinectV2Server.hh"

#include <opencv2/opencv.hpp>

KinectWrapper::KinectWrapper() {
	debug_msg_name = "[Kinect Wrapper]";
	verbose = true;

	image_counter = 0;
	enable_rgb = true;
	enable_depth = true;
	enable_ir = false;
	serial = "";
	dev = 0;
}

KinectWrapper::~KinectWrapper() {
}

/*
 * Initialize connection to camera
 */
int KinectWrapper::initializeCam() {
	printDebugMsg("Initializing Kinect...");

	// discover the device
	if (freenect2.enumerateDevices() == 0) {
		printErrorMsg("No device connected!");
		return -1;
	}

	// connect to device
	serial = freenect2.getDefaultDeviceSerialNumber();
	dev = freenect2.openDevice(serial);

	if (dev == 0) {
		printErrorMsg("Failure opening device!");
		return -1;
	}

	printDebugMsg("Initializing done!");

	return 0;
}

void KinectWrapper::startVideo() {
	if (initializeCam() != 0) {
		return;
	}

	// define which kind of data (listeners) should be requested (RGB, IR and depth)
	int listener_types = 0;
	if (enable_rgb)
		listener_types |= libfreenect2::Frame::Color;
	if (enable_depth)
		listener_types |= libfreenect2::Frame::Depth;
	if (enable_ir)
		listener_types |= libfreenect2::Frame::Ir;

	listener = new libfreenect2::SyncMultiFrameListener(listener_types);

	dev->setColorFrameListener(listener);
	dev->setIrAndDepthFrameListener(listener);

	printDebugMsg("Opening video stream...");

	// start streaming
	if (!dev->startStreams(enable_rgb, enable_depth)) {
		printErrorMsg("Video stream can not be opened!");
		return;
	}

	printDebugMsg("Opening stream done!");

	printDebugMsg("Kinect device information:");
	printDebugMsg(std::string(" serial: ") + std::string(dev->getSerialNumber()));
	printDebugMsg(std::string(" firmware: ") + std::string(dev->getFirmwareVersion()));
}

void KinectWrapper::stopVideo() {
	dev->stop();
	dev->close();
	delete dev;
	delete listener;
}

void KinectWrapper::getImage(DomainVision::CommRGBDImage& image) {
	libfreenect2::FrameMap frames;
	libfreenect2::Freenect2Device::ColorCameraParams colorParams;
	libfreenect2::Freenect2Device::IrCameraParams irParams;

	colorParams = dev->getColorCameraParams();

	irParams = dev->getIrCameraParams();

	// get new frames (index 1 = color frame; index 2 = ir frame; index 4 = depth frame)
	if (!listener->waitForNewFrame(frames, 10 * 1000)) { // 10 seconds
		printErrorMsg("No new frame, timeout!");
		return;
	}

	const libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
	libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

	if (rgb->status != 0 || depth->status != 0) {
		printErrorMsg("Error in image data!");
		image.setIs_valid(false);
		return;
	}

	if (rgb->format != libfreenect2::Frame::BGRX && rgb->format != libfreenect2::Frame::RGBX) {
		printErrorMsg("received invalid frame format");
		image.setIs_valid(false);
		return;
	}

	//mirror image vertically, because kinect driver delivers it wrong
	cv::Mat color = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
	cv::Mat tmp;
	cv::Mat flipped_color_image;
	cv::flip(color, tmp, 1);




	//reduce size
	if (rgb->format == libfreenect2::Frame::BGRX){
#ifdef WITH_OPENCV_4_2_VERSION
		cv::cvtColor(tmp, flipped_color_image, cv::COLOR_BGR2RGB);
#else
		cv::cvtColor(tmp, flipped_color_image, CV_BGRA2RGB);
#endif
	}else{
#ifdef WITH_OPENCV_4_2_VERSION
		cv::cvtColor(tmp, flipped_color_image, cv::COLOR_RGBA2RGB);
#else
		cv::cvtColor(tmp, flipped_color_image, CV_RGBA2RGB);
#endif
	}

//	if (rgb->format == libfreenect2::Frame::BGRX) {
//		cv::cvtColor(tmp, flipped_color_image, CV_RGBA2RGB);
//	} else {
//		cv::cvtColor(tmp, flipped_color_image, CV_RGBA2RGB);
//	}

	//todo set flag in ini
	bool save_images = false;
	if(save_images){
		img_count++;
		//todo set path in ini
		std::stringstream ss;
		ss << "/home/kinect_rgb_" << img_count << ".jpg";
		std::string path = ss.str();

		imwrite(path, flipped_color_image );
	}


	DomainVision::CommVideoImage color_img;
	DomainVision::ImageParameters color_params;
	color_params.setWidth(rgb->width);
	color_params.setHeight(rgb->height);
	color_params.setFormat(DomainVision::FormatType::RGB24);
	color_params.setDepth(3*8); //Bits per pixel
	color_img.setParameter(color_params);
	color_img.set_parameters(rgb->width, rgb->height, DomainVision::FormatType::RGB24);
	//const std::vector<unsigned char> color_vector(*flipped_color_image.data, *flipped_color_image.dataend);
	color_img.set_data(flipped_color_image.data);



	std::vector<double> rgb_intrinsic(16, 0.0);
	rgb_intrinsic[0] = dev->getColorCameraParams().fx;
	rgb_intrinsic[5] = dev->getColorCameraParams().fy;
	rgb_intrinsic[2] = dev->getColorCameraParams().cx;
	rgb_intrinsic[6] = dev->getColorCameraParams().cy;
	rgb_intrinsic[10] = 1;
	rgb_intrinsic[15] = 1;
	color_img.setIntrinsic_m(rgb_intrinsic);

	image.setColor_image(color_img);


	// registration setup
	// registration combines frames of depth and color camera
	libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	libfreenect2::Frame undistorted_depth_frame(512, 424, 4);
	// big depth frame has one empty line at the top and one at the bottom
	libfreenect2::Frame big_depth_frame(1920, 1082, 4);
	libfreenect2::Frame registered_frame(512, 424, 4);
	int color_depth_map;

	// create RGB image with depth information
	registration->apply(rgb, depth, &undistorted_depth_frame, &registered_frame, true, &big_depth_frame);

	int depth_img_width = big_depth_frame.width;
	int depth_img_height = big_depth_frame.height;

	//mirror big depth image vertically, because kinect driver delivers it wrong
	cv::Mat flipped_undist_depth_frame;
	cv::Mat tmp_undist_depth_mat(depth_img_height, depth_img_width, CV_32FC1, big_depth_frame.data);
	//cv::Mat tmp_undist_depth_mat(depth_img_height, depth_img_width, CV_32FC1, undistorted_depth_frame.data);
	cv::flip(tmp_undist_depth_mat, flipped_undist_depth_frame, 1);

	//convert distance values from millimeter into meter
	flipped_undist_depth_frame = flipped_undist_depth_frame * 0.001;

	// set undistorted depth image to communication object
	DomainVision::CommDepthImage depth_img;
	depth_img.setWidth(depth_img_width);
	depth_img.setHeight(depth_img_height);
	depth_img.setFormat(DomainVision::DepthFormatType::FLOAT);
	depth_img.setPixel_size(32);
	//create vector with starting pointer and end pointer
	const std::vector<unsigned char> depth_vector(*flipped_undist_depth_frame.data, *flipped_undist_depth_frame.dataend);
	assert (flipped_undist_depth_frame.data != NULL);
	const float* depth_data = reinterpret_cast<const float*>(flipped_undist_depth_frame.data);
	depth_img.set_distances(depth_data, (int)depth_img_width, (int)depth_img_height);

	std::vector<double> depth_intrinsic(16, 0.0);
	std::vector<double> depth_extrinsic(12, 0.0);

	//if upscaled depth image is used the rgb intrinsics must be used
	//for point point calculations
	if (depth_img.getWidth() == big_depth_frame.width){
		depth_intrinsic = rgb_intrinsic;

		//rotation matrix is identity matrix
		// translation is zero
		depth_extrinsic[0] = 1;
		depth_extrinsic[4] = 1;
		depth_extrinsic[8] = 1;
	}else{
		depth_intrinsic[0] = dev->getIrCameraParams().fx;
		depth_intrinsic[5] = dev->getIrCameraParams().fy;
		depth_intrinsic[2] = dev->getIrCameraParams().cx;
		depth_intrinsic[6] = dev->getIrCameraParams().cy;
		depth_intrinsic[10] = 1;
		depth_intrinsic[15] = 1;
	}

	depth_img.setIntrinsic_m(depth_intrinsic);
	depth_img.setExtrinsic_m(depth_extrinsic);

	depth_img.setMin_distcance(COMP->getGlobalState().getHardware_properties().getMin_distance());
	depth_img.setMax_distcance(COMP->getGlobalState().getHardware_properties().getMax_distance());

	image.setDepth_image(depth_img);


	image_counter++;
	std::stringstream tmp_str_stream;
	tmp_str_stream << image_counter;
	printDebugMsg("Images received: " + tmp_str_stream.str());

	image.setSeq_count(image_counter);
	image.setIs_valid(true);

	listener->release(frames);

	delete registration;
}

void KinectWrapper::printDebugMsg(std::string msgText) {
	if (verbose) {
		std::cout << debug_msg_name << ' ' << msgText << std::endl;
	}
}

void KinectWrapper::printErrorMsg(std::string msgText) {
	std::cerr << debug_msg_name << ' ' << msgText << std::endl;
}

