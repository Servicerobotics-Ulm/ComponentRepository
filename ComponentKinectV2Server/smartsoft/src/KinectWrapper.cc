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
	read_camera_params = false;
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

	/* Device startup*/
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

		undistorted_depth_frame = new libfreenect2::Frame(512, 424, 4);
		big_depth_frame= new libfreenect2::Frame(1920, 1082, 4);
		registered_frame= new libfreenect2::Frame(512, 424, 4);

		printDebugMsg("Kinect device information:");
		printDebugMsg(std::string(" serial: ") + std::string(dev->getSerialNumber()));
		printDebugMsg(std::string(" firmware: ") + std::string(dev->getFirmwareVersion()));



	if (!dev->startStreams(enable_rgb, enable_depth)) {
			printErrorMsg("RGB, DEPTH streams failed to start!");
			return;
		}

	if(!read_camera_params)
		{

			rgb_intrinsics.resize(16, 0.0);
			rgb_extrinsics.resize(16, 0.0);

			depth_intrinsics.resize(16, 0.0);
			depth_extrinsics.resize(16, 0.0);

			irParams = dev->getIrCameraParams();
			colorParams = dev->getColorCameraParams();

			std::cout << "colorParams: "
					<<colorParams.cx << ", " <<colorParams.cy <<", "
					<<colorParams.fx << ", " <<colorParams.fy
					<<std::endl;
			std::cout << "irParams: "
					<<irParams.cx << ", " <<irParams.cy <<", "
					<<irParams.fx << ", " <<irParams.fy
					<<std::endl;

			printDebugMsg("Opening stream done!");


			rgb_intrinsics[0] = colorParams.fx;
			rgb_intrinsics[5] = colorParams.fy;
			rgb_intrinsics[2] = colorParams.cx;
			rgb_intrinsics[6] = colorParams.cy;
			rgb_intrinsics[10] = 1;
			rgb_intrinsics[15] = 1;

			depth_intrinsics[0] = colorParams.fx;
			depth_intrinsics[5] = colorParams.fy;
			depth_intrinsics[2] = colorParams.cx;
			depth_intrinsics[6] = colorParams.cy;
			depth_intrinsics[10] = 1;
			depth_intrinsics[15] = 1;


			//high resolution
			if (resolution == KinectWrapper::Resolution::RES_1920_X_1080){
				depth_intrinsics = rgb_intrinsics;

				//rotation matrix is identity matrix
				// translation is zero
				depth_extrinsics[0] = 1;
				depth_extrinsics[4] = 1;
				depth_extrinsics[8] = 1;
			}else{
				depth_intrinsics[0] = irParams.fx;
				depth_intrinsics[5] = irParams.fy;
				depth_intrinsics[2] = irParams.cx;
				depth_intrinsics[6] = irParams.cy;
				depth_intrinsics[10] = 1;
				depth_intrinsics[15] = 1;

				rgb_intrinsics = depth_intrinsics;

				depth_extrinsics[0] = 1;
				depth_extrinsics[4] = 1;
				depth_extrinsics[8] = 1;

			}

			read_camera_params = true;
		}


	registration = new libfreenect2::Registration(irParams, colorParams);
	std::cout << "[KinectWrapper] : kinect started streaming" <<std::endl;
}

void KinectWrapper::stopVideo() {
	dev->stop();
	std::cout << "[KinectWrapper] : stopping data processing.....\n";
	dev->close();
	delete registration;
	delete undistorted_depth_frame;
	delete big_depth_frame;
	delete registered_frame;
	delete listener;
	delete dev;

	std::cout << "[KinectWrapper] : kinect stopped streaming\n";
}

void KinectWrapper::getImage(DomainVision::CommRGBDImage& image) {

	// get new frames (index 1 = color frame; index 2 = ir frame; index 4 = depth frame)
	if (!listener->waitForNewFrame(frames, 10 * 1000)) { // 10 seconds
		printErrorMsg("No new frame, timeout!");
		listener->release(frames);
		return;
	}
	libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
	libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

	if (rgb->status != 0 || depth->status != 0) {
		listener->release(frames);
		printErrorMsg("Error in image data!");
		image.setIs_valid(false);
		return;
	}

	if (rgb->format != libfreenect2::Frame::BGRX && rgb->format != libfreenect2::Frame::RGBX) {
		listener->release(frames);
		printErrorMsg("received invalid frame format");
		image.setIs_valid(false);
		return;
	}


	//requested resolution is equal to original rgb frame size, prepare a big depth image mapped to rgb frame
	if(resolution == Resolution::RES_1920_X_1080)
	{
		//rgb, big_depth_frames are used in RGBD image with intrinsics from color
		std::cout << " High resolution" <<std::endl;
		registration->apply(rgb, depth, undistorted_depth_frame, registered_frame, true, big_depth_frame);

		std::cout << "rgb   : " <<rgb->width   << " x " << rgb->height << "\n";
		std::cout << "depth : " <<big_depth_frame->width << " x " << big_depth_frame->height << "\n";

		/////////////////////////////////// Prepare RGB Image //////////////////////////////////////////
		// rgb 1920x 1080, depth 1920x1082
		//mirror image vertically, because kinect driver delivers it wrong
		cv::Mat color = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
		cv::Mat tmp;
		cv::Mat flipped_color_image;
		cv::flip(color, tmp, 1);

		//convert to RGB format
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

		//todo set flag in ini
		//		bool save_images = false;
		//		if(save_images){
		//			img_count++;
		//			//todo set path in ini
		//			std::stringstream ss;
		//			ss << "/home/kinect_rgb_" << img_count << ".jpg";
		//			std::string path = ss.str();
		//
		//			imwrite(path, flipped_color_image );
		//		}

		DomainVision::CommVideoImage color_img;
		DomainVision::ImageParameters color_params;
		color_params.setWidth(rgb->width);
		color_params.setHeight(rgb->height);
		color_params.setFormat(DomainVision::FormatType::RGB24);
		color_params.setDepth(3*8); //Bits per pixel
		color_img.setParameter(color_params);
		color_img.set_parameters(rgb->width, rgb->height, DomainVision::FormatType::RGB24);

		color_img.set_data(flipped_color_image.data);
		color_img.setIntrinsic_m(rgb_intrinsics);
		image.setColor_image(color_img);

		/////////////////////////////////// Prepare Depth Image //////////////////////////////////////////


		int depth_img_width = big_depth_frame->width;
		int depth_img_height = big_depth_frame->height;

		//mirror big depth image vertically, because kinect driver delivers it wrong
		cv::Mat flipped_undist_depth_frame;
		cv::Mat tmp_undist_depth_mat(depth_img_height, depth_img_width, CV_32FC1, big_depth_frame->data);

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
		//const std::vector<unsigned char> depth_vector(*flipped_undist_depth_frame.data, *flipped_undist_depth_frame.dataend);
		assert (flipped_undist_depth_frame.data != NULL);
		const float* depth_data = reinterpret_cast<const float*>(flipped_undist_depth_frame.data);
		depth_img.set_distances(depth_data, (int)depth_img_width, (int)depth_img_height);

		depth_img.setIntrinsic_m(depth_intrinsics);
		depth_img.setExtrinsic_m(depth_extrinsics);

		depth_img.setMin_distcance(COMP->getGlobalState().getHardware_properties().getMin_distance());
		depth_img.setMax_distcance(COMP->getGlobalState().getHardware_properties().getMax_distance());

		image.setDepth_image(depth_img);
	}
	//requested resolution is equal to original depth frame size, prepare a rgb image mapped to depth frame
	else if(resolution == Resolution::RES_512_X_424)
	{
		//registered_frame, depth frame are used in RGBD image with intrinsics from depth
		registration->apply(rgb, depth, undistorted_depth_frame, registered_frame);
		std::cout << " Low resolution" <<std::endl;

		std::cout << "rgb   : " <<registered_frame->width   << " x " << registered_frame->height << "\n";
		std::cout << "depth : " <<depth->width << " x " << depth->height << "\n";

		/////////////////////////////////// Prepare RGB Image //////////////////////////////////////////
		// rgb 512x 424, depth 512x424
		//mirror image vertically, because kinect driver delivers it wrong
		cv::Mat color = cv::Mat(registered_frame->height, registered_frame->width, CV_8UC4, registered_frame->data);
		cv::Mat tmp;
		cv::Mat flipped_color_image;
		cv::flip(color, tmp, 1);

		//convert to RGB format
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

		DomainVision::CommVideoImage color_img;
		DomainVision::ImageParameters color_params;
		color_params.setWidth(registered_frame->width);
		color_params.setHeight(registered_frame->height);
		color_params.setFormat(DomainVision::FormatType::RGB24);
		color_params.setDepth(3*8); //Bits per pixel
		color_img.setParameter(color_params);
		color_img.set_parameters(registered_frame->width, registered_frame->height, DomainVision::FormatType::RGB24);

		color_img.set_data(flipped_color_image.data);
		color_img.setIntrinsic_m(rgb_intrinsics);
		image.setColor_image(color_img);

		//Since the color image after registration has black pixels
		//Prepare RGB image of size 512x424 from RGB frame of 1920X1080
		//this image is supplied only to rgb push port
		cv::Mat high_res_color_image = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);


		cv::Mat high_res_flipped_color_image;
		{
		cv::Mat tmp;
		cv::flip(high_res_color_image, tmp, 1);

		//convert to RGB format
		if (rgb->format == libfreenect2::Frame::BGRX){
			#ifdef WITH_OPENCV_4_2_VERSION
					cv::cvtColor(tmp, high_res_flipped_color_image, cv::COLOR_BGR2RGB);
			#else
					cv::cvtColor(tmp, high_res_flipped_color_image, CV_BGRA2RGB);
			#endif
				}else{
			#ifdef WITH_OPENCV_4_2_VERSION
					cv::cvtColor(tmp, high_res_flipped_color_image, cv::COLOR_RGBA2RGB);
			#else
					cv::cvtColor(tmp, high_res_flipped_color_image, CV_RGBA2RGB);
			#endif
		}
		}

		cv::Size tmp_size(753, 424); // size maintaining the aspect ratio(height same as depth image height) of high resolution RGB image size
		cv::Mat tmp_low_res_color_image;
		cv::resize(high_res_flipped_color_image, tmp_low_res_color_image, tmp_size);

		cv::Mat low_res_color_image = cv::Mat(tmp_low_res_color_image,cv::Rect(121,0,512,424)).clone();
		std::cout << "low_res_color_image : " << low_res_color_image.cols << " x " << low_res_color_image.rows <<std::endl;
		COMP->imageTask->set_low_res_rgb_image(low_res_color_image);



		/////////////////////////////////// Prepare Depth Image //////////////////////////////////////////
		int depth_img_width = depth->width;
		int depth_img_height = depth->height;

		//mirror big depth image vertically, because kinect driver delivers it wrong
		cv::Mat flipped_undist_depth_frame;
		//cv::Mat tmp_undist_depth_mat(depth_img_height, depth_img_width, CV_32FC1, depth->data);
		cv::Mat tmp_undist_depth_mat(depth_img_height, depth_img_width, CV_32FC1, undistorted_depth_frame->data);

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
		//const std::vector<unsigned char> depth_vector(*flipped_undist_depth_frame.data, *flipped_undist_depth_frame.dataend);
		assert (flipped_undist_depth_frame.data != NULL);
		const float* depth_data = reinterpret_cast<const float*>(flipped_undist_depth_frame.data);
		depth_img.set_distances(depth_data, (int)depth_img_width, (int)depth_img_height);

		depth_img.setIntrinsic_m(depth_intrinsics);
		depth_img.setExtrinsic_m(depth_extrinsics);

		depth_img.setMin_distcance(COMP->getGlobalState().getHardware_properties().getMin_distance());
		depth_img.setMax_distcance(COMP->getGlobalState().getHardware_properties().getMax_distance());

		image.setDepth_image(depth_img);

	}

	image_counter++;
	std::stringstream tmp_str_stream;
	tmp_str_stream << image_counter;
	printDebugMsg("Images received: " + tmp_str_stream.str());

	image.setSeq_count(image_counter);
	image.setIs_valid(true);

	listener->release(frames);
}

void KinectWrapper::printDebugMsg(std::string msgText) {
	if (verbose) {
		std::cout << debug_msg_name << ' ' << msgText << std::endl;
	}
}

void KinectWrapper::printErrorMsg(std::string msgText) {
	std::cerr << debug_msg_name << ' ' << msgText << std::endl;
}

void KinectWrapper::set_resolution(const KinectWrapper::Resolution& res)
{
	resolution = res;
}

