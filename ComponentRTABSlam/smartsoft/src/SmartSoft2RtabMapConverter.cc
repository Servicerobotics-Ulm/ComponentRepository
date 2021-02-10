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


#include "SmartSoft2RtabMapConverter.hh"
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

rtabmap::CameraModel cameraModelFromSmartSoft(DomainVision::CommRGBDImage &scan, const rtabmap::Transform & localTransform)
{

//    DomainVision::Rs_intrinsics intrinsics = scan.getRs_color_intrinsics();
//    DomainVision::Rs_extrinsics extrinsics = scan.getRs_extrinsics();

	DomainVision::CommVideoImage comm_rgb_image = scan.getColor_image();
	DomainVision::CommDepthImage comm_depth_image = scan.getDepth_image();

	st_intrinsics color_intrinsics, depth_intrinsics;
	st_extrinsics depth_to_color_extrinsics;


	// fill color intrinsics from CommRGBDImage
	arma::mat comm_color_intrinsic = arma::zeros(4,4);

	comm_color_intrinsic = comm_rgb_image.get_intrinsic();

	color_intrinsics.fx = comm_color_intrinsic(0,0);
	color_intrinsics.fy = comm_color_intrinsic(1,1);
	color_intrinsics.cx = comm_color_intrinsic(0,2);
	color_intrinsics.cy = comm_color_intrinsic(1,2);
	color_intrinsics.cols = comm_rgb_image.getParameter().width;
	color_intrinsics.rows= comm_rgb_image.getParameter().height;

//	DomainVision::ImageDistortionModel color_distortion_model = comm_rgb_image.getDistortion_model();
//
//	if(color_distortion_model == DomainVision::ImageDistortionModel::BROWN_CONRADY)
//		color_intrinsics.distortion_model = rs::core::distortion_type::modified_brown_conrady;
//	else
//		color_intrinsics.distortion_model = rs::core::distortion_type::none;
//
//	arma::mat comm_color_distortion = arma::zeros(1,5);
//
//	comm_color_distortion = comm_rgb_image.get_distortion();
//	color_intrinsics.distortion_coeffs[0] = comm_color_distortion(0,0);
//	color_intrinsics.distortion_coeffs[1] = comm_color_distortion(0,1);
//	color_intrinsics.distortion_coeffs[2] = comm_color_distortion(0,2);
//	color_intrinsics.distortion_coeffs[3] = comm_color_distortion(0,3);
//	color_intrinsics.distortion_coeffs[4] = comm_color_distortion(0,4);





//    float distort_coeffs[5];
//
//    scan.get_rs_distortion_coeffs(&distort_coeffs[0]);
//
//	// K is the camera intrinsic 3x3 CV_64FC1
//	cv::Mat K = cv::Mat(3, 3, CV_64FC1);
//
//	K.at<double>(0,0) = (double)intrinsics.getFx(); 	//0
//	K.at<double>(0,1) = 0.0;							//1
//	K.at<double>(0,2) = (double)intrinsics.getPpx();	//2
//
//	K.at<double>(1,0) = 0.0;							//3
//	K.at<double>(1,1) = (double)intrinsics.getFy();		//4
//	K.at<double>(1,2) = (double)intrinsics.getPpy();	//5
//
//	K.at<double>(2,0) = 0.0;							//6
//	K.at<double>(2,1) = 0.0;							//7
//	K.at<double>(2,2) = 1;								//8
//
//	/*---------------------------------------------------------------------------------------------------------------*/
//
//	// D is the distortion coefficients 1x5 CV_64FC1
//	cv::Mat D= cv::Mat(1, 5, CV_64FC1, &distort_coeffs);
//	//cv::Mat D= cv::Mat::zeros(1,5, CV_64FC1);
//
//	D.at<double>(0,0) = (double)distort_coeffs[0];
//	D.at<double>(0,1) = (double)distort_coeffs[1];
//	D.at<double>(0,2) = (double)distort_coeffs[2];
//	D.at<double>(0,3) = (double)distort_coeffs[3];
//	D.at<double>(0,4) = (double)distort_coeffs[4];
//
//	/*---------------------------------------------------------------------------------------------------------------*/
//	// R is the rectification matrix 3x3 CV_64FC1 (computed from stereo or Identity)
//	//cv::Mat R = cv::Mat(3, 3, CV_64FC1);
//	double r_array[9] ={0, 0, 0, 0, 0, 0, 0, 0, 0};
//	cv::Mat R= cv::Mat(3, 3, CV_64FC1, &r_array);
//
//	R.at<double>(0,0) = 1.0; 							//0
//	R.at<double>(0,1) = 0.0;							//1
//	R.at<double>(0,2) = 0.0;							//2
//
//	R.at<double>(1,0) = 0.0;							//3
//	R.at<double>(1,1) = 1.0;							//4
//	R.at<double>(1,2) = 0.0;							//5
//
//	R.at<double>(2,0) = 0.0;							//6
//	R.at<double>(2,1) = 0.0;							//7
//	R.at<double>(2,2) = 1.0;							//8
//
//
//	/*---------------------------------------------------------------------------------------------------------------*/
//	// P is the projection matrix 3x4 CV_64FC1 (computed from stereo or equal to [K [0 0 1]'])
//	//cv::Mat P = cv::Mat(3, 4, CV_64FC1);
//	double p_array[12] ={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//	cv::Mat P= cv::Mat(3, 4, CV_64FC1, &p_array);
//
//	P.at<double>(0,0) = (double)intrinsics.getFx(); 	//0
//	P.at<double>(0,1) = 0.0;							//1
//	P.at<double>(0,2) = (double)intrinsics.getPpx();	//2
//	P.at<double>(0,3) = 0.0;							//3
//
//	P.at<double>(1,0) = 0.0; 							//4
//	P.at<double>(1,1) = (double)intrinsics.getFy();		//5
//	P.at<double>(1,2) = (double)intrinsics.getPpy();	//6
//	P.at<double>(1,3) = 0.0;							//7
//
//	P.at<double>(2,0) = 0.0; 							//8
//	P.at<double>(2,1) = 0.0;							//9
//	P.at<double>(2,2) = 1.0;							//10
//	P.at<double>(2,3) = 0.0;							//11


//	return rtabmap::CameraModel(
//				"smartsoft",
//				cv::Size(scan.getRgb_width(), scan.getRgb_height()),
//				K, D, R, P,
//				localTransform);


	return rtabmap::CameraModel(
			color_intrinsics.fx, //fx
			color_intrinsics.fx, //fy
			color_intrinsics.cx,  //cx
			color_intrinsics.cy,  //cy
			localTransform,
			0,
			cv::Size(color_intrinsics.cols, color_intrinsics.rows));
}


rtabmap::Transform transformFromSmartSoftCommPose3d( const CommBasicObjects::CommPose3d &base_pose)
{


	double base_yaw 	= base_pose.get_azimuth(),
		   base_pitch   = base_pose.get_elevation(),
		   base_roll 	= base_pose.get_roll();

	double base_x 	= base_pose.getPosition().getX() / 1000,
		   base_y 	= base_pose.getPosition().getY() / 1000,
		   base_z 	= base_pose.getPosition().getZ() / 1000;

	Eigen::Affine3f t = pcl::getTransformation (base_x, base_y, base_z, base_roll, base_pitch, base_yaw);

	rtabmap::Transform transform = rtabmap::Transform(t(0,0), t(0,1), t(0,2), t(0,3),
													  t(1,0), t(1,1), t(1,2), t(1,3),
													  t(2,0), t(2,1), t(2,2), t(2,3));
	return transform;
}


rtabmap::Transform transformFromSmartSoftBasePose( const CommBasicObjects::CommBasePose &base_pose)
{
	{


		double base_yaw 	= base_pose.get_base_azimuth(),
			   base_pitch   = base_pose.get_base_elevation(),
			   base_roll 	= base_pose.get_base_elevation();

		double base_x 	= base_pose.get_x() / 1000,
			   base_y 	= base_pose.get_y() / 1000,
			   base_z 	= base_pose.get_z() / 1000;



		rtabmap::Transform transform = rtabmap::Transform(base_x, base_y, base_z, base_roll, base_pitch, base_yaw);

		return transform;
	}
}

