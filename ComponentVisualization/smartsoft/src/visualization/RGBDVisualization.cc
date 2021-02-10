/*--------------------------------------------------------------------------

 Copyright (C) 2017

 Created on: Oct 27, 2017
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

#include "RGBDVisualization.hh"
#include <mrpt/opengl.h>
#include <mrpt/opengl/CFrustum.h>
#include <mrpt/opengl/CAxis.h>


using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::math;

#ifdef WITH_MRPT_2_0_VERSION
#else
using namespace mrpt::utils;
#include <mrpt/utils/adapters.h>
#endif

//#include <cmath>
#include <ctime>
using namespace std;

RGBDVisualization::RGBDVisualization(CDisplayWindow3D& window3D, const std::string& identifier):
AbstractVisualization(window3D, identifier)
{
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr &ptrScene = window3D.get3DSceneAndLock();
		{
			// point cloud
			opengl::CPointCloudColoured::Ptr cloud = opengl::CPointCloudColoured::Create();
			cloud->setName(identifier + "_cloud");
			cloud->setPointSize(2.0);
			ptrScene->insert(cloud);

			//camera origin
			mrpt::opengl::CSetOfObjects::Ptr gl_corner = mrpt::opengl::stock_objects::CornerXYZSimple(0.25);
			gl_corner->setName(identifier + "_camera_origin");
			gl_corner->setScale(0.5);
			ptrScene->insert(gl_corner);

		}
#else
	opengl::COpenGLScenePtr &ptrScene = window3D.get3DSceneAndLock();
		{
			// point cloud
			opengl::CPointCloudColouredPtr cloud = opengl::CPointCloudColoured::Create();
			cloud->setName(identifier + "_cloud");
			cloud->setPointSize(2.0);
			ptrScene->insert(cloud);

			//camera origin
			mrpt::opengl::CSetOfObjectsPtr gl_corner = mrpt::opengl::stock_objects::CornerXYZSimple(0.25);
			gl_corner->setName(identifier + "_camera_origin");
			gl_corner->setScale(0.5);
			ptrScene->insert(gl_corner);

		}
#endif
		window3D.unlockAccess3DScene();

		first_image_flag = true;

}

RGBDVisualization::~RGBDVisualization() {
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr & ptrScene = window3D.get3DSceneAndLock();
		{
		    // remove pointcloud
			opengl::CPointCloudColoured::Ptr cloud = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + "_cloud"));
			ptrScene->removeObject(cloud);

			// remove origin
			mrpt::opengl::CSetOfObjects::Ptr gl_corner = std::dynamic_pointer_cast<opengl::CSetOfObjects>(ptrScene->getByName(identifier + "_camera_origin"));
			ptrScene->removeObject(gl_corner);

		}
#else
	opengl::COpenGLScenePtr & ptrScene = window3D.get3DSceneAndLock();
		{
		    // remove pointcloud
			opengl::CPointCloudColouredPtr cloud = (opengl::CPointCloudColouredPtr) ptrScene->getByName(identifier + "_cloud");
			ptrScene->removeObject(cloud);

			// remove origin
			mrpt::opengl::CSetOfObjectsPtr gl_corner = (opengl::CSetOfObjectsPtr) ptrScene->getByName(identifier + "_camera_origin");
			ptrScene->removeObject(gl_corner);

		}
#endif
		window3D.unlockAccess3DScene();
		window3D.forceRepaint();
}

void RGBDVisualization::displayImage(DomainVision::CommRGBDImage& rgbd_image)
{

	if (rgbd_image.getIs_valid())
	{

		if(first_image_flag)
			read_intrinsics_extrinsics(rgbd_image);

		double depth_frame_x, depth_frame_y, depth_frame_z;
		double color_frame_x, color_frame_y, color_frame_z;
		float r = 0, g = 0, b = 0;
		std::vector<ColPoint3d> vec_points;

		//const uint8_t* rgbImg = image.get_rgb_image();
		DomainVision::CommVideoImage comm_color_image = rgbd_image.getColor_image();
		DomainVision::CommDepthImage comm_depth_image = rgbd_image.getDepth_image();

		CommBasicObjects::CommPose3d sensor_pose = rgbd_image.getSensor_pose();

		double sensor_yaw = sensor_pose.get_azimuth(), sensor_pitch = sensor_pose.get_elevation(), sensor_roll = sensor_pose.get_roll();
		double sensor_x = sensor_pose.getPosition().getX() / 1000, sensor_y = sensor_pose.getPosition().getY() / 1000, sensor_z = sensor_pose.getPosition().getZ() / 1000;

		mrpt::poses::CPose3D mrpt_sensorPose(sensor_x, sensor_y, sensor_z,sensor_yaw, sensor_pitch, sensor_roll);
		//mrpt::poses::CPose3D mrpt_sensor_pose =
		createColorPointCloud (vec_points, &comm_color_image, &comm_depth_image);
#ifdef WITH_MRPT_2_0_VERSION
		opengl::COpenGLScene::Ptr & ptrScene = window3D.get3DSceneAndLock();
		//////////////////////////////////////////
		// show pointcloud
		{
			opengl::CPointCloudColoured::Ptr cloud = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + "_cloud"));
			cloud->clear();
			cloud->setPose(mrpt_sensorPose);
			//set points to cloud
			for(const ColPoint3d current_pont : vec_points)
			{
				cloud->push_back(current_pont.x, current_pont.y, current_pont.z, current_pont.r, current_pont.g, current_pont.b);
			}
			mrpt::opengl::CSetOfObjects::Ptr gl_origin = std::dynamic_pointer_cast<opengl::CSetOfObjects>(ptrScene->getByName(identifier + "_camera_origin"));
			gl_origin->setPose(mrpt_sensorPose);
		}
#else
		opengl::COpenGLScenePtr & ptrScene = window3D.get3DSceneAndLock();
		//////////////////////////////////////////
		// show pointcloud
		{
			opengl::CPointCloudColouredPtr cloud = (opengl::CPointCloudColouredPtr) ptrScene->getByName(identifier + "_cloud");
			cloud->clear();
			cloud->setPose(mrpt_sensorPose);

			for(const ColPoint3d current_pont : vec_points)
			{
				cloud->push_back(current_pont.x, current_pont.y, current_pont.z, current_pont.r, current_pont.g, current_pont.b);
			}

			mrpt::opengl::CSetOfObjectsPtr gl_origin = (mrpt::opengl::CSetOfObjectsPtr) ptrScene->getByName(identifier + "_camera_origin");
			gl_origin->setPose(mrpt_sensorPose);

		}
#endif

		// Frustum
//		{
//
//			//mrpt::opengl::CFrustum::Ptr gl_frustum = mrpt::make_aligned_shared<mrpt::opengl::CFrustum>(0.2f, 5.0f, 90.0f, 5.0f, 2.0f, true, true);
//			mrpt::opengl::CFrustumPtr gl_frustum = mrpt::opengl::CFrustum::Create(0.2f, 5.0f, 90.0f, 5.0f, 2.0f, true, false);
//
//			//mrpt::opengl::CFrustum gl_frustum(0.5, 4.5, 60,5, 1, true,true);
//			//gl_frustum->setPose(mrpt_sensorPose);
//			ptrScene->insert(gl_frustum);
//		}




		window3D.unlockAccess3DScene();
		window3D.forceRepaint();
	}
}

void RGBDVisualization::clear()
{
#ifdef WITH_MRPT_2_0_VERSION
	opengl::COpenGLScene::Ptr & ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudColoured::Ptr cloud = std::dynamic_pointer_cast<opengl::CPointCloudColoured>(ptrScene->getByName(identifier + "_cloud"));
		cloud->clear();
	}
#else
	opengl::COpenGLScenePtr & ptrScene = window3D.get3DSceneAndLock();
	{
		opengl::CPointCloudColouredPtr cloud = (opengl::CPointCloudColouredPtr) ptrScene->getByName(identifier + "_cloud");
		cloud->clear();
	}
#endif
	window3D.unlockAccess3DScene();
	window3D.forceRepaint();
}
/*Convert image pixel to real world point
 * equations and functions are similar to realsense library
 * https://github.com/IntelRealSense/librealsense/blob/master/include/librealsense2/rsutil.h
 *
 * */
void RGBDVisualization::deproject(const st_intrinsics& intrinsics, const uint32_t& r, const uint32_t& c, const float &depth_val_meters, float &out_x, float &out_y, float &out_z)
{
	float x = (c - intrinsics.cx) / intrinsics.fx;
	float y = (r - intrinsics.cy) / intrinsics.fy;
	if(intrinsics.distortion_model == DomainVision::ImageDistortionModel::INVERSE_BROWN_CONRADY)//RS2_DISTORTION_INVERSE_BROWN_CONRADY
	{
		float r2  = x*x + y*y;
		float f =    1 + intrinsics.distortion_coeffs[0]*r2    + intrinsics.distortion_coeffs[1]*r2*r2 + intrinsics.distortion_coeffs[4]*r2*r2*r2;
		float ux = x*f + 2*intrinsics.distortion_coeffs[2]*x*y + intrinsics.distortion_coeffs[3]*(r2 + 2*x*x);
		float uy = y*f + 2*intrinsics.distortion_coeffs[3]*x*y + intrinsics.distortion_coeffs[2]*(r2 + 2*y*y);
		x = ux;
		y = uy;
		//std::cout << "distortion in de projection" <<std::endl;
	}
	out_x = depth_val_meters * x;
	out_y = depth_val_meters * y;
	out_z = depth_val_meters;

}
/*Convert real world point to image pixel*/
void RGBDVisualization::project(const st_intrinsics& intrinsics, uint32_t& out_r, uint32_t& out_c, const float &in_x, const float &in_y, const float &in_z)
{

	float x = in_x/in_z;
	float y = in_y/in_z;


	if(intrinsics.distortion_model == DomainVision::ImageDistortionModel::MODIFIED_BROWN_CONRADY) //RS2_DISTORTION_MODIFIED_BROWN_CONRADY
	{
		float r2  = x*x + y*y;
		float f = 1 + intrinsics.distortion_coeffs[0]*r2 + intrinsics.distortion_coeffs[1]*r2*r2 + intrinsics.distortion_coeffs[4]*r2*r2*r2;
		x *= f;
		y *= f;
		float dx = x + 2*intrinsics.distortion_coeffs[2]*x*y + intrinsics.distortion_coeffs[3]*(r2 + 2*x*x);
		float dy = y + 2*intrinsics.distortion_coeffs[3]*x*y + intrinsics.distortion_coeffs[2]*(r2 + 2*y*y);
		x = dx;
		y = dy;
	}

	out_c =  x * intrinsics.fx + intrinsics.cx;
    out_r =  y * intrinsics.fy + intrinsics.cy;
}

void RGBDVisualization::transform (const st_extrinsics& extrinsics, float &x, float &y, float &z)
{

	float from_point_x = x;
	float from_point_y = y;
	float from_point_z = z;

	float to_point_x = extrinsics.rotation[0] * from_point_x + extrinsics.rotation[3] * from_point_y + extrinsics.rotation[6] * from_point_z + extrinsics.translation[0];
	float to_point_y = extrinsics.rotation[1] * from_point_x + extrinsics.rotation[4] * from_point_y + extrinsics.rotation[7] * from_point_z + extrinsics.translation[1];
	float to_point_z = extrinsics.rotation[2] * from_point_x + extrinsics.rotation[5] * from_point_y + extrinsics.rotation[8] * from_point_z + extrinsics.translation[2];

	x= to_point_x;
	y= to_point_y;
	z= to_point_z;

	//std::cout <<"from: " <<from_point_x <<", "<<from_point_y <<", "<<from_point_z <<", "<<"   To: " <<to_point_x <<", "<<to_point_y <<", "<<to_point_z <<", "<<std::endl;

}

void RGBDVisualization::calcPointXYZ (const uint32_t& r, const uint32_t& c, const float &depth_val_meters, float &x, float &y, float &z,
		                             const st_intrinsics& intrinsics, const st_extrinsics& extrinsics)
{

  //depth value is not valid
  if(isnan(depth_val_meters) || depth_val_meters <= 0.001)
  {
    //depth value is not valid
	const float bad_point = std::numeric_limits<float>::quiet_NaN();
    x = y = z = bad_point;
  }
  else
  {
	  // find corresponding x,y,z of give depth pixel and depth value
	  deproject(intrinsics,r, c, depth_val_meters, x, y, z);
	  // transform x,y,z into rgb coordinate system using extrinsics
	  transform(extrinsics, x, y, z);


  }
}
#ifdef WITH_MRPT_2_0_VERSION
void RGBDVisualization::createColorPointCloud (std::vector<ColPoint3d>& points, DomainVision::CommVideoImage *comm_color_image,
		                                                                       DomainVision::CommDepthImage *comm_depth_image)
#else
void RGBDVisualization::createColorPointCloud (std::vector<ColPoint3d>& points, DomainVision::CommVideoImage *comm_color_image,
		                                                                       DomainVision::CommDepthImage *comm_depth_image)
#endif
{
	// get rgb data
	const uint8_t *color_imageData = comm_color_image->get_data();

	// get depth data
	DomainVision::DepthFormatType depth_format = comm_depth_image->getFormat();
	uint32_t depth_width 							= comm_depth_image->getWidth();
	uint32_t depth_height 							= comm_depth_image->getHeight();

	const uint16_t* depth_data_uint16;
	const float* depth_data_float;
	if(depth_format==DomainVision::DepthFormatType::UINT16)
	{
		depth_data_uint16 = comm_depth_image->get_distances_uint16();
	}else if (depth_format==DomainVision::DepthFormatType::FLOAT)
	{
		depth_data_float = comm_depth_image->get_distances_float();
	}

	float x_in_m, y_in_m, z_in_m;

	for (uint32_t depth_row = 0; depth_row < depth_height ; ++depth_row){//along y
//		if(depth_row%2==0)
//			continue;
		for (uint32_t depth_col = 0; depth_col < depth_width;++depth_col){//along x-axis
//			if(depth_col%2==0)
//						continue;

			float depth_meters =0.0;
			if(depth_format==DomainVision::DepthFormatType::UINT16)
			{
				const uint16_t depth_val_ptr = depth_data_uint16[depth_row*depth_width+depth_col];

				depth_meters = depth_val_ptr/1000.0f;
			}else if(depth_format==DomainVision::DepthFormatType::FLOAT)
			{
				const float depth_val_ptr = depth_data_float[depth_width*depth_row+depth_col];

				depth_meters= depth_val_ptr;
				//std::cout << " depth = " <<depth_meters<<std::endl;

			}else
			{
				std::cout << "Unknow Depth Format" <<std::endl;
				std::abort();
			}
            //find x, y, z for given depth pixel in rgb frame
			calcPointXYZ (depth_row, depth_col, depth_meters, x_in_m, y_in_m, z_in_m, depth_intrinsics, depth_to_color_extrinsics);


			if(isinf(x_in_m) || isinf(z_in_m) || isinf(y_in_m)){
				x_in_m = 10.0;
				y_in_m = 10.0;
				z_in_m = 10.0;
			}
			if(x_in_m != x_in_m || y_in_m != y_in_m || z_in_m != z_in_m){
				x_in_m = 10.0;
				y_in_m = 10.0;
				z_in_m = 10.0;
			}


            //find color pixel for the given x, y, z
			uint32_t rgb_pixel_row = depth_row;
			uint32_t rgb_pixel_col = depth_col;
			project(color_intrinsics, rgb_pixel_row, rgb_pixel_col, x_in_m, y_in_m, z_in_m);


			uint8_t r, g,b;

			// if point not projected to the picture fill some color
			if(rgb_pixel_row<0 || rgb_pixel_row > color_intrinsics.rows || rgb_pixel_col < 0 || rgb_pixel_col > color_intrinsics.cols )
			{
	             r = g = b = 255;
			}
			else
			{
				const uint8_t* pixel = color_imageData + rgb_pixel_row * 3 * color_intrinsics.cols + rgb_pixel_col * 3;
				r = pixel[0];
				g = pixel[1];
				b = pixel[2];
			}

			//cloud->push_back(z_in_m, x_in_m * (-1), y_in_m * (-1), r/255.0, g/255.0, b/255.0);
			//cloud->push_back(x_in_m, y_in_m, z_in_m, r/255.0, g/255.0, b/255.0);
			points.push_back(ColPoint3d(x_in_m, y_in_m, z_in_m, r/255.0, g/255.0, b/255.0));
			//float range = sqrt((x_in_m*x_in_m)+(y_in_m*y_in_m)+(z_in_m*z_in_m));
			//if(range>0.5&&range<4.0)
			//cloud->push_back(x_in_m, y_in_m , z_in_m, r/255.0, g/255.0, b/255.0);
			//else
			//	std::cout << "range  = "<<range<<std::endl;
		}


	}

//	for (int i =0; i<1; i++)
//	{
//		z_in_m = 1;
//		x_in_m = 0;
//		y_in_m = 0;
//
//
//		//std::cout << "Z = " <<z_in_m << "  i =" <<i <<std::endl;
//		cloud->push_back(x_in_m, y_in_m, z_in_m, r/255.0, g/255.0, b/255.0);
//
//	}



}
void RGBDVisualization::read_intrinsics_extrinsics(const DomainVision::CommRGBDImage& rgbd_image)
{
	// get color intrinsics
	{
	DomainVision::CommVideoImage comm_rgb_image = rgbd_image.getColor_image();
	arma::mat comm_color_intrinsic = arma::zeros(4,4);
	comm_color_intrinsic = comm_rgb_image.get_intrinsic();

	color_intrinsics.fx 	= comm_color_intrinsic(0,0);
	color_intrinsics.fy 	= comm_color_intrinsic(1,1);
	color_intrinsics.cx 	= comm_color_intrinsic(0,2);
	color_intrinsics.cy 	= comm_color_intrinsic(1,2);
	color_intrinsics.cols 	= comm_rgb_image.getParameter().width;
	color_intrinsics.rows 	= comm_rgb_image.getParameter().height;
	color_intrinsics.distortion_model = comm_rgb_image.getDistortion_model();

	arma::mat comm_color_distortion = arma::zeros(1,5);

	comm_color_distortion = comm_rgb_image.get_distortion();
	color_intrinsics.distortion_coeffs[0] = comm_color_distortion(0,0);
	color_intrinsics.distortion_coeffs[1] = comm_color_distortion(0,1);
	color_intrinsics.distortion_coeffs[2] = comm_color_distortion(0,2);
	color_intrinsics.distortion_coeffs[3] = comm_color_distortion(0,3);
	color_intrinsics.distortion_coeffs[4] = comm_color_distortion(0,4);
	}

	// get depth intrinsics
	{

	DomainVision::CommDepthImage comm_depth_image = rgbd_image.getDepth_image();

	arma::mat comm_depth_intrinsic = arma::zeros(4,4);
	comm_depth_intrinsic = comm_depth_image.get_intrinsic();

	depth_intrinsics.fx 	= comm_depth_intrinsic(0,0);
	depth_intrinsics.fy 	= comm_depth_intrinsic(1,1);
	depth_intrinsics.cx 	= comm_depth_intrinsic(0,2);
	depth_intrinsics.cy 	= comm_depth_intrinsic(1,2);
	depth_intrinsics.cols 	= comm_depth_image.getWidth();
	depth_intrinsics.rows 	= comm_depth_image.getHeight();
	depth_intrinsics.distortion_model = comm_depth_image.getDistortion_model();

	arma::mat comm_depth_distortion = arma::zeros(1,5);

	comm_depth_distortion = comm_depth_image.get_distortion();
	depth_intrinsics.distortion_coeffs[0] = comm_depth_distortion(0,0);
	depth_intrinsics.distortion_coeffs[1] = comm_depth_distortion(0,1);
	depth_intrinsics.distortion_coeffs[2] = comm_depth_distortion(0,2);
	depth_intrinsics.distortion_coeffs[3] = comm_depth_distortion(0,3);
	depth_intrinsics.distortion_coeffs[4] = comm_depth_distortion(0,4);

    //get depth extrinsics
	arma::mat comm_depth_extrinsics = arma::zeros(1,12);
	comm_depth_extrinsics = comm_depth_image.get_extrinsic();

	depth_to_color_extrinsics.rotation[0] =     comm_depth_extrinsics(0,0);
	depth_to_color_extrinsics.rotation[1] =     comm_depth_extrinsics(0,1);
	depth_to_color_extrinsics.rotation[2] =     comm_depth_extrinsics(0,2);
	depth_to_color_extrinsics.rotation[3] =     comm_depth_extrinsics(0,3);
	depth_to_color_extrinsics.rotation[4] =     comm_depth_extrinsics(0,4);
	depth_to_color_extrinsics.rotation[5] =     comm_depth_extrinsics(0,5);
	depth_to_color_extrinsics.rotation[6] =     comm_depth_extrinsics(0,6);
	depth_to_color_extrinsics.rotation[7] =     comm_depth_extrinsics(0,7);
	depth_to_color_extrinsics.rotation[8] =     comm_depth_extrinsics(0,8);

	depth_to_color_extrinsics.translation[0] = comm_depth_extrinsics(0,9);
	depth_to_color_extrinsics.translation[1] = comm_depth_extrinsics(0,10);
	depth_to_color_extrinsics.translation[2] = comm_depth_extrinsics(0,11);
	}

	first_image_flag = false;

	//display_intrinsics_extrinsics();

}

void RGBDVisualization::display_intrinsics_extrinsics()
{

	std::cout<<TEXT_COLOR_RESET<< TEXT_COLOR_GREEN; // make the screen output Green
	std::cout << "---------------------------------------------------------------" <<std::endl;
	std::cout <<std::setw(40)<<"Colour Intrinsics"<<std::setw(20)<<"Depth Intrinsics"<<std::endl;
	std::cout << "-----------------------------------------------------------------------------------------" <<std::endl;
	std::cout <<std::setw(20)<<  "Cx"<<std::setw(20)<< color_intrinsics.cx<<std::setw(20)<<  depth_intrinsics.cx<<std::endl;
	std::cout <<std::setw(20)<<  "Cy"<<std::setw(20)<< color_intrinsics.cy<<std::setw(20)<<  depth_intrinsics.cy<<std::endl;
	std::cout <<std::setw(20)<<  "Fx"<<std::setw(20)<< color_intrinsics.fx<<std::setw(20)<<  depth_intrinsics.fx<<std::endl;
	std::cout <<std::setw(20)<<  "Fy"<<std::setw(20)<< color_intrinsics.fy<<std::setw(20)<<  depth_intrinsics.fy<<std::endl;


	std::stringstream rgb_size, depth_size;
	rgb_size << color_intrinsics.cols<<" x "<<  color_intrinsics.rows;
	depth_size << depth_intrinsics.cols<<" x "<<  depth_intrinsics.rows;

	std::cout <<std::setw(20)<<"Size"<<std::setw(20)<< rgb_size.str()<<std::setw(20)<<depth_size.str()<<std::endl;

	std::cout <<std::setw(20)<<  "Distortion model"<<std::setw(20) <<((color_intrinsics.distortion_model==DomainVision::ImageDistortionModel::BROWN_CONRADY)?"Brown Conrady":"None")
		        		  << std::setw(20)<<((depth_intrinsics.distortion_model==DomainVision::ImageDistortionModel::BROWN_CONRADY)?"Brown Conrady":"None")<<std::endl;

	std::cout <<std::setw(20)<<  "Distortion coeffs"<<std::setw(20)<<color_intrinsics.distortion_coeffs[0] <<std::setw(20)<<depth_intrinsics.distortion_coeffs[0]<<std::endl;
	std::cout <<std::setw(40)<<color_intrinsics.distortion_coeffs[1] <<std::setw(20)<<depth_intrinsics.distortion_coeffs[1]<<std::endl;
	std::cout <<std::setw(40)<<color_intrinsics.distortion_coeffs[2] <<std::setw(20)<<depth_intrinsics.distortion_coeffs[2]<<std::endl;
	std::cout <<std::setw(40)<<color_intrinsics.distortion_coeffs[3] <<std::setw(20)<<depth_intrinsics.distortion_coeffs[3]<<std::endl;
	std::cout <<std::setw(40)<<color_intrinsics.distortion_coeffs[4] <<std::setw(20)<<depth_intrinsics.distortion_coeffs[4]<<std::endl;
	std::cout << "-----------------------------------------------------------------------------------------" <<std::endl;
	std::cout <<"Depth to Colour Extrinsics"<<std::endl;
	std::cout << "-----------------------------------------------------------------------------------------" <<std::endl;

	std::cout <<std::setw(20)<<  "Rotation"<<" = "
			<<std::setw( 2)<< "[ "<<std::setw(13)<<depth_to_color_extrinsics.rotation[0] <<std::setw(15)<< depth_to_color_extrinsics.rotation[3]<<std::setw(15)<< depth_to_color_extrinsics.rotation[6]<<" ]"<<std::endl
			<<std::setw(25)<< "[ "<<std::setw(13)<<depth_to_color_extrinsics.rotation[1] <<std::setw(15)<< depth_to_color_extrinsics.rotation[4]<<std::setw(15)<< depth_to_color_extrinsics.rotation[7]<<" ]"<<std::endl
			<<std::setw(25)<< "[ "<<std::setw(13)<<depth_to_color_extrinsics.rotation[2] <<std::setw(15)<< depth_to_color_extrinsics.rotation[5]<<std::setw(15)<< depth_to_color_extrinsics.rotation[8]<<" ]"<<std::endl;

	std::cout <<std::endl;
	std::cout <<std::setw(20)<< "Translation"<<" = "<<"[ "<<std::setw(13) << depth_to_color_extrinsics.translation[0] <<std::setw(15) << depth_to_color_extrinsics.translation[1]<<std::setw(15) << depth_to_color_extrinsics.translation[2]<<" ]"<<std::endl;
	std::cout <<std::endl;
	std::cout << "-----------------------------------------------------------------------------------------" <<std::endl;
	std::cout<< TEXT_COLOR_RESET; // make the screen output normal


}
