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

#ifndef REALSENSEVISUALIZATION_H_
#define REALSENSEVISUALIZATION_H_

#include "AbstractVisualization.hh"
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CPointCloud.h>
#include <DomainVision/CommRGBDImage.hh>

#define TEXT_COLOR_RESET   "\033[0m"
#define TEXT_COLOR_GREEN   "\033[32m"      /* Green */
#include <iostream>
#include <iomanip>

class RGBDVisualization: public AbstractVisualization {
private:
	struct ColPoint3d
		{
			ColPoint3d(float x, float y, float z, float r, float g, float b)
			{
				this->x = x;
				this->y = y;
				this->z = z;

				this->r = r;
				this->g = g;
				this->b = b;
			}

			float x, y, z;
			float r, g, b;
		};

	struct st_intrinsics{
		int cols, rows;
		float cx, cy, fx, fy;
		float distortion_coeffs[5];
		DomainVision::ImageDistortionModel distortion_model;
	}color_intrinsics, depth_intrinsics;

	//Depth extrinsics with respect to color
	struct st_extrinsics{
		float rotation[9];
		float translation[3];
	}depth_to_color_extrinsics;

	bool first_image_flag;  /**use this flag to read intrinsic and extrinsic parameters from the first received RGBD image**/

public:
	RGBDVisualization(CDisplayWindow3D& window3D, const std::string& identifier);
	virtual ~RGBDVisualization();

	void displayImage(DomainVision::CommRGBDImage& image);
    void clear();
    void inline calcPointXYZ (const uint32_t& r, const uint32_t& c, const float &depth_val_meters, float &x, float &y, float &z,
    		                             const st_intrinsics& intrinsics, const st_extrinsics& extrinsics);
    void inline transform (const st_extrinsics& extrinsics, float &x, float &y, float &z);
    void inline deproject(const st_intrinsics& intrinsics, const uint32_t& r, const uint32_t& c, const float &depth_val_meters, float &out_x, float &out_y, float &out_z);
    void inline project(const st_intrinsics& intrinsics, uint32_t& out_r, uint32_t& out_c, const float &in_x, const float &in_y, const float &in_z);
#ifdef WITH_MRPT_2_0_VERSION
    void createColorPointCloud (opengl::CPointCloudColoured::Ptr cloud, DomainVision::CommVideoImage *comm_color_image,
        		                                                                       DomainVision::CommDepthImage *comm_depth_image);
#else
    void createColorPointCloud (opengl::CPointCloudColouredPtr cloud, DomainVision::CommVideoImage *comm_color_image,
    		                                                                       DomainVision::CommDepthImage *comm_depth_image);
#endif
    void read_intrinsics_extrinsics(const DomainVision::CommRGBDImage& rgbd_image);
    void display_intrinsics_extrinsics();
};

#endif /* REALSENSEVISUALIZATION_H_ */
