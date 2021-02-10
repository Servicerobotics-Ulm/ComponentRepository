//--------------------------------------------------------------------------
// Code generated by the SmartSoft MDSD Toolchain Version 0.10.3
// The SmartSoft Toolchain has been developed by:
//
// ZAFH Servicerobotic Ulm
// Christian Schlegel (schlegel@hs-ulm.de)
// University of Applied Sciences
// Prittwitzstr. 10
// 89075 Ulm (Germany)
//
// Information about the SmartSoft MDSD Toolchain is available at:
// smart-robotics.sourceforge.net
//
// This file is generated once. Modify this file to your needs.
// If you want the toolchain to re-generate this file, please
// delete it before running the code generator.
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//
//  Copyright (C) 2010 Manuel Wopfner, Matthias Lutz
//
//        schlegel@hs-ulm.de
//        lutz@hs-ulm.de
//
//        ZAFH Servicerobotic Ulm
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm
//        Germany
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 2.1
//  of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this library; if not, write to the Free Software Foundation, Inc.,
//  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//
//  This work is based on previous work by the folks from PlayerStage.
//
//--------------------------------------------------------------------------

//----------------------------------------------------------------------------
//
// CREDITS:
//
// The code for the amcl algorithm was taken from the
// Playerstage Project, which is distributed under GPL, and you can find at
// http://playerstage.sourceforge.net/
//
// Player - One Hell of a Robot Server
// Copyright (C) 2000
//    Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
//
//----------------------------------------------------------------------------

#ifndef _ACMLTYPES_HH
#define _ACMLTYPES_HH

#include "map/map.h"
#include "pf/pf.h"
#include "sensors/amcl_odom.h"
#include "sensors/amcl_laser.h"
#include "sensors/amcl_visual_tag.h"

using namespace amcl;

// Pose hypothesis
typedef struct {
	// Total weight (weights sum to 1)
	double weight;

	// Mean of pose esimate
	pf_vector_t pf_pose_mean;

	// Covariance of pose estimate
	pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

enum class amcl_sensor_to_use
{
	LASER_ONLY,
	VISUAL_TAGS_ONLY,
	LASER_PLUS_VISUAL_TAGS
};


// Map info
typedef struct {
	double resolution;
	int negate;
	double occ_threshold, free_threshold;
	std::string mapfname;
	pf_vector_t origin;

} amcl_map_info_t;

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

double normalize(double z);
double angle_diff(double a, double b);
double pi_to_pi(double angle);


std::ostream& operator<<(std::ostream& in, const amcl_hyp_t& hyp);
std::ostream& operator<<(std::ostream& in, const pf_vector_t& pose);
pf_vector_t operator-(const pf_vector_t& a, const pf_vector_t& b);
amcl_sensor_to_use get_sensors_to_use(int val);
int get_map_info(const std::string yaml_file_name, amcl_map_info_t& map_info);
#endif
